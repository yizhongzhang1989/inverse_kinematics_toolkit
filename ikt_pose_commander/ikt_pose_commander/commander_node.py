#!/usr/bin/env python3
"""ikt_pose_commander — accept a Cartesian target pose, solve IK, move the arm.

Pipeline per target (one configured kinematic group = frame + joints + a
controller pair):

    PoseStamped  ->  (TF to base)  ->  ikt_inverse_kinematics solve
                 ->  SAFETY GATE   ->  command the arm

This node ACTUALLY commands the robot (unlike ikt_inverse_kinematics, which is
advisory only). It is therefore safety-gated:

  * starts **disabled** — no motion until ``~/enable`` is called;
  * rejects IK solutions that are not ``reachable``;
  * rejects solutions whose joint change from the current measured pose exceeds
    ``max_step_rad`` (jump protection);
  * **speed-limits** every JTC move (duration from max joint delta / max speed);
  * holds (does not command) on stale ``/joint_states`` or missing model.

Two command modes (param ``command_mode``):

  * ``jtc`` (default, SAFE): each accepted target -> ONE speed-limited
    ``FollowJointTrajectory`` goal to the arm's JointTrajectoryController.
  * ``fpc`` (streaming): each accepted target -> a ``Float64MultiArray`` to the
    arm's forward_position_controller ``commands`` topic (continuous servoing;
    relies on the rm_control hardware shaper for smoothing).

Run one instance per arm (set ``controlled_frame`` / ``joints`` / controller
names), mirroring the cartesian_control_manager left/right pattern.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    _HAS_FJT = True
except ImportError:  # pragma: no cover
    _HAS_FJT = False

try:
    from controller_manager_msgs.srv import SwitchController, ListControllers
    _HAS_CM = True
except ImportError:  # pragma: no cover
    _HAS_CM = False

# In-process IK (the advisory solver). Pure-Python core: no topic round-trip.
try:
    from ikt_core.robot_model import RobotModel
    from ikt_core import ik_core
    from ikt_core.tasks import Task
    _IK_IMPORT_ERROR: Optional[str] = None
except Exception as _exc:  # noqa: BLE001  pragma: no cover
    RobotModel = None  # type: ignore
    ik_core = None  # type: ignore
    Task = None  # type: ignore
    _IK_IMPORT_ERROR = f"{type(_exc).__name__}: {_exc}"


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


# Runtime-config keys, split by HOW they apply. ``_LIVE_KEYS`` take effect
# immediately (even while enabled); ``_STRUCTURAL_KEYS`` change the kinematic
# group / controllers and are refused while enabled (disable first). Launch
# params, the ``~/configure`` topic, AND ``ros2 param set`` all funnel through
# the same apply path -> ONE unified way to set any of them, at launch or live.
_LIVE_KEYS = (
    "base_frame", "max_joint_speed", "min_move_time", "max_step_rad",
    "joint_states_stale_after", "joint_centering_weight", "damping",
    "tol_pos", "tol_ori", "max_iters", "default_stiffness",
)
_STRUCTURAL_KEYS = (
    "controlled_frame", "joints", "jtc_controller", "fpc_controller",
    "command_mode",
)


class PoseCommander(Node):
    def __init__(self) -> None:
        super().__init__("ikt_pose_commander")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_pose_topic", "~/target_pose")
        # base_frame: the frame a target with an EMPTY ``header.frame_id`` is
        # assumed to be expressed in. Every target is transformed into the model
        # root for the solver, so any TF frame works as a reference. Live /
        # runtime-settable (see _LIVE_KEYS).
        self.declare_parameter("base_frame", "")
        # Robot-INDEPENDENT: everything below is empty by default. The node
        # builds its model from the live /robot_description and is configured at
        # runtime (``~/configure`` topic or the dashboard) by naming just the
        # link to control; the joints (kinematic path to that link) and the
        # JTC/FPC controllers (matched in /controller_manager) are auto-derived.
        # The params are still honoured if set, so an explicit launch config
        # also works.
        self.declare_parameter("controlled_frame", "")
        self.declare_parameter("joints", [""])
        self.declare_parameter("jtc_controller", "")
        self.declare_parameter("fpc_controller", "")
        self.declare_parameter("command_mode", "jtc")          # jtc | fpc
        self.declare_parameter("start_enabled", False)         # SAFETY: off
        self.declare_parameter("switch_controllers", True)
        self.declare_parameter("controller_manager", "/controller_manager")
        # solver
        self.declare_parameter("default_stiffness",
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_centering_weight", 1e-2)
        self.declare_parameter("damping", 1e-2)
        self.declare_parameter("tol_pos", 1e-3)
        self.declare_parameter("tol_ori", 3.5e-3)
        self.declare_parameter("max_iters", 200)
        # safety
        self.declare_parameter("max_joint_speed", 0.5)         # rad/s (JTC dur)
        self.declare_parameter("min_move_time", 0.5)           # s
        self.declare_parameter("max_step_rad", 0.8)            # jump reject
        self.declare_parameter("joint_states_stale_after", 0.5)  # s
        self.declare_parameter("status_rate_hz", 10.0)

        gp = self.get_parameter
        self._desc_topic = str(gp("robot_description_topic").value)
        self._js_topic = str(gp("joint_states_topic").value)
        self._target_topic = str(gp("target_pose_topic").value)
        self._base_frame = str(gp("base_frame").value or "")
        # Active config (may start empty -> unconfigured). Filled by _apply_config.
        self._frame = str(gp("controlled_frame").value or "")
        self._joints = [str(j) for j in (gp("joints").value or []) if str(j)]
        self._jtc = str(gp("jtc_controller").value or "")
        self._fpc = str(gp("fpc_controller").value or "")
        self._mode = str(gp("command_mode").value).strip().lower()
        self._do_switch = bool(gp("switch_controllers").value)
        self._cm = str(gp("controller_manager").value or "/controller_manager")
        self._stiffness = [float(v) for v in gp("default_stiffness").value]
        self._centering = float(gp("joint_centering_weight").value)
        self._damping = float(gp("damping").value)
        self._tol_pos = float(gp("tol_pos").value)
        self._tol_ori = float(gp("tol_ori").value)
        self._max_iters = int(gp("max_iters").value)
        self._max_speed = max(1e-3, float(gp("max_joint_speed").value))
        self._min_time = max(0.0, float(gp("min_move_time").value))
        self._max_step = float(gp("max_step_rad").value)
        self._js_stale = float(gp("joint_states_stale_after").value)
        status_rate = max(0.5, float(gp("status_rate_hz").value))

        if self._mode not in ("jtc", "fpc"):
            raise ValueError("command_mode must be 'jtc' or 'fpc'")
        if _IK_IMPORT_ERROR is not None:
            self.get_logger().error(
                "ikt_core import failed: %s — the commander "
                "cannot solve IK. Is the package built/sourced?"
                % _IK_IMPORT_ERROR)

        # ---- state (guarded by _lock) -----------------------------------
        self._lock = threading.Lock()
        self._urdf = ""
        self._model: Optional[RobotModel] = None
        self._joint_pos: Dict[str, float] = {}
        self._js_stamp = 0.0
        self._enabled = False
        self._configured = False
        self._last_msg = "initialised (disabled, unconfigured)"
        # Throttle repeated _set_msg logs: a high-rate target stream into a
        # disabled/unconfigured commander would otherwise flood the log at the
        # stream rate. We always update the status field, but only emit an INFO
        # line when the message text changes or a heartbeat interval elapses.
        self._last_logged_msg = ""
        self._last_log_time = 0.0
        self._last_target_stamp = 0.0
        self._last_solution = None
        self._last_reason = ""
        self._last_delta = 0.0
        self._goal_handle = None
        # a pending config request (from launch params, ~/configure, or a staged
        # ``ros2 param set``) to apply once the model is available
        self._cfg_dirty = False
        self._req_cfg: Optional[dict] = None
        if self._frame:
            self._req_cfg = {"controlled_frame": self._frame,
                             "joints": self._joints or None,
                             "jtc_controller": self._jtc or None,
                             "fpc_controller": self._fpc or None,
                             "command_mode": self._mode}

        self._cbg = ReentrantCallbackGroup()
        cb = self._cbg

        # ---- pubs / subs ------------------------------------------------
        self.create_subscription(String, self._desc_topic,
                                 self._on_urdf, _latched_qos(),
                                 callback_group=cb)
        self.create_subscription(JointState, self._js_topic,
                                 self._on_js, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(PoseStamped, self._target_topic,
                                 self._on_target, 10, callback_group=cb)
        self.create_subscription(String, "~/configure",
                                 self._on_configure, 10, callback_group=cb)
        # Controller-dependent endpoints are (re)created on configure.
        self._fpc_pub = None
        self._jtc_client = None
        self._status_pub = self.create_publisher(String, "~/status", 10)

        # ---- switch + list clients --------------------------------------
        self._cli_switch = None
        self._cli_list = None
        if _HAS_CM:
            self._cli_switch = self.create_client(
                SwitchController, f"{self._cm}/switch_controller",
                callback_group=cb)
            self._cli_list = self.create_client(
                ListControllers, f"{self._cm}/list_controllers",
                callback_group=cb)

        # ---- TF (optional, for PoseStamped in non-base frames) ----------
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        except Exception:  # noqa: BLE001  pragma: no cover
            self._tf_buffer = None

        # ---- services ---------------------------------------------------
        self.create_service(Trigger, "~/enable", self._srv_enable,
                            callback_group=cb)
        self.create_service(Trigger, "~/disable", self._srv_disable,
                            callback_group=cb)
        self.create_service(Trigger, "~/stop", self._srv_disable,
                            callback_group=cb)

        self.create_timer(1.0 / status_rate, self._publish_status,
                          callback_group=cb)

        # Unified runtime config via standard parameters: ``ros2 param set``.
        # Live tunables apply at once; structural ones are staged and applied by
        # the status timer (so this callback never blocks on discovery). Same
        # effect as the ``~/configure`` topic.
        self.add_on_set_parameters_callback(self._on_set_params)

        if bool(gp("start_enabled").value):
            # honoured only after the first model+js arrive; _try_enable guards.
            self.get_logger().warn(
                "start_enabled=true — commander will engage as soon as a model "
                "and joint states are available. Ensure the area is clear.")
            self._want_enable = True
        else:
            self._want_enable = False

        self.get_logger().info(
            "ikt_pose_commander up (DISABLED, %s). Reads /robot_description "
            "online; configure by naming the link to control (~/configure or "
            "the dashboard), then ~/enable. mode=%s"
            % ("pre-configured for '%s'" % self._frame if self._frame
               else "UNCONFIGURED", self._mode))

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_urdf(self, msg: String) -> None:
        if not msg.data:
            return
        with self._lock:
            if msg.data == self._urdf and self._model is not None:
                return
            self._urdf = msg.data
        if RobotModel is None:
            return
        try:
            model = RobotModel(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("failed to build model: %r" % exc)
            return
        with self._lock:
            self._model = model
        self.get_logger().info(
            "built kinematic model from /robot_description: %d DOF, %d links."
            % (model.nq, len(model.link_frame_names())))
        # Apply any pending config (launch param or an earlier ~/configure that
        # arrived before the model).
        with self._lock:
            req = self._req_cfg
        if req is not None:
            ok, m = self._apply_config(req)
            self._set_msg(("configured: " if ok else "configure failed: ") + m)
        if self._want_enable:
            self._want_enable = False
            self._try_enable()

    def _on_js(self, msg: JointState) -> None:
        now = time.monotonic()
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_pos[n] = float(p)
            self._js_stamp = now

    # ------------------------------------------------------------------ #
    # Runtime configuration (robot-independent): name the link, derive the rest
    # ------------------------------------------------------------------ #
    def _on_configure(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self._set_msg("configure ignored: bad JSON (%s)" % exc)
            return
        if not isinstance(req, dict):
            self._set_msg("configure ignored: need a JSON object")
            return
        if not any(k in req for k in (_LIVE_KEYS + _STRUCTURAL_KEYS)):
            self._set_msg("configure ignored: no known config keys")
            return
        # Live tunables apply immediately, even before a model exists.
        live = self._apply_live(req)
        if not any(k in req for k in _STRUCTURAL_KEYS):
            self._set_msg("configured (live): " + (live or "no changes"))
            return
        with self._lock:
            have_model = self._model is not None
        if not have_model:
            with self._lock:
                base = dict(self._req_cfg or {})
                base.update(req)
                self._req_cfg = base
            self._set_msg("configure queued: waiting for /robot_description"
                          + (("; live: " + live) if live else ""))
            return
        ok, m = self._apply_structural(req)
        if live:
            m = "live: " + live + "; " + m
        self._set_msg(("configured: " if ok else "configure failed: ") + m)

    def _on_set_params(self, params):
        """Unified runtime setter through standard parameters.

        Live tunables apply immediately; structural params are staged and
        applied by the status timer (so we never block on controller discovery
        inside this callback), and are rejected while enabled to keep the
        parameter store consistent with the active config.
        """
        live: dict = {}
        structural: dict = {}
        for p in params:
            if p.name in _LIVE_KEYS:
                live[p.name] = p.value
            elif p.name in _STRUCTURAL_KEYS:
                structural[p.name] = p.value
        if structural:
            with self._lock:
                enabled = self._enabled
            if enabled:
                return SetParametersResult(
                    successful=False,
                    reason="disable before changing structural config (%s)"
                    % ", ".join(sorted(structural)))
        if live:
            m = self._apply_live(live)
            if m:
                self._set_msg("param set: " + m)
        if structural:
            with self._lock:
                base = dict(self._req_cfg or {})
                base.update(structural)
                self._req_cfg = base
                self._cfg_dirty = True
        return SetParametersResult(successful=True)

    def _apply_config(self, req: dict):
        """Unified apply: live tunables + (optional) structural reconfig.

        Launch params and every runtime setter (``~/configure`` topic,
        ``ros2 param set``) funnel through here so there is a single behaviour.
        Live keys always apply; structural keys are refused while enabled.
        """
        live = self._apply_live(req)
        if any(k in req for k in _STRUCTURAL_KEYS):
            ok, m = self._apply_structural(req)
            return ok, (("live: " + live + "; " + m) if live else m)
        return True, ("live: " + live if live else "no changes")

    def _apply_live(self, req: dict) -> str:
        """Apply pose-independent tunables that are safe to change anytime.

        Returns a short description of what changed ("" if nothing). Covers the
        base reference frame, the solver weights/tolerances, and the safety
        limits. Robust to string values (so JSON and typed params both work).
        """
        changed: List[str] = []
        with self._lock:
            for key, attr, lo in (
                ("max_joint_speed", "_max_speed", 1e-3),
                ("min_move_time", "_min_time", 0.0),
                ("max_step_rad", "_max_step", None),
                ("joint_states_stale_after", "_js_stale", None),
                ("joint_centering_weight", "_centering", None),
                ("damping", "_damping", None),
                ("tol_pos", "_tol_pos", None),
                ("tol_ori", "_tol_ori", None),
            ):
                if req.get(key) is None:
                    continue
                try:
                    v = float(req[key])
                except (TypeError, ValueError):
                    continue
                if lo is not None:
                    v = max(lo, v)
                setattr(self, attr, v)
                changed.append("%s=%g" % (key, v))
            if req.get("max_iters") is not None:
                try:
                    self._max_iters = max(1, int(req["max_iters"]))
                    changed.append("max_iters=%d" % self._max_iters)
                except (TypeError, ValueError):
                    pass
            if req.get("default_stiffness") is not None:
                try:
                    s = [float(x) for x in req["default_stiffness"]]
                    if len(s) == 6:
                        self._stiffness = s
                        changed.append("default_stiffness")
                except (TypeError, ValueError):
                    pass
            if "base_frame" in req and req["base_frame"] is not None:
                self._base_frame = str(req["base_frame"] or "")
                changed.append("base_frame=%s" % (self._base_frame or "(root)"))
        return ", ".join(changed)

    def _apply_structural(self, req: dict):
        """Apply a kinematic-group / controller reconfig. Refused while enabled.

        ``controlled_frame`` (or ``joints``) selects the group; when both are
        omitted the current group is kept and only e.g. ``command_mode`` changes.
        ``joints`` and the JTC/FPC controller names are optional; when omitted
        they are derived from the model (joints = kinematic path to the link)
        and from /controller_manager (controllers whose required command
        interfaces cover those joints).
        """
        with self._lock:
            if self._enabled:
                return False, "refused: disable before reconfiguring"
            model = self._model
            cur_frame, cur_joints = self._frame, list(self._joints)
            cur_jtc, cur_fpc, cur_mode = self._jtc, self._fpc, self._mode
            cur_configured = self._configured
        if model is None:
            return False, "no model yet"

        changing_group = ("controlled_frame" in req) or ("joints" in req)
        if changing_group:
            frame = str(req.get("controlled_frame") or cur_frame or "")
            if not frame or not model.has_frame(frame):
                return False, f"unknown link/frame '{frame}'"
            joints = req.get("joints")
            if joints:
                joints = [str(j) for j in joints if str(j)]
                missing = [j for j in joints if j not in model.joint_names]
                if missing:
                    return False, f"joints not in URDF: {missing}"
            else:
                joints = model.supporting_joints(frame)
                if not joints:
                    return False, f"no movable joints support frame '{frame}'"
        else:
            if not cur_configured:
                return False, "not configured; set controlled_frame first"
            frame, joints = cur_frame, cur_joints

        mode = str(req.get("command_mode") or cur_mode).strip().lower()
        if mode not in ("jtc", "fpc"):
            return False, "command_mode must be 'jtc' or 'fpc'"

        # controllers: explicit > kept (only if the group is unchanged) > derived
        jtc = str(req.get("jtc_controller") or "")
        fpc = str(req.get("fpc_controller") or "")
        if not changing_group:
            jtc = jtc or cur_jtc
            fpc = fpc or cur_fpc
        if not jtc or not fpc:
            disc = self._discover_controllers(joints)
            jtc = jtc or disc.get("jtc", "")
            fpc = fpc or disc.get("fpc", "")
        # the controller for the ACTIVE mode must be known; the other is optional
        if mode == "jtc" and not jtc:
            return False, ("no JointTrajectoryController found driving %s; "
                           "set jtc_controller explicitly" % joints)
        if mode == "fpc" and not fpc:
            return False, ("no ForwardCommandController found driving %s; "
                           "set fpc_controller explicitly" % joints)

        with self._lock:
            self._frame, self._joints = frame, joints
            self._jtc, self._fpc, self._mode = jtc, fpc, mode
            self._configured = True
        self._rebuild_clients()
        return True, (f"link={frame} joints={len(joints)} mode={mode} "
                      f"jtc={jtc or '-'} fpc={fpc or '-'}")

    def _discover_controllers(self, joints) -> dict:
        """Find JTC + FPC controllers whose command interfaces cover ``joints``.

        Uses /controller_manager/list_controllers and matches each controller's
        ``required_command_interfaces`` (populated for active AND inactive
        controllers) against ``{joint}/position``. Returns {'jtc':.., 'fpc':..}.
        """
        out = {"jtc": "", "fpc": ""}
        if not _HAS_CM or self._cli_list is None:
            return out
        if not self._cli_list.wait_for_service(timeout_sec=3.0):
            return out
        fut = self._cli_list.call_async(ListControllers.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=4.0):
            return out
        resp = fut.result()
        want = {f"{j}/position" for j in joints}
        best = {"jtc": -1, "fpc": -1}
        for ctl in getattr(resp, "controller", []):
            req_if = set(getattr(ctl, "required_command_interfaces", []) or [])
            cover = len(want & req_if)
            if cover == 0:
                continue
            t = ctl.type
            if t.endswith("JointTrajectoryController") and cover > best["jtc"]:
                best["jtc"] = cover
                out["jtc"] = ctl.name
            elif t.endswith("ForwardCommandController") and cover > best["fpc"]:
                best["fpc"] = cover
                out["fpc"] = ctl.name
        return out

    def _rebuild_clients(self) -> None:
        """(Re)create the FPC publisher + JTC action client for current names."""
        with self._lock:
            jtc, fpc = self._jtc, self._fpc
        # FPC command publisher
        if self._fpc_pub is not None:
            try:
                self.destroy_publisher(self._fpc_pub)
            except Exception:  # noqa: BLE001
                pass
            self._fpc_pub = None
        if fpc:
            self._fpc_pub = self.create_publisher(
                Float64MultiArray, f"/{fpc}/commands", 10)
        # JTC action client
        if self._jtc_client is not None:
            try:
                self._jtc_client.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._jtc_client = None
        if jtc and _HAS_FJT:
            self._jtc_client = ActionClient(
                self, FollowJointTrajectory,
                f"/{jtc}/follow_joint_trajectory", callback_group=self._cbg)

    # ------------------------------------------------------------------ #
    # Main path: target -> solve -> gate -> command
    # ------------------------------------------------------------------ #
    def _on_target(self, msg: PoseStamped) -> None:
        with self._lock:
            self._last_target_stamp = time.monotonic()
            enabled = self._enabled
            model = self._model
            configured = self._configured
        if not configured:
            self._set_msg("target ignored: UNCONFIGURED (set the link via "
                          "~/configure or the dashboard)")
            return
        if not enabled:
            self._set_msg("target ignored: commander DISABLED (call ~/enable)")
            return
        if model is None:
            self._set_msg("target ignored: no robot_description yet")
            return
        if not self._js_fresh():
            self._set_msg("target ignored: /joint_states stale")
            return

        xyz, quat = self._resolve_pose(msg)
        if xyz is None:
            self._set_msg("target ignored: TF transform unavailable")
            return

        seed = self._build_seed(model)
        sol = self._solve(model, seed, xyz, quat)
        with self._lock:
            self._last_solution = sol
            self._last_reason = sol.reason.value

        if not sol.reachable:
            self._set_msg("target REJECTED: unreachable (%s)" % sol.reason.value)
            return

        # jump protection: max joint change over the CONTROLLED joints vs current
        q_cmd = {j: float(sol.q[model.q_index(j)]) for j in self._joints}
        with self._lock:
            cur = {j: self._joint_pos.get(j) for j in self._joints}
        if any(cur[j] is None for j in self._joints):
            self._set_msg("target ignored: current joint pos unknown")
            return
        max_delta = max(abs(q_cmd[j] - cur[j]) for j in self._joints)
        with self._lock:
            self._last_delta = max_delta
        if max_delta > self._max_step:
            self._set_msg(
                "target REJECTED: step %.3f rad > max_step_rad %.3f "
                "(jump protection)" % (max_delta, self._max_step))
            return

        if self._mode == "jtc":
            self._command_jtc(q_cmd, max_delta)
        else:
            self._command_fpc(q_cmd)

    def _build_seed(self, model: "RobotModel") -> np.ndarray:
        q = model.neutral()
        with self._lock:
            jp = dict(self._joint_pos)
        for jn in model.joint_names:
            if jn in jp:
                q[model.q_index(jn)] = jp[jn]
        return q

    def _solve(self, model, seed, xyz, quat):
        params = ik_core.SolveParams(
            max_iters=self._max_iters, tol_pos=self._tol_pos,
            tol_ori=self._tol_ori, damping=self._damping,
            joint_centering_weight=self._centering)
        task = Task(self._frame, tuple(float(v) for v in xyz),
                    tuple(float(v) for v in quat), tuple(self._stiffness))
        return ik_core.solve(model, seed, [task], params=params,
                             active_joints=self._joints)

    def _resolve_pose(self, msg: PoseStamped):
        """Return (xyz, quat_wxyz) for the target expressed in the SOLVER frame
        (the model root), or (None, None).

        Pinocchio expresses the controlled frame's placement in the model root
        frame, so every target must be resolved into that root frame before the
        solve. ``base_frame`` is the *default* frame a bare target (empty
        ``header.frame_id``) is interpreted in; an explicit ``header.frame_id``
        always wins. Either is transformed to the root via TF, so any TF frame
        (a robot link or an external frame, e.g. a camera) is a valid reference.
        """
        p, o = msg.pose.position, msg.pose.orientation
        xyz = np.array([p.x, p.y, p.z])
        quat = np.array([o.w, o.x, o.y, o.z])
        n = float(np.linalg.norm(quat))
        quat = quat / n if n > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])

        root = self._model_root()
        with self._lock:
            base = self._base_frame
        # The frame the target is expressed in: explicit frame_id wins, else the
        # configured base_frame, else the root itself.
        src = msg.header.frame_id or base or root
        if self._tf_buffer is None or src == root:
            return xyz, quat
        try:
            import tf2_geometry_msgs  # noqa: F401  (registers Pose transforms)
            stamped = msg
            if not msg.header.frame_id:
                # stamp the assumed source frame so TF can resolve it (latest)
                stamped = PoseStamped()
                stamped.header.frame_id = src
                stamped.pose = msg.pose
            out = self._tf_buffer.transform(
                stamped, root, timeout=rclpy.duration.Duration(seconds=0.2))
            p, o = out.pose.position, out.pose.orientation
            return (np.array([p.x, p.y, p.z]),
                    np.array([o.w, o.x, o.y, o.z]))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "TF resolve %s->%s failed: %r" % (src, root, exc))
            return None, None

    def _model_root(self) -> str:
        with self._lock:
            if self._model is None:
                return "base_link"
            names = self._model.frame_names()
        return names[1] if len(names) > 1 else "base_link"

    # ------------------------------------------------------------------ #
    # Commanding
    # ------------------------------------------------------------------ #
    def _command_jtc(self, q_cmd: Dict[str, float], max_delta: float) -> None:
        if self._jtc_client is None:
            self._set_msg("cannot command: FollowJointTrajectory unavailable")
            return
        if not self._jtc_client.server_is_ready():
            self._jtc_client.wait_for_server(timeout_sec=1.0)
        if not self._jtc_client.server_is_ready():
            self._set_msg("cannot command: %s action server not ready"
                          % self._jtc)
            return
        duration = max(self._min_time, max_delta / self._max_speed)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joints)
        pt = JointTrajectoryPoint()
        pt.positions = [q_cmd[j] for j in self._joints]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        goal.trajectory.points = [pt]
        fut = self._jtc_client.send_goal_async(goal)

        def _on_resp(f):
            try:
                handle = f.result()
            except Exception as exc:  # noqa: BLE001  pragma: no cover
                self._set_msg("JTC goal error: %r" % exc)
                return
            if handle is None or not handle.accepted:
                self._set_msg("JTC goal rejected")
                return
            with self._lock:
                self._goal_handle = handle
        fut.add_done_callback(_on_resp)
        self._set_msg("JTC move sent (%.2fs, step %.3f rad)"
                      % (duration, max_delta))

    def _command_fpc(self, q_cmd: Dict[str, float]) -> None:
        m = Float64MultiArray()
        m.data = [q_cmd[j] for j in self._joints]
        self._fpc_pub.publish(m)
        self._set_msg("FPC command streamed")

    # ------------------------------------------------------------------ #
    # Enable / disable (controller switching)
    # ------------------------------------------------------------------ #
    def _srv_enable(self, request, response):
        ok, msg = self._try_enable()
        response.success = ok
        response.message = msg
        return response

    def _try_enable(self):
        with self._lock:
            model = self._model
            configured = self._configured
            mode, jtc, fpc = self._mode, self._jtc, self._fpc
        if not configured:
            return False, ("unconfigured; name the link to control via "
                           "~/configure or the dashboard first")
        if model is None:
            return False, "no robot_description yet; cannot enable"
        if not self._js_fresh():
            return False, "/joint_states stale; cannot enable"
        # Switch to the controller this mode commands.
        want = fpc if mode == "fpc" else jtc
        other = jtc if mode == "fpc" else fpc
        if not want:
            return False, f"no controller configured for mode '{mode}'"
        if self._do_switch:
            if mode == "fpc":
                # seed FPC with the CURRENT pose first so activation can't jump
                self._seed_fpc_current()
            deact = [other] if other else []
            if not self._switch(activate=[want], deactivate=deact):
                return False, self._last_msg or "controller switch failed"
        with self._lock:
            self._enabled = True
        self._set_msg("ENABLED (mode=%s, controller=%s)" % (mode, want))
        return True, "enabled"

    def _srv_disable(self, request, response):
        with self._lock:
            self._enabled = False
            mode, jtc, fpc = self._mode, self._jtc, self._fpc
        # Return to JTC, which holds the current pose.
        if self._do_switch and mode == "fpc" and jtc and fpc:
            self._switch(activate=[jtc], deactivate=[fpc])
        # cancel any in-flight JTC goal
        with self._lock:
            handle = self._goal_handle
            self._goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001  pragma: no cover
                pass
        self._set_msg("DISABLED; holding pose")
        response.success = True
        response.message = "disabled; holding pose"
        return response

    def _seed_fpc_current(self) -> None:
        with self._lock:
            cur = {j: self._joint_pos.get(j) for j in self._joints}
            pub = self._fpc_pub
        if pub is None or any(cur[j] is None for j in self._joints):
            return
        m = Float64MultiArray()
        m.data = [float(cur[j]) for j in self._joints]
        pub.publish(m)

    def _switch(self, activate: List[str], deactivate: List[str]) -> bool:
        if not _HAS_CM or self._cli_switch is None:
            self._set_msg("controller_manager unavailable")
            return False
        if not self._cli_switch.wait_for_service(timeout_sec=3.0):
            self._set_msg("/controller_manager/switch_controller unreachable")
            return False
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout.sec = 3
        fut = self._cli_switch.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=4.0):
            self._set_msg("timeout switching controllers")
            return False
        resp = fut.result()
        if not bool(getattr(resp, "ok", False)):
            self._set_msg("controller_manager refused the switch")
            return False
        return True

    # ------------------------------------------------------------------ #
    # Helpers / status
    # ------------------------------------------------------------------ #
    def _js_fresh(self) -> bool:
        with self._lock:
            stamp = self._js_stamp
        return stamp > 0.0 and (time.monotonic() - stamp) <= self._js_stale

    def _set_msg(self, msg: str) -> None:
        with self._lock:
            self._last_msg = msg
        # Dedupe + heartbeat-throttle: only log when the text changes or after a
        # quiet interval, so a 50 Hz target stream doesn't spam the console.
        now = time.monotonic()
        if msg == self._last_logged_msg and (now - self._last_log_time) < 5.0:
            return
        self._last_logged_msg = msg
        self._last_log_time = now
        self.get_logger().info("[commander] %s" % msg)

    def _publish_status(self) -> None:
        # Apply any config staged by a ``ros2 param set`` (done here, off the
        # set-parameters callback, so controller discovery never blocks it).
        if self._cfg_dirty:
            with self._lock:
                req = dict(self._req_cfg or {})
                self._cfg_dirty = False
            ok, m = self._apply_config(req)
            self._set_msg(("reconfigured: " if ok else "reconfigure failed: ")
                          + m)
        with self._lock:
            model = self._model
            enabled = self._enabled
            configured = self._configured
            sol = self._last_solution
            msg = self._last_msg
            delta = self._last_delta
            reason = self._last_reason
            frame, joints = self._frame, list(self._joints)
            jtc, fpc, mode = self._jtc, self._fpc, self._mode
        status = {
            "enabled": enabled,
            "configured": configured,
            "mode": mode,
            "controlled_frame": frame,
            "base_frame": self._base_frame or "(model root)",
            "joints": joints,
            "jtc_controller": jtc,
            "fpc_controller": fpc,
            "have_model": model is not None,
            "joint_states_fresh": self._js_fresh(),
            "last_message": msg,
            "last_reason": reason,
            "last_step_rad": delta,
            "max_step_rad": self._max_step,
            "commands_robot": True,
        }
        if model is not None:
            # URDF introspection so the dashboard can offer link/joint choices
            # entirely from the live robot (no offline config).
            status["available_links"] = model.link_frame_names()
            status["available_joints"] = list(model.joint_names)
        if sol is not None:
            status["last_solve"] = {
                "reachable": bool(sol.reachable),
                "reason": sol.reason.value,
                "max_pos_err": sol.max_pos_err(),
                "max_ori_err": sol.max_ori_err(),
            }
        m = String()
        m.data = json.dumps(status)
        self._status_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
