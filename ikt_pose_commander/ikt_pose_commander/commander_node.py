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
import math
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

# Time-synchronized joint-space streamer (the "move directly toward the target"
# trajectory generator). Kept in its own module so it is unit-testable without ROS.
from ikt_pose_commander.trajectory import SyncedJointTrajectory


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
    "base_frame", "max_joint_speed", "max_joint_accel", "min_move_time",
    "max_step_rad",
    "joint_states_stale_after", "joint_centering_weight", "damping",
    "tol_pos", "tol_ori", "max_iters", "default_stiffness",
    "allow_unreachable", "reach_gain",
    "control_rate_hz",
)
_STRUCTURAL_KEYS = (
    "controlled_frame", "joints", "fixed_joints", "jtc_controller",
    "fpc_controller", "command_mode",
)

# FPC republish deadband: skip streaming a setpoint that is essentially the
# previous one (anti-chatter, esp. best-effort holding at a joint limit).
_FPC_DEADBAND_RAD = 1e-4
# Settle band for the accel-limited FPC generator: within this distance of the
# goal the target velocity is zeroed so the joint parks cleanly (no chatter).
_FPC_SETTLE_RAD = 5e-4
# JTC re-command deadband: with a control loop, skip re-sending a trajectory
# goal when the solved config is essentially unchanged (avoids goal spam /
# restarting the same trajectory every tick).
_JTC_DEADBAND_RAD = 1e-3
# Hard cap on the control-loop rate. The effective FPC stream must stay within
# the servoj input window (see duco_servoj_internals); 250 Hz is the ceiling.
_CONTROL_RATE_MAX_HZ = 250.0


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
        # fixed_joints: joints the IK must NOT move (held at their current
        # measured value), e.g. a lifter/torso joint that is on the kinematic
        # path to the arm tip but is driven separately. They are filtered out of
        # the active joint group, so the solver freezes them and the arm solves
        # AROUND them; controller auto-discovery then matches the arm-only set.
        # Settable at launch, via ~/configure / ros2 param set, and the dashboard
        # (structural -> applied while disabled). Empty = none fixed.
        self.declare_parameter("fixed_joints", [""])
        self.declare_parameter("jtc_controller", "")
        self.declare_parameter("fpc_controller", "")
        self.declare_parameter("command_mode", "fpc")          # fpc | jtc
        self.declare_parameter("start_enabled", False)         # SAFETY: off
        self.declare_parameter("switch_controllers", True)
        self.declare_parameter("controller_manager", "/controller_manager")
        # solver. ``default_stiffness`` = per-DOF Cartesian stiffness
        # [x y z rx ry rz]: 0 lets that DOF float free, a positive value
        # constrains it (1 = fully rigid). e.g. [1 1 1 0 0 0] = position only.
        self.declare_parameter("default_stiffness",
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        # best-effort reach (Req 5): when a target is unreachable, command the
        # solver's closest config (still gated) so the arm STRETCHES TOWARD the
        # target instead of refusing to move. Default ON for this workspace --
        # operators expect the arm to keep tracking an out-of-reach / edited
        # target (bounded by the speed/accel limits). Set
        # ``allow_unreachable:=false`` (or untick it in the dashboard) to restore
        # the conservative reject-on-unreachable behaviour.
        self.declare_parameter("allow_unreachable", True)
        # FPC approach scaling in (0, 1]: command cur + reach_gain*(q_cmd-cur)
        # for gradual, smoother stretching. 1.0 = full step (today's behaviour).
        self.declare_parameter("reach_gain", 1.0)
        self.declare_parameter("joint_centering_weight", 1e-2)
        self.declare_parameter("damping", 1e-2)
        self.declare_parameter("tol_pos", 1e-3)
        self.declare_parameter("tol_ori", 3.5e-3)
        self.declare_parameter("max_iters", 200)
        # safety
        self.declare_parameter("max_joint_speed", 0.5)         # rad/s (JTC dur)
        # max_joint_accel caps how fast the FPC stream velocity ramps -> smooth
        # accel/decel (trapezoidal velocity) instead of instant start/stop. Used
        # only by the control-loop (control_rate_hz>0) FPC generator; JTC timing
        # is unaffected.
        self.declare_parameter("max_joint_accel", 3.0)         # rad/s^2 (FPC)
        self.declare_parameter("min_move_time", 0.5)           # s
        self.declare_parameter("max_step_rad", 0.8)            # jump reject
        self.declare_parameter("joint_states_stale_after", 0.5)  # s
        # control loop (Req 2): >0 Hz re-solves the latest target and streams a
        # velocity-limited, INTERPOLATED joint setpoint to the FPC on a fixed
        # timer -> smooth motion (no raw jump to the IK solution). Default
        # 200 Hz (FPC is the intended operating mode). 0 = pure event-driven.
        # Capped at _CONTROL_RATE_MAX_HZ.
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("status_rate_hz", 10.0)

        gp = self.get_parameter
        self._desc_topic = str(gp("robot_description_topic").value)
        self._js_topic = str(gp("joint_states_topic").value)
        self._target_topic = str(gp("target_pose_topic").value)
        self._base_frame = str(gp("base_frame").value or "")
        # Active config (may start empty -> unconfigured). Filled by _apply_config.
        self._frame = str(gp("controlled_frame").value or "")
        self._joints = [str(j) for j in (gp("joints").value or []) if str(j)]
        # Joints frozen out of the IK (held at current value). See the param doc.
        self._fixed_joints = [str(j) for j in (gp("fixed_joints").value or [])
                              if str(j)]
        # The full kinematic group BEFORE fixing (so toggling fixed_joints is
        # reversible without losing the original joint set). self._joints is
        # always self._group_joints minus self._fixed_joints.
        self._group_joints: List[str] = []
        self._jtc = str(gp("jtc_controller").value or "")
        self._fpc = str(gp("fpc_controller").value or "")
        # Full ordered joint list each controller drives (>= self._joints). Read
        # from the controller_manager at configure time so FPC/JTC commands are
        # always full-width: a sub-group like link_4 (4 joints) otherwise sends a
        # short array/goal the 6-joint controller drops/rejects -> no motion. The
        # joints outside the controlled group are held at their current position.
        self._jtc_joints: List[str] = []
        self._fpc_joints: List[str] = []
        self._mode = str(gp("command_mode").value).strip().lower()
        self._do_switch = bool(gp("switch_controllers").value)
        self._cm = str(gp("controller_manager").value or "/controller_manager")
        self._stiffness = [float(v) for v in gp("default_stiffness").value]
        self._allow_unreachable = bool(gp("allow_unreachable").value)
        self._reach_gain = min(1.0, max(1e-3, float(gp("reach_gain").value)))
        self._centering = float(gp("joint_centering_weight").value)
        self._damping = float(gp("damping").value)
        self._tol_pos = float(gp("tol_pos").value)
        self._tol_ori = float(gp("tol_ori").value)
        self._max_iters = int(gp("max_iters").value)
        self._max_speed = max(1e-3, float(gp("max_joint_speed").value))
        self._max_accel = max(1e-3, float(gp("max_joint_accel").value))
        self._min_time = max(0.0, float(gp("min_move_time").value))
        self._max_step = float(gp("max_step_rad").value)
        self._js_stale = float(gp("joint_states_stale_after").value)
        self._control_rate = max(0.0, min(_CONTROL_RATE_MAX_HZ,
                                          float(gp("control_rate_hz").value)))
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
        # Serialises Pinocchio model.data access (solve + FK) across the
        # event-driven target callback, the status timer, and the control-loop
        # timer (Phase 3) -- they share one RobotModel.data and must not race.
        self._fk_lock = threading.Lock()
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
        # On ~/enable we capture the measured joints as the return-to-start pose.
        self._start_q: Optional[Dict[str, float]] = None
        self._last_fpc_cmd: Optional[List[float]] = None
        self._last_best_effort = False
        self._last_jtc_cmd: Optional[List[float]] = None
        # Frozen hold angles for fixed_joints: the CONSTANT value each held
        # joint is commanded at (captured when it is fixed, refreshed on
        # ~/enable). Commanding the live measured value instead would let
        # encoder noise leak through the synchronized FPC generator and make a
        # "fixed" joint slowly creep -- and the arm never settle. Joint -> rad.
        self._fixed_hold: Dict[str, float] = {}
        # Velocity-limited FPC stream setpoint (full ctrl-joint width). Under a
        # control loop (control_rate_hz>0) each tick advances this toward the IK
        # goal by at most max_joint_speed/rate per joint -> a smooth interpolated
        # ramp instead of a raw jump to the solved config. None = re-seed from
        # the current measured joints on the next tick (reset on enable/disable).
        #
        # The stream is produced by a TIME-SYNCHRONIZED generator: all joints
        # share one scalar trapezoidal speed and the SAME progress fraction, so
        # they move along a straight joint-space line and arrive together (the
        # end-effector travels directly toward the target instead of curving /
        # shaking). See trajectory.SyncedJointTrajectory.
        self._traj: Optional[SyncedJointTrajectory] = None
        # control loop (Phase 3): latest resolved target + its timer
        self._last_target = None              # (xyz, quat) in the solver frame
        # Goal cache (control-loop / FPC): the solved+gated controlled-joint goal
        # is reused until the target pose moves beyond a small threshold, so the
        # redundant 7-DOF IK is NOT re-solved every tick. Re-solving each tick let
        # the solution drift in the null space, which made the arm shake around
        # the target. Reset on enable/disable.
        self._cached_goal: Optional[Dict[str, float]] = None
        self._cached_target_xyz: Optional[np.ndarray] = None
        self._cached_target_quat: Optional[np.ndarray] = None
        # Rejection cache: when a target is refused (unreachable), remember its
        # pose so an UNCHANGED repeated target
        # (e.g. a dashboard heartbeat re-sending the last gizmo pose, or the
        # control loop re-processing the same _last_target at the loop rate) is
        # not re-solved every tick. Without this the full IK runs at the control
        # rate on a hopeless target, burning CPU and starving the joint_states
        # callback (-> spurious "stale" holds). Invalidated when the target moves
        # or on enable/disable. Holding while rejected does not move the arm, so
        # the seed is unchanged and the verdict would be identical anyway.
        self._rejected_target_xyz: Optional[np.ndarray] = None
        self._rejected_target_quat: Optional[np.ndarray] = None
        self._rejected_msg: str = ""
        self._control_timer = None
        self._control_timer_hz = 0.0
        # a pending config request (from launch params, ~/configure, or a staged
        # ``ros2 param set``) to apply once the model is available
        self._cfg_dirty = False
        self._req_cfg: Optional[dict] = None
        if self._frame:
            self._req_cfg = {"controlled_frame": self._frame,
                             "joints": self._joints or None,
                             "fixed_joints": self._fixed_joints or None,
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
        # ...cached by controller name and REUSED across reconfigures. Destroying
        # an ActionClient that still has a status pending in the executor's ready
        # list crashes rclpy ("action client pointer is invalid"), so we never
        # tear these down mid-life — we only swap which cached one is active.
        self._fpc_pub_cache = {}
        self._jtc_client_cache = {}
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
        self.create_service(Trigger, "~/return_to_start",
                            self._srv_return_to_start, callback_group=cb)

        self.create_timer(1.0 / status_rate, self._publish_status,
                          callback_group=cb)
        # start the control loop if a non-zero rate was set at launch
        if self._control_rate > 0.0:
            self._reconcile_control_timer()

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
        ok, m = self._rebuild_model_from_urdf()
        if not ok:
            self.get_logger().error("failed to build model: %s" % m)
            return
        with self._lock:
            model = self._model
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

    def _rebuild_model_from_urdf(self):
        """(Re)build ``self._model`` from ``self._urdf``."""
        with self._lock:
            urdf = self._urdf
        if not urdf or RobotModel is None:
            return False, "no robot_description"
        try:
            model = RobotModel(urdf)
        except Exception as exc:  # noqa: BLE001
            return False, "model build failed: %r" % exc
        with self._lock:
            self._model = model
        return True, "ok"

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
        inside this callback). Structural changes are rejected while enabled to
        keep the parameter store consistent with the active config — EXCEPT a
        ``fixed_joints``-only change, which is applied live (it keeps the
        controllers), mirroring the ``~/configure`` topic.
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
            # ``fixed_joints`` alone is safe while enabled (applied live below);
            # any other structural key requires disabling first.
            non_live_structural = [k for k in structural if k != "fixed_joints"]
            if enabled and non_live_structural:
                return SetParametersResult(
                    successful=False,
                    reason="disable before changing structural config (%s)"
                    % ", ".join(sorted(non_live_structural)))
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
        rate_changed = False
        with self._lock:
            for key, attr, lo in (
                ("max_joint_speed", "_max_speed", 1e-3),
                ("max_joint_accel", "_max_accel", 1e-3),
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
            if req.get("allow_unreachable") is not None:
                self._allow_unreachable = self._as_bool(req["allow_unreachable"])
                changed.append("allow_unreachable=%s" % self._allow_unreachable)
            if req.get("reach_gain") is not None:
                try:
                    self._reach_gain = min(1.0, max(1e-3,
                                                    float(req["reach_gain"])))
                    changed.append("reach_gain=%g" % self._reach_gain)
                except (TypeError, ValueError):
                    pass
            if req.get("control_rate_hz") is not None:
                try:
                    self._control_rate = max(0.0, min(
                        _CONTROL_RATE_MAX_HZ, float(req["control_rate_hz"])))
                    changed.append("control_rate_hz=%g" % self._control_rate)
                    rate_changed = True
                except (TypeError, ValueError):
                    pass
            if "base_frame" in req and req["base_frame"] is not None:
                self._base_frame = str(req["base_frame"] or "")
                changed.append("base_frame=%s" % (self._base_frame or "(root)"))
            # Any live change must re-drive the robot. Invalidate the goal +
            # rejection caches so the background control loop (control_rate_hz>0)
            # RE-SOLVES the stored target on its next tick with the new settings,
            # instead of re-streaming the previously cached goal. Without this a
            # parameter edit (e.g. default_stiffness) only takes effect once the
            # TARGET POSE moves -- the loop keeps serving the stale cached goal.
            if changed:
                self._cached_goal = None
                self._cached_target_xyz = None
                self._cached_target_quat = None
                self._rejected_target_xyz = None
                self._rejected_target_quat = None
        if rate_changed:
            self._reconcile_control_timer()
        return ", ".join(changed)

    @staticmethod
    def _as_bool(value) -> bool:
        """Parse a bool from JSON bool/number or a string ('true'/'1'/'on')."""
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _reconcile_control_timer(self) -> None:
        """Create/destroy/retune the control-loop timer to match _control_rate.

        Called whenever ``control_rate_hz`` changes. The timer self-guards (does
        nothing unless enabled + configured + a target exists), so it is safe to
        run it whenever the rate is >0. Not called while holding ``_lock``.
        """
        with self._lock:
            hz = self._control_rate
            have = self._control_timer is not None
            have_hz = self._control_timer_hz
        want = hz > 0.0
        if want and (not have or abs(have_hz - hz) > 1e-6):
            if have and self._control_timer is not None:
                self.destroy_timer(self._control_timer)
            self._control_timer = self.create_timer(
                1.0 / hz, self._control_tick, callback_group=self._cbg)
            with self._lock:
                self._control_timer_hz = hz
        elif not want and have:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
            with self._lock:
                self._control_timer_hz = 0.0

    def _control_tick(self) -> None:
        """Control-loop tick (Phase 3): re-solve + command the latest target.

        Re-seeds from the CURRENT joints each tick so a single target keeps
        being tracked and a slow target stream is upsampled into smooth FPC
        streaming. The FPC/JTC deadbands stop redundant commands once converged.
        """
        with self._lock:
            rate = self._control_rate
            enabled = self._enabled
            configured = self._configured
            model = self._model
            tgt = self._last_target
        if rate <= 0.0 or not enabled or not configured or model is None:
            return
        if tgt is None or not self._js_fresh():
            return
        self._process_target(model, tgt[0], tgt[1])

    def _apply_structural(self, req: dict):
        """Apply a kinematic-group / controller reconfig.

        Selects the control link (``controlled_frame``), derives the joints, and
        picks the JTC/FPC controllers. ``joints`` and controller names are
        auto-derived when omitted. A general reconfig is **refused while enabled**
        (disable first), but a change to **only** ``fixed_joints`` is applied
        **live** (see below) since it keeps the controllers.
        """
        with self._lock:
            enabled = self._enabled
            urdf = self._urdf
            cur_frame, cur_joints = self._frame, list(self._joints)
            cur_group = list(self._group_joints)
            cur_fixed = list(self._fixed_joints)
            cur_jtc, cur_fpc, cur_mode = self._jtc, self._fpc, self._mode
            cur_configured = self._configured
        if not urdf or RobotModel is None:
            return False, "no model yet"

        # Fast path: a change to ONLY ``fixed_joints`` (no frame/group/controller/
        # mode change) keeps the controlled frame and the controllers identical —
        # it just freezes/releases joints within the SAME group. So it is applied
        # **live, even while enabled**: we re-derive the active set and drop the
        # motion caches so the next target re-solves around the newly fixed
        # joints, without dropping engagement. (Any other structural change still
        # requires disabling first.)
        fixed_only = ("fixed_joints" in req and not any(
            k in req for k in ("controlled_frame", "joints", "jtc_controller",
                               "fpc_controller", "command_mode")))
        if fixed_only and cur_configured and cur_frame:
            try:
                model = RobotModel(urdf)
            except Exception as exc:  # noqa: BLE001
                return False, "model build failed: %r" % exc
            fixed = [str(j) for j in (req.get("fixed_joints") or []) if str(j)]
            missing = [j for j in fixed if j not in model.joint_names]
            if missing:
                return False, f"fixed_joints not in URDF: {missing}"
            group = cur_group or model.supporting_joints(cur_frame)
            fixed_set = set(fixed)
            joints = [j for j in group if j not in fixed_set]
            if not joints:
                return False, ("all joints in the group are fixed "
                               "(group=%s, fixed=%s)" % (group, fixed))
            with self._lock:
                self._model = model
                self._joints = joints
                self._group_joints = group
                self._fixed_joints = fixed
                # Freeze each fixed joint at a CONSTANT angle: keep the existing
                # freeze point for joints already held, capture the current
                # measured angle for newly fixed ones, drop released joints.
                self._fixed_hold = {
                    j: (self._fixed_hold[j] if j in self._fixed_hold
                        else float(self._joint_pos[j]))
                    for j in fixed
                    if j in self._fixed_hold or j in self._joint_pos}
                # Re-solve the current target against the new active set: drop the
                # goal/rejection cache + FPC stream so the next tick re-seeds from
                # the current joints and solves with the joint(s) now frozen.
                self._cached_goal = None
                self._cached_target_xyz = None
                self._cached_target_quat = None
                self._rejected_target_xyz = None
                self._rejected_target_quat = None
                self._traj = None
                if not self._enabled:
                    # disabled: re-capture the start pose on the next ~/enable.
                    self._start_q = None
            fixed_on_path = [j for j in fixed if j in group]
            return True, (
                "fixed=%d joints=%d (live%s; controllers unchanged)"
                % (len(fixed_on_path), len(joints),
                   ", enabled" if enabled else ""))

        # General reconfig (frame / group / controllers / mode): disabled only.
        if enabled:
            return False, "refused: disable before reconfiguring"

        changing_group = ("controlled_frame" in req) or ("joints" in req)
        if changing_group:
            frame = str(req.get("controlled_frame") or cur_frame or "")
        else:
            if not cur_configured:
                return False, "not configured; set controlled_frame first"
            frame = cur_frame
        if not frame:
            return False, "controlled_frame required"

        try:
            model = RobotModel(urdf)
        except Exception as exc:  # noqa: BLE001
            return False, "model build failed: %r" % exc
        if not model.has_frame(frame):
            return False, f"unknown link/frame '{frame}'"

        # group_joints = the FULL kinematic group BEFORE fixing:
        #   explicit ``joints`` > derived from the control link > kept.
        group = req.get("joints")
        if group:
            group = [str(j) for j in group if str(j)]
            missing = [j for j in group if j not in model.joint_names]
            if missing:
                return False, f"joints not in URDF: {missing}"
        elif changing_group or not cur_group:
            group = model.supporting_joints(frame)
            if not group:
                return False, f"no movable joints support frame '{frame}'"
        else:
            group = cur_group

        # fixed_joints = joints held OUT of the IK: explicit in req > kept.
        fixing = "fixed_joints" in req
        if fixing:
            fixed = [str(j) for j in (req.get("fixed_joints") or []) if str(j)]
            missing = [j for j in fixed if j not in model.joint_names]
            if missing:
                return False, f"fixed_joints not in URDF: {missing}"
        else:
            fixed = cur_fixed
        fixed_set = set(fixed)

        # Active controlled joints = group minus the fixed ones. The IK solves
        # only these; the fixed joints stay at their current measured value and
        # the arm solves around them.
        joints = [j for j in group if j not in fixed_set]
        if not joints:
            return False, ("all joints in the group are fixed "
                           "(group=%s, fixed=%s)" % (group, fixed))

        mode = str(req.get("command_mode") or cur_mode).strip().lower()
        if mode not in ("jtc", "fpc"):
            return False, "command_mode must be 'jtc' or 'fpc'"

        # controllers: explicit > kept (only if the active set is unchanged) >
        # derived. Re-derive when the group OR the fixed set changed, since that
        # changes which joints the controller must drive.
        group_or_fix_changed = changing_group or fixing
        jtc = str(req.get("jtc_controller") or "")
        fpc = str(req.get("fpc_controller") or "")
        if not group_or_fix_changed:
            jtc = jtc or cur_jtc
            fpc = fpc or cur_fpc
        if not jtc or not fpc:
            disc = self._discover_controllers(joints)
            jtc = jtc or disc.get("jtc", "")
            fpc = fpc or disc.get("fpc", "")
        if mode == "jtc" and not jtc:
            return False, ("no JointTrajectoryController found driving %s; "
                           "set jtc_controller explicitly" % joints)
        if mode == "fpc" and not fpc:
            return False, ("no ForwardCommandController found driving %s; "
                           "set fpc_controller explicitly" % joints)

        # Full joint set each controller drives, so FPC/JTC commands can span it
        # (holding the joints outside the controlled sub-group at current pos).
        cj = self._controller_joint_map()
        fpc_joints = cj.get(fpc, []) if fpc else []
        jtc_joints = cj.get(jtc, []) if jtc else []

        with self._lock:
            self._model = model
            self._frame, self._joints = frame, joints
            self._group_joints, self._fixed_joints = group, fixed
            # Freeze each fixed joint at a CONSTANT angle (see _fixed_hold):
            # keep existing freeze points, capture the current measured angle
            # for newly fixed joints, drop released ones.
            self._fixed_hold = {
                j: (self._fixed_hold[j] if j in self._fixed_hold
                    else float(self._joint_pos[j]))
                for j in fixed
                if j in self._fixed_hold or j in self._joint_pos}
            self._jtc, self._fpc, self._mode = jtc, fpc, mode
            self._jtc_joints, self._fpc_joints = jtc_joints, fpc_joints
            self._configured = True
            # The captured start pose belonged to the previous group; invalidate
            # it so it is re-captured on the next ~/enable (and a return-to-start
            # before re-enabling is refused, not crashing).
            self._start_q = None
        self._rebuild_clients()
        fixed_on_path = [j for j in fixed if j in group]
        return True, (f"link={frame} joints={len(joints)} "
                      f"fixed={len(fixed_on_path)} "
                      f"mode={mode} jtc={jtc or '-'} fpc={fpc or '-'}")

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
        # Score each candidate by (joint coverage, is-active) so a tie on
        # coverage prefers the ACTIVE controller -- e.g. a UR ships BOTH an
        # inactive ``joint_trajectory_controller`` and the active
        # ``scaled_joint_trajectory_controller``; the active one is the one that
        # actually drives the arm.
        best = {"jtc": (-1, -1), "fpc": (-1, -1)}
        for ctl in getattr(resp, "controller", []):
            req_if = set(getattr(ctl, "required_command_interfaces", []) or [])
            cover = len(want & req_if)
            if cover == 0:
                continue
            score = (cover, 1 if getattr(ctl, "state", "") == "active" else 0)
            t = ctl.type
            if t.endswith("JointTrajectoryController"):
                if score > best["jtc"]:
                    best["jtc"] = score
                    out["jtc"] = ctl.name
            # A position forward-command controller streams the same
            # Float64MultiArray of joint positions on ``~/commands``. Match the
            # ros2_control ForwardCommandController AND position_controllers'
            # JointGroupPositionController (what UR ships as
            # ``forward_position_controller``) so FPC mode transfers unchanged.
            elif (t.endswith("ForwardCommandController")
                  or t.endswith("JointGroupPositionController")):
                if score > best["fpc"]:
                    best["fpc"] = score
                    out["fpc"] = ctl.name
        return out

    def _controller_joint_map(self) -> Dict[str, List[str]]:
        """{controller: [ordered joints it position-commands]} from the CM.

        Lets FPC/JTC commands span a controller's FULL joint set even when only
        a sub-group is controlled (the extra joints are held). The order matches
        the controller's command-interface order == its command-array order.
        """
        out: Dict[str, List[str]] = {}
        if not _HAS_CM or self._cli_list is None:
            return out
        if not self._cli_list.wait_for_service(timeout_sec=2.0):
            return out
        fut = self._cli_list.call_async(ListControllers.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=2.0):
            return out
        resp = fut.result()
        for ctl in getattr(resp, "controller", []):
            js = [itf[:-len("/position")]
                  for itf in (getattr(ctl, "required_command_interfaces", [])
                              or [])
                  if itf.endswith("/position")]
            if js:
                out[ctl.name] = js
        return out

    def _rebuild_clients(self) -> None:
        """Point the FPC publisher + JTC action client at the current controllers.

        Endpoints are created lazily ONCE per controller name and cached; we only
        swap which cached endpoint is "active". They are never destroyed while the
        node spins, because tearing down an ActionClient that still has a status
        pending in the executor's ready list crashes rclpy with "action client
        pointer is invalid" (hit when reconfiguring right after a JTC move).
        """
        with self._lock:
            jtc, fpc = self._jtc, self._fpc
        # FPC command publisher (cached by controller name, never destroyed here)
        if fpc:
            pub = self._fpc_pub_cache.get(fpc)
            if pub is None:
                pub = self.create_publisher(
                    Float64MultiArray, f"/{fpc}/commands", 10)
                self._fpc_pub_cache[fpc] = pub
            self._fpc_pub = pub
        else:
            self._fpc_pub = None
        # JTC action client (cached by controller name, never destroyed here)
        if jtc and _HAS_FJT:
            client = self._jtc_client_cache.get(jtc)
            if client is None:
                client = ActionClient(
                    self, FollowJointTrajectory,
                    f"/{jtc}/follow_joint_trajectory", callback_group=self._cbg)
                self._jtc_client_cache[jtc] = client
            self._jtc_client = client
        else:
            self._jtc_client = None

    # ------------------------------------------------------------------ #
    # Main path: target -> solve -> gate -> command
    # ------------------------------------------------------------------ #
    def _on_target(self, msg: PoseStamped) -> None:
        with self._lock:
            self._last_target_stamp = time.monotonic()
            enabled = self._enabled
            model = self._model
            configured = self._configured
            rate = self._control_rate
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

        # Store the latest target. With a control loop (control_rate_hz>0) the
        # timer re-solves/commands it; otherwise process it now (event-driven).
        with self._lock:
            self._last_target = (xyz, quat)
        if rate <= 0.0:
            self._process_target(model, xyz, quat)

    def _process_target(self, model, xyz, quat) -> None:
        """Solve IK for one target and command it through the safety gates.

        Shared by the event-driven path (``_on_target``) and the control-loop
        timer (``_control_tick``). Re-seeds from the current joints each call.

        Goal caching (control loop, FPC AND JTC): the solved + gated
        controlled-joint goal is cached and REUSED while the target pose is
        essentially unchanged, so the redundant 7-DOF IK is solved once per
        target rather than every tick. Re-solving each tick let the solution
        drift in the null space; in FPC that made the end-effector shake around
        the target, and in JTC it made each tick re-send a slightly different
        ``FollowJointTrajectory`` goal that preempted the in-flight trajectory
        (the arm kept re-aiming mid-move -> shaking). Caching gives one stable
        goal: FPC streams to it in a straight line; JTC sends it once and the
        trajectory controller executes it without interruption.
        """
        with self._lock:
            rate = self._control_rate
            mode = self._mode
            cached_goal = self._cached_goal
            c_xyz = self._cached_target_xyz
            c_quat = self._cached_target_quat
            best_effort_cached = self._last_best_effort
            rej_xyz = self._rejected_target_xyz
            rej_quat = self._rejected_target_quat
        # Cache hit: same target, already solved + gated under the control loop.
        # FPC keeps streaming toward the cached goal (no re-solve, no null-space
        # drift). JTC does NOTHING: the trajectory controller is already
        # executing/holding the cached goal, and re-sending would restart the
        # trajectory and cause shaking.
        if (rate > 0.0 and cached_goal is not None
                and not self._target_moved(xyz, quat, c_xyz, c_quat)):
            if mode == "fpc":
                self._command_fpc(cached_goal, best_effort_cached)
            return

        # Rejection cache hit: the same target was just refused. Re-processing it
        # every control tick would re-run the full (failed) IK solve, burn CPU
        # and starve the joint_states callback. Skip the solve while it is
        # unchanged; the arm is held by the controller meanwhile.
        if (rate > 0.0 and rej_xyz is not None
                and not self._target_moved(xyz, quat, rej_xyz, rej_quat)):
            return

        seed = self._build_seed(model)
        sol = self._solve(model, seed, xyz, quat)
        with self._lock:
            self._last_solution = sol
            self._last_reason = sol.reason.value
            allow = self._allow_unreachable

        best_effort = False
        if not sol.reachable:
            if not allow:
                with self._lock:
                    self._last_best_effort = False
                    self._cached_goal = None
                self._remember_rejection(xyz, quat)
                self._set_msg(
                    "target REJECTED: unreachable (%s)" % sol.reason.value)
                return
            # best-effort (Req 5): command the solver's closest config (arm
            # stretches toward the target). Still bounded by the max_step gate
            # and the Phase-0 Cartesian gate below. Status keeps reachable=false.
            best_effort = True
        with self._lock:
            self._last_best_effort = best_effort
            # A solvable target: clear any stale rejection cache.
            self._rejected_target_xyz = None
            self._rejected_target_quat = None

        # Commanded full configuration (active joints solved; the rest frozen at
        # the seed). Jump protection acts on this BEFORE anything is sent to a
        # controller.
        q_full = np.asarray(sol.q, dtype=float)

        # jump protection: max joint change over the CONTROLLED joints vs current
        q_cmd = {j: float(q_full[model.q_index(j)]) for j in self._joints}
        with self._lock:
            cur = {j: self._joint_pos.get(j) for j in self._joints}
        if any(cur[j] is None for j in self._joints):
            self._set_msg("target ignored: current joint pos unknown")
            return
        max_delta = max(abs(q_cmd[j] - cur[j]) for j in self._joints)
        with self._lock:
            self._last_delta = max_delta
            mode = self._mode
            rate = self._control_rate
        # Jump protection guards the EVENT-DRIVEN path (control_rate_hz=0), where
        # a target is commanded in one shot with no velocity limiting. Under the
        # control loop (rate>0) BOTH modes are inherently speed-limited, so a
        # large step (e.g. a best-effort stretch toward an unreachable target,
        # which near a singularity needs a big joint move for a small Cartesian
        # one) executes as a slow, smooth, bounded trajectory rather than a
        # dangerous instant jump:
        #   * FPC ramps to the goal via the synchronized accel-limited generator;
        #   * JTC sends ONE FollowJointTrajectory whose duration scales with the
        #     joint delta (max(min_move_time, max_delta/max_joint_speed)).
        # So the hard reject applies only when the control loop is off.
        speed_limited = rate > 0.0
        if max_delta > self._max_step and not speed_limited:
            self._set_msg(
                "target REJECTED: step %.3f rad > max_step_rad %.3f "
                "(jump protection)" % (max_delta, self._max_step))
            return

        # Cache the solved + gated goal so subsequent control-loop ticks for this
        # same target reuse it (no re-solve, no null-space wander). Applies to
        # both modes: FPC streams toward it; JTC sends it once and then the cache
        # hit above suppresses re-sends until the target actually moves.
        with self._lock:
            self._cached_goal = dict(q_cmd)
            self._cached_target_xyz = np.asarray(xyz, dtype=float)
            self._cached_target_quat = (None if quat is None
                                        else np.asarray(quat, dtype=float))
        if mode == "jtc":
            self._command_jtc(q_cmd, max_delta, best_effort)
        else:
            self._command_fpc(q_cmd, best_effort)

    def _target_moved(self, xyz, quat, c_xyz, c_quat,
                      pos_tol: float = 1e-3, ang_tol: float = 2e-3) -> bool:
        """True if the target pose moved beyond pos_tol (m) / ang_tol (rad) from
        the cached one (or there is no cache) -> the IK goal must be re-solved."""
        if c_xyz is None:
            return True
        if float(np.linalg.norm(np.asarray(xyz) - c_xyz)) > pos_tol:
            return True
        if (quat is None) != (c_quat is None):
            return True
        if quat is not None:
            dot = abs(float(np.dot(np.asarray(quat), c_quat)))
            if 2.0 * math.acos(min(1.0, dot)) > ang_tol:
                return True
        return False

    def _remember_rejection(self, xyz, quat) -> None:
        """Record a refused target so an unchanged repeat is not re-solved every
        control tick (CPU / joint_states-starvation guard)."""
        with self._lock:
            self._rejected_target_xyz = np.asarray(xyz, dtype=float)
            self._rejected_target_quat = (None if quat is None
                                          else np.asarray(quat, dtype=float))

    def _build_seed(self, model: "RobotModel") -> np.ndarray:
        q = model.neutral()
        with self._lock:
            jp = dict(self._joint_pos)
            # In the FPC control loop, seed the controlled joints from the smooth
            # commanded stream so the IK goal is deterministic (it does not track
            # measured-joint noise) -> steady hold + a consistent solve. Falls
            # back to measured joints (event-driven, JTC, or first tick).
            stream = (dict(self._traj.stream)
                      if (self._mode == "fpc" and self._control_rate > 0.0
                          and self._traj is not None) else None)
        for jn in model.joint_names:
            if jn in jp:
                q[model.q_index(jn)] = jp[jn]
        if stream is not None:
            for jn, v in stream.items():
                if jn in model.joint_names:
                    q[model.q_index(jn)] = float(v)
        return q

    def _solve(self, model, seed, xyz, quat):
        params = ik_core.SolveParams(
            max_iters=self._max_iters, tol_pos=self._tol_pos,
            tol_ori=self._tol_ori, damping=self._damping,
            joint_centering_weight=self._centering)
        task = self._build_task(xyz, quat)
        with self._fk_lock:
            return ik_core.solve(model, seed, [task], params=params,
                                 active_joints=self._joints)

    def _build_task(self, xyz, quat):
        """Build the IK Task from the per-DOF ``default_stiffness`` 6-vector.

        ``default_stiffness`` = [x y z rx ry rz] sets each Cartesian DOF's
        stiffness directly: ``0`` lets that DOF float free, a positive value
        constrains it (``1`` = fully rigid). e.g. ``[1 1 1 0 0 0]`` reaches the
        target position with orientation free; ``[1 1 1 0 0 1]`` adds yaw only.
        """
        with self._lock:
            frame = self._frame
            stiff = tuple(self._stiffness)
        xyz_t = tuple(float(v) for v in xyz)
        quat_t = tuple(float(v) for v in quat)
        return Task(frame, xyz_t, quat_t, stiff)

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
    def _command_jtc(self, q_cmd: Dict[str, float], max_delta: float,
                     best_effort: bool = False) -> None:
        if self._jtc_client is None:
            self._set_msg("cannot command: FollowJointTrajectory unavailable")
            return
        # Command the controller's FULL joint set: solved values for the
        # controlled sub-group, current position (hold) for the rest. A 6-joint
        # JTC rejects a partial-joint goal, so a sub-group (e.g. link_4) would
        # otherwise never move.
        with self._lock:
            ctrl_joints = list(self._jtc_joints) or list(self._joints)
            controlled = set(self._joints)
            cur = {j: self._joint_pos.get(j) for j in ctrl_joints}
            fixed_hold = dict(self._fixed_hold)
            last = self._last_jtc_cmd
            rate = self._control_rate
        data = []
        for j in ctrl_joints:
            if j in controlled and j in q_cmd:
                data.append(float(q_cmd[j]))
            elif j in fixed_hold:
                # Frozen joint: hold at the constant captured angle (not the
                # live measured value, which would creep tick to tick).
                data.append(float(fixed_hold[j]))
            else:
                data.append(
                    float(cur[j] if cur[j] is not None else q_cmd.get(j, 0.0)))
        # Deadband (control-loop only): don't re-send a goal for an unchanged
        # solved config (would restart the same trajectory every tick). In
        # event-driven mode every target is intentional, so always send.
        with self._lock:
            last = self._last_jtc_cmd
            rate = self._control_rate
        if rate > 0.0 and last is not None and len(last) == len(data) and \
                max(abs(a - b) for a, b in zip(data, last)) < _JTC_DEADBAND_RAD:
            return
        if not self._jtc_client.server_is_ready():
            self._jtc_client.wait_for_server(timeout_sec=1.0)
        if not self._jtc_client.server_is_ready():
            self._set_msg("cannot command: %s action server not ready"
                          % self._jtc)
            return
        with self._lock:
            self._last_jtc_cmd = data
        duration = max(self._min_time, max_delta / self._max_speed)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ctrl_joints)
        pt = JointTrajectoryPoint()
        pt.positions = data
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
        self._set_msg("JTC move sent (%.2fs, step %.3f rad)%s"
                      % (duration, max_delta,
                         " [best-effort]" if best_effort else ""))

    def _command_fpc(self, q_cmd: Dict[str, float],
                     best_effort: bool = False) -> None:
        with self._lock:
            ctrl_joints = list(self._fpc_joints) or list(self._joints)
            controlled = set(self._joints)
            cur = {j: self._joint_pos.get(j) for j in ctrl_joints}
            fixed_hold = dict(self._fixed_hold)
            gain = self._reach_gain
            last = self._last_fpc_cmd
            rate = self._control_rate
            max_speed = self._max_speed
            max_accel = self._max_accel
            traj = self._traj
        # Goal = the controller's FULL joint set: solved values for the
        # controlled sub-group, current position (hold) for the rest. The
        # ForwardCommandController drops a command whose width != its joint
        # count, so a sub-group (e.g. link_4) would otherwise never move.
        goal = {}
        for j in ctrl_joints:
            if j in controlled and j in q_cmd:
                goal[j] = float(q_cmd[j])
            elif j in fixed_hold:
                # Frozen joint: command the CONSTANT captured angle. Using the
                # live measured value would feed encoder noise into the
                # synchronized generator and make the joint slowly creep.
                goal[j] = float(fixed_hold[j])
            else:
                goal[j] = float(cur[j] if cur[j] is not None
                                else q_cmd.get(j, 0.0))

        if rate > 0.0:
            # TIME-SYNCHRONIZED trajectory generation: all joints advance along
            # the SAME joint-space direction governed by ONE scalar trapezoidal
            # speed sized for the lead (largest-travel) joint. They therefore
            # stay phase-locked and reach the goal together -> the end-effector
            # moves directly toward the target (straight joint-space segment)
            # instead of curving while early-finishing joints wait, and it parks
            # without the residual shaking the old per-joint profiles produced.
            # The lead speed is acceleration-limited and braked by sqrt(2*a*d) so
            # the arm settles on the goal without overshoot. The move naturally
            # slows when the IK goal demands large joint travel (e.g. nearing a
            # singularity). The FPC has no interpolation of its own, so we stream
            # this smoothed setpoint. Seed from the current joints on the first
            # tick after enable.
            if traj is None or list(traj.joints) != list(ctrl_joints):
                seed_q = {j: (fixed_hold[j] if j in fixed_hold
                              else (cur[j] if cur[j] is not None else goal[j]))
                          for j in ctrl_joints}
                traj = SyncedJointTrajectory(ctrl_joints, seed_q,
                                             settle_rad=_FPC_SETTLE_RAD)
            dt = 1.0 / rate
            data = traj.step(goal, dt, max_speed, max_accel)
            with self._lock:
                self._traj = traj
                self._last_fpc_cmd = data
        else:
            # Event-driven (rate=0): publish the solved setpoint directly, with
            # optional reach_gain approach scaling (gradual stretch per cycle).
            use_gain = 0.0 < gain < 1.0 and all(
                cur[j] is not None for j in ctrl_joints if j in controlled)
            data = []
            for j in ctrl_joints:
                v = goal[j]
                if j in controlled and j in q_cmd and use_gain \
                        and cur[j] is not None:
                    v = float(cur[j] + gain * (goal[j] - cur[j]))
                data.append(v)
            # Deadband: skip republishing an essentially unchanged setpoint
            # (anti-chatter; the controller latches it).
            if last is not None and len(last) == len(data) and \
                    max(abs(a - b) for a, b in zip(data, last)) \
                    < _FPC_DEADBAND_RAD:
                return
            with self._lock:
                self._last_fpc_cmd = data
        m = Float64MultiArray()
        m.data = data
        self._fpc_pub.publish(m)
        self._set_msg("FPC command streamed%s"
                      % (" [best-effort]" if best_effort else ""))

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
            joints = list(self._joints)
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
        # Capture the start pose (measured joints) for return-to-start.
        seed = self._build_seed(model)
        with self._lock:
            self._enabled = True
            self._start_q = {j: float(seed[model.q_index(j)]) for j in joints}
            # Refresh each fixed joint's freeze point to where it physically is
            # right now, so motion begins holding the current pose (no snap if a
            # held joint was moved while disabled).
            self._fixed_hold = {
                j: float(self._joint_pos[j])
                for j in self._fixed_joints if j in self._joint_pos}
            self._last_fpc_cmd = None
            self._last_jtc_cmd = None
            self._traj = None
            self._cached_goal = None
            self._cached_target_xyz = None
            self._cached_target_quat = None
            self._rejected_target_xyz = None
            self._rejected_target_quat = None
            self._last_target = None
            self._last_best_effort = False
        self._set_msg(
            "ENABLED (mode=%s, controller=%s)" % (mode, want))
        return True, "enabled"

    def _srv_disable(self, request, response):
        with self._lock:
            self._enabled = False
            self._last_target = None
            self._traj = None
            self._cached_goal = None
            self._cached_target_xyz = None
            self._cached_target_quat = None
            self._rejected_target_xyz = None
            self._rejected_target_quat = None
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
            ctrl_joints = list(self._fpc_joints) or list(self._joints)
            cur = {j: self._joint_pos.get(j) for j in ctrl_joints}
            pub = self._fpc_pub
        if pub is None or any(cur[j] is None for j in ctrl_joints):
            return
        m = Float64MultiArray()
        m.data = [float(cur[j]) for j in ctrl_joints]
        pub.publish(m)

    def _srv_return_to_start(self, request, response):
        ok, msg = self._return_to_start()
        response.success = ok
        response.message = msg
        return response

    def _return_to_start(self, timeout: float = 20.0):
        """Command a JTC move back to the pose captured at ~/enable and wait.

        SAFETY (Phase 0): used at the end of every test to bring the arm home.
        Always moves via the trajectory controller (activating it first if the
        active mode is FPC), so it works regardless of the command mode. Blocks
        on the goal result up to ``timeout`` and reports completion.
        """
        with self._lock:
            model = self._model
            configured = self._configured
            start_q = dict(self._start_q) if self._start_q else None
            joints = list(self._joints)
            jtc, fpc, mode = self._jtc, self._fpc, self._mode
            do_switch = self._do_switch
        if start_q is None:
            return False, "no start pose captured; enable first"
        # The start pose is captured per-joint at ~/enable. If the controlled
        # group changed since then (different joints), the captured pose no
        # longer covers the current joints -> refuse cleanly instead of raising
        # a KeyError (which previously crashed the node).
        if any(j not in start_q for j in joints):
            return False, ("start pose was captured for a different controlled "
                           "group; re-enable before return-to-start")
        if model is None or not configured:
            return False, "not configured"
        if not self._js_fresh():
            return False, "/joint_states stale; cannot return to start"
        if not _HAS_FJT or self._jtc_client is None:
            return False, "JTC action client unavailable"
        # Ensure the trajectory controller is active (return-to-start is a JTC
        # move even for FPC tests).
        if do_switch:
            deact = [fpc] if (mode == "fpc" and fpc) else []
            if not self._switch(activate=[jtc], deactivate=deact):
                return False, "failed to activate JTC for return-to-start"
        if not self._jtc_client.server_is_ready():
            self._jtc_client.wait_for_server(timeout_sec=2.0)
        if not self._jtc_client.server_is_ready():
            return False, "JTC action server not ready"
        with self._lock:
            cur = {j: self._joint_pos.get(j) for j in joints}
        if any(cur[j] is None for j in joints):
            return False, "current joint pos unknown"
        max_delta = max(abs(start_q[j] - cur[j]) for j in joints)
        duration = max(self._min_time, max_delta / self._max_speed)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [start_q[j] for j in joints]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        goal.trajectory.points = [pt]

        done = threading.Event()
        state: Dict[str, object] = {}

        def _on_result(_f):
            state["done_result"] = True
            done.set()

        def _on_goal(f):
            try:
                handle = f.result()
            except Exception as exc:  # noqa: BLE001
                state["err"] = "goal error: %r" % exc
                done.set()
                return
            if handle is None or not handle.accepted:
                state["err"] = "JTC goal rejected"
                done.set()
                return
            with self._lock:
                self._goal_handle = handle
            handle.get_result_async().add_done_callback(_on_result)

        self._jtc_client.send_goal_async(goal).add_done_callback(_on_goal)
        if not done.wait(timeout=timeout):
            return False, "return-to-start timed out after %.1fs" % timeout
        if "err" in state:
            return False, str(state["err"])
        self._set_msg("returned to start pose (%.2fs move)" % duration)
        return True, "returned to start"

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
            fixed_joints = list(self._fixed_joints)
            group_joints = list(self._group_joints)
            jtc, fpc, mode = self._jtc, self._fpc, self._mode
            cmd_joints = list(self._fpc_joints if mode == "fpc"
                              else self._jtc_joints)
            control_rate = self._control_rate
            allow_unreachable = self._allow_unreachable
            reach_gain = self._reach_gain
            best_effort = self._last_best_effort
        status = {
            "enabled": enabled,
            "configured": configured,
            "mode": mode,
            "controlled_frame": frame,
            "base_frame": self._base_frame or "(model root)",
            "joints": joints,
            "fixed_joints": fixed_joints,
            "group_joints": group_joints,
            "command_joints": cmd_joints,
            "jtc_controller": jtc,
            "fpc_controller": fpc,
            "have_model": model is not None,
            "joint_states_fresh": self._js_fresh(),
            "last_message": msg,
            "last_reason": reason,
            "last_step_rad": delta,
            "max_step_rad": self._max_step,
            "max_joint_speed": self._max_speed,
            "max_joint_accel": self._max_accel,
            "min_move_time": self._min_time,
            "joint_states_stale_after": self._js_stale,
            "default_stiffness": list(self._stiffness),
            "joint_centering_weight": self._centering,
            "damping": self._damping,
            "tol_pos": self._tol_pos,
            "tol_ori": self._tol_ori,
            "max_iters": self._max_iters,
            "control_rate_hz": control_rate,
            "allow_unreachable": allow_unreachable,
            "reach_gain": reach_gain,
            "best_effort": best_effort,
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
