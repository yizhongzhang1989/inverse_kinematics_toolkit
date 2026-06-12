"""Headless IK solver ROS node (advisory only).

Wraps ``ik_core`` + ``robot_model`` behind a ROS 2 API. It reads the URDF from
``/robot_description`` (latched) and the seed configuration from
``/joint_states``, accepts solve requests as JSON over a topic pair, and
publishes the solution as a ``JointState`` plus a JSON status string.

IT NEVER COMMANDS THE ROBOT. ``~/solution`` is an IK *result*, explicitly not a
controller command; a separate consumer decides whether/how to actuate.

JSON solve request (std_msgs/String on ``~/solve_request``)::

    {
      "id": "optional-correlation-id",
      "tasks": [
        {"frame": "right_arm_Link7",
         "xyz": [x, y, z],
         "quat": [w, x, y, z],            # optional; omit for point-only
         "stiffness": [sx,sy,sz,srx,sry,srz],   # optional; default pose
         "frame_id": "base_link"},        # optional; TF source frame (R8)
        ...
      ],
      "virtual_frames": [                  # optional tool frames (R2)
        {"name": "tool", "parent": "right_arm_Link7",
         "xyz": [0,0,0.1], "rpy": [0,0,0]}],
      "arm_angles": [{"chain": "right_arm", "psi": 0.3, "stiffness": 0.5}],
      "relative": {"frame_a": "...", "frame_b": "...",
                   "xyz": [...], "quat": [...], "stiffness": [...]},  # R9
      "active_joints": ["right_arm_joint1", ...],   # optional; default all
      "seed": [q0, q1, ...]                # optional; default current joint state
    }

The JSON response (``~/solve_response``) carries the full Solution + diagnostics.
"""

from __future__ import annotations

import json
import threading
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from . import ik_core
from .robot_model import RobotModel
from .tasks import Task, Reason
from .arm_angle import SRSChain, make_arm_angle_extra_task, compute_all_psi
from .relative import make_relative_extra_task

try:
    import tf2_ros
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

try:
    # Optional typed service (Phase 6). The node works without it (JSON API);
    # if the interfaces package is built, a typed SolveIK service is also offered.
    from ikt_interfaces.srv import SolveIK
    from ikt_interfaces.msg import IKResult
    _HAVE_TYPED = True
except Exception:  # pragma: no cover
    _HAVE_TYPED = False


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class IKNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("base_frame", "")
        self.declare_parameter("max_iters", 200)
        self.declare_parameter("tol_pos", 1e-3)
        self.declare_parameter("tol_ori", 3.5e-3)
        self.declare_parameter("damping", 1e-2)
        self.declare_parameter("joint_centering_weight", 1e-2)
        self.declare_parameter("default_task_stiffness",
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("arm_angle_stiffness", 0.5)
        self.declare_parameter("status_rate_hz", 5.0)

        self._desc_topic = self.get_parameter("robot_description_topic").value
        self._js_topic = self.get_parameter("joint_states_topic").value
        self._base_frame = self.get_parameter("base_frame").value or ""

        self._lock = threading.Lock()
        self._model: Optional[RobotModel] = None
        self._urdf: str = ""
        self._vframe_key = ()              # cache key for the augmented model
        self._joint_pos: Dict[str, float] = {}
        self._srs_chains: Dict[str, SRSChain] = self._load_srs_chains()
        self._last_solution = None

        # ---- pubs / subs ------------------------------------------------
        self.create_subscription(String, self._desc_topic,
                                 self._on_urdf, _latched_qos())
        self.create_subscription(JointState, self._js_topic,
                                 self._on_js, 50)
        self.create_subscription(String, "~/solve_request",
                                 self._on_solve_request, 10)
        self._sol_pub = self.create_publisher(JointState, "~/solution", 10)
        self._resp_pub = self.create_publisher(String, "~/solve_response", 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)

        if _HAVE_TF:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None

        # Optional typed service (Phase 6); reuses the same _solve() core.
        if _HAVE_TYPED:
            self._solve_srv = self.create_service(
                SolveIK, "~/solve", self._on_solve_service)
            self.get_logger().info("Typed SolveIK service available at ~/solve.")

        rate = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / max(0.5, rate), self._publish_status)

        self.get_logger().info(
            "ikt_inverse_kinematics ik_node started (advisory only — NEVER "
            "commands the robot). Waiting for %s and %s."
            % (self._desc_topic, self._js_topic))

    # ------------------------------------------------------------------ #
    # Param helpers
    # ------------------------------------------------------------------ #
    def _load_srs_chains(self) -> Dict[str, SRSChain]:
        """Read srs_chains.<name>.{shoulder,elbow,wrist,base,tip} params.

        Arm names are taken from the ``srs_chain_names`` string-array param
        (EMPTY by default — the node is robot-independent and offers arm-angle
        only for chains the user explicitly declares). For each, the per-field
        dotted params are declared-then-read so this works regardless of
        param-load ordering.
        """
        chains: Dict[str, SRSChain] = {}
        # Declare with an explicit STRING_ARRAY descriptor: an empty-list default
        # has no inferrable type in rclpy (raises on get), so we must type it.
        try:
            from rcl_interfaces.msg import ParameterDescriptor, ParameterType
            self.declare_parameter(
                "srs_chain_names", [],
                ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        except Exception:
            pass
        try:
            names = list(self.get_parameter("srs_chain_names").value or [])
        except Exception:
            names = []
        for arm in names:
            def g(field, default=""):
                p = f"srs_chains.{arm}.{field}"
                try:
                    self.declare_parameter(p, default)
                except Exception:
                    pass
                try:
                    return self.get_parameter(p).value or default
                except Exception:
                    return default
            sh, el, wr = g("shoulder"), g("elbow"), g("wrist")
            if sh and el and wr:
                chains[arm] = SRSChain(arm, sh, el, wr, g("base"), g("tip"))
        return chains

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_urdf(self, msg: String) -> None:
        with self._lock:
            if msg.data and msg.data != self._urdf:
                self._urdf = msg.data
                self._vframe_key = ()
                try:
                    self._model = RobotModel(msg.data)
                    self.get_logger().info(
                        "Built kinematic model: %d DOF, %d frames."
                        % (self._model.nq, len(self._model.frame_names())))
                except Exception as exc:  # noqa: BLE001
                    self._model = None
                    self.get_logger().error("Failed to build model: %r" % exc)

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_pos[n] = float(p)

    def _current_seed(self, model: RobotModel) -> np.ndarray:
        q = model.neutral()
        for jn in model.joint_names:
            if jn in self._joint_pos:
                q[model.q_index(jn)] = self._joint_pos[jn]
        return q

    # ------------------------------------------------------------------ #
    # Solve request handling
    # ------------------------------------------------------------------ #
    def _on_solve_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self._respond({"ok": False, "error": f"bad JSON: {exc}"})
            return
        try:
            resp = self._solve(req)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("solve failed: %r" % exc)
            resp = {"ok": False, "error": repr(exc), "id": req.get("id")}
        self._respond(resp)

    def _respond(self, payload: dict) -> None:
        m = String()
        m.data = json.dumps(payload)
        self._resp_pub.publish(m)

    def _on_solve_service(self, request, response):
        """Typed SolveIK service: convert to the dict core and back (Phase 6)."""
        req = {"tasks": [], "id": "typed"}
        for t in request.tasks:
            p = t.target.position
            o = t.target.orientation
            entry = {"frame": t.frame,
                     "xyz": [p.x, p.y, p.z],
                     "quat": [o.w, o.x, o.y, o.z],
                     "stiffness": list(t.stiffness)}
            if t.frame_id:
                entry["frame_id"] = t.frame_id
            req["tasks"].append(entry)
        if request.active_joints:
            req["active_joints"] = list(request.active_joints)
        if len(request.seed) > 0:
            req["seed"] = list(request.seed)
        try:
            out = self._solve(req)
        except Exception as exc:  # noqa: BLE001
            response.ok = False
            response.message = repr(exc)
            return response
        response.ok = bool(out.get("ok", False))
        response.message = out.get("error", "") or ""
        res = IKResult()
        res.reachable = bool(out.get("reachable", False))
        res.reason = str(out.get("reason", ""))
        res.iters = int(out.get("iters", 0))
        res.joint_names = list(out.get("joint_names", []))
        res.q = [float(v) for v in out.get("q", [])]
        res.max_pos_err = float(out.get("max_pos_err", 0.0) or 0.0)
        res.max_ori_err = float(out.get("max_ori_err", 0.0) or 0.0)
        res.blocking_joints = list(out.get("blocking_joints", []))
        res.manipulability = float(out.get("manipulability", 0.0) or 0.0)
        res.sigma_min = float(out.get("sigma_min", 0.0) or 0.0)
        res.delta_norm = float(out.get("delta_norm", 0.0) or 0.0)
        response.result = res
        return response

    def _solve(self, req: dict) -> dict:
        with self._lock:
            base_model = self._model
            urdf = self._urdf
        if base_model is None:
            return {"ok": False, "error": "no robot_description yet",
                    "id": req.get("id")}

        # virtual tool frames -> (cached) augmented model
        vframes = req.get("virtual_frames") or []
        if vframes:
            key = tuple(sorted(json.dumps(v, sort_keys=True) for v in vframes))
            with self._lock:
                if key == self._vframe_key and self._model is not None \
                        and self._model.virtual_frame_names:
                    model = self._model
                else:
                    model = RobotModel(urdf, virtual_frames=vframes)
                    self._model = model
                    self._vframe_key = key
        else:
            model = base_model

        # seed
        if req.get("seed") is not None:
            seed = np.asarray(req["seed"], dtype=float).reshape(model.nq)
        else:
            with self._lock:
                seed = self._current_seed(model)

        # tasks (with optional TF resolution of the target frame, R8)
        default_stiff = list(self.get_parameter("default_task_stiffness").value)
        tasks: List[Task] = []
        for t in req.get("tasks", []):
            frame = t["frame"]
            if not model.has_frame(frame):
                return {"ok": False, "error": f"unknown frame '{frame}'",
                        "id": req.get("id")}
            xyz, quat = self._resolve_target(t)
            if xyz is None:
                return {"ok": False, "reachable": False,
                        "reason": Reason.TF_UNAVAILABLE.value,
                        "id": req.get("id"),
                        "error": "TF transform unavailable"}
            stiff = t.get("stiffness", default_stiff)
            if "quat" in t and t["quat"] is not None:
                tasks.append(Task(frame, tuple(xyz), tuple(quat), tuple(stiff)))
            else:
                # point-only if no orientation given
                s = list(stiff)
                s[3] = s[4] = s[5] = 0.0
                tasks.append(Task(frame, tuple(xyz), (1.0, 0.0, 0.0, 0.0),
                                  tuple(s)))

        # extra tasks: arm-angle psi (R6) and relative pose (R9)
        extras = []
        for aa in req.get("arm_angles", []):
            chain = self._srs_chains.get(aa["chain"])
            if chain is None:
                return {"ok": False, "error": f"unknown srs chain '{aa['chain']}'",
                        "id": req.get("id")}
            st = float(aa.get("stiffness",
                              self.get_parameter("arm_angle_stiffness").value))
            extras.append(make_arm_angle_extra_task(model, chain,
                                                    float(aa["psi"]), st))
        rel = req.get("relative")
        if rel:
            extras.append(make_relative_extra_task(
                model, rel["frame_a"], rel["frame_b"],
                rel.get("xyz", [0, 0, 0]),
                rel.get("quat", [1, 0, 0, 0]),
                rel.get("stiffness", [1, 1, 1, 1, 1, 1])))

        # optional soft self-collision penalty (E2), request- or param-enabled
        sc = req.get("self_collision")
        if sc and sc.get("capsules"):
            from .collision import Capsule, make_self_collision_extra_task
            caps = [Capsule(c["frame_a"], c["frame_b"], float(c.get("radius", 0.05)))
                    for c in sc["capsules"]]
            extras.append(make_self_collision_extra_task(
                model, caps, float(sc.get("min_distance", 0.05)),
                float(sc.get("weight", 1.0))))

        params = ik_core.SolveParams(
            max_iters=int(self.get_parameter("max_iters").value),
            tol_pos=float(self.get_parameter("tol_pos").value),
            tol_ori=float(self.get_parameter("tol_ori").value),
            damping=float(self.get_parameter("damping").value),
            joint_centering_weight=float(
                self.get_parameter("joint_centering_weight").value),
        )
        sol = ik_core.solve(model, seed, tasks, params=params,
                            active_joints=req.get("active_joints"),
                            extra_tasks=extras or None)

        # publish solution JointState (advisory) + return JSON
        self._publish_solution(model, sol)
        with self._lock:
            self._last_solution = (model, sol)
        return self._solution_to_json(model, sol, req.get("id"))

    def _resolve_target(self, t: dict):
        """Return (xyz, quat_wxyz) in the solve/base frame, or (None, None)."""
        xyz = t.get("xyz")
        quat = t.get("quat", [1.0, 0.0, 0.0, 0.0])
        src = t.get("frame_id")
        if not src or self._tf_buffer is None or src == self._base_frame:
            return (np.asarray(xyz, dtype=float) if xyz is not None else None,
                    np.asarray(quat, dtype=float))
        # transform PoseStamped from src into base_frame via tf2 (R8)
        try:
            from geometry_msgs.msg import PoseStamped
            import tf2_geometry_msgs  # noqa: F401  (registers Pose transforms)
            ps = PoseStamped()
            ps.header.frame_id = src
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
                (float(xyz[0]), float(xyz[1]), float(xyz[2]))
            ps.pose.orientation.w = float(quat[0])
            ps.pose.orientation.x = float(quat[1])
            ps.pose.orientation.y = float(quat[2])
            ps.pose.orientation.z = float(quat[3])
            target_frame = self._base_frame or self._model_root()
            out = self._tf_buffer.transform(
                ps, target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            p = out.pose.position
            o = out.pose.orientation
            return (np.array([p.x, p.y, p.z]),
                    np.array([o.w, o.x, o.y, o.z]))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("TF resolve failed: %r" % exc)
            return None, None

    def _model_root(self) -> str:
        with self._lock:
            if self._model is None:
                return "base_link"
            names = self._model.frame_names()
        return names[1] if len(names) > 1 else "base_link"

    # ------------------------------------------------------------------ #
    # Outputs
    # ------------------------------------------------------------------ #
    def _publish_solution(self, model: RobotModel, sol) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(sol.joint_names)
        js.position = [float(sol.q[model.q_index(j)]) for j in sol.joint_names]
        self._sol_pub.publish(js)

    def _solution_to_json(self, model: RobotModel, sol, req_id) -> dict:
        chains = self._srs_chains
        psis = compute_all_psi(model, sol.q, chains) if chains else {}
        return {
            "ok": True,
            "id": req_id,
            "reachable": bool(sol.reachable),
            "reason": sol.reason.value,
            "iters": int(sol.iters),
            "joint_names": list(sol.joint_names),
            "q": [float(sol.q[model.q_index(j)]) for j in sol.joint_names],
            "max_pos_err": sol.max_pos_err(),
            "max_ori_err": sol.max_ori_err(),
            "residuals": [{"frame": r.frame, "pos_err": r.pos_err,
                           "ori_err": r.ori_err} for r in sol.residuals],
            "blocking_joints": list(sol.blocking_joints),
            "manipulability": sol.manipulability,
            "sigma_min": sol.sigma_min,
            "arm_angles": psis,
            "delta_norm": sol.delta_norm(),
        }

    def _publish_status(self) -> None:
        with self._lock:
            model = self._model
            have_js = bool(self._joint_pos)
            last = self._last_solution
        status = {
            "have_model": model is not None,
            "have_joint_states": have_js,
            "dof": model.nq if model else 0,
            "srs_chains": list(self._srs_chains.keys()),
            "advisory_only": True,
        }
        if model is not None:
            # Introspection so a dashboard can populate selection dropdowns
            # entirely from the live URDF (no offline config). Links are the
            # operable targets; joints are the movable DOF.
            status["links"] = model.link_frame_names()
            status["joints"] = list(model.joint_names)
        if model is not None and have_js:
            with self._lock:
                q = self._current_seed(model)
            status["arm_angles_now"] = compute_all_psi(model, q,
                                                       self._srs_chains)
        if last is not None:
            _, sol = last
            status["last"] = {"reachable": bool(sol.reachable),
                              "reason": sol.reason.value,
                              "max_pos_err": sol.max_pos_err(),
                              "max_ori_err": sol.max_ori_err()}
        m = String(); m.data = json.dumps(status)
        self._status_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
