#!/usr/bin/env python3
"""Optional web dashboard for ikt_pose_commander — a thin HTTP/ROS client.

It talks to a running ``commander_node`` over its ROS API (it does not import
the commander's internals):

  * subscribes ``<ns>/status`` (JSON) for monitoring;
  * calls ``<ns>/enable`` / ``<ns>/disable`` (std_srvs/Trigger);
  * publishes ``<ns>/target_pose`` (geometry_msgs/PoseStamped) to command motion.

It also renders a **3D canvas** (Three.js) showing the robot at the live
``/joint_states`` configuration plus a triad at the **commanded target pose** —
so a SpaceMouse / teleop stream can be visually checked. For that it builds its
own Pinocchio model from ``/robot_description`` purely to compute per-link
forward kinematics for rendering (same approach as the ikt_inverse_kinematics
dashboard; this is the shared FK library, not commander internals), and it
watches the same ``<ns>/target_pose`` topic it publishes to, so the marker
tracks whatever source is driving the commander (its own jog/send OR the
SpaceMouse ``pose_node``).

It also keeps its own TF listener so it can *capture* the controlled frame's
current pose and *jog* it (capture → offset one axis → publish). The commander
runs fine with this dashboard absent; the dashboard degrades gracefully when the
commander is absent.

Mirrors the cartesian_controller_dashboard / ikt_inverse_kinematics dashboard
pattern: a ThreadingHTTPServer in a daemon thread, rclpy on a
MultiThreadedExecutor with a ReentrantCallbackGroup so synchronous service calls
issued from the HTTP handler thread don't deadlock the ROS spin. Default port
8180 (8080/8100/8120/8140/8160 already used by other toolkit dashboards).
"""

from __future__ import annotations

import json
import mimetypes
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from urllib.parse import parse_qs, quote, unquote, urlparse

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

try:
    import tf2_ros
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore

# The 3D viewer builds a Pinocchio model purely to compute per-link forward
# kinematics for rendering (same approach as the ikt_inverse_kinematics
# dashboard). This is the shared FK library, not commander internals.
try:
    from ikt_core.robot_model import RobotModel
    _RM_IMPORT_ERROR: Optional[str] = None
except Exception as _exc:  # noqa: BLE001  pragma: no cover
    RobotModel = None  # type: ignore
    _RM_IMPORT_ERROR = f"{type(_exc).__name__}: {_exc}"

_STATIC_DIR = Path(__file__).resolve().parent / "static"


def _latched_qos() -> QoSProfile:
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def parse_visuals(urdf_xml: str) -> List[dict]:
    """Extract ``<link><visual>`` mesh entries from a URDF string.

    Returns ``[{link, filename, xyz, rpy, scale}]`` (mesh visuals only;
    primitive-only links fall back to the skeleton view).
    """
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for link in root.findall("link"):
        lname = link.get("name")
        if not lname:
            continue
        for vis in link.findall("visual"):
            geom = vis.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None or not mesh.get("filename"):
                continue
            origin = vis.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                if origin.get("xyz"):
                    xyz = [float(x) for x in origin.get("xyz").split()]
                if origin.get("rpy"):
                    rpy = [float(x) for x in origin.get("rpy").split()]
            scale = [1.0, 1.0, 1.0]
            if mesh.get("scale"):
                scale = [float(x) for x in mesh.get("scale").split()]
            out.append({"link": lname, "filename": mesh.get("filename"),
                        "xyz": xyz, "rpy": rpy, "scale": scale})
    return out


def parse_joint_tree(urdf_xml: str) -> List[dict]:
    """Extract the URDF joint topology for the 3D skeleton.

    Returns ``[{parent, child, type}]`` (parent/child are link names). The
    viewer draws a line segment between the world origins of ``parent`` and
    ``child`` for each joint, which gives a proper kinematic skeleton (rather
    than disconnected per-link dots).
    """
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for j in root.findall("joint"):
        p = j.find("parent")
        c = j.find("child")
        if p is None or c is None:
            continue
        pl, cl = p.get("link"), c.get("link")
        if pl and cl:
            out.append({"parent": pl, "child": cl,
                        "name": j.get("name", ""),
                        "type": j.get("type", "fixed")})
    return out


def _quat_wxyz_to_R(q) -> np.ndarray:
    w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    n = (w * w + x * x + y * y + z * z) ** 0.5
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _R_to_quat_wxyz(R) -> list:
    R = np.asarray(R, dtype=float)
    t = float(np.trace(R))
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w, x, y, z = 0.25 / s, (R[2, 1] - R[1, 2]) * s, \
            (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
            w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
            y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
            w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
            y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    q /= (np.linalg.norm(q) or 1.0)
    return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]


class CommanderDashboard(Node):
    def __init__(self) -> None:
        super().__init__("ikt_pose_commander_dashboard")
        self.declare_parameter("port", 8180)
        self.declare_parameter("commander_ns", "/ikt_pose_commander")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("status_stale_after", 2.0)
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        # Rate at which the dashboard republishes its latest target so it ALWAYS
        # streams target_pose (like the SpaceMouse bridge), not only on change.
        self.declare_parameter("stream_rate_hz", 100.0)

        self._port = int(self.get_parameter("port").value)
        self._ns = str(self.get_parameter("commander_ns").value).rstrip("/")
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._stale = float(self.get_parameter("status_stale_after").value)
        self._desc_topic = str(self.get_parameter("robot_description_topic").value)
        self._js_topic = str(self.get_parameter("joint_states_topic").value)
        self._stream_rate = float(self.get_parameter("stream_rate_hz").value)
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        # Pinocchio FK mutates a shared data buffer; serialize FK calls.
        self._fk_lock = threading.Lock()
        self._status: Optional[dict] = None
        self._status_stamp = 0.0
        # 3D viewer state (robot model built from /robot_description)
        self._urdf = ""
        self._model: Optional["RobotModel"] = None
        self._visuals: List[dict] = []
        self._joint_tree: List[dict] = []
        self._joint_pos: Dict[str, float] = {}
        self._pkg_dirs: Dict[str, Optional[str]] = {}
        # live commanded target (from <ns>/target_pose or status; incl. pose_node)
        self._target: Optional[dict] = None
        self._target_stamp = 0.0
        # Continuous target stream: republish the latest dashboard target at
        # stream_rate_hz so it ALWAYS sends (mirrors the SpaceMouse bridge), not
        # only on gizmo change. Active from a stream send until a disable.
        self._stream_pose: Optional[Tuple[list, list, str]] = None
        self._stream_active = False

        self.create_subscription(String, f"{self._ns}/status", self._on_status,
                                 10, callback_group=self._cbg)
        self.create_subscription(String, self._desc_topic, self._on_urdf,
                                 _latched_qos(), callback_group=self._cbg)
        self.create_subscription(JointState, self._js_topic, self._on_js,
                                 qos_profile_sensor_data, callback_group=self._cbg)
        # Watch the same target topic we publish to, so the canvas shows the
        # live target regardless of who sent it (this dashboard's jog/send OR
        # the SpaceMouse pose_node).
        self.create_subscription(PoseStamped, f"{self._ns}/target_pose",
                                 self._on_target, 10, callback_group=self._cbg)
        self._target_pub = self.create_publisher(
            PoseStamped, f"{self._ns}/target_pose", 10)
        if _RM_IMPORT_ERROR is not None:
            self.get_logger().warning(
                "ikt_core.robot_model unavailable (%s) - 3D robot view "
                "disabled; control panel still works." % _RM_IMPORT_ERROR)
        self._configure_pub = self.create_publisher(
            String, f"{self._ns}/configure", 10)
        self._cli_enable = self.create_client(
            Trigger, f"{self._ns}/enable", callback_group=self._cbg)
        self._cli_disable = self.create_client(
            Trigger, f"{self._ns}/disable", callback_group=self._cbg)
        self._cli_return = self.create_client(
            Trigger, f"{self._ns}/return_to_start", callback_group=self._cbg)
        self._cli_snap = self.create_client(
            Trigger, f"{self._ns}/snap_target", callback_group=self._cbg)
        # Optional teleop bridge reanchor (set_pose -> EE) so Snap recenters the
        # source too; harmless if no bridge is running.
        self._cli_reanchor = self.create_client(
            Trigger, "/spacemouse_teleop/reanchor", callback_group=self._cbg)
        # SpaceMouse forwarding gate: toggle whether the teleop bridge streams
        # the puck pose to target_pose, so control can be handed between the
        # SpaceMouse and this dashboard. The bridge latches ``forwarding``.
        self._sm_forwarding: Optional[bool] = None
        self._cli_set_forwarding = self.create_client(
            SetBool, "/spacemouse_teleop/set_forwarding",
            callback_group=self._cbg)
        self.create_subscription(
            Bool, "/spacemouse_teleop/forwarding", self._on_sm_forwarding,
            _latched_qos(), callback_group=self._cbg)
        # Continuous target streamer (always-send, like the bridge): the
        # _stream_tick timer republishes the latest target at stream_rate_hz.
        _srate = max(1.0, self._stream_rate)
        self.create_timer(1.0 / _srate, self._stream_tick,
                          callback_group=self._cbg)

        if _HAVE_TF:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            "ikt_pose_commander dashboard on http://%s:%d  (commander_ns=%s, "
            "base_frame=%s) — UI only; commands go through the commander's gates."
            % (self._host, self._port, self._ns, self._base_frame))

    # ------------------------------------------------------------------ #
    # ROS callbacks / helpers
    # ------------------------------------------------------------------ #
    def _on_status(self, msg: String) -> None:
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = s
            self._status_stamp = time.monotonic()

    def _on_sm_forwarding(self, msg: Bool) -> None:
        with self._lock:
            fwd = bool(msg.data)
            if fwd and not self._sm_forwarding:
                # SpaceMouse just took over -> stop our own target stream so the
                # two sources never both drive target_pose (double rate/conflict).
                self._stream_active = False
            self._sm_forwarding = fwd

    def _on_urdf(self, msg: String) -> None:
        if not msg.data or RobotModel is None:
            return
        with self._lock:
            if msg.data == self._urdf and self._model is not None:
                return
            self._urdf = msg.data
        try:
            model = RobotModel(msg.data)
            vis = parse_visuals(msg.data)
            jtree = parse_joint_tree(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("failed to build model/visuals: %r" % exc)
            return
        with self._lock:
            self._model = model
            self._visuals = vis
            self._joint_tree = jtree
        self.get_logger().info(
            "3D view: built model (%d DOF, %d links, %d mesh visuals, %d joints)."
            % (model.nq, len(model.link_frame_names()), len(vis), len(jtree)))

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_pos[n] = float(p)

    def _on_target(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._target = {
                "xyz": [float(p.x), float(p.y), float(p.z)],
                "quat": [float(o.w), float(o.x), float(o.y), float(o.z)],
                "frame_id": msg.header.frame_id or self._base_frame,
            }
            self._target_stamp = time.monotonic()

    # ------------------------------------------------------------------ #
    # 3D viewer FK / mesh helpers (mirror the ikt_inverse_kinematics dashboard)
    # ------------------------------------------------------------------ #
    def _full_q(self, model: "RobotModel") -> np.ndarray:
        q = model.neutral()
        with self._lock:
            jp = dict(self._joint_pos)
        for jn in model.joint_names:
            if jn in jp:
                q[model.q_index(jn)] = jp[jn]
        return q

    def _mesh_url(self, filename: str) -> str:
        if filename.startswith("package://"):
            pkg, _, rel = filename[len("package://"):].partition("/")
            return f"/mesh?pkg={quote(pkg)}&path={quote(rel)}"
        if filename.startswith("file://"):
            return f"/mesh?path={quote(filename[len('file://'):])}"
        return f"/mesh?path={quote(filename)}"

    def _package_dir(self, pkg: str) -> Optional[str]:
        if pkg in self._pkg_dirs:
            return self._pkg_dirs[pkg]
        resolved = None
        if get_package_share_directory is not None:
            try:
                resolved = get_package_share_directory(pkg)
            except Exception:
                resolved = None
        self._pkg_dirs[pkg] = resolved
        return resolved

    def read_mesh(self, pkg: str, rel: str) -> Optional[bytes]:
        rel = unquote(rel or "")
        if pkg:
            base = self._package_dir(unquote(pkg))
            if base is None:
                return None
            path = Path(base) / rel
        else:
            path = Path(rel)
        try:
            return path.resolve().read_bytes()
        except Exception:
            return None

    def _target_in_base(self) -> Optional[dict]:
        """Live commanded target, expressed in the render (base) frame.

        Transforms from the message's ``frame_id`` to ``base_frame`` via TF when
        they differ (the common SpaceMouse pose_node case already publishes in the
        base frame, so this is usually identity).
        """
        with self._lock:
            tgt = dict(self._target) if self._target else None
            stamp = self._target_stamp
        if tgt is None:
            return None
        xyz, quat, fid = tgt["xyz"], tgt["quat"], tgt["frame_id"]
        transformed_from = None
        if fid and fid != self._base_frame and self._tf_buffer is not None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame, fid, rclpy.time.Time())
                t, r = tf.transform.translation, tf.transform.rotation
                T = np.eye(4)
                T[:3, :3] = _quat_wxyz_to_R([r.w, r.x, r.y, r.z])
                T[:3, 3] = [t.x, t.y, t.z]
                P = np.eye(4)
                P[:3, :3] = _quat_wxyz_to_R(quat)
                P[:3, 3] = xyz
                M = T @ P
                xyz = [float(M[0, 3]), float(M[1, 3]), float(M[2, 3])]
                quat = _R_to_quat_wxyz(M[:3, :3])
                transformed_from, fid = fid, self._base_frame
            except Exception:
                pass
        age = time.monotonic() - stamp
        return {"xyz": xyz, "quat": quat, "frame_id": fid,
                "age": round(age, 2), "fresh": age <= self._stale,
                "transformed_from": transformed_from}

    def _target_from_status(self) -> Optional[dict]:
        """The commander's internal goal pose (from ``~/status``), in the render
        frame. Used as a fallback for the 3D marker when nothing is publishing on
        ``~/target_pose`` (e.g. after a snap, or before the bridge sends its
        first target). The internal target is already in the model root frame
        (== base_frame here)."""
        with self._lock:
            s = self._status
            stamp = self._status_stamp
        if not s:
            return None
        tp = s.get("target_pose")
        if not isinstance(tp, dict):
            return None
        xyz, quat = tp.get("xyz"), tp.get("quat")
        if not (isinstance(xyz, list) and len(xyz) == 3
                and isinstance(quat, list) and len(quat) == 4):
            return None
        age = time.monotonic() - stamp if stamp else None
        return {"xyz": [float(v) for v in xyz],
                "quat": [float(v) for v in quat],
                "frame_id": self._base_frame,
                "age": round(age, 2) if age is not None else None,
                "fresh": age is not None and age <= self._stale,
                "transformed_from": None, "source": "status"}

    def _best_target(self) -> Optional[dict]:
        """Live commanded target for the 3D marker: prefer a FRESH absolute
        target on ``~/target_pose`` (gizmo / spacemouse-absolute), else fall back
        to the commander's internal goal pose from ``~/status`` (snap / delta)."""
        topic_t = self._target_in_base()
        if topic_t is not None and topic_t.get("fresh"):
            return topic_t
        status_t = self._target_from_status()
        if status_t is not None:
            return status_t
        return topic_t

    def snapshot(self) -> dict:
        with self._lock:
            s = self._status
            age = time.monotonic() - self._status_stamp if self._status_stamp \
                else None
            model = self._model
            visuals = list(self._visuals)
            joint_tree = list(self._joint_tree)
        fresh = age is not None and age <= self._stale
        out = {"status": s, "fresh": fresh,
               "age": round(age, 2) if age is not None else None,
               "commander_ns": self._ns, "base_frame": self._base_frame,
               "target": self._best_target(), "has_model_viz": model is not None}
        with self._lock:
            fwd = self._sm_forwarding
        out["spacemouse"] = {
            "bridge": self._cli_set_forwarding.service_is_ready(),
            "forwarding": fwd}
        if model is not None:
            q = self._full_q(model)
            with self._fk_lock:
                link_tf = model.all_link_transforms(q)
            out["link_tf"] = link_tf
            out["has_meshes"] = bool(visuals)
            out["visuals"] = [
                {"link": v["link"], "url": self._mesh_url(v["filename"]),
                 "xyz": v["xyz"], "rpy": v["rpy"], "scale": v["scale"]}
                for v in visuals]
            out["skeleton"] = {k: [m[0][3], m[1][3], m[2][3]]
                               for k, m in link_tf.items()}
            # Joint topology (parent/child link names) so the viewer can draw a
            # proper kinematic skeleton (lines parent->child), not just dots.
            out["joint_tree"] = joint_tree
            # Link / joint introspection for the 3D viewer's labels, link list,
            # highlight, and the joint-angle + TCP-pose panels.
            out["links"] = model.link_frame_names()
            out["joints"] = list(model.joint_names)
            out["joint_values"] = {jn: float(q[model.q_index(jn)])
                                   for jn in model.joint_names}
            # Per-joint (lower, upper) limits for the on-canvas joint bars;
            # non-finite (continuous) limits are sent as null so the viewer
            # falls back to a +/-pi display range.
            try:
                lo, hi = model.joint_limits()
                jl = {}
                for jn in model.joint_names:
                    i = model.q_index(jn)
                    lov, hiv = float(lo[i]), float(hi[i])
                    jl[jn] = [lov if np.isfinite(lov) else None,
                              hiv if np.isfinite(hiv) else None]
                out["joint_limits"] = jl
            except Exception:  # noqa: BLE001
                out["joint_limits"] = {}
            out["controlled_frame"] = (s or {}).get("controlled_frame")
            # Name of the model root frame (frames[0] is "universe"); the 3D
            # gizmo sends targets expressed in this frame so they map 1:1 to
            # the canvas, which renders link transforms in the root frame.
            fn = model.frame_names()
            out["root_frame"] = fn[1] if len(fn) > 1 else ""
        return out

    def _controlled_frame(self) -> Optional[str]:
        with self._lock:
            s = self._status or {}
        return s.get("controlled_frame")

    def configure(self, cfg: dict) -> dict:
        """Relay a config request to the commander's ``~/configure`` topic.

        Any subset of commander config keys is accepted (the commander
        validates and applies them live or structurally). ``controlled_frame``
        is only needed for the initial group setup; live tunables
        (default_stiffness, allow_unreachable, speed/step limits,
        control_rate_hz, ...) can be sent on their own -> full
        dashboard parity with the topic/param/service path.
        """
        if not isinstance(cfg, dict) or not cfg:
            return {"ok": False, "message": "empty config"}
        m = String()
        m.data = json.dumps(cfg)
        for _ in range(3):
            self._configure_pub.publish(m)
            time.sleep(0.02)
        return {"ok": True,
                "message": "configure sent (%s)" % ", ".join(sorted(cfg))}

    def call_trigger(self, enable: bool, timeout: float = 5.0) -> dict:
        cli = self._cli_enable if enable else self._cli_disable
        if not enable:
            with self._lock:
                self._stream_active = False   # stop the dashboard target stream
        if not cli.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "message": "service unavailable "
                    "(is the commander running?)"}
        fut = cli.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"ok": False, "message": "service call timed out"}
        res = fut.result()
        return {"ok": bool(res.success), "message": res.message}

    def call_return_to_start(self, timeout: float = 20.0) -> dict:
        """Call the commander's ``~/return_to_start`` service (JTC move home)."""
        if not self._cli_return.wait_for_service(timeout_sec=5.0):
            return {"ok": False, "message": "return_to_start unavailable"}
        fut = self._cli_return.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"ok": False, "message": "return_to_start timed out"}
        res = fut.result()
        return {"ok": bool(res.success), "message": res.message}

    def call_snap_target(self, timeout: float = 5.0) -> dict:
        """Call the commander's ``~/snap_target`` service.

        Snaps the commander's internal target onto the controlled frame's
        CURRENT pose (forward kinematics of the measured joints) -- the
        server-side counterpart of the 3D "Snap target -> link" button, used to
        seed the goal before delta jogging or to re-centre with no jump.
        """
        if not self._cli_snap.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "message": "snap_target unavailable "
                    "(is the commander running + configured?)"}
        fut = self._cli_snap.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"ok": False, "message": "snap_target timed out"}
        res = fut.result()
        return {"ok": bool(res.success), "message": res.message}

    def call_reanchor(self, timeout: float = 2.0) -> dict:
        """Reset the teleop source to the current EE (bridge ~/reanchor ->
        set_pose) so a Snap also recenters the puck and the goal stays put.
        No-op if no bridge is running."""
        if not self._cli_reanchor.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "message": "no teleop bridge"}
        fut = self._cli_reanchor.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"ok": False, "message": "reanchor timed out"}
        r = fut.result()
        return {"ok": bool(r.success), "message": r.message}

    def call_set_forwarding(self, enable: bool, timeout: float = 2.0) -> dict:
        """Toggle the teleop bridge's forwarding (~/set_forwarding, SetBool).

        ON  = SpaceMouse drives target_pose; OFF = this dashboard does. Returns
        a no-op message if no bridge is running."""
        if not self._cli_set_forwarding.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "message": "no teleop bridge"}
        req = SetBool.Request()
        req.data = bool(enable)
        fut = self._cli_set_forwarding.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"ok": False, "message": "set_forwarding timed out"}
        r = fut.result()
        return {"ok": bool(r.success), "message": r.message}

    def capture(self, timeout: float = 2.0
                ) -> Tuple[Optional[list], Optional[list]]:
        """Look up the controlled frame's current pose in base_frame."""
        frame = self._controlled_frame()
        if frame is None:
            return None, None
        if self._tf_buffer is None:
            return None, None
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame, frame, rclpy.time.Time())
                t = tf.transform.translation
                r = tf.transform.rotation
                return ([t.x, t.y, t.z], [r.w, r.x, r.y, r.z])
            except Exception:
                time.sleep(0.05)
        return None, None

    def _build_target_msg(self, xyz: list, quat: list, frame_id: str
                          ) -> PoseStamped:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id or self._base_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        m.pose.orientation.w = float(quat[0])
        m.pose.orientation.x = float(quat[1])
        m.pose.orientation.y = float(quat[2])
        m.pose.orientation.z = float(quat[3])
        return m

    def send_pose(self, xyz: list, quat: list, frame_id: str) -> dict:
        m = self._build_target_msg(xyz, quat, frame_id)
        # publish a few times so a just-matched subscriber surely receives it
        for _ in range(3):
            self._target_pub.publish(m)                  # target pose -> commander
            time.sleep(0.02)
        return {"ok": True, "xyz": [m.pose.position.x, m.pose.position.y,
                                    m.pose.position.z], "frame_id": m.header.frame_id}

    def stream_pose(self, xyz: list, quat: list, frame_id: str) -> dict:
        """Publish a target setpoint AND keep streaming it.

        Publishes immediately, then stores it as the active stream target so the
        ``_stream_tick`` timer keeps republishing at ``stream_rate_hz`` -- the
        dashboard then ALWAYS sends target_pose (like the SpaceMouse bridge),
        not only when the gizmo moves. Streaming stops on disable.
        """
        m = self._build_target_msg(xyz, quat, frame_id)
        with self._lock:
            # While the SpaceMouse bridge is forwarding it OWNS target_pose, so
            # ignore the dashboard target entirely (don't arm the stream) -- the
            # two sources never both drive, and nothing stale resumes later.
            active = not self._sm_forwarding
            if active:
                self._stream_pose = (list(xyz), list(quat), frame_id)
                self._stream_active = True
        if active:
            self._target_pub.publish(m)
        return {"ok": True, "xyz": [m.pose.position.x, m.pose.position.y,
                                    m.pose.position.z], "frame_id": m.header.frame_id}

    def _stream_tick(self) -> None:
        with self._lock:
            # Suppress while the SpaceMouse bridge is forwarding so the two
            # sources never both publish target_pose (double rate + conflict).
            active = self._stream_active and not self._sm_forwarding
            sp = self._stream_pose
        if not (active and sp):
            return
        xyz, quat, frame_id = sp
        self._target_pub.publish(self._build_target_msg(xyz, quat, frame_id))

    def jog(self, axis: str, delta: float) -> dict:
        xyz, quat = self.capture()
        if xyz is None:
            return {"ok": False, "message": "could not capture current pose "
                    "(TF unavailable or no status yet)"}
        idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if idx is None:
            return {"ok": False, "message": f"bad axis '{axis}'"}
        xyz[idx] += float(delta)
        out = self.send_pose(xyz, quat, self._base_frame)
        out["message"] = f"jog {axis}{'+' if delta >= 0 else ''}{delta} sent"
        return out

    # ------------------------------------------------------------------ #
    # HTTP
    # ------------------------------------------------------------------ #
    def _start_http(self) -> None:
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # silence access log
                pass

            def _send(self, code, body, ctype="application/json"):
                if isinstance(body, str):
                    body = body.encode()
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> dict:
                n = int(self.headers.get("Content-Length", 0))
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    return json.loads(raw or b"{}")
                except Exception:
                    return {}

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    return self._serve_static("index.html")
                if path == "/api/state":
                    return self._send(200, json.dumps(dash.snapshot()))
                if path == "/mesh":
                    q = parse_qs(urlparse(self.path).query)
                    data = dash.read_mesh((q.get("pkg") or [""])[0],
                                          (q.get("path") or [""])[0])
                    if data is None:
                        return self._send(404, b'{"error":"mesh not found"}')
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "public, max-age=86400")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if path.startswith("/static/"):
                    return self._serve_static(path[len("/static/"):])
                if path.startswith("/vendor/") or path.endswith(
                        (".css", ".js", ".html")):
                    return self._serve_static(path.lstrip("/"))
                return self._send(404, '{"error":"not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                if path == "/api/configure":
                    return self._send(200, json.dumps(
                        dash.configure(self._read_json())))
                if path == "/api/enable":
                    return self._send(200, json.dumps(dash.call_trigger(True)))
                if path == "/api/disable":
                    return self._send(200, json.dumps(dash.call_trigger(False)))
                if path == "/api/return_to_start":
                    return self._send(200, json.dumps(
                        dash.call_return_to_start()))
                if path == "/api/snap_target":
                    return self._send(200, json.dumps(
                        dash.call_snap_target()))
                if path == "/api/reanchor":
                    return self._send(200, json.dumps(dash.call_reanchor()))
                if path == "/api/spacemouse_forwarding":
                    body = self._read_json()
                    return self._send(200, json.dumps(
                        dash.call_set_forwarding(bool(body.get("enabled")))))
                if path == "/api/capture":
                    xyz, quat = dash.capture()
                    if xyz is None:
                        return self._send(200, json.dumps(
                            {"ok": False, "message": "capture failed"}))
                    return self._send(200, json.dumps(
                        {"ok": True, "xyz": xyz, "quat": quat,
                         "frame_id": dash._base_frame}))
                if path == "/api/send":
                    b = self._read_json()
                    try:
                        out = dash.send_pose(b["xyz"], b.get(
                            "quat", [1, 0, 0, 0]), b.get("frame_id", ""))
                    except Exception as exc:  # noqa: BLE001
                        return self._send(200, json.dumps(
                            {"ok": False, "message": f"bad request: {exc}"}))
                    return self._send(200, json.dumps(out))
                if path == "/api/target":
                    b = self._read_json()
                    try:
                        out = dash.stream_pose(b["xyz"], b.get(
                            "quat", [1, 0, 0, 0]), b.get("frame_id", ""))
                    except Exception as exc:  # noqa: BLE001
                        return self._send(200, json.dumps(
                            {"ok": False, "message": f"bad request: {exc}"}))
                    return self._send(200, json.dumps(out))
                if path == "/api/jog":
                    b = self._read_json()
                    return self._send(200, json.dumps(
                        dash.jog(b.get("axis", "x"), b.get("delta", 0.01))))
                return self._send(404, '{"error":"not found"}')

            def _serve_static(self, relpath):
                fp = (_STATIC_DIR / relpath).resolve()
                if not str(fp).startswith(str(_STATIC_DIR.resolve())) \
                        or not fp.is_file():
                    return self._send(404, "not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] or "application/octet-stream"
                if fp.suffix == ".js":
                    ctype = "text/javascript"
                return self._send(200, fp.read_bytes(), ctype)

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommanderDashboard()
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
