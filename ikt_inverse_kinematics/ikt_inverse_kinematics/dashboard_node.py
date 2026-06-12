#!/usr/bin/env python3
"""Web dashboard for ikt_inverse_kinematics with a 3D robot viewer.

This dashboard is a UI/visualization client of ``ik_node`` (advisory only — it
never commands the robot). On top of the previous thin client it adds:

  * a **3D canvas** (Three.js) that renders the robot from
    ``/robot_description`` meshes at the live ``/joint_states`` configuration,
    plus a translucent **ghost** of the last IK solution and a TCP-frame triad
    on the operated link;
  * an interactive control panel exercising **every** ik_node function: pick
    any link, set/capture a target pose, per-DOF stiffness, virtual tool
    frames, arm-angle (psi) tasks, the dual-arm relative-pose constraint,
    active-joint masking and TF-framed targets.

It builds its own Pinocchio :class:`RobotModel` purely to compute per-link
forward kinematics for rendering; all *solving* is delegated to ``ik_node`` over
its ROS JSON API, so the dashboard genuinely tests the node.

Mesh + 3D approach mirrors ``src/rm_dashboard``: the backend computes per-link
4x4 transforms and serves meshes via ``/mesh``; the browser loads STL meshes and
places them at ``link_tf[link] * visual_origin``.
"""

from __future__ import annotations

import json
import mimetypes
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import parse_qs, quote, unquote, urlparse

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore

from .robot_model import RobotModel

_STATIC_DIR = Path(__file__).resolve().parent / "static"


def _latched_qos() -> QoSProfile:
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _f3(v) -> list:
    return [float(v[0]), float(v[1]), float(v[2])]


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


class IKDashboard(Node):
    def __init__(self) -> None:
        super().__init__("ik_dashboard")
        self.declare_parameter("port", 8160)
        self.declare_parameter("ik_ns", "/ik_node")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("base_frame", "")
        self._port = int(self.get_parameter("port").value)
        self._ns = str(self.get_parameter("ik_ns").value or "/ik_node").rstrip("/")
        self._desc_topic = str(self.get_parameter("robot_description_topic").value)
        self._js_topic = str(self.get_parameter("joint_states_topic").value)
        self._base_frame = str(self.get_parameter("base_frame").value or "")
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        # Pinocchio FK mutates the model's shared `data` buffer, so the HTTP
        # poll thread and the solve thread must not run FK concurrently.
        self._fk_lock = threading.Lock()
        self._urdf = ""
        self._model: Optional[RobotModel] = None
        self._visuals: List[dict] = []
        self._joint_pos: Dict[str, float] = {}
        self._status: Optional[dict] = None
        self._pkg_dirs: Dict[str, Optional[str]] = {}
        self._pending: Dict[str, dict] = {}   # solve id -> {event, resp}

        self.create_subscription(String, self._desc_topic, self._on_urdf,
                                 _latched_qos(), callback_group=self._cbg)
        self.create_subscription(JointState, self._js_topic, self._on_js,
                                 qos_profile_sensor_data, callback_group=self._cbg)
        self.create_subscription(String, f"{self._ns}/status",
                                 self._on_status, 10, callback_group=self._cbg)
        self.create_subscription(String, f"{self._ns}/solve_response",
                                 self._on_response, 10, callback_group=self._cbg)
        self._req_pub = self.create_publisher(
            String, f"{self._ns}/solve_request", 10)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            "ikt_inverse_kinematics 3D dashboard on http://%s:%d (ik_ns=%s) — "
            "UI/visualization only; solving delegated to ik_node."
            % (self._host, self._port, self._ns))

    # ------------------------------------------------------------------ #
    # ROS callbacks
    # ------------------------------------------------------------------ #
    def _on_urdf(self, msg: String) -> None:
        if not msg.data:
            return
        with self._lock:
            if msg.data == self._urdf and self._model is not None:
                return
            self._urdf = msg.data
        try:
            model = RobotModel(msg.data)
            vis = parse_visuals(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("failed to build model/visuals: %r" % exc)
            return
        with self._lock:
            self._model = model
            self._visuals = vis
        self.get_logger().info(
            "built model: %d DOF, %d links, %d mesh visuals."
            % (model.nq, len(model.link_frame_names()), len(vis)))

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_pos[n] = float(p)

    def _on_status(self, msg: String) -> None:
        try:
            with self._lock:
                self._status = json.loads(msg.data)
        except Exception:
            pass

    def _on_response(self, msg: String) -> None:
        try:
            resp = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            entry = self._pending.get(resp.get("id"))
            if entry is not None:
                entry["resp"] = resp
                entry["event"].set()

    # ------------------------------------------------------------------ #
    # FK / state helpers
    # ------------------------------------------------------------------ #
    def _full_q(self, model: RobotModel,
                overlay: Optional[Dict[str, float]] = None) -> np.ndarray:
        q = model.neutral()
        with self._lock:
            jp = dict(self._joint_pos)
        for jn in model.joint_names:
            if jn in jp:
                q[model.q_index(jn)] = jp[jn]
        if overlay:
            for jn, v in overlay.items():
                if jn in model.joint_names:
                    q[model.q_index(jn)] = float(v)
        return q

    def snapshot(self) -> dict:
        with self._lock:
            model = self._model
            visuals = list(self._visuals)
            status = self._status
            base = self._base_frame
        out: dict = {"has_model": model is not None, "ik_ns": self._ns,
                     "ik_status": status, "base_frame": base}
        if model is None:
            out["has_meshes"] = False
            return out
        q = self._full_q(model)
        with self._fk_lock:
            link_tf = model.all_link_transforms(q)
        out["links"] = model.link_frame_names()
        out["joints"] = list(model.joint_names)
        out["joint_values"] = {jn: float(q[model.q_index(jn)])
                               for jn in model.joint_names}
        out["link_tf"] = link_tf
        out["has_meshes"] = bool(visuals)
        out["visuals"] = [{"link": v["link"], "url": self._mesh_url(v["filename"]),
                           "xyz": v["xyz"], "rpy": v["rpy"], "scale": v["scale"]}
                          for v in visuals]
        out["skeleton"] = {"links": {k: [m[0][3], m[1][3], m[2][3]]
                                     for k, m in link_tf.items()}}
        return out

    def fk(self, frame: str) -> Optional[dict]:
        with self._lock:
            model = self._model
        if model is None or not model.has_frame(frame):
            return None
        with self._fk_lock:
            xyz, quat = model.fk(self._full_q(model), frame)
        return {"frame": frame, "xyz": _f3(xyz), "quat": [float(x) for x in quat]}

    def solve(self, req: dict, timeout: float = 6.0) -> dict:
        """Send a solve request to ik_node; augment the response with a ghost
        ``solution_link_tf`` (FK of the solved configuration) for the viewer."""
        with self._lock:
            model = self._model
        rid = req.get("id") or uuid.uuid4().hex[:12]
        req["id"] = rid
        event = threading.Event()
        with self._lock:
            self._pending[rid] = {"event": event, "resp": None}
        m = String()
        m.data = json.dumps(req)
        # Publish exactly once (avoid ik_node solving the same request 3x); wait
        # briefly for the subscription to be up on a cold first request.
        deadline = time.time() + 1.0
        while self._req_pub.get_subscription_count() < 1 and time.time() < deadline:
            time.sleep(0.02)
        self._req_pub.publish(m)
        ok = event.wait(timeout=timeout)
        with self._lock:
            entry = self._pending.pop(rid, None)
        if not ok or entry is None or entry.get("resp") is None:
            return {"ok": False, "error": "no response from ik_node (timeout)",
                    "id": rid}
        resp = entry["resp"]
        if model is not None and resp.get("ok") and resp.get("q") \
                and resp.get("joint_names"):
            overlay = {jn: v for jn, v in zip(resp["joint_names"], resp["q"])}
            with self._fk_lock:
                resp["solution_link_tf"] = model.all_link_transforms(
                    self._full_q(model, overlay))
        return resp

    # ------------------------------------------------------------------ #
    # mesh resolution / serving (mirrors src/rm_dashboard)
    # ------------------------------------------------------------------ #
    def _mesh_url(self, filename: str) -> str:
        if filename.startswith("package://"):
            rest = filename[len("package://"):]
            pkg, _, rel = rest.partition("/")
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

    # ---- HTTP -------------------------------------------------------------
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
                self.send_header("Access-Control-Allow-Origin", "*")
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
                if path.startswith("/vendor/") or path.endswith(
                        (".js", ".css", ".html")):
                    return self._serve_static(path.lstrip("/"))
                return self._send(404, b'{"error":"not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                if path == "/api/solve":
                    return self._send(200, json.dumps(dash.solve(self._read_json())))
                if path == "/api/fk":
                    out = dash.fk(self._read_json().get("frame", ""))
                    if out is None:
                        return self._send(200, json.dumps(
                            {"ok": False, "error": "unknown frame / no model"}))
                    out["ok"] = True
                    return self._send(200, json.dumps(out))
                return self._send(404, b'{"error":"not found"}')

            def _serve_static(self, relpath):
                fp = (_STATIC_DIR / relpath).resolve()
                if not str(fp).startswith(str(_STATIC_DIR.resolve())) \
                        or not fp.is_file():
                    return self._send(404, b"not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] or "application/octet-stream"
                if fp.suffix == ".js":
                    ctype = "text/javascript"
                self._send(200, fp.read_bytes(), ctype)

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKDashboard()
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
