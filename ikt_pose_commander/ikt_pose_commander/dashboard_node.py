#!/usr/bin/env python3
"""Optional web dashboard for ikt_pose_commander — a thin HTTP/ROS client.

INDEPENDENT FROM THE CORE: this node imports no commander/IK internals. It talks
to a running ``commander_node`` purely over its ROS API:

  * subscribes ``<ns>/status`` (JSON) for monitoring;
  * calls ``<ns>/enable`` / ``<ns>/disable`` (std_srvs/Trigger);
  * publishes ``<ns>/target_pose`` (geometry_msgs/PoseStamped) to command motion.

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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional, Tuple
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import tf2_ros
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

_STATIC_DIR = Path(__file__).resolve().parent / "static"


class CommanderDashboard(Node):
    def __init__(self) -> None:
        super().__init__("ikt_pose_commander_dashboard")
        self.declare_parameter("port", 8180)
        self.declare_parameter("commander_ns", "/ikt_pose_commander_right")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("status_stale_after", 2.0)

        self._port = int(self.get_parameter("port").value)
        self._ns = str(self.get_parameter("commander_ns").value).rstrip("/")
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._stale = float(self.get_parameter("status_stale_after").value)
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._status: Optional[dict] = None
        self._status_stamp = 0.0

        self.create_subscription(String, f"{self._ns}/status", self._on_status,
                                 10, callback_group=self._cbg)
        self._target_pub = self.create_publisher(
            PoseStamped, f"{self._ns}/target_pose", 10)
        self._configure_pub = self.create_publisher(
            String, f"{self._ns}/configure", 10)
        self._cli_enable = self.create_client(
            Trigger, f"{self._ns}/enable", callback_group=self._cbg)
        self._cli_disable = self.create_client(
            Trigger, f"{self._ns}/disable", callback_group=self._cbg)

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

    def snapshot(self) -> dict:
        with self._lock:
            s = self._status
            age = time.monotonic() - self._status_stamp if self._status_stamp \
                else None
        fresh = age is not None and age <= self._stale
        return {"status": s, "fresh": fresh,
                "age": round(age, 2) if age is not None else None,
                "commander_ns": self._ns, "base_frame": self._base_frame}

    def _controlled_frame(self) -> Optional[str]:
        with self._lock:
            s = self._status or {}
        return s.get("controlled_frame")

    def configure(self, cfg: dict) -> dict:
        """Relay a config request to the commander's ``~/configure`` topic.

        ``cfg`` needs at least ``controlled_frame``; joints + controllers are
        auto-derived by the commander from the URDF + controller_manager.
        """
        frame = (cfg or {}).get("controlled_frame")
        if not frame:
            return {"ok": False, "message": "need 'controlled_frame'"}
        m = String()
        m.data = json.dumps(cfg)
        for _ in range(3):
            self._configure_pub.publish(m)
            time.sleep(0.02)
        return {"ok": True, "message": f"configure sent (link={frame})"}

    def call_trigger(self, enable: bool, timeout: float = 5.0) -> dict:
        cli = self._cli_enable if enable else self._cli_disable
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

    def send_pose(self, xyz: list, quat: list, frame_id: str) -> dict:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id or self._base_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        m.pose.orientation.w = float(quat[0])
        m.pose.orientation.x = float(quat[1])
        m.pose.orientation.y = float(quat[2])
        m.pose.orientation.z = float(quat[3])
        # publish a few times so a just-matched subscriber surely receives it
        for _ in range(3):
            self._target_pub.publish(m)
            time.sleep(0.02)
        return {"ok": True, "xyz": [m.pose.position.x, m.pose.position.y,
                                    m.pose.position.z], "frame_id": m.header.frame_id}

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
                if path.startswith("/static/") or path.endswith(
                        (".css", ".js", ".html")):
                    return self._serve_static(Path(path).name)
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
                if path == "/api/jog":
                    b = self._read_json()
                    return self._send(200, json.dumps(
                        dash.jog(b.get("axis", "x"), b.get("delta", 0.01))))
                return self._send(404, '{"error":"not found"}')

            def _serve_static(self, name):
                fp = _STATIC_DIR / name
                if not fp.is_file():
                    return self._send(404, "not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] or "text/plain"
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
