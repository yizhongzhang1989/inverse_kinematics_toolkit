"""RViz interactive-marker bridge for the IK solver (E4).

Publishes one draggable 6-DOF ``InteractiveMarker`` per configured task frame.
On drag-release it sends a JSON solve request to ik_node and logs the verdict
(reachable / reason / residual). A pure client of ik_node's ROS API — it never
commands the robot. Open RViz, add an "InteractiveMarkers" display on
``/ik_markers/update``, and drag.

Params:
  frames        string[]  task frames to expose (default right/left tip)
  ik_ns         string    ik_node namespace (default /ik_node)
  base_frame    string    marker fixed frame (default the first frame's root)
"""

from __future__ import annotations

import json
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from interactive_markers import InteractiveMarkerServer
    from visualization_msgs.msg import (InteractiveMarker,
                                        InteractiveMarkerControl,
                                        Marker)
    _HAVE_IM = True
except Exception:  # pragma: no cover
    _HAVE_IM = False


class MarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_marker")
        self.declare_parameter("frames", [""])
        self.declare_parameter("ik_ns", "/ik_node")
        self.declare_parameter("base_frame", "base_link")
        self._frames: List[str] = [f for f in
                                   (self.get_parameter("frames").value or [])
                                   if f]
        ns = self.get_parameter("ik_ns").value or "/ik_node"
        self._base = self.get_parameter("base_frame").value or "base_link"

        self._req_pub = self.create_publisher(String, f"{ns}/solve_request", 10)
        self.create_subscription(String, f"{ns}/solve_response",
                                 self._on_resp, 10)

        if not _HAVE_IM:
            self.get_logger().warn(
                "interactive_markers / visualization_msgs not available; "
                "marker_node is a no-op. Install rviz/interactive_markers.")
            return

        self._server = InteractiveMarkerServer(self, "ik_markers")
        if not self._frames:
            self.get_logger().info(
                "no 'frames' set — marker_node idle. Set the 'frames' param to "
                "the link(s) you want draggable markers for (any URDF link).")
        for fr in self._frames:
            self._make_marker(fr)
        self._server.applyChanges()
        self.get_logger().info(
            "IK interactive markers up for [%s] (drag in RViz; advisory only)."
            % ", ".join(self._frames))

    def _make_marker(self, frame: str) -> None:
        im = InteractiveMarker()
        im.header.frame_id = self._base
        im.name = frame
        im.description = f"IK: {frame}"
        im.scale = 0.2

        # a small sphere so the control is visible
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = \
            0.1, 0.6, 1.0, 0.9
        vis = InteractiveMarkerControl()
        vis.always_visible = True
        vis.markers.append(sphere)
        im.controls.append(vis)

        # 3 translation + 3 rotation controls
        for axis, (x, y, z) in {"x": (1, 0, 0), "y": (0, 1, 0),
                                "z": (0, 0, 1)}.items():
            for mode, name in ((InteractiveMarkerControl.MOVE_AXIS, "move"),
                               (InteractiveMarkerControl.ROTATE_AXIS, "rot")):
                c = InteractiveMarkerControl()
                c.name = f"{name}_{axis}"
                c.interaction_mode = mode
                import math
                # orient the control along the axis (quaternion w,x,y,z)
                c.orientation.w = 1.0
                c.orientation.x = float(x)
                c.orientation.y = float(z)
                c.orientation.z = float(y)
                im.controls.append(c)

        self._server.insert(im, feedback_callback=self._on_feedback)

    def _on_feedback(self, fb) -> None:
        if _HAVE_IM and fb.event_type != fb.MOUSE_UP:
            return
        p = fb.pose.position
        o = fb.pose.orientation
        req = {"id": f"marker:{fb.marker_name}",
               "tasks": [{"frame": fb.marker_name,
                          "xyz": [p.x, p.y, p.z],
                          "quat": [o.w, o.x, o.y, o.z]}]}
        m = String(); m.data = json.dumps(req)
        self._req_pub.publish(m)

    def _on_resp(self, msg: String) -> None:
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        if str(r.get("id", "")).startswith("marker:"):
            self.get_logger().info(
                "IK %s: reachable=%s reason=%s pos_err=%.4f"
                % (r.get("id"), r.get("reachable"), r.get("reason"),
                   r.get("max_pos_err", float("nan"))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerNode()
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
