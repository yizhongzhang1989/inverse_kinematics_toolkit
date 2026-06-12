#!/usr/bin/env python3
"""send_pose — tiny CLI to publish one target PoseStamped to the commander.

Convenience for testing without writing a publisher. Publishes a single
``geometry_msgs/PoseStamped`` (latched briefly) to the commander's target topic.

Examples
--------
    # absolute pose in the base frame
    ros2 run ikt_pose_commander send_pose \
        --topic /ikt_pose_commander/target_pose \
        --xyz 0.4 -0.2 0.6 --quat 1 0 0 0 --frame-id base_link

    # capture a frame's current pose as the target (no motion if already there)
    ros2 run ikt_pose_commander send_pose --capture right_arm_Link7
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="send_pose")
    ap.add_argument("--topic", default="/ikt_pose_commander/target_pose")
    ap.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--quat", nargs=4, type=float,
                    metavar=("W", "X", "Y", "Z"), default=[1.0, 0.0, 0.0, 0.0])
    ap.add_argument("--frame-id", default="",
                    help="source frame of the pose (TF-resolved by commander); "
                         "empty = already in the commander's base frame")
    ap.add_argument("--capture", metavar="FRAME",
                    help="instead of --xyz, look up this frame's current pose "
                         "from /tf and send THAT (a no-op target)")
    args = ap.parse_args(argv)

    if not args.capture and not args.xyz:
        ap.error("provide --xyz X Y Z (and optional --quat) or --capture FRAME")

    rclpy.init()
    node = Node("send_pose")
    pub = node.create_publisher(PoseStamped, args.topic, 1)

    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()

    if args.capture:
        import tf2_ros
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, node)
        base = args.frame_id or "base_link"
        deadline = time.time() + 5.0
        tf = None
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform(base, args.capture,
                                          rclpy.time.Time())
                break
            except Exception:  # noqa: BLE001
                continue
        if tf is None:
            print(f"could not look up {args.capture} in {base}", file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        t = tf.transform.translation
        r = tf.transform.rotation
        msg.header.frame_id = base
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
            t.x, t.y, t.z
        msg.pose.orientation = r
    else:
        msg.header.frame_id = args.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.xyz
        w, x, y, z = args.quat
        msg.pose.orientation.w = w
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z

    # publish a few times so the latched-less subscriber surely receives it
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    print(f"sent target to {args.topic}: "
          f"xyz=[{msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
          f"{msg.pose.position.z:.3f}] frame_id='{msg.header.frame_id}'")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
