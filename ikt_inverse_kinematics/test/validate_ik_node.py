#!/usr/bin/env python3
"""End-to-end validation client for ik_node (no hardware motion).

Subscribes to /robot_description + /joint_states, builds a local RobotModel,
picks a *reachable* target by FK from a small perturbation of the current pose,
fires a JSON solve request at ik_node, waits for the JSON response, and checks
the solver reproduced the target. Pure client — it never commands the robot.

Usage:
  python3 validate_ik_node.py --frame right_arm_Link7 --ns /ik_node
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ikt_inverse_kinematics.robot_model import RobotModel


def latched():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Validator(Node):
    def __init__(self, ns, frame, active):
        super().__init__("ik_validator")
        self.frame = frame
        self.active = active
        self.urdf = None
        self.jpos = {}
        self.resp = None
        self.create_subscription(String, "/robot_description", self._u, latched())
        self.create_subscription(JointState, "/joint_states", self._j, 50)
        self.pub = self.create_publisher(String, f"{ns}/solve_request", 10)
        self.create_subscription(String, f"{ns}/solve_response", self._r, 10)

    def _u(self, m): self.urdf = m.data
    def _j(self, m):
        for n, p in zip(m.name, m.position):
            self.jpos[n] = float(p)
    def _r(self, m): self.resp = json.loads(m.data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="right_arm_Link7")
    ap.add_argument("--ns", default="/ik_node")
    ap.add_argument("--offset", type=float, default=0.05,
                    help="cartesian target offset (m) from current pose")
    args = ap.parse_args()
    active = [f"{args.frame.split('_Link')[0]}_joint{i}" for i in range(1, 8)]

    rclpy.init()
    node = Validator(args.ns, args.frame, active)
    t0 = time.time()
    while time.time() - t0 < 8.0 and (node.urdf is None or not node.jpos):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.urdf is None or not node.jpos:
        print("FAIL: no robot_description / joint_states"); return 2

    model = RobotModel(node.urdf)
    q = model.neutral()
    for jn in model.joint_names:
        if jn in node.jpos:
            q[model.q_index(jn)] = node.jpos[jn]

    # Pick a guaranteed-reachable target: FK of a small random perturbation of
    # the current configuration on the active joints. This avoids workspace
    # boundary / singular targets (e.g. the all-zero mock home is singular) and
    # keeps the implied motion small.
    rng = np.random.default_rng(0)
    q_pert = q.copy()
    for jn in active:
        q_pert[model.q_index(jn)] += rng.uniform(-0.15, 0.15)
    target_xyz, target_quat = model.fk(q_pert, args.frame)
    p_cur, _ = model.fk(q, args.frame)
    req = {"id": "validate1",
           "tasks": [{"frame": args.frame,
                      "xyz": [float(v) for v in target_xyz],
                      "quat": [float(v) for v in target_quat]}],
           "active_joints": active}
    print(f"current {args.frame} xyz={np.round(p_cur,4)} -> target "
          f"{np.round(target_xyz,4)} (|d|={np.linalg.norm(target_xyz-p_cur):.3f} m)")

    # wait for ik_node to subscribe, then publish
    t0 = time.time()
    while self_count(node) == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    m = String(); m.data = json.dumps(req); node.pub.publish(m)

    t0 = time.time()
    while node.resp is None and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.resp is None:
        print("FAIL: no solve_response from ik_node"); return 2

    r = node.resp
    print("RESPONSE:", json.dumps(r, indent=2)[:800])
    ok = r.get("ok") and r.get("reachable") and r.get("max_pos_err", 1) <= 2e-3
    print("VALIDATE", "PASS" if ok else "FAIL")
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


def self_count(node):
    return node.pub.get_subscription_count()


if __name__ == "__main__":
    sys.exit(main())
