#!/usr/bin/env python3
"""solve_and_send — solve IK via the standalone ``ik_node`` and (optionally)
drive a forward_position_controller directly, WITHOUT the commander (R4).

This is the easy "IK result -> move" path for using the inverse-kinematics
package on its own:

  1. read ``/robot_description`` + ``/joint_states`` (so it can self-check),
  2. call the ``ik_node`` typed service ``<ik-ns>/solve`` for one Cartesian task,
  3. print the solution + diagnostics (reachable / reason / errors / q),
  4. apply the SAME 30 cm Cartesian safety check as the commander (FK of the
     solved config vs the current pose of the target frame),
  5. only with ``--apply`` (and within the radius) publish ONE Float64MultiArray
     to ``/<controller>/commands`` (the forward_position_controller must be
     active). Default is print-only.

It shares ``ikt_core`` with the commander, so the solver math is identical.

Examples
--------
    # print-only (safe): solve and show what would be sent
    ros2 run ikt_inverse_kinematics solve_and_send --frame link_6 \
        --xyz 0.10 0.25 1.04 --point

    # actually drive the controller (FPC must be active), <=5 cm move
    ros2 run ikt_inverse_kinematics solve_and_send --frame link_6 \
        --xyz 0.10 0.25 1.04 --point --apply --radius 0.30
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from ikt_core.robot_model import RobotModel
from ikt_interfaces.srv import SolveIK
from ikt_interfaces.msg import IKTask


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class _Helper(Node):
    def __init__(self) -> None:
        super().__init__("solve_and_send")
        self._urdf: str = ""
        self._jp: Dict[str, float] = {}
        self.create_subscription(String, "/robot_description",
                                 self._on_urdf, _latched_qos())
        self.create_subscription(JointState, "/joint_states",
                                 self._on_js, qos_profile_sensor_data)

    def _on_urdf(self, msg: String) -> None:
        if msg.data:
            self._urdf = msg.data

    def _on_js(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self._jp[n] = float(p)

    def wait_for_inputs(self, timeout: float = 8.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._urdf and self._jp:
                return True
        return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="solve_and_send")
    ap.add_argument("--frame", required=True, help="target frame (link/tip)")
    ap.add_argument("--xyz", nargs=3, type=float, required=True,
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--quat", nargs=4, type=float, default=[1.0, 0.0, 0.0, 0.0],
                    metavar=("W", "X", "Y", "Z"))
    ap.add_argument("--point", action="store_true",
                    help="position-only (orientation free)")
    ap.add_argument("--frame-id", default="", help="TF source frame of target")
    ap.add_argument("--ik-ns", default="/ik_node", help="ik_node namespace")
    ap.add_argument("--controller", default="forward_position_controller")
    ap.add_argument("--radius", type=float, default=0.30,
                    help="max Cartesian move of --frame (m) before refusing")
    ap.add_argument("--apply", action="store_true",
                    help="actually publish to the controller (default: print)")
    args = ap.parse_args(argv)

    rclpy.init()
    node = _Helper()
    rc = 0
    try:
        if not node.wait_for_inputs():
            print("ERROR: no /robot_description or /joint_states", file=sys.stderr)
            return 1
        model = RobotModel(node._urdf)
        if not model.has_frame(args.frame):
            print("ERROR: unknown frame '%s'" % args.frame, file=sys.stderr)
            return 1
        joints = model.supporting_joints(args.frame)

        # build the typed solve request (one Cartesian task)
        cli = node.create_client(SolveIK, args.ik_ns.rstrip("/") + "/solve")
        if not cli.wait_for_service(timeout_sec=5.0):
            print("ERROR: %s/solve service unavailable (is ik_node running?)"
                  % args.ik_ns, file=sys.stderr)
            return 1
        task = IKTask()
        task.frame = args.frame
        task.target = Pose()
        task.target.position.x, task.target.position.y, task.target.position.z \
            = args.xyz
        task.target.orientation.w, task.target.orientation.x, \
            task.target.orientation.y, task.target.orientation.z = args.quat
        task.stiffness = ([1.0, 1.0, 1.0, 0.0, 0.0, 0.0] if args.point
                          else [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        task.frame_id = args.frame_id
        req = SolveIK.Request()
        req.tasks = [task]
        req.active_joints = []
        req.seed = []

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=8.0)
        resp = fut.result()
        if resp is None or not resp.ok:
            print("ERROR: solve failed: %s"
                  % (resp.message if resp else "no response"), file=sys.stderr)
            return 1
        r = resp.result
        qmap = {n: q for n, q in zip(r.joint_names, r.q)}
        print("solve: reachable=%s reason=%s pos_err=%.4f ori_err=%.4f "
              "manip=%.4f sigma_min=%.4f"
              % (r.reachable, r.reason, r.max_pos_err, r.max_ori_err,
                 r.manipulability, r.sigma_min))
        print("active joints (%s): %s" % (args.frame,
              ", ".join("%s=%.4f" % (j, qmap.get(j, float("nan")))
                        for j in joints)))

        # 30 cm Cartesian gate: FK of solved vs current config of --frame
        q_cur = model.neutral()
        for jn in model.joint_names:
            if jn in node._jp:
                q_cur[model.q_index(jn)] = node._jp[jn]
        q_tgt = q_cur.copy()
        for jn in model.joint_names:
            if jn in qmap:
                q_tgt[model.q_index(jn)] = qmap[jn]
        ee_cur, _ = model.fk(q_cur, args.frame)
        ee_tgt, _ = model.fk(q_tgt, args.frame)
        disp = float(np.linalg.norm(ee_tgt - ee_cur))
        print("cartesian move of %s = %.4f m (limit %.2f m)"
              % (args.frame, disp, args.radius))

        data = [float(qmap[j]) for j in joints]
        if not args.apply:
            print("PRINT-ONLY (use --apply to drive %s). Would send: %s"
                  % (args.controller, [round(v, 4) for v in data]))
            return 0
        if disp > args.radius:
            print("REFUSED to apply: move %.3f m exceeds radius %.2f m"
                  % (disp, args.radius), file=sys.stderr)
            return 2
        pub = node.create_publisher(
            Float64MultiArray, "/%s/commands" % args.controller, 10)
        # give the publisher a moment to connect, then send once
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        m = Float64MultiArray()
        m.data = data
        pub.publish(m)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        print("APPLIED: published %d-DOF command to /%s/commands"
              % (len(data), args.controller))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
