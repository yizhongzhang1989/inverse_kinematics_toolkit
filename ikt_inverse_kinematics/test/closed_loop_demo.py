#!/usr/bin/env python3
"""Closed-loop IK demo: solve via ik_node, then MOVE the arm to the result.

This is the *consumer* the IK package is designed for: it asks ik_node for a
joint solution to a Cartesian target, then commands the existing
joint_trajectory_controller (JTC) to move there, and finally confirms (by FK on
the achieved joint state) that the end-effector reached the requested pose. The
IK package itself stays advisory-only; THIS script is the thing that actuates.

It is deliberately a *separate*, heavily-gated client (not part of the solver):

SAFETY GATES (the robot cannot be recovered remotely if it faults):
  * Target is the FK of a SMALL random perturbation of the CURRENT pose, so the
    Cartesian move is bounded (default <= 8 cm; hard cap --max-cart 0.10 m).
  * The solve must return reachable=true; otherwise NO motion.
  * Every solved joint delta must be <= --max-joint-step (default 0.25 rad); the
    total ||dq|| <= --max-total-step; otherwise NO motion.
  * One arm only; the other arm's joints are never included.
  * The JTC move is slow (default 10 s, smooth spline) to a single point.
  * Refuses if /joint_states is stale.

Use --execute to actually command motion; without it the script is a dry run
(solve + gate-check + report, no command published). Start with mock hardware.
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ikt_core.robot_model import RobotModel


def _latched():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class ClosedLoop(Node):
    def __init__(self, ns, jtc):
        super().__init__("ik_closed_loop_demo")
        self.urdf = None
        self.jpos = {}
        self.js_stamp = 0.0
        self.resp = None
        self.create_subscription(String, "/robot_description", self._u, _latched())
        self.create_subscription(JointState, "/joint_states", self._j, 50)
        self.req_pub = self.create_publisher(String, f"{ns}/solve_request", 10)
        self.create_subscription(String, f"{ns}/solve_response", self._r, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{jtc}/joint_trajectory", 10)

    def _u(self, m):
        self.urdf = m.data

    def _j(self, m):
        for n, p in zip(m.name, m.position):
            self.jpos[n] = float(p)
        self.js_stamp = time.monotonic()

    def _r(self, m):
        self.resp = json.loads(m.data)

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not pred():
            rclpy.spin_once(self, timeout_sec=0.05)
        return pred()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="right_arm_Link7")
    ap.add_argument("--ns", default="/ik_node")
    ap.add_argument("--jtc", default="right_arm_joint_trajectory_controller")
    ap.add_argument("--max-cart", type=float, default=0.10, help="hard cap, m")
    ap.add_argument("--cart-offset", type=float, default=0.08, help="target dist, m")
    ap.add_argument("--max-joint-step", type=float, default=0.25, help="rad/joint")
    ap.add_argument("--max-total-step", type=float, default=0.6, help="rad ||dq||")
    ap.add_argument("--move-secs", type=float, default=10.0)
    ap.add_argument("--execute", action="store_true",
                    help="actually command JTC motion (default: dry run)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    arm = args.frame.split("_Link")[0]
    active = [f"{arm}_joint{i}" for i in range(1, 8)]

    rclpy.init()
    node = ClosedLoop(args.ns, args.jtc)
    if not node.spin_until(lambda: node.urdf is not None and bool(node.jpos), 8.0):
        print("FAIL: no robot_description / joint_states"); return 2
    if time.monotonic() - node.js_stamp > 0.5:
        print("FAIL: /joint_states stale"); return 2

    model = RobotModel(node.urdf)
    q = model.neutral()
    for jn in model.joint_names:
        if jn in node.jpos:
            q[model.q_index(jn)] = node.jpos[jn]

    # Build a bounded, guaranteed-reachable target: FK of a small perturbation
    # of the current config on the active joints, scaled so the Cartesian move
    # is about --cart-offset and never exceeds --max-cart.
    rng = np.random.default_rng(args.seed)
    p_cur, _ = model.fk(q, args.frame)
    target_xyz = target_quat = None
    for _ in range(40):
        q_pert = q.copy()
        for jn in active:
            q_pert[model.q_index(jn)] += rng.uniform(-0.12, 0.12)
        xyz, quat = model.fk(q_pert, args.frame)
        d = float(np.linalg.norm(xyz - p_cur))
        if 0.02 <= d <= args.cart_offset:
            target_xyz, target_quat = xyz, quat
            break
    if target_xyz is None:
        print("FAIL: could not sample a small reachable target"); return 2
    cart_d = float(np.linalg.norm(target_xyz - p_cur))
    if cart_d > args.max_cart:
        print(f"FAIL(SAFETY): cartesian move {cart_d:.3f} m > cap {args.max_cart}")
        return 2

    req = {"id": "cl1", "tasks": [{"frame": args.frame,
           "xyz": [float(v) for v in target_xyz],
           "quat": [float(v) for v in target_quat]}],
           "active_joints": active}
    node.spin_until(lambda: node.req_pub.get_subscription_count() > 0, 5.0)
    node.resp = None
    m = String(); m.data = json.dumps(req); node.req_pub.publish(m)
    if not node.spin_until(lambda: node.resp is not None, 10.0):
        print("FAIL: no solve_response"); return 2

    r = node.resp
    print(f"target {np.round(target_xyz,4)} cart_move={cart_d:.3f} m")
    print(f"solve: reachable={r.get('reachable')} reason={r.get('reason')} "
          f"pos_err={r.get('max_pos_err'):.5f} ori_err={r.get('max_ori_err'):.5f}")
    if not (r.get("ok") and r.get("reachable")):
        print("ABORT: solution not reachable — no motion."); return 1

    # Gate: per-joint and total joint deltas on the ACTIVE arm.
    names = r["joint_names"]; qsol = dict(zip(names, r["q"]))
    deltas = {jn: qsol[jn] - node.jpos.get(jn, qsol[jn]) for jn in active}
    max_dj = max(abs(v) for v in deltas.values())
    tot = math.sqrt(sum(v * v for v in deltas.values()))
    print(f"joint deltas: max={max_dj:.3f} rad, ||dq||={tot:.3f} rad")
    if max_dj > args.max_joint_step:
        print(f"ABORT(SAFETY): max joint step {max_dj:.3f} > {args.max_joint_step}")
        return 1
    if tot > args.max_total_step:
        print(f"ABORT(SAFETY): total step {tot:.3f} > {args.max_total_step}")
        return 1
    # other-arm joints must be untouched by the solve (small tolerance absorbs
    # live /joint_states jitter between the seed snapshot and this check; the
    # gate is to catch gross cross-arm coupling, not microradian noise).
    other = [jn for jn in names if not jn.startswith(arm)]
    for jn in other:
        if abs(qsol[jn] - node.jpos.get(jn, qsol[jn])) > 1e-3:
            print(f"ABORT(SAFETY): solve moved non-active joint {jn}"); return 1

    if not args.execute:
        print("DRY RUN ok (all safety gates passed). Re-run with --execute to move.")
        node.destroy_node(); rclpy.shutdown(); return 0

    # Command the JTC to the solved active-joint angles (slow, single point).
    traj = JointTrajectory()
    traj.joint_names = active
    pt = JointTrajectoryPoint()
    pt.positions = [float(qsol[jn]) for jn in active]
    pt.velocities = [0.0] * len(active)
    pt.time_from_start.sec = int(args.move_secs)
    traj.points = [pt]
    for _ in range(3):
        node.traj_pub.publish(traj); rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)
    print(f"commanded JTC move over {args.move_secs}s; waiting...")
    node.spin_until(lambda: False, args.move_secs + 2.0)

    # Verify by FK on the achieved joint state.
    q2 = model.neutral()
    for jn in model.joint_names:
        if jn in node.jpos:
            q2[model.q_index(jn)] = node.jpos[jn]
    p_ach, _ = model.fk(q2, args.frame)
    reached = float(np.linalg.norm(p_ach - target_xyz))
    print(f"achieved {args.frame} xyz={np.round(p_ach,4)}  "
          f"target={np.round(target_xyz,4)}  err={reached*1000:.1f} mm")
    ok = reached <= 0.01
    print("CLOSED-LOOP", "PASS" if ok else "INCOMPLETE")
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
