"""Console CLI for ikt_inverse_kinematics.

Two offline subcommands (no ROS required) plus a thin online solve:

  ikt validate --urdf <file> [--frame F] [--n 200]
      FK->IK round-trip success-rate report on a URDF (the §10 harness).
  ikt solve --urdf <file> --frame F --xyz x y z [--quat w x y z]
      one-shot solve from the URDF's neutral pose; prints the Solution.
  ikt fk --urdf <file> --frame F [--q ...]
      forward kinematics of a frame (handy to capture a reachable target).

The URDF can be a file path or '-' to read stdin (e.g. piped from xacro).
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

import numpy as np


def _read_urdf(path: str) -> str:
    if path == "-":
        return sys.stdin.read()
    with open(path, "r") as fh:
        return fh.read()


def _model(urdf_path: str):
    from .robot_model import RobotModel
    return RobotModel(_read_urdf(urdf_path))


def _active_for_frame(model, frame: str) -> Optional[List[str]]:
    prefix = frame.split("_Link")[0]
    js = [j for j in model.joint_names if j.startswith(prefix)]
    return js or None


def cmd_fk(args) -> int:
    model = _model(args.urdf)
    q = model.neutral()
    if args.q:
        vals = [float(x) for x in args.q]
        active = _active_for_frame(model, args.frame) or model.joint_names
        for jn, v in zip(active, vals):
            q[model.q_index(jn)] = v
    xyz, quat = model.fk(q, args.frame)
    print(f"frame {args.frame}")
    print(f"xyz  = [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
    print(f"quat = [{quat[0]:.5f}, {quat[1]:.5f}, {quat[2]:.5f}, {quat[3]:.5f}]")
    return 0


def cmd_solve(args) -> int:
    from . import ik_core
    from .tasks import Task
    model = _model(args.urdf)
    seed = model.neutral()
    quat = tuple(args.quat) if args.quat else (1.0, 0.0, 0.0, 0.0)
    stiff = tuple(args.stiffness) if args.stiffness else (1, 1, 1, 1, 1, 1)
    task = Task(args.frame, tuple(args.xyz), quat, stiff)
    active = _active_for_frame(model, args.frame)
    sol = ik_core.solve(model, seed, [task],
                        params=ik_core.SolveParams(max_iters=args.max_iters),
                        active_joints=active)
    print(f"reachable={sol.reachable} reason={sol.reason.value} iters={sol.iters}")
    print(f"pos_err={sol.max_pos_err()*1000:.3f} mm  ori_err={sol.max_ori_err():.5f} rad")
    print(f"manipulability={sol.manipulability:.5f}  sigma_min={sol.sigma_min:.5f}")
    if sol.blocking_joints:
        print(f"blocking_joints={sol.blocking_joints}")
    print("q = [" + ", ".join(f"{sol.q[model.q_index(j)]:.4f}"
                              for j in (active or model.joint_names)) + "]")
    return 0 if sol.reachable else 1


def cmd_validate(args) -> int:
    from . import ik_core
    from .tasks import Task
    model = _model(args.urdf)
    frame = args.frame
    active = _active_for_frame(model, frame)
    lo, hi = model.joint_limits()
    rng = np.random.default_rng(args.seed)
    params = ik_core.SolveParams(max_iters=args.max_iters)

    def rand_q():
        q = model.neutral()
        for jn in (active or model.joint_names):
            i = model.q_index(jn)
            a, b = lo[i], hi[i]
            if not (np.isfinite(a) and np.isfinite(b)):
                a, b = -np.pi, np.pi
            q[i] = 0.5 * (a + b) + 0.5 * (b - a) * args.range * rng.uniform(-1, 1)
        return q

    ok = 0
    pos_errs = []
    for _ in range(args.n):
        q_true = rand_q()
        p_t, quat_t = model.fk(q_true, frame)
        # Seed from a perturbation of the true solution: this matches the
        # intended use (IK seeded from the current/previous pose -> minimal
        # change) and isolates solver quality from random-target reachability.
        q_seed = q_true.copy()
        for jn in (active or model.joint_names):
            q_seed[model.q_index(jn)] += rng.uniform(-args.seed_pert, args.seed_pert)
        q_seed = np.clip(q_seed, lo, hi)
        sol = ik_core.solve(model, q_seed,
                            [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                            params=params, active_joints=active)
        if sol.reachable and sol.max_pos_err() <= 1.5e-3:
            ok += 1
        pos_errs.append(sol.max_pos_err())
    rate = ok / max(1, args.n)
    pe = np.asarray(pos_errs)
    print(f"FK->IK round-trip on {frame}: {ok}/{args.n} = {rate:.1%}")
    print(f"pos_err mm: p50={np.percentile(pe,50)*1000:.3f} "
          f"p95={np.percentile(pe,95)*1000:.3f} max={pe.max()*1000:.3f}")
    gate = 0.95
    print(f"GATE {'PASS' if rate >= gate else 'FAIL'} (>= {gate:.0%})")
    return 0 if rate >= gate else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ikt",
                                 description="ikt_inverse_kinematics CLI (advisory only)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    pf = sub.add_parser("fk", help="forward kinematics of a frame")
    pf.add_argument("--urdf", required=True)
    pf.add_argument("--frame", required=True)
    pf.add_argument("--q", nargs="*", help="active-joint angles (rad)")
    pf.set_defaults(func=cmd_fk)

    ps = sub.add_parser("solve", help="one-shot IK solve from neutral")
    ps.add_argument("--urdf", required=True)
    ps.add_argument("--frame", required=True)
    ps.add_argument("--xyz", nargs=3, type=float, required=True)
    ps.add_argument("--quat", nargs=4, type=float)
    ps.add_argument("--stiffness", nargs=6, type=float)
    ps.add_argument("--max-iters", type=int, default=200)
    ps.set_defaults(func=cmd_solve)

    pv = sub.add_parser("validate", help="FK->IK round-trip success rate")
    pv.add_argument("--urdf", required=True)
    pv.add_argument("--frame", required=True,
                    help="frame to validate IK on (any link in the URDF)")
    pv.add_argument("--n", type=int, default=200)
    pv.add_argument("--seed", type=int, default=0)
    pv.add_argument("--range", type=float, default=0.7,
                    help="fraction of joint range to sample targets within")
    pv.add_argument("--seed-pert", type=float, default=0.4,
                    help="rad: IK seed = q_true +/- this (minimal-change use case)")
    pv.add_argument("--max-iters", type=int, default=200)
    pv.set_defaults(func=cmd_validate)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
