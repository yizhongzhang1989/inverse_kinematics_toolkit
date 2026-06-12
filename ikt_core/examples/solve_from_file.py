#!/usr/bin/env python3
"""Example: solve inverse kinematics from a URDF file, as a plain Python script.

No ROS required — just ``numpy`` + ``pinocchio`` + this package. Run it with a
bundled sample URDF or your own::

    python3 solve_from_file.py                       # uses bundled arm_6dof
    python3 solve_from_file.py /path/to/your.urdf tool0

It demonstrates the high-level :class:`IK` facade: build once, capture a
reachable target via FK, then solve and inspect the result.
"""

import sys

from ikt_core import IK, assets


def main() -> int:
    # 1) Choose a URDF: a CLI path, or a bundled sample.
    if len(sys.argv) > 1:
        urdf = sys.argv[1]
        frame = sys.argv[2] if len(sys.argv) > 2 else None
    else:
        urdf = assets.sample_urdf_path("arm_6dof")
        frame = "tool0"
        print(f"(no URDF given; using bundled sample: {urdf})")

    # 2) Build the solver once (reuse it for many solves).
    ik = IK.from_urdf_file(urdf)
    print("joints:", ik.joint_names)
    print("links :", ik.link_names)
    if frame is None:
        frame = ik.link_names[-1]
    print("operating on frame:", frame)

    # 3) Pick a reachable target: FK of a non-trivial configuration.
    seed = ik.neutral()
    js = ik.supporting_joints(frame)
    bend = dict(zip(js, [0.3, -0.6, 0.8, 0.2, 0.5, -0.4, 0.0]))
    xyz, quat = ik.fk(bend, frame)
    print(f"target xyz = {xyz.round(4).tolist()}")

    # 4) Solve full pose (seed from neutral = a real, non-trivial solve).
    sol = ik.solve(frame, xyz, quat, seed=seed)
    print(f"\nreachable = {sol.reachable}  reason = {sol.reason.value}")
    print(f"pos_err   = {sol.max_pos_err() * 1000:.3f} mm")
    print(f"ori_err   = {sol.max_ori_err():.5f} rad   iters = {sol.iters}")
    print("solution (active joints):")
    for jn, v in zip(sol.active_joints or ik.joint_names, sol.q_active()):
        print(f"  {jn:>14} = {v:+.4f} rad")

    # 5) A position-only one-liner, for contrast.
    from ikt_core import solve_ik
    sol2 = solve_ik(urdf, frame, [xyz[0], xyz[1], xyz[2]], position_only=True)
    print(f"\nposition-only one-liner: reachable={sol2.reachable} "
          f"pos_err={sol2.max_pos_err() * 1000:.3f} mm")
    return 0 if sol.reachable else 1


if __name__ == "__main__":
    raise SystemExit(main())
