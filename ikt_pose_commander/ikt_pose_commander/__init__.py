"""ikt_pose_commander — target pose -> IK -> robot motion.

A node that subscribes to a Cartesian target pose, solves it with the
``ikt_inverse_kinematics`` solver (in-process), and commands the arm. Unlike the
IK package (advisory only), this package ACTUALLY moves the robot, so it is
safety-gated (starts disabled; rejects unreachable / large-jump solutions;
speed-limited; holds on stale input).
"""
