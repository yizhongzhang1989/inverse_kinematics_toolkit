"""ikt_inverse_kinematics — robot-agnostic, advisory-only inverse kinematics.

Public layers (see IMPLEMENTATION_PLAN.md):
  * ``ik_core``    — pure-Python weighted LM-DLS solver (no rclpy).
  * ``robot_model``— URDF string -> Pinocchio kinematic model (frames/Jacobians).
  * ``tasks``      — Task / VirtualFrame / RelativeTask / Solution dataclasses.
  * ``arm_angle``  — S-R-S arm-angle (psi) compute/report + desired-psi task.
  * ``ik_node``    — headless ROS node wrapping the solver (advisory only).

The solver NEVER commands the robot; it publishes IK *results* only.
"""

__all__ = [
    "ik_core",
    "robot_model",
    "tasks",
    "arm_angle",
]
