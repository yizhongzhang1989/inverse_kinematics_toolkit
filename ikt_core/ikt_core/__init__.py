"""ikt_core — robot-agnostic, ROS-free inverse-kinematics core.

The reusable, ``rclpy``-free heart of the inverse_kinematics_toolkit. Load a
URDF and solve IK via the high-level :class:`IK` class or the :func:`solve_ik`
one-liner::

    from ikt_core import IK
    ik = IK.from_urdf_file("arm.urdf")
    sol = ik.solve("tool0", xyz=[0.4, -0.2, 0.5])

The ROS layer (a solver node that reads the URDF from a file/string parameter or
``/robot_description``, a 3D web dashboard, an RViz marker bridge) lives in the
separate ``ikt_inverse_kinematics`` package, which depends on this one. The
solver is advisory only — it returns joint solutions; it never commands a robot.

Public layers:
  * ``api``        — high-level :class:`IK` facade + :func:`solve_ik`.
  * ``ik_core``    — pure-Python weighted LM-DLS solver.
  * ``robot_model``— URDF string -> Pinocchio kinematic model (frames/Jacobians).
  * ``tasks``      — Task / VirtualFrame / RelativeTask / Solution dataclasses.
  * ``arm_angle``  — S-R-S arm-angle (psi) compute/report + desired-psi task.
  * ``relative``   — dual-arm relative-pose task.
  * ``collision``  — capsule self-collision soft penalty.
  * ``urdf_utils`` — URDF file/xacro/string loader + tool-frame augmentation.
  * ``assets``     — bundled sample URDFs for examples + tests.
  * ``cli``        — the ``ikt`` console command (validate / solve / fk).

Only ``numpy`` + ``pinocchio`` are required; importing this package pulls no ROS.
"""

from . import arm_angle, assets, ik_core, robot_model, tasks, urdf_utils
from .api import IK, solve_ik
from .ik_core import SolveParams, solve
from .robot_model import RobotModel
from .tasks import Reason, Solution, Task
from .urdf_utils import augment_urdf, read_urdf, run_xacro

__all__ = [
    # high-level API
    "IK",
    "solve_ik",
    # core
    "RobotModel",
    "Task",
    "Solution",
    "Reason",
    "SolveParams",
    "solve",
    # urdf helpers
    "read_urdf",
    "run_xacro",
    "augment_urdf",
    # submodules
    "api",
    "ik_core",
    "robot_model",
    "tasks",
    "arm_angle",
    "urdf_utils",
    "assets",
]


