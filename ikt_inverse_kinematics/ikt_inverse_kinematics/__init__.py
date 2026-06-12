"""ikt_inverse_kinematics — ROS 2 layer over the ``ikt_core`` IK solver.

This package wraps the ROS-free :mod:`ikt_core` solver in ROS nodes:

  * ``ik_node``        — headless solver node; reads the URDF from a file/string
                         parameter or ``/robot_description`` + ``/joint_states``,
                         exposes a JSON (and optional typed) solve API. Advisory
                         only — it publishes IK *results*, never robot commands.
  * ``dashboard_node`` — optional 3D web dashboard (Three.js) that renders the
                         robot and drives every solve function.
  * ``marker_node``    — optional RViz interactive-marker bridge.

For the reusable, ROS-free solver (the :class:`IK` class, ``solve_ik``, the
``ikt`` CLI and the bundled sample URDFs) import :mod:`ikt_core` instead.
"""
