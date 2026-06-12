# ikt_core

The **robot-agnostic, ROS-free inverse-kinematics core** of the
`inverse_kinematics_toolkit`. Pure Python (only `numpy` + `pinocchio`) — it
imports no `rclpy`, so it works as an ordinary Python library *and* as a
colcon/ament package. **Advisory only:** it returns joint solutions; it never
commands a robot.

The ROS layer (a solver node, a 3D web dashboard, an RViz marker bridge) lives
in the separate `ikt_inverse_kinematics` package, which depends on this one.

## Use as a Python library

```bash
pip install .            # add .[pinocchio] for a pure-pip Pinocchio backend
```

```python
from ikt_core import IK, solve_ik, assets

# build once from a URDF file / .xacro / raw XML string, reuse for many solves
ik = IK.from_urdf_file("arm.urdf")
print(ik.joint_names, ik.link_names)

xyz, quat = ik.fk(ik.neutral(), "tool0")        # a reachable target via FK
sol = ik.solve("tool0", xyz, quat)              # full-pose solve
print(sol.reachable, sol.reason.value, sol.joint_dict())

# position-only one-liner
sol = solve_ik("arm.urdf", "tool0", [0.4, -0.2, 0.5], position_only=True)

# bundled sample URDFs — no external files needed
ik = IK.from_urdf_file(assets.sample_urdf_path("srs_7dof"))
```

`IK.solve(...)` auto-derives the moving joints for the frame, defaults the seed
to the neutral pose, normalises quaternions, and accepts `stiffness`,
`position_only`, `active_joints`, `tool_frames`, `arm_angle` and `relative`
options. `IK.solve_many([...])` solves multiple tips at once. See
[`examples/solve_from_file.py`](examples/solve_from_file.py).

## Why a "package"? (layout)

`ikt_core` is shipped as an `ament_python`/setuptools package — it has a
`package.xml`, a `setup.py`/`pyproject.toml`, and a nested `ikt_core/ikt_core/`
directory — rather than a loose folder of scripts. That is deliberate and serves
both use cases from one source:

- **ROS workspace:** the sibling ROS packages import it as `from ikt_core import
  …`. For that to resolve after `colcon build`, `ikt_core` must be installed into
  the workspace, and colcon only installs a shared Python library if it is an
  `ament_python` package — hence `package.xml`.
- **Plain Python:** the same `setup.py` / `pyproject.toml` make `pip install .`
  work with no ROS at all.

The doubled `ikt_core/ikt_core/` is the standard Python project layout: the
**outer** directory is the package *project* (build metadata, `test/`,
`examples/`); the **inner** directory is the importable module named `ikt_core`.
Either way you just write `import ikt_core`.

## Modules

```
api.py         high-level IK class + solve_ik() one-liner
ik_core.py     pure-Python weighted LM-DLS solver
robot_model.py URDF string -> Pinocchio model (frames / Jacobians / limits)
tasks.py       Task / RelativeTask / ArmAngleTask / VirtualFrame / Solution
arm_angle.py   S-R-S arm-angle psi compute/report + desired-psi task
relative.py    dual-arm relative-pose task
collision.py   capsule self-collision soft penalty
urdf_utils.py  URDF file/xacro/string loader + virtual tool-frame augmentation
assets.py      bundled sample URDFs (urdf/*.urdf)
cli.py         `ikt` console command (validate / solve / fk)
urdf/          sample URDFs: planar_3r, arm_6dof, srs_7dof, dual_arm
```

## CLI

```bash
ikt fk       --urdf arm.urdf --frame tool0
ikt solve    --urdf arm.urdf --frame tool0 --xyz 0.4 -0.2 0.5
ikt validate --urdf arm.urdf --frame tool0 --n 200
```

## Dependencies

- **Pinocchio** — kinematics backend (required). System: `apt install
  python3-pinocchio` (or `ros-<distro>-pinocchio`); pure pip: `pip install pin`
  (or `pip install .[pinocchio]`).
- `numpy`. Nothing else — no ROS.

## Build (in a colcon workspace)

```bash
colcon build --symlink-install --packages-select ikt_core
```
