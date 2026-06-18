# inverse_kinematics_toolkit

A robot-agnostic, **advisory-only** inverse-kinematics toolkit. Its core is a
pure-Python (ROS-free) IK solver you can `pip install` and `import`; on top of
that it provides ROS 2 nodes (Humble, `ament_python` + one `ament_cmake`
interface package): a headless solver, a safety-gated pose commander, typed
service interfaces, and 3D web dashboards. Every package builds its kinematic
model from a URDF (a file/string or the live `/robot_description`), so nothing
here names a specific robot.

## Packages

| Package | Type | Purpose |
|---|---|---|
| [`ikt_core`](ikt_core) | ament_python | **ROS-free IK core + Python library.** Pinocchio-backed solver, the high-level `IK` class + `solve_ik`, the `ikt` CLI, and bundled sample URDFs. Only `numpy` + `pinocchio`; imports no rclpy. |
| [`ikt_common`](ikt_common) | ament_python | Shared launch-time utilities: centralized config loader (`config_manager`), URDF helpers (`urdf_loader`), workspace helpers, and the packaged `toolkit_defaults.yaml`. |
| [`ikt_interfaces`](ikt_interfaces) | ament_cmake | Typed messages/services: `IKTask`, `IKResult`, `SolveIK`. |
| [`ikt_inverse_kinematics`](ikt_inverse_kinematics) | ament_python | ROS 2 layer over `ikt_core`: advisory solver node (URDF from file/string/topic, JSON + typed API), optional RViz marker, and a 3D Three.js dashboard. |
| [`ikt_pose_commander`](ikt_pose_commander) | ament_python | Target pose → IK → robot motion, safety-gated (reachability / jump / speed / staleness), with an optional dashboard. Solves in-process via `ikt_core`. |

Dependency order: `ikt_core`, `ikt_common`, `ikt_interfaces` →
`ikt_inverse_kinematics` → `ikt_pose_commander`.

## Build

This repo is normally consumed as a git submodule of a larger workspace. From
the workspace root:

```bash
colcon build --packages-select ikt_core ikt_common ikt_interfaces \
  ikt_inverse_kinematics ikt_pose_commander --symlink-install
source install/setup.bash
```

It also builds **standalone**. Drop the repo into a ROS 2 workspace `src/` and
`colcon build`; the only external runtime dependency beyond ROS 2 is
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pin`) and NumPy.

## Use as a plain Python library (no ROS)

`ikt_core` works as an ordinary Python package — load a URDF and solve IK with a
class or a one-liner, needing only `numpy` + `pinocchio`:

```bash
cd ikt_core
pip install .              # add .[pinocchio] for a pure-pip Pinocchio backend
```

```python
from ikt_core import IK, solve_ik, assets

# 1) Build a solver from a URDF file, a .xacro, or a raw URDF/XML string.
ik = IK.from_urdf_file("arm.urdf")
print(ik.joint_names)                       # movable joints, in q order
print(ik.link_names)                        # frames you can target

# 2) Pick a reachable target (here: FK of a non-trivial config).
xyz, quat = ik.fk({"joint2": -0.6, "joint3": 0.8}, "tool0")

# 3) Solve. active_joints/seed are auto-derived; quats are normalised.
sol = ik.solve("tool0", xyz, quat)
print(sol.reachable, sol.reason.value)      # True 'ok'
print(sol.joint_dict())                     # {joint: angle} for the whole model
print(sol.q_active())                       # just the joints that moved tool0

# Position-only, custom stiffness, an explicit seed, a virtual tool frame:
sol = ik.solve("tool0", [0.4, -0.2, 0.5], position_only=True,
               seed=ik.neutral(),
               tool_frames=[{"name": "tcp", "parent": "tool0",
                             "xyz": [0, 0, 0.05]}])

# True one-liner; works on the bundled sample URDFs (no external files needed).
sol = solve_ik(assets.sample_urdf_path("srs_7dof"), "tool0", [0.3, 0.1, 0.6])
```

Bundled sample URDFs: `planar_3r`, `arm_6dof`, `srs_7dof`, `dual_arm`
(via `ikt_core.assets`). Runnable example:
[`ikt_core/examples/solve_from_file.py`](ikt_core/examples/solve_from_file.py).

> **Why is `ikt_core` a "package" (with `package.xml` and a nested
> `ikt_core/ikt_core/`) and not just a folder of scripts?** Two reasons. (1) The
> ROS nodes here import it with `from ikt_core import …`; for that to resolve
> after `colcon build`, `ikt_core` must be *installed* into the workspace, and
> the only way colcon installs a shared Python library is as an `ament_python`
> package (hence `package.xml`). (2) The same `setup.py` / `pyproject.toml` make
> `pip install .` work for the no-ROS use case above. The doubled
> `ikt_core/ikt_core/` is the standard layout: the outer directory is the
> package *project* (build metadata), the inner directory is the importable
> module named `ikt_core`. So the import name you use is just `ikt_core`.

## Quick start (ROS)

```bash
# headless solver + 3D dashboard (http://localhost:8160), URDF from /robot_description
ros2 launch ikt_inverse_kinematics ik_with_dashboard.launch.py

# OR provide a URDF/xacro file directly (no /robot_description needed)
ros2 launch ikt_inverse_kinematics ik.launch.py urdf_file:=/path/to/arm.urdf

# pose commander (+ dashboard on :8180) — stays DISABLED until ~/enable
ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180
```

## Configuration

Launch-arg defaults are centralized in `ikt_common/config/toolkit_defaults.yaml`
(sections `ikt_inverse_kinematics:` and `ikt_pose_commander:`). Override per
workspace via `config/robot_config.yaml` or the `ROBOT_CONFIG_PATH` environment
variable; CLI `key:=value` overrides win over everything. See each package's
`README.md` for details.

## Apply to a new robot

Nothing in the toolkit names a specific robot — every package builds its model
from your URDF and adapts at runtime. There is **no per-robot code to write**.

**Python library (`ikt_core`) — works on any URDF immediately:**

```python
from ikt_core import IK
ik = IK.from_urdf_file("my_robot.urdf")      # or .xacro, or a raw URDF string
print(ik.joint_names, ik.link_names)         # discover frames/joints to target
sol = ik.solve("my_tip_link", [x, y, z], [qw, qx, qy, qz])
```

Frame and joint names are whatever your URDF uses — there is no naming
convention to follow. `solve(...)` auto-selects the joints on the kinematic path
to the chosen frame; pass `active_joints=` to override, or `active_joints=None`
for all joints.

**ROS (solver + dashboard + motion) — checklist:**

1. **URDF / state.** Have a bringup publish `/robot_description` and
   `/joint_states` (any robot's `robot_state_publisher` + driver). Or skip the
   topic and pass the model directly:
   `ros2 launch ikt_inverse_kinematics ik.launch.py urdf_file:=/path/my_robot.urdf`.
2. **Controllers (only for motion).** Load a `JointTrajectoryController` (for
   `jtc` mode) and/or a forward **position** controller — a
   `forward_command_controller/ForwardCommandController` **or** a
   `position_controllers/JointGroupPositionController` (for `fpc` mode) — for your
   joints in `/controller_manager`. The commander **auto-discovers** them by
   matching `<joint>/position` command interfaces and prefers an **active**
   controller; names don't matter (see
   [`ikt_pose_commander`](ikt_pose_commander/README.md)).
3. **Solve / visualize:** `ros2 launch ikt_inverse_kinematics ik_with_dashboard.launch.py`
   → open http://localhost:8160, pick any link, Solve. (Advisory; never moves the robot.)
4. **Move:** `ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180`
   → open http://localhost:8180, **Configure** the link to control (joints +
   controllers auto-derive), then **Snap robot** (one JTC move) or **Track robot**
   (drag the 3D gizmo — live FPC). FPC streaming is smoothed by a fixed-rate
   (200 Hz) acceleration-limited loop. Starts **disabled**; every move is gated
   by a Cartesian envelope + jump/speed limits.
5. **Optional — arm-angle (7-DOF S-R-S) redundancy.** Only this feature needs
   per-robot frame names. Set `srs_chain_names` + `srs_chains.<name>.{shoulder,
   elbow,wrist}` in your `robot_config.yaml` (template in
   [`ikt_inverse_kinematics/config/ik_defaults.yaml`](ikt_inverse_kinematics/config/ik_defaults.yaml)).
   Everything else works without it.

**Multiple arms.** Run **one `ikt_pose_commander` instance per arm**, each with a
distinct `instance_name:=<arm>` (node/namespace suffix) and `dashboard_port:=`,
configured to that arm's tip link. All instances share the one
`/robot_description` + `/joint_states`; controllers auto-derive per arm, so each
arm needs its **own** controller set. See
[`ikt_pose_commander` → Multiple arms](ikt_pose_commander/README.md#multiple-arms).

**Requirements:** the URDF needs movable (revolute/prismatic) joints to the link
you target; continuous joints are treated as unlimited. Mesh-free URDFs are
fine (meshes only affect the dashboard's 3D rendering, not the solve). The
dashboards default the operated link to a likely tip (a `*Link7` / `tool` /
`tcp` / `gripper` / `flange` link, else the last link) and group active joints
by link-name prefix — both are just UI defaults you can override in the dropdown.

## Safety

`ikt_inverse_kinematics` is **advisory only**: it publishes IK *results*, never
controller commands. Actuation happens solely through `ikt_pose_commander`,
which starts disabled and enforces a **Cartesian motion envelope** (a radius
around the pose captured at enable), per-step jump, joint-speed and staleness
gates before any motion; its FPC output is **acceleration-limited at a fixed rate**
(default 200 Hz) for smooth, jerk-free streaming.