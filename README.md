# inverse_kinematics_toolkit

A robot-agnostic, **advisory-only** inverse-kinematics toolkit for ROS 2
(Humble, `ament_python` + one `ament_cmake` interface package). It provides a
headless IK solver, a safety-gated pose commander, typed service interfaces, and
3D web dashboards. Every package builds its kinematic model from the live
`/robot_description`, so nothing here names a specific robot.

## Packages

| Package | Type | Purpose |
|---|---|---|
| [`ikt_common`](ikt_common) | ament_python | Shared utilities: centralized config loader (`config_manager`), URDF helpers (`urdf_loader`), workspace helpers, and the packaged `toolkit_defaults.yaml`. |
| [`ikt_interfaces`](ikt_interfaces) | ament_cmake | Typed messages/services: `IKTask`, `IKResult`, `SolveIK`. |
| [`ikt_inverse_kinematics`](ikt_inverse_kinematics) | ament_python | Pinocchio-backed IK solver node (advisory only — never commands the robot), JSON + typed API, CLI (`ikt`), optional RViz marker, and a 3D Three.js dashboard. |
| [`ikt_pose_commander`](ikt_pose_commander) | ament_python | Target pose → IK → robot motion, safety-gated (reachability / jump / speed / staleness), with an optional dashboard. Depends on `ikt_inverse_kinematics`. |

Dependency order: `ikt_common`, `ikt_interfaces` → `ikt_inverse_kinematics` →
`ikt_pose_commander`.

## Build

This repo is normally consumed as a git submodule of a larger workspace. From
the workspace root:

```bash
colcon build --packages-select ikt_common ikt_interfaces \
  ikt_inverse_kinematics ikt_pose_commander --symlink-install
source install/setup.bash
```

It also builds **standalone**. Drop the repo into a ROS 2 workspace `src/` and
`colcon build`; the only external runtime dependency beyond ROS 2 is
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pin`) and NumPy.

## Quick start

```bash
# headless solver + 3D dashboard (http://localhost:8160)
ros2 launch ikt_inverse_kinematics ik_with_dashboard.launch.py

# pose commander (+ dashboard on :8180) — stays DISABLED until ~/enable
ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180
```

## Configuration

Launch-arg defaults are centralized in `ikt_common/config/toolkit_defaults.yaml`
(sections `ikt_inverse_kinematics:` and `ikt_pose_commander:`). Override per
workspace via `config/robot_config.yaml` or the `ROBOT_CONFIG_PATH` environment
variable; CLI `key:=value` overrides win over everything. See each package's
`README.md` for details.

## Safety

`ikt_inverse_kinematics` is **advisory only**: it publishes IK *results*, never
controller commands. Actuation happens solely through `ikt_pose_commander`,
which starts disabled and enforces reachability, per-step jump, joint-speed and
staleness gates before any motion.