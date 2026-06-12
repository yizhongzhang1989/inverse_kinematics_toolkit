# ikt_inverse_kinematics

A **robot-agnostic, advisory-only inverse-kinematics service** for the
`cartesian_controllers_toolkit`. Given a URDF (read live from
`/robot_description`) it solves, for one or more chosen links, the joint angles
that place those links at requested target poses — subject to joint limits,
singularity robustness, a soft rest-posture bias, and **per-DOF task stiffness**.

> **The solver never commands the robot.** It publishes IK *results* (a
> `JointState` solution + JSON diagnostics). A separate consumer decides whether
> and how to actuate. `~/solution` is explicitly **not** a controller command.

## Capabilities

| # | Feature |
|---|---------|
| R1 | Pose → joints for **any frame** (intermediate links included) and **multiple tips** (dual-arm), solved simultaneously |
| R2 | **Virtual tool frames** — attach a 6-DOF offset to a link and solve on it (via `ikt_common.urdf_loader.augment_urdf`) |
| R3 | **Constraints** — hard joint limits (box), singularity damping, soft "prefer rest posture" |
| R4 | **Per-DOF stiffness** — weight each Cartesian DOF; low orientation weight ⇒ position dominates (avoids unwanted rotations) |
| R6 | **Arm-angle ψ** — report and *control* the 7-DOF elbow-swivel redundancy (S-R-S), the bridge to the FZI null-space fix |
| R7 | **Reachability verdict** — `reachable` + reason (`joint_limit` / `singular` / `task_conflict` / `max_iters`) + closest-reachable pose |
| R8 | **TF-framed targets** — accept a target in any frame; transformed to the base frame via tf2 |
| R9 | **Dual-arm relative-pose** — constrain the transform between two tips (rigidly hold one object) |
| E1 | **Task templates** — `pose` / `point` / `position_yaw` / `axis_gaze` presets over the stiffness weights |

## Architecture

```
ik_core.py     pure-Python weighted LM-DLS solver (no rclpy) — unit-tested offline
robot_model.py URDF string -> Pinocchio model (frames / Jacobians / limits)
tasks.py       Task / RelativeTask / ArmAngleTask / VirtualFrame / Solution
arm_angle.py   S-R-S arm-angle psi compute/report + desired-psi task (R6)
relative.py    dual-arm relative-pose task (R9)
ik_node.py     headless ROS node: reads URDF + joint states, JSON solve API (advisory)
dashboard_node.py  optional 3D web UI (http.server + Three.js) — renders the
               robot from /robot_description meshes, ghosts the IK solution,
               and drives every solve function; pure client of the ROS API
static/        index.html + app.js (Three.js viewer) + dashboard.css + vendor/
```

The math is one **weighted, damped, box-constrained least-squares** problem
(Levenberg–Marquardt / damped Gauss–Newton with backtracking line search). The
task term carries the per-DOF stiffness `Wt`; the soft rest-posture term is
applied in the **task null space** so it never degrades the Cartesian task.

## Dependencies

- **Pinocchio** (`python3-pinocchio`) — kinematics backend (required).
- `numpy`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`,
  `ikt_common`.

## Build

```bash
colcon build --symlink-install --packages-select ikt_inverse_kinematics
source install/setup.bash
```

## Run (headless solver)

```bash
# after a robot (or mock) bringup is publishing /robot_description + /joint_states
ros2 launch ikt_inverse_kinematics ik.launch.py
# with the 3D web dashboard (http://localhost:8160):
ros2 launch ikt_inverse_kinematics ik_with_dashboard.launch.py
```

## 3D dashboard

The dashboard (`dashboard_node`, port 8160) renders the live robot in 3D and is
a visual test-bench for the whole package. It is **advisory-only** — it never
commands the robot; every solve is delegated to `ik_node` over its ROS API.

It builds its own Pinocchio model (for per-link FK) and serves the robot's STL
meshes (resolving `package://` via the ament index), mirroring `src/rm_dashboard`.
The browser (Three.js + OrbitControls + STLLoader) renders:

* the **current** robot at the live `/joint_states` configuration (solid);
* a translucent **ghost** of the last IK solution;
* a **target-pose marker** (sphere + triad) and an axis triad at the world origin.

The view auto-frames the robot on load; **fit view** (top-right) re-centres it
at any time, and the canvas tracks the window/layout on resize.

Controls exercise every node function: pick any link/frame, **Capture current**
pose, edit target xyz + rpy, per-DOF **stiffness** sliders + presets
(pose / position-only / pos+yaw), **active-joint** group masking (auto-derived
per-arm), **virtual tool frame** (R2) — naming a tool frame makes it selectable
as the operated frame and **Capture current** reports its pose — **arm-angle ψ**
(R6, needs `srs_chains` on `ik_node`, and the solved ψ is shown in the result),
and the dual-arm **relative-pose** constraint (R9). The result panel shows the
reachability verdict, reason, position/orientation residual, iterations,
manipulability, σ_min and Δq.

Backend HTTP API: `GET /api/state` (model + per-link transforms + visuals + ik
status), `GET /mesh?pkg=&path=`, `POST /api/fk` (capture a frame's pose),
`POST /api/solve` (relays to `ik_node`, returns the solution + a ghost
`solution_link_tf`).

### Solve over the JSON API

```bash
ros2 topic pub --once /ik_node/solve_request std_msgs/String \
  '{data: "{\"id\":\"a\",\"tasks\":[{\"frame\":\"right_arm_Link7\",\"xyz\":[0.4,-0.2,0.9],\"quat\":[1,0,0,0]}],\"active_joints\":[\"right_arm_joint1\",\"right_arm_joint2\",\"right_arm_joint3\",\"right_arm_joint4\",\"right_arm_joint5\",\"right_arm_joint6\",\"right_arm_joint7\"]}"}'
ros2 topic echo /ik_node/solve_response --once
```

The solution is also published on `~/solution` (`sensor_msgs/JointState`) and the
live diagnostics (reachability, ψ, manipulability, limit-proximity) on
`~/status` (JSON string).

## CLI (offline, no ROS)

```bash
xacro src/robot_description/urdf/robot.urdf.xacro use_mock_hardware:=true > /tmp/r.urdf
ikt validate --urdf /tmp/r.urdf --frame right_arm_Link7 --n 200   # round-trip gate
ikt fk       --urdf /tmp/r.urdf --frame right_arm_Link7
ikt solve    --urdf /tmp/r.urdf --frame right_arm_Link7 --xyz 0.4 -0.2 0.9
```

## Using the result to move the robot

The package is advisory; a *consumer* actuates. See
[`test/closed_loop_demo.py`](test/closed_loop_demo.py) for a heavily-gated
reference consumer that solves via `ik_node`, checks safety limits (reachable,
bounded Cartesian move, bounded per-joint step, other arm untouched), then
commands the existing `joint_trajectory_controller` and verifies the
end-effector reached the target by FK. Validated on the mock robot (0.8 mm) and
on the physical RM75 (≈4 mm, real servo accuracy).

## Tests

```bash
# offline unit tests (no ROS); the env's anyio pytest plugin must be disabled:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  python_3rdlib/cartesian_controllers_toolkit/ikt_inverse_kinematics/test -q
```

Covers FK + analytic Jacobian, the FK→IK round-trip gate (≥95%), dual-arm,
tool-frame, joint-limit and singularity handling, position-only stiffness,
arm-angle ψ (report + control), and the dual-arm relative-pose constraint.

## Status / roadmap

Implemented: R1–R4, R6–R9, E1 (task templates), headless node, dashboard, CLI.
Planned (design hooks in place): E2 soft self-collision penalty, E3 streaming
continuity, E4 RViz interactive marker, a typed `ikt_interfaces` service
package (Phase 6), and the F-tier niceties (batch solve, mimic joints, named
pose library).
