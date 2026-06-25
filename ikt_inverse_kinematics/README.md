# ikt_inverse_kinematics

The **ROS 2 layer** over the [`ikt_core`](../ikt_core) inverse-kinematics
solver. `ik_node` reads a URDF from a file/string parameter or from
`/robot_description`, builds the model with `ikt_core`, and exposes a JSON (and
optional typed) solve API; an optional 3D web dashboard and an RViz marker
bridge round it out. Given a URDF it solves, for one or more chosen links, the
joint angles that place those links at requested target poses — subject to joint
limits, singularity robustness, a soft rest-posture bias, and **per-DOF task
stiffness**.

> **The solver never commands the robot.** It publishes IK *results* (a
> `JointState` solution + JSON diagnostics). A separate consumer decides whether
> and how to actuate. `~/solution` is explicitly **not** a controller command.

## Just want a Python library? Use `ikt_core`

The reusable, ROS-free solver — the `IK` class, `solve_ik`, the `ikt` CLI and the
bundled sample URDFs — lives in [`ikt_core`](../ikt_core) and needs only
`numpy` + `pinocchio`:

```python
from ikt_core import IK, solve_ik, assets
ik = IK.from_urdf_file("arm.urdf")
sol = ik.solve("tool0", [0.4, -0.2, 0.5])
```

This package adds the ROS nodes on top of that core.

## Capabilities

| # | Feature |
|---|---------|
| R1 | Pose → joints for **any frame** (intermediate links included) and **multiple tips** (dual-arm), solved simultaneously |
| R2 | **Virtual tool frames** — attach a 6-DOF offset to a link and solve on it (via `ikt_core.urdf_utils.augment_urdf`) |
| R3 | **Constraints** — hard joint limits (box), singularity damping, soft "prefer rest posture" |
| R4 | **Per-DOF stiffness** — weight each Cartesian DOF; low orientation weight ⇒ position dominates (avoids unwanted rotations) |
| R6 | **Arm-angle ψ** — report and *control* the 7-DOF elbow-swivel redundancy (S-R-S), the bridge to the FZI null-space fix |
| R7 | **Reachability verdict** — `reachable` + reason (`joint_limit` / `singular` / `task_conflict` / `max_iters`) + closest-reachable pose |
| R8 | **TF-framed targets** — accept a target in any frame; transformed to the base frame via tf2 |
| R9 | **Dual-arm relative-pose** — constrain the transform between two tips (rigidly hold one object) |
| E1 | **Task templates** — `pose` / `point` / `position_yaw` / `axis_gaze` presets over the stiffness weights |

## Architecture

The ROS-free solver core (`ik_core`, `robot_model`, `tasks`, `arm_angle`,
`relative`, `collision`, `urdf_utils`, `assets`, the `IK` API and the `ikt` CLI)
lives in the [`ikt_core`](../ikt_core) package. This package contains only the
ROS nodes:

```
ik_node.py     headless ROS node: URDF from file/string/topic; builds the model
               via ikt_core, exposes the JSON + typed solve API (advisory)
dashboard_node.py  optional 3D web UI (http.server + Three.js) — renders the
               robot from /robot_description meshes, ghosts the IK solution,
               and drives every solve function; pure client of the ROS API
marker_node.py optional RViz interactive-marker bridge
static/        index.html + app.js (Three.js viewer) + dashboard.css + vendor/
launch/        ik / dashboard / ik_with_dashboard launch files
config/        ik_defaults.yaml (solver tuning)
```

The math is one **weighted, damped, box-constrained least-squares** problem
(Levenberg–Marquardt / damped Gauss–Newton with backtracking line search). The
task term carries the per-DOF stiffness `Wt`; the soft rest-posture term is
applied in the **task null space** so it never degrades the Cartesian task.

## Dependencies

- [`ikt_core`](../ikt_core) — the solver core (and, transitively, **Pinocchio**
  + `numpy`). Pinocchio: `apt install python3-pinocchio` / `ros-<distro>-pinocchio`,
  or `pip install pin`.
- ROS: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`,
  `tf2_geometry_msgs`, `ikt_interfaces` (typed service), and `ikt_common`
  (launch-time centralized config).

## Build

```bash
colcon build --symlink-install --packages-select ikt_core ikt_inverse_kinematics
source install/setup.bash
```

## Run (headless solver)

```bash
# URDF from the live topic (after a bringup publishes /robot_description):
ros2 launch ikt_inverse_kinematics ik.launch.py
# OR provide a URDF/xacro file directly (no /robot_description needed):
ros2 launch ikt_inverse_kinematics ik.launch.py urdf_file:=/path/to/arm.urdf
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

The package is advisory; a *consumer* actuates. The easy "solve → move" path is
the **`solve_and_send`** CLI (commander **not** required):

```bash
# solve via ik_node and PRINT what would be sent (safe; default)
ros2 run ikt_inverse_kinematics solve_and_send --frame link_6 \
    --xyz 0.45 -0.03 0.68 --point

# actually drive the controller (forward_position_controller must be active),
# gated by the same 30 cm Cartesian check the commander uses:
ros2 run ikt_inverse_kinematics solve_and_send --frame link_6 \
    --xyz 0.45 -0.03 0.68 --point --apply --radius 0.30
```

It reads `/robot_description` + `/joint_states`, calls the `ik_node` typed
service `<ik-ns>/solve`, prints the solution + diagnostics, applies the 30 cm
Cartesian gate (FK of the solved config vs the current pose of `--frame`), and
only with `--apply` publishes **one** `Float64MultiArray` to
`/<controller>/commands`. Shares `ikt_core` with the commander, so the solver
math is identical. Validated with the commander stopped on the mock and on the
real Duco GCR5-910 (drove the FPC controller directly to the solved pose).

For a more thorough, multi-gate reference consumer see
[`test/closed_loop_demo.py`](test/closed_loop_demo.py): solves via `ik_node`,
checks safety limits (reachable, bounded Cartesian move, bounded per-joint step,
other arm untouched), commands the `joint_trajectory_controller`, and verifies
the EE reached the target by FK. Validated on the mock (0.8 mm) and the physical
RM75 (≈4 mm, real servo accuracy).

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
