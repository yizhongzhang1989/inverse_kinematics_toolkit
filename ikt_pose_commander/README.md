# ikt_pose_commander

Take a Cartesian **target pose** on a topic, solve IK in-process with
[`ikt_inverse_kinematics`](../ikt_inverse_kinematics/README.md), and **move the
arm** — safely. Robot-agnostic: it reads `/robot_description` + `/joint_states`,
you name the link, and it auto-derives the joints and controller.

```
~/target_pose ─▶ IK solve ─▶ safety gates ─▶ fpc stream  (or jtc trajectory) ─▶ robot
```

## Safety (read once)

- Starts **DISABLED** — nothing moves until `~/enable`.
- Speed/accel limited; holds on stale `/joint_states`; **Disable** returns to the
  trajectory controller and holds.
- `~/return_to_start` drives back to the pose captured at enable.

## Build

```bash
colcon build --packages-select ikt_inverse_kinematics ikt_pose_commander --symlink-install
source install/setup.bash
```

## Quick start

```bash
# 1) launch (reads /robot_description; starts UNCONFIGURED + disabled)
ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180

# 2) pick the link to control (joints + controller auto-derive)
ros2 topic pub --once /ikt_pose_commander/configure std_msgs/msg/String \
    '{data: "{\"controlled_frame\": \"link_6\", \"command_mode\": \"fpc\"}"}'

# 3) enable, then snap the goal to where the arm is (no jump)
ros2 service call /ikt_pose_commander/enable std_srvs/srv/Trigger
ros2 service call /ikt_pose_commander/snap_target std_srvs/srv/Trigger

# 4) send a target pose (base_link frame)
ros2 topic pub --once /ikt_pose_commander/target_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: base_link}, pose: {position: {x: 0.45, y: 0.0, z: 0.6},
      orientation: {w: 1.0}}}'

# 5) stop (holds current pose)
ros2 service call /ikt_pose_commander/disable std_srvs/srv/Trigger
```

Configure + command in **one** message with `ikt_interfaces/PoseCommand` (empty
`frame_link`/`control_link` reuse the last; bare pose controls the tip):

```bash
ros2 topic pub --once /ikt_pose_commander/pose_command ikt_interfaces/msg/PoseCommand \
    '{control_link: link_6, frame_link: base_link, has_pose: true,
      pose: {position: {x: 0.45, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}'
```

Pin a link at launch (skip step 2), or pin the controller if the match is
ambiguous:

```bash
ros2 launch ikt_pose_commander commander.launch.py \
    controlled_frame:=link_6 fpc_controller:=forward_position_controller
```

## Command modes

| `command_mode` | how | use |
|---|---|---|
| `fpc` (default) | 200 Hz accel-limited stream to `/<fpc>/commands` | continuous jogging / pose stream |
| `jtc` | one speed-limited `FollowJointTrajectory` per target | discrete moves |

Near a singularity the `fpc` stream switches to per-joint speeds
(`singularity_decouple`, on by default) so the arm passes through instead of
crawling; cap big joints with `joint_speed_limits` (JSON: joint→rad/s).

## Snap & teleop

Re-centre the goal on the arm's current pose anytime (jump-free):

```bash
ros2 service call /ikt_pose_commander/snap_target std_srvs/srv/Trigger
```

SpaceMouse jogging is fed via [`spacemouse_teleop`](../../../src/spacemouse_teleop)
(translates `/spacemouse/curr_pose` → `~/pose_command`, anchored to the EE):

```bash
ros2 launch spacemouse spacemouse.launch.py integration_frame:=world dashboard_port:=8080
ros2 launch ikt_pose_commander commander.launch.py controlled_frame:=compliance_link dashboard_port:=8180
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py
# enable on :8180, then jog
```

## Dashboard (optional)

Launched automatically with `dashboard_port:=8180`, or standalone. A thin web
client — pick the link, drag a 3D gizmo, enable/stop, live-tune params; all goes
through the same safety gates:

```bash
ros2 launch ikt_pose_commander dashboard.launch.py port:=8180   # http://localhost:8180
```

The page renders the live robot in 3D (from `/robot_description` meshes) as a set
of left-panel cards. Every action goes through the commander's safety gates, so
the UI can never bypass reachability / jump / speed limits:

* **Configure** — pick the controlled + base link, click Configure (while
  disabled). **Snap target → current pose** seeds the goal on the live pose.
* **Engage** — *Snap robot (jtc)* one discrete move; *Track robot (fpc)* streams
  the gizmo live; *Stop / Disengage* disables and holds.
* **Parameters** — live-tune stiffness, speeds, singularity knobs, tolerances.

Default port **8180**; for multiple arms run one per arm on distinct ports.

## Multiple arms

One instance per arm via `instance_name` (own namespace + dashboard, shared
`/robot_description`); each arm needs its **own** controller set:

```bash
ros2 launch ikt_pose_commander commander.launch.py instance_name:=left  dashboard_port:=8180 controlled_frame:=left_arm_Link7
ros2 launch ikt_pose_commander commander.launch.py instance_name:=right dashboard_port:=8181 controlled_frame:=right_arm_Link7
```

## Fixed joints (lifter / torso)

List chain joints driven separately in `fixed_joints` — held at their value, IK
solves around them, controller match is arm-only. Settable at launch, by config,
or per-joint checkboxes on the dashboard:

```bash
ros2 launch ikt_pose_commander commander.launch.py controlled_frame:=arm_tip \
    fixed_joints:="['torso_lift_joint']"
```

## ROS interface

| | name | type |
|---|---|---|
| sub | `~/target_pose` | `geometry_msgs/PoseStamped` (any TF frame) |
| sub | `~/pose_command` | `ikt_interfaces/PoseCommand` (link + frame + pose) |
| sub | `~/configure` | `std_msgs/String` JSON config |
| sub | `/robot_description`, `/joint_states` | model + seed |
| pub | `/<fpc>/commands` | `std_msgs/Float64MultiArray` (fpc) |
| pub | `~/status` | `std_msgs/String` JSON (state, joints, last solve, tunables) |
| srv | `~/enable` `~/disable` `~/stop` `~/snap_target` `~/return_to_start` | `std_srvs/Trigger` |

## Config

Set at launch, by `ros2 param set <ns> <key>`, or `~/configure` JSON. **Live**
keys apply anytime; **structural** keys (`controlled_frame`, `joints`,
`fixed_joints`, `*_controller`, `command_mode`) need disable first.

Most-used: `controlled_frame`, `base_frame`, `command_mode`, `default_stiffness`
(`[x y z rx ry rz]`; `0`=free, `1`=rigid), `max_joint_speed`, `max_joint_accel`,
`allow_unreachable` (default true = stretch toward target), `start_enabled`.

```bash
ros2 param set /ikt_pose_commander default_stiffness "[1,1,1,0,0,0]"   # position-only
```

Validated on real Duco GCR5-910, UR15, and dual-arm RM75; relies on
[`ikt_inverse_kinematics`](../ikt_inverse_kinematics/README.md) for the solve.
