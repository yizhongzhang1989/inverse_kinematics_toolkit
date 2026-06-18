# ikt_pose_commander

Accept a Cartesian **target pose** on a topic, solve it with the
[`ikt_inverse_kinematics`](../ikt_inverse_kinematics/README.md) solver
(in-process), and **command the arm** to that pose.

Unlike `ikt_inverse_kinematics` (which is *advisory only* — it just publishes IK
results), this package **actually moves the robot**. It is therefore
**safety-gated**.

```
PoseStamped ──▶ (TF→base) ──▶ ikt_inverse_kinematics solve ──▶ SAFETY GATE ──▶ command
   ~/target_pose                 (per-DOF stiffness, limits,        │            │
                                  rest posture, reachability)       │            ├─ jtc: one speed-limited
                                                                    │            │   FollowJointTrajectory goal
                                            reject if: disabled,     │            └─ fpc: Float64MultiArray
                                            unreachable, jump >       │                stream to /commands
                                            max_step, stale state ───┘
```

## Why a node, not a `ros2_control` controller

In `ros2_control` a controller can't *call* another controller, and
`forward_position_controller` is **not chainable** (it's a plain
`ControllerInterface`, verified on ros2_control 2.53.1). Also, the IK is
Python/Pinocchio and can't run inside a C++ real-time `update()`. So the right
design (Plan C) is a **node** that computes IK and *commands* the existing
controller — either the `JointTrajectoryController` (action) or the
`forward_position_controller` (`commands` topic). Smoothness is produced **inside
this node** by a fixed-rate, acceleration-limited trajectory generator (see
*Command modes* below), so it does not depend on any robot-specific hardware
shaper.

## Safety model (read before enabling)

* **Starts DISABLED.** No motion until you call `~/enable`. (`start_enabled` can
  override, but defaults to `false`.)
* **Reachability gate:** solutions with `reachable == false` (joint-limit,
  singular, task-conflict, max-iters) are rejected by default. Set
  `allow_unreachable=true` to instead command the solver's **best-effort closest
  config** (the arm *stretches toward* an out-of-reach target) — still bounded
  by every gate below.
* **30 cm Cartesian envelope (`safety_radius_m`, default 0.30):** on `~/enable`
  the controlled frame's pose is captured as the **start**; thereafter every
  command is checked: in `jtc` a target whose predicted EE leaves the sphere is
  **rejected**; in `fpc` the joint step is **clamped** so the EE lands on the
  sphere boundary. Runs *regardless of* reachability — it is the primary
  software bound on motion.
* **Measured-TCP watchdog (independent backstop):**
  [`tools/ikt_safety_watchdog.py`](../../../tools/ikt_safety_watchdog.py) reads
  the **real** TCP at ≥5 Hz and calls `~/disable` if it leaves the sphere (fails
  safe on read errors too). Run it for every real-robot test.
* **Return-to-start:** `~/return_to_start` (Trigger) commands a JTC move back to
  the captured start pose and waits for completion — use it between tests.
* **Jump protection:** a solve whose max joint change from the *current measured*
  pose exceeds `max_step_rad` is rejected.
* **Speed limiting (JTC):** every move's duration is
  `max(min_move_time, max_joint_delta / max_joint_speed)`.
* **Stale-input hold:** no command if `/joint_states` is stale or no model yet.
* **Disable / stop** returns the arm to its `JointTrajectoryController`, which
  holds the current pose; any in-flight goal is cancelled.

> Singular start configs are refused by design. The mock robot boots at
> all-joints-zero, which for the RM75 is a *fully-extended singularity*
> (`sigma_min ≈ 0`); the gate will reject Cartesian moves from there. Bend the
> arm off the singularity first (a joint-space JTC move, or `align` via teleop).
> A real arm is essentially never at exactly zero.

## Build

```bash
cd <ws> && export COLCON_DEFAULTS_FILE=$PWD/colcon_defaults.yaml
colcon build --packages-select ikt_inverse_kinematics ikt_pose_commander --symlink-install
source install/setup.bash
```

## Run

Robot-independent: launch with **no arguments**. The node reads
`/robot_description` online and starts **unconfigured**; you pick the link to
control at runtime (the dashboard, or a `~/configure` message) and the joints +
JTC/FPC controllers are auto-derived.

```bash
# zero-config: works on ANY robot once a bringup publishes
# /robot_description + /joint_states and the controllers are loaded
ros2 launch ikt_pose_commander commander.launch.py            # headless
ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180  # + UI
```

**Configure by naming only the link** (joints = kinematic path to it; JTC/FPC =
matched in `/controller_manager`):

```bash
# via the dashboard: pick the link in the "Configure" dropdown, click Configure
# or by topic:
ros2 topic pub --once /ikt_pose_commander/configure std_msgs/msg/String \
    '{data: "{\"controlled_frame\": \"<your_tip_link>\", \"command_mode\": \"fpc\"}"}'
```

You may still pin a fixed config at launch (skips the runtime step):

```bash
ros2 launch ikt_pose_commander commander.launch.py \
    instance_name:=left controlled_frame:=left_arm_Link7
# joints + controllers are still auto-derived from that link unless you also
# pass joints:=[...] / jtc_controller:= / fpc_controller:= explicitly.
```

Enable, then send a pose:

```bash
ros2 service call /ikt_pose_commander/enable std_srvs/srv/Trigger

# capture the current EE pose (a no-op target — safe first check)
ros2 run ikt_pose_commander send_pose \
    --topic /ikt_pose_commander/target_pose --capture <your_tip_link>

# an absolute pose in a known frame
ros2 run ikt_pose_commander send_pose \
    --topic /ikt_pose_commander/target_pose \
    --xyz 0.45 -0.78 1.06 --quat 1 0 0 0 --frame-id base_link
```

Or publish `geometry_msgs/PoseStamped` to `~/target_pose` from your own node.
`header.frame_id` may be any TF frame; it is transformed into the solve frame.

Disable / stop (return to JTC hold):

```bash
ros2 service call /ikt_pose_commander_right/disable std_srvs/srv/Trigger
```

## Command modes

| `command_mode` | path | when |
|---|---|---|
| `fpc` (default) | `Float64MultiArray` to `/<fpc_controller>/commands` per target | continuous servoing / a streamed target pose (the normal path) |
| `jtc` | one speed-limited `FollowJointTrajectory` goal per target | discrete moves; conservative bring-up |

`fpc` is the default. A **fixed-rate control loop** (`control_rate_hz`, default
**200 Hz**) re-solves the latest target every tick and feeds the
`forward_position_controller` through a built-in **acceleration-limited
trajectory generator**: each joint is driven toward the IK goal with its velocity
capped at `max_joint_speed` and ramped within `max_joint_accel`, so the streamed
setpoint is a smooth trapezoidal-velocity profile with no jerk — the FPC itself
does no interpolation. A stopping-distance brake parks the joint on the goal
without overshoot, and the generator seeds IK from its **own commanded stream**
(not the noisy measured joints), giving a rock-steady hold. The net effect: a
sparse external pose stream (e.g. a ~25 Hz dashboard drag) is up-sampled into
smooth 200 Hz motion. Set `control_rate_hz:=0` for the legacy event-driven path
(one setpoint per received target, no interpolation). Switch to `jtc` for
discrete, speed-limited `FollowJointTrajectory` goals.

**Naming the controller.** Both controllers are **auto-derived** from
`/controller_manager` by matching the controlled link's joints to a controller's
`<joint>/position` command interfaces — the JTC is any
`JointTrajectoryController`; the FPC is a `ForwardCommandController` **or** a
`JointGroupPositionController` (so Duco- and UR-style stacks both work), and on a
coverage tie an **active** controller wins. To pin the exact name instead — e.g.
force the precise `forward_position_controller` — set it in **config** (the
`ikt_pose_commander:` section, key `fpc_controller` / `jtc_controller`) **or**
pass it as a **launch argument** of the same name:

```bash
ros2 launch ikt_pose_commander commander.launch.py \
    fpc_controller:=forward_position_controller   # or jtc_controller:=arm_1_controller
```

## Dashboard (optional, independent)

A web dashboard (`dashboard_node`) monitors and drives the commander — **without**
importing any commander/IK internals. It is a thin HTTP/ROS client of the
commander's API (`~/status`, `~/configure`, `~/enable`, `~/disable`,
`~/target_pose`) plus its own TF listener. The commander runs fine without it; it
is launched automatically when `dashboard_port` is set, or standalone:

```bash
ros2 launch ikt_pose_commander dashboard.launch.py \
    commander_ns:=/ikt_pose_commander base_frame:=base_link port:=8180
# then open http://localhost:8180
```

The page renders the live robot in 3D (from `/robot_description` meshes) as four
left-panel cards. Every action goes through the commander's safety gates, so the
UI can never bypass reachability / envelope / jump / speed limits:

* **Configure** — pick the link to control and the base reference link from the
  live URDF, then **Configure** (joints + JTC/FPC controllers auto-derive). Do
  this while the commander is disabled.
* **Target frame** — a draggable 3D gizmo (move / rotate handles) is the goal
  pose the controlled link is driven to match. **Snap target → link** resets the
  gizmo onto the current link pose.
* **Engage** (commands the real robot) — **Snap robot (jtc)** sends one discrete
  JTC move to the target frame; **Track robot (fpc)** live-streams the gizmo pose
  (~25 Hz) while you drag, which the commander's 200 Hz accel-limited loop turns
  into smooth motion; **Stop / Disengage** disables and holds.
* **Parameters** — live-edit the solver / motion / safety tunables (stiffness
  preset, `safety_radius_m`, `max_joint_speed`, `max_joint_accel`,
  `control_rate_hz`, tolerances, …). Each field posts to `~/configure` on change
  and applies immediately.

Default port **8180** (8080/8100/8120/8140/8160 are used by the other toolkit
dashboards). For a multi-arm robot run one dashboard per arm on distinct ports
(see *Multiple arms* below).

## Multiple arms

The commander is single-chain: one instance controls **one** link through one
controller set. A dual-arm (or N-arm) robot is handled by running **one commander
instance per arm**, distinguished by `instance_name`. Each instance:

* is named `ikt_pose_commander_<instance_name>` (node + namespace), so its
  topics/services live under `/ikt_pose_commander_<name>/...`;
* gets its own dashboard on its own `dashboard_port`;
* reads the **same** shared `/robot_description` + `/joint_states`, and is
  configured to that arm's tip link — the joints (kinematic path to the tip) and
  the JTC/FPC controllers auto-derive per arm.

```bash
# one shared bringup publishes the dual-arm /robot_description + /joint_states
# and loads a controller set PER ARM, then start one commander per arm:
ros2 launch ikt_pose_commander commander.launch.py \
    instance_name:=left  dashboard_port:=8180 controlled_frame:=left_arm_Link7
ros2 launch ikt_pose_commander commander.launch.py \
    instance_name:=right dashboard_port:=8181 controlled_frame:=right_arm_Link7
# dashboards: http://localhost:8180 (left) and http://localhost:8181 (right)
```

**Requirement: one controller set per arm.** Auto-discovery matches a controller
to the controlled link by its joints, so each arm needs its **own** JTC and/or
FPC controller (e.g. `left_arm_controller` + `right_arm_controller`, plus per-arm
`*_forward_position_controller`s). Two instances must **not** drive the same
controller `commands` topic — give each arm a distinct controller. (If a single
controller spans all joints, the subgroup logic lets one instance command its arm
while *holding* the others, but two instances writing one FPC topic would fight.)
Pin names with `jtc_controller:=` / `fpc_controller:=` (or per-instance config) if
the joint match is ambiguous.

Each instance captures and enforces its **own** `safety_radius_m` envelope at its
own `~/enable`, so the arms are independently gated. Coordinated *relative-pose*
bimanual commanding (two arms rigidly holding one object) is a separate,
not-yet-wired feature — see the roadmap.

## ROS interface

**Subscribes**
* `~/target_pose` (`geometry_msgs/PoseStamped`) — the target for `controlled_frame`.
  `header.frame_id` may be any TF frame; an empty one is interpreted in
  `base_frame`. Either way the target is transformed into the model root for the
  solve. One-shot or streamed — a stream just retargets; it never interrupts the
  robot (especially in `fpc` mode, where each target overwrites the setpoint).
* `~/configure` (`std_msgs/String`, JSON) — unified runtime config (see below).
* `/robot_description` (`std_msgs/String`, latched) — builds the kinematic model.
* `/joint_states` (`sensor_msgs/JointState`) — IK seed + current pose.

**Publishes**
* `/<fpc_controller>/commands` (`std_msgs/Float64MultiArray`) — in `fpc` mode.
* `~/status` (`std_msgs/String` JSON) — enabled, mode, controlled/base frame,
  `joints` + `command_joints` (the full controller joint set), last message, last
  solve (reachable/reason/residual), last step size, freshness, the live tunables
  (`max_joint_speed`, `max_joint_accel`, `control_rate_hz`, `max_step_rad`,
  tolerances, …), the safety/feature fields (`safety_radius_m`, `start_ee`,
  `ee_displacement`, `clamp_scale`, `allow_unreachable`, `stiffness_preset`,
  `reach_gain`, `best_effort`), and `available_links` / `available_joints` from
  the live URDF so the dashboard can populate its dropdowns.

**Actions / services used**
* `/<jtc_controller>/follow_joint_trajectory` (sends moves in `jtc` mode).
* `/controller_manager/switch_controller` (activates the mode's controller on enable).

**Services offered**
* `~/enable`, `~/disable`, `~/stop` (`std_srvs/Trigger`).
* `~/return_to_start` (`std_srvs/Trigger`) — JTC move back to the pose captured
  at the last `~/enable`; blocks until the move completes.

### Runtime configuration (unified)

Every config knob is set the **same** way at launch (a parameter) and at runtime
— through **either** of two equivalent channels, both funnelling through one
apply path:

* `ros2 param set <ns> <key> <value>` — the standard ROS way.
* `~/configure` (`std_msgs/String` carrying a JSON object of any subset of keys)
  — used by the dashboard.

Keys split by how they apply:

* **Live** (take effect immediately, even while enabled): `base_frame`,
  `max_joint_speed`, `max_joint_accel`, `min_move_time`, `max_step_rad`,
  `joint_states_stale_after`, `joint_centering_weight`, `damping`, `tol_pos`,
  `tol_ori`, `max_iters`, `default_stiffness`, `safety_radius_m`,
  `allow_unreachable`, `stiffness_preset`, `reach_gain`, `control_rate_hz`.
* **Structural** (change the kinematic group / controllers; **refused
  while enabled** — disable first): `controlled_frame`, `joints`,
  `jtc_controller`, `fpc_controller`, `command_mode`. Naming only
  `controlled_frame` re-derives `joints` + controllers automatically.

**New tunables (this revision):**

| key | kind | meaning |
|---|---|---|
| `safety_radius_m` | live | radius (m) of the Cartesian motion envelope around the start pose (default 0.30). |
| `allow_unreachable` | live | `true` = best-effort: command the closest config and *stretch toward* unreachable targets (default `false`). |
| `stiffness_preset` | live | `full_pose` \| `position_only` \| `position_yaw` \| `custom`. How hard each DOF reaches; `custom` uses `default_stiffness`. |
| `reach_gain` | live | (0,1] FPC approach scaling, **event-driven path only** (`control_rate_hz=0`): command `cur + reach_gain·(q_cmd−cur)` per target for a gradual stretch (default 1.0). Ignored by the accel-limited control loop. |
| `control_rate_hz` | live | fixed-rate FPC control loop (default **200** Hz): re-solves the latest target and streams an accel-limited setpoint each tick. `0` = legacy event-driven (one setpoint per target, no interpolation). Capped at 250 Hz. |
| `max_joint_accel` | live | per-joint acceleration cap (rad/s², default **3.0**) for the FPC control-loop generator — the **smoothness knob** (lower = gentler starts/stops). JTC timing is unaffected. |

```bash
# set the controlled (target) link and the base reference link at runtime
ros2 param set /ikt_pose_commander controlled_frame compliance_link
ros2 param set /ikt_pose_commander base_frame base_link
# or both at once over the unified topic
ros2 topic pub --once /ikt_pose_commander/configure std_msgs/msg/String \
  '{data: "{\"controlled_frame\": \"compliance_link\", \"base_frame\": \"base_link\", \"command_mode\": \"fpc\"}"}'
```

`base_frame` is the **default reference frame** a bare target (empty
`header.frame_id`) is interpreted in; it is transformed to the model root for the
solver, so any TF frame — a robot link or an external frame — is a valid base.

The **dashboard** (`dashboard_node`, optional) is a separate node that consumes
this API over HTTP/JSON; it adds no new robot-facing interface. Its Configure
card now also offers a **base-link** selector alongside the controlled-link one.

## Key parameters

See [config/commander_defaults.yaml](config/commander_defaults.yaml). Most-used:
`controlled_frame` (the target link), `base_frame` (the target reference link),
`joints`, `jtc_controller`, `fpc_controller`, `command_mode`, `default_stiffness`
(per-DOF IK weighting — e.g. zero the last 3 for position-only),
`max_joint_speed`, `max_step_rad`, `start_enabled`. All are settable at launch
**and** at runtime (see *Runtime configuration* above).

## Relationship to the rest of the toolkit

* **`ikt_inverse_kinematics`** — the advisory solver this node calls in-process
  (reuses its per-DOF stiffness, rest posture, reachability verdict). Never
  commands the robot.
* **`cartesian_control_manager` / FZI `cartesian_motion_controller`** — an
  alternative pose→motion path that uses FZI's forward-dynamics differential IK
  (with the 7-DOF null-space drift). Use `ikt_pose_commander` when you want *your*
  IK semantics in the loop instead.

## Status / roadmap

* Implemented & validated on **mock** (model load, disabled-by-default,
  reachability + Cartesian-envelope + jump + stale gates, enable/disable
  controller switch, JTC + FPC paths, subgroup commanding).
* Validated on the **real Duco GCR5-910** and a **real UR15** with no per-robot
  code: the model, IK, controller auto-discovery and safety gates transfer
  directly. JTC moves track to <1 mm; the **200 Hz acceleration-limited FPC**
  generator streams smooth motion (peak accel held at the `max_joint_accel` cap,
  rock-steady hold) on both robots' controller types
  (`ForwardCommandController` and `JointGroupPositionController`).
* **Dashboard** (independent HTTP/ROS client, port 8180) — Configure / Target
  frame gizmo / Snap-Track engage / live Parameters, validated on both robots.
* Not yet: dual-arm **relative-pose** commanding (one object held by two arms —
  blocked on the `ikt_inverse_kinematics` R9 frame fix). Independent per-arm
  control already works (see *Multiple arms*).
* **Verify on the live URDF first** with a Snap-target (no-op) before any real
  motion, keep within your clearance, and keep a hand on the e-stop.
