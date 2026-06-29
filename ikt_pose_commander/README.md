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
                                       hold if disabled / stale;     │            └─ fpc: Float64MultiArray
                                       speed / jump limited           │                stream to /commands
                                       (both modes)          ────────┘
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
* **Reachability gate:** when a solution is `reachable == false` (joint-limit,
  singular, task-conflict, max-iters) the commander commands the solver's
  **best-effort closest config** — the arm *stretches toward* an out-of-reach (or
  still-being-edited) target instead of refusing to move — still bounded by every
  gate below. This is the **default** (`allow_unreachable=true`). Set
  `allow_unreachable=false` (or untick it in the dashboard) to instead **reject**
  unreachable solutions and hold position until the target becomes reachable again.
* **Return-to-start:** `~/return_to_start` (Trigger) commands a JTC move back to
  the captured start pose and waits for completion — use it between tests.
* **Jump protection (event-driven path only):** when the control loop is off
  (`control_rate_hz=0`), a solve whose max joint change from the *current
  measured* pose exceeds `max_step_rad` is rejected. Under the control loop
  (default 200 Hz) **both** modes are inherently speed-limited — FPC ramps via
  the synchronized accel-limited generator; JTC sends one `FollowJointTrajectory`
  whose duration scales with the joint delta — so a large (e.g. best-effort)
  step executes as a slow, smooth, bounded trajectory instead of being rejected.
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
**200 Hz**) tracks the latest target and feeds the `forward_position_controller`
through a built-in **time-synchronized, acceleration-limited trajectory
generator**: all controlled joints advance along the *same* joint-space direction
governed by **one** scalar trapezoidal speed sized for the lead (largest-travel)
joint. They therefore stay **phase-locked** — the end-effector travels *directly*
toward the target (a straight joint-space segment, not a curve) and **every joint
reaches the goal at the same instant**, instead of small-travel joints finishing
early while larger ones lag (the cause of the old "curve, then shake around the
target" behaviour). The scalar speed is capped at `max_joint_speed`, ramped within
`max_joint_accel`, and braked on a discrete-time stopping curve so the arm parks
on the goal without overshoot or chatter; a large move simply slows down (and
*near a singularity* the generator can switch to a per-joint profile so the arm
doesn't crawl — see **Passing through a singularity** below). The IK goal is
**solved once per target and
cached** — re-used until the target pose moves more than ~1 mm / ~2 mrad — so the
redundant (e.g. 7-DOF) solution no longer drifts in the null space between ticks
(which previously made the arm jitter while holding). The generator seeds IK from
its **own commanded stream** (not the noisy measured joints), giving a rock-steady
hold, and an unchanged *unreachable* target is likewise not re-solved every tick
(so a hopeless target can't starve the `/joint_states` callback). The FPC itself
does no interpolation, so a sparse external pose stream (e.g. a ~25 Hz dashboard
drag) is up-sampled into smooth 200 Hz motion. Set `control_rate_hz:=0` for the
legacy event-driven path (one setpoint per received target, no interpolation).

### Passing through a singularity (per-joint decoupling)

The synchronized profile has one downside **near a singularity**: because every
joint is slaved to one shared speed, a heavily-damped IK goal that barely
advances (or demands a big reconfiguration) makes the *whole* arm **crawl** —
every joint creeping. When `singularity_decouple` is on (the default), the
generator detects singularity proximity from the solver's `sigma_min` (the
smallest Jacobian singular value) and, once it drops below `singularity_sigma`,
switches to a **decoupled per-joint** profile: each joint slews toward its goal
at its **own** acceleration-limited speed, so small-travel joints finish quickly
and the large **proximal joints are held to their own (lower) cap**. The
trade-off is explicit — the end-effector **no longer tracks a straight Cartesian
path** through the singular region — which is exactly what you want for *moving
through* a singularity instead of stalling at it. Hysteresis
(`singularity_exit_ratio`, disengage above `singularity_sigma * ratio`) plus a
velocity-continuous hand-off keep the switch jerk-bounded; away from
singularities the motion is the normal synchronized straight line.

Set **per-joint caps** with `joint_speed_limits` / `joint_accel_limits` — a JSON
object string mapping joint name → limit (rad/s, rad/s²); any joint not listed
falls back to the scalar `max_joint_speed` / `max_joint_accel`. Slow the big
joints, e.g.:

```bash
ros2 param set /ikt_pose_commander joint_speed_limits \
    '{"arm_1_joint_1": 0.3, "arm_1_joint_2": 0.3, "arm_1_joint_3": 0.4}'
```

Disable the behaviour entirely (always synchronized, today's straight-line
motion everywhere) with `singularity_decouple:=false`. The dashboard's last-solve
readout reports the live `sigma_min` and a `decoupled_active` flag so you can see
when the arm is in the singular region.

Switch to `jtc` for discrete, speed-limited `FollowJointTrajectory` goals. The
**same per-target goal cache** applies here: under the control loop a held target
is solved once and the trajectory is sent **once**, then re-sends are suppressed
until the target actually moves. Without it, re-solving every tick on a redundant
arm produced tiny null-space-different goals that **preempted the in-flight
trajectory** each tick — the controller kept restarting the move and the arm
**shook**. With the cache, JTC sends one clean trajectory per target (Snap = one
move) and holds.

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

## Setting the target + snapping

The commander drives the robot from an **absolute** target pose: a
`geometry_msgs/PoseStamped` on `~/target_pose`, TF-resolved from its
`header.frame_id` into the solver (model-root) frame. Each message **sets** the
goal — the commander always tracks the **latest** one, so intermediate poses may
be dropped freely (transport delay never corrupts the goal). This is the path
used by the 3D gizmo, `send_pose`, scripts, and the SpaceMouse bridge below.

**Snapping the goal to the current pose.** `~/snap_target` (`std_srvs/Trigger`)
sets the internal goal to the controlled frame's **current pose** (forward
kinematics of the measured joints). Use it to re-centre the target onto the live
pose at any time (no jump). The dashboard's *Snap target → current pose* button
calls it.

```bash
ros2 service call /ikt_pose_commander/snap_target std_srvs/srv/Trigger
ros2 topic pub /ikt_pose_commander/target_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: base_link}, pose: {position: {x: 0.6, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}'
```

## Unified command: `~/pose_command` (frame_link + control_link + pose)

One message (`ikt_interfaces/PoseCommand`) carries all three pieces — the frame
the pose is in, the link to control, and the target pose — so a single topic can
both select the control link and command it:

```
string frame_link     # frame the pose is defined in ("" = reuse previous)
string control_link   # link to control            ("" = reuse previous)
bool   has_pose       # true -> pose is set this msg; false -> reuse previous
geometry_msgs/Pose pose
```

Every field is **optional**: an empty `frame_link`/`control_link` or
`has_pose:=false` reuses the last value. If a field is **never** set the default
is the model's **first link** (`frame_link`, the root) and **last link**
(`control_link`, the tip). `control_link` is applied live — sending a new one
re-derives the joints/controllers and snaps to the current pose (no jump), even
while enabled. This is what the dashboard now uses to drive the robot. The
legacy `~/target_pose` + `~/configure` inputs remain for back-compat.

```bash
ros2 topic pub --once /ikt_pose_commander/pose_command ikt_interfaces/msg/PoseCommand \
    '{frame_link: base_link, control_link: link_6, has_pose: true, \
      pose: {position: {x: 0.6, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}'
```


## SpaceMouse teleop (via `spacemouse_teleop`)

The commander stays **device-agnostic**: SpaceMouse teleop is fed through the
[`spacemouse_teleop`](../../../src/spacemouse_teleop) translator, which
turns the 3Dconnexion `pose_node`'s absolute puck pose (`/spacemouse/curr_pose`)
into an absolute `~/target_pose` anchored to the arm's current EE (jump-free).
Every output is a full pose, so the commander tracks the latest and may drop
intermediate ones. Run the SpaceMouse stack, the commander, and the bridge in
separate terminals, then enable the commander:

```bash
ros2 launch spacemouse spacemouse.launch.py \
    integration_frame:=world max_trans_speed:=0.15 max_rot_speed:=0.6 \
    dashboard_port:=8080
ros2 launch ikt_pose_commander commander.launch.py \
    controlled_frame:=compliance_link dashboard_port:=8180
ros2 launch spacemouse_teleop spacemouse_teleop.launch.py
# enable from the :8180 dashboard (or `ros2 service call .../enable`), then jog
```

## Dashboard (optional, independent)

A web dashboard (`dashboard_node`) monitors and drives the commander — **without**
importing any commander/IK internals. It is a thin HTTP/ROS client of the
commander's API (`~/status`, `~/configure`, `~/enable`, `~/disable`,
`~/snap_target`, `~/target_pose`) plus its own TF listener. The commander runs
fine without it; it is launched automatically when `dashboard_port` is set, or
standalone:

```bash
ros2 launch ikt_pose_commander dashboard.launch.py \
    commander_ns:=/ikt_pose_commander base_frame:=base_link port:=8180
# then open http://localhost:8180
```

The page renders the live robot in 3D (from `/robot_description` meshes) as a set
of left-panel cards. Every action goes through the commander's safety gates, so
the UI can never bypass reachability / jump / speed limits:

* **Configure** — pick the link to control and the base reference link from the
  live URDF, then **Configure** (joints + JTC/FPC controllers auto-derive). Do
  this while the commander is disabled. **Snap target → current pose** calls
  `~/snap_target` to set the goal onto the controlled link's live pose.
* **Target mode** — switch the commander between **Absolute** (the gizmo / send
  path sets the goal) and **Delta (jog)** (incremental poses on `~/target_delta`,
  e.g. the SpaceMouse, are added to the goal). Snap first to seed the goal.
* **Target frame** — a draggable 3D gizmo (move / rotate handles) is the goal
  pose the controlled link is driven to match. **Snap target → link** resets the
  gizmo onto the current link pose.
* **Engage** (commands the real robot) — **Snap robot (jtc)** sends one discrete
  JTC move to the target frame; **Track robot (fpc)** live-streams the gizmo pose
  (~25 Hz) while you drag, which the commander's 200 Hz accel-limited loop turns
  into smooth motion; **Stop / Disengage** disables and holds.
* **Parameters** — live-edit the solver / motion tunables (per-DOF
  `default_stiffness`, `max_joint_speed`, `max_joint_accel`,
  `control_rate_hz`, the singularity knobs `singularity_decouple` /
  `singularity_sigma` / `joint_speed_limits`, tolerances, …). Each field posts to
  `~/configure` on change and applies immediately.

The 3D target marker follows a fresh absolute target on `~/target_pose` when
present, otherwise the commander's internal goal pose (`~/status`) — so the
snapped / delta-jogged goal is shown even when nothing publishes `~/target_pose`.

Default port **8180** (8080/8100/8120/8140/8160 are used by the other toolkit
dashboards). For a multi-arm robot run one dashboard per arm on distinct ports
(see *Multiple arms* below).

## Multiple arms

The commander is single-chain: one instance controls **one** link through one
controller set. A dual-arm (or N-arm) robot is handled by running **one commander
instance per arm**, distinguished by `instance_name`. Each instance:

* is named `ikt_pose_commander_<instance_name>` (node + namespace), so its
  topics/services live under `/ikt_pose_commander_<name>/...`;
* gets its own dashboard on its own `dashboard_port`, named
  `ikt_pose_commander_dashboard_<instance_name>` so multiple dashboards never
  collide on one ROS node name;
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

Each instance is gated and enabled independently (its own controllers + enable
state), so the arms don't interfere. Coordinated *relative-pose*
bimanual commanding (two arms rigidly holding one object) is a separate,
not-yet-wired feature — see the roadmap.

## Fixing joints (extra DOFs, e.g. a lifter / torso)

Not every joint on the path to the tip belongs to the arm IK. A torso-lift,
column, or rail joint is often on the kinematic chain but driven **separately**.
List such joints in **`fixed_joints`** and the commander holds them **out of the
IK**: they keep their current measured value while the arm solves *around* them,
and controller auto-discovery then matches the **arm-only** joint set (so it
finds the arm controller, not a non-existent arm+lifter one).

* **At launch:** `... commander.launch.py controlled_frame:=arm_tip
  fixed_joints:="['torso_lift_joint']"`
* **Live / on the dashboard:** it is a **structural** key (applies while
  **disabled** — disable first if enabled). The **Configure** card lists every
  movable joint with a **checkbox**; tick a joint to freeze it (it shows a
  **FIXED** tag and the joint gets an amber **🔒 FIXED** marker on the 3D canvas
  at that joint's location). Untick to release. Equivalent to
  `ros2 param set <ns> fixed_joints "['torso_lift_joint']"` or a `~/configure`
  JSON.

The status reports `group_joints` (full chain), `fixed_joints` (held), and
`joints` (the active set the IK actually solves = group − fixed).

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
  `joints` (active, IK-solved) + `fixed_joints` (held out of the IK) +
  `group_joints` (the full kinematic group = active ∪ fixed) + `command_joints`
  (the full controller joint set), last message, last
  solve (reachable/reason/residual), last step size, freshness, the live tunables
  (`max_joint_speed`, `max_joint_accel`, `control_rate_hz`, `max_step_rad`,
  tolerances, …), the feature fields (`allow_unreachable`,
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
  `tol_ori`, `max_iters`, `default_stiffness`,
  `allow_unreachable`, `reach_gain`, `control_rate_hz`.
* **Structural** (change the kinematic group / controllers; **refused
  while enabled** — disable first): `controlled_frame`, `joints`,
  `fixed_joints`, `jtc_controller`, `fpc_controller`, `command_mode`. Naming only
  `controlled_frame` re-derives `joints` + controllers automatically.

**New tunables (this revision):**

| key | kind | meaning |
|---|---|---|
| `fixed_joints` | structural | joint names the IK must **not** move (held at their current value), e.g. a lifter/torso joint on the path to the arm tip that is driven separately. Filtered out of the active joint group, so the solver freezes them and the arm solves **around** them; controller auto-discovery then matches the arm-only set. Empty = none. Settable at launch (`fixed_joints:="['torso_lift_joint']"`), via `~/configure` / `ros2 param set`, and on the dashboard (per-joint checkboxes in the Configure card; fixed joints also get a **🔒 FIXED** marker on the 3D canvas). |
| `allow_unreachable` | live | `true` (**default**) = best-effort: command the closest config and *stretch toward* unreachable / still-being-edited targets. `false` = reject unreachable solutions and hold position until the target is reachable again. |
| `default_stiffness` | live | per-DOF Cartesian stiffness `[x y z rx ry rz]`: `0` = that DOF floats **free**, positive = constrained (`1` = **rigid**). e.g. `[1 1 1 0 0 0]` = position-only, `[1 1 1 0 0 1]` = position + yaw. |
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
  reachability + jump + stale gates, enable/disable
  controller switch, JTC + FPC paths, subgroup commanding).
* Validated on the **real Duco GCR5-910** and a **real UR15** with no per-robot
  code: the model, IK, controller auto-discovery and safety gates transfer
  directly. JTC moves track to <1 mm; the **200 Hz acceleration-limited FPC**
  generator streams smooth motion (peak accel held at the `max_joint_accel` cap,
  rock-steady hold) on both robots' controller types
  (`ForwardCommandController` and `JointGroupPositionController`).
* Validated on a **real dual-arm RealMan RM75** (7-DOF per arm) controlling one
  arm at a time. The FPC generator was upgraded to a **time-synchronized**
  (phase-locked) profile so all joints reach the goal together and the
  end-effector moves directly toward the target — fixing a curved/shaking
  approach seen with the earlier per-joint independent profiles. Verified on the
  ROS mock + live arm: straight joint-space path, zero arrival spread, zero
  settling chatter, IK goal cached per target (no null-space drift). Best-effort
  reach (`allow_unreachable`) defaults **on** here so the arm keeps tracking an
  out-of-reach or being-edited target. The per-target **goal cache** applies to
  **both** FPC and JTC: in JTC mode a held target is sent as **one** trajectory
  (not re-sent every control tick), which fixed a separate JTC-only shaking from
  null-space-different goals preempting the in-flight trajectory. Jump protection
  is scoped to the event-driven path, so JTC also **moves toward** an unreachable
  target (one speed-limited best-effort trajectory), matching FPC. Unit tests:
  [`test/test_trajectory.py`](test/test_trajectory.py).
* **Dashboard** (independent HTTP/ROS client, port 8180) — Configure / Target
  frame gizmo / Snap-Track engage / live Parameters, validated on both robots.
* Not yet: dual-arm **relative-pose** commanding (one object held by two arms —
  blocked on the `ikt_inverse_kinematics` R9 frame fix). Independent per-arm
  control already works (see *Multiple arms*).
* **Verify on the live URDF first** with a Snap-target (no-op) before any real
  motion, keep within your clearance, and keep a hand on the e-stop.
