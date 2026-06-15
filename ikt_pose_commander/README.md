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
`forward_position_controller` (`commands` topic). The `rm_control` hardware
shaper (velocity/accel limiting at 200 Hz) is the downstream safety backstop.

## Safety model (read before enabling)

* **Starts DISABLED.** No motion until you call `~/enable`. (`start_enabled` can
  override, but defaults to `false`.)
* **Reachability gate:** solutions with `reachable == false` (joint-limit,
  singular, task-conflict, max-iters) are rejected — never commanded.
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
    '{data: "{\"controlled_frame\": \"<your_tip_link>\", \"command_mode\": \"jtc\"}"}'
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
| `jtc` (default) | one speed-limited `FollowJointTrajectory` goal per target | safe, discrete moves; first real-robot tests |
| `fpc` | `Float64MultiArray` to `/<fpc>/commands` per target | continuous servoing / streaming targets |

`fpc` streams a setpoint per incoming target (no internal interpolation); it
relies on the `rm_control` hardware shaper for smoothing and on the jump gate to
reject discontinuities. Use `jtc` until you need streaming.

## Dashboard (optional, independent)

A web dashboard monitors the commander and drives it — **without** importing any
commander/IK internals. It is a thin HTTP/ROS client of the commander's API
(`~/status`, `~/enable`, `~/disable`, `~/target_pose`) plus its own TF listener
for capture/jog. The commander runs fine without it.

```bash
# after the commander is running (see above)
ros2 launch ikt_pose_commander dashboard.launch.py \
    commander_ns:=/ikt_pose_commander_right base_frame:=base_link port:=8180
# then open http://localhost:8180
```

The page shows live status (enabled, mode, model/joint-state freshness,
controlled frame, last message, last-solve reachable/reason/residual/step) and
offers controls: **Enable / Disable**, **Capture current** (fill the target with
the controlled frame's current pose), **Send target** (xyz + quaternion +
optional `frame_id`), and **Jog** buttons (±X/±Y/±Z by a step — capture current →
offset → send). Every command still goes through the commander's safety gates,
so the dashboard cannot bypass reachability / jump / speed limits. Default port
**8180** (8080/8100/8120/8140/8160 are used by the other toolkit dashboards).

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
* `~/status` (`std_msgs/String` JSON) — enabled, mode, last message, last solve
  (reachable/reason/residual), last step size, freshness.

**Actions / services used**
* `/<jtc_controller>/follow_joint_trajectory` (sends moves in `jtc` mode).
* `/controller_manager/switch_controller` (activates the mode's controller on enable).

**Services offered**
* `~/enable`, `~/disable`, `~/stop` (`std_srvs/Trigger`).

### Runtime configuration (unified)

Every config knob is set the **same** way at launch (a parameter) and at runtime
— through **either** of two equivalent channels, both funnelling through one
apply path:

* `ros2 param set <ns> <key> <value>` — the standard ROS way.
* `~/configure` (`std_msgs/String` carrying a JSON object of any subset of keys)
  — used by the dashboard.

Keys split by how they apply:

* **Live** (take effect immediately, even while enabled): `base_frame`,
  `max_joint_speed`, `min_move_time`, `max_step_rad`, `joint_states_stale_after`,
  `joint_centering_weight`, `damping`, `tol_pos`, `tol_ori`, `max_iters`,
  `default_stiffness`.
* **Structural** (change the kinematic group / controllers; **refused while
  enabled** — disable first): `controlled_frame`, `joints`, `jtc_controller`,
  `fpc_controller`, `command_mode`. Naming only `controlled_frame` re-derives
  `joints` + controllers automatically.

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

* Implemented & smoke-tested on the mock robot (model load, disabled-by-default,
  reachability + jump + stale gates, enable/disable controller switch, JTC move
  verified to move the EE, FPC publish path).
* Validated on the **real robot** (right arm, JTC): enable, capture-current
  no-op, a +1 cm move (EE tracked to <1 mm IK residual), and far-target gate
  rejection all confirmed; clean teardown, arms healthy.
* **Dashboard** (independent HTTP/ROS client, port 8180) smoke-tested on the
  mock: serves UI + `/api/state`, and Enable / Capture / Jog drive the commander
  through its gates (jog +Z executed a real move).
* Not yet: streaming continuity (velocity/jerk-bounded successive solutions);
  dual-arm relative-pose commanding (blocked on the `ikt_inverse_kinematics` R9
  frame fix); a dashboard.
* **Verify on the live URDF first** with a capture-current (no-op) target before
  any real motion, and keep a hand on the e-stop.
