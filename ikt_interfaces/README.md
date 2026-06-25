# ikt_interfaces

Typed ROS 2 interfaces for the IK toolkit: a `SolveIK` service and the `IKTask`
/ `IKResult` messages. They let C++ (or any typed) consumers call the advisory
solver in [`ikt_inverse_kinematics`](../ikt_inverse_kinematics/README.md)
without the JSON-string topic API.

> The solver is **advisory only** — the service returns an IK solution +
> diagnostics; it never commands the robot.

This package is robot-agnostic: all frames/joints are plain strings, so the same
interfaces work for any URDF.

## Interfaces

### `srv/SolveIK`

```
# request
IKTask[]  tasks            # one or more simultaneous tasks (multi-tip / dual-arm)
string[]  active_joints    # joints allowed to move ([] = all)
float64[] seed             # optional seed configuration ([] = current /joint_states)
---
# response
bool      ok               # request well-formed and a solve ran
string    message          # error detail when ok == false
IKResult  result
```

### `msg/IKTask`

```
string             frame        # any frame in the model (link, tip or tool frame)
geometry_msgs/Pose target       # target pose in the solve/base frame (or frame_id)
float64[6]         stiffness    # diag(Wt): [x y z rx ry rz]; 0 lets that DOF float
string             frame_id     # optional TF source frame for target ("" = base)
```

### `msg/IKResult`

```
bool      reachable        # true if all active pose tasks met their tolerance
string    reason           # ok | joint_limit | singular | task_conflict | max_iters | tf_unavailable
int32     iters
string[]  joint_names      # order of q
float64[] q                # solved joint angles (rad)
float64   max_pos_err      # metres
float64   max_ori_err      # rad
string[]  blocking_joints  # joints at a limit (when reason == joint_limit)
float64   manipulability   # sqrt(det(J J^T)) of the (first) task
float64   sigma_min        # smallest singular value (singularity proximity)
float64   delta_norm       # ||q - seed|| (continuity / motion size)
```

## Use

`ik_node` offers the typed service at `~/solve` (i.e. `/ik_node/solve`) when this
package is built. Inspect or call it from the CLI:

```bash
ros2 interface show ikt_interfaces/srv/SolveIK
ros2 service type /ik_node/solve            # -> ikt_interfaces/srv/SolveIK
```

```python
from ikt_interfaces.srv import SolveIK
from ikt_interfaces.msg import IKTask
# fill SolveIK.Request().tasks = [IKTask(frame=..., target=Pose(...), stiffness=[...])]
```

There is also an equivalent JSON-string topic API (`~/solve_request` →
`~/solve_response`) that needs no typed dependency; see
[`ikt_inverse_kinematics`](../ikt_inverse_kinematics/README.md).

## Build

```bash
colcon build --packages-select ikt_interfaces
```

`ament_cmake` + `rosidl` package; depends on `geometry_msgs` and `std_msgs`.
