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

## Algorithm

`ik_core.py` is a single **weighted, damped least-squares (Levenberg-Marquardt)
Gauss-Newton** solver with hard joint limits. One formulation covers every
feature — single tip, multi-tip / dual-arm, position-only, joint centering,
7-DOF arm-angle and relative-pose tasks — by stacking their rows into the same
least-squares problem:

$$
\begin{aligned}
\min_{q}\quad & \tfrac{1}{2}\sum_{k}\left\lVert W_{t,k}^{1/2}\,e_k(q)\right\rVert^{2}
\;+\; \tfrac{1}{2}\left\lVert W_q^{1/2}\,(q - q_{\mathrm{rest}})\right\rVert^{2} \\
\text{s.t.}\quad & q_{\min} \le q \le q_{\max}
\end{aligned}
$$

- $e_k(q)$ is the 6-DOF pose error (position + log-map orientation) of task $k$,
  evaluated on the Pinocchio model in the `LOCAL_WORLD_ALIGNED` frame.
- $W_{t,k}$ is the per-DOF **task stiffness** (a zero weight drops that DOF — e.g.
  position-only); $W_q$ is the **joint-centering / rest-posture** weight.

Each iteration:

1. **Stack** every task's weighted error $W^{1/2}e$ and weighted Jacobian
   $W^{1/2}J$ (plus any extra arm-angle / relative-pose rows the caller
   injects). Columns for frozen (inactive) joints are zeroed so those joints
   never move.
2. **Damped step** via the normal equations
   $(J^\top W J + \mu I)\,\Delta q_{\mathrm{task}} = J^\top W e$; the damping
   $\mu$ keeps the step finite through singularities.
3. **Posture bias** is projected into the task **null space** —
   $\Delta q_{\mathrm{null}} = \left(I - (J^\top W J + \mu I)^{-1} J^\top W J\right) W_q\,(q_{\mathrm{rest}} - q)$
   — so joint centering pulls the redundant DOF toward $q_{\mathrm{rest}}$
   without degrading the Cartesian task.
4. **Box-projected backtracking line search:** the combined
   $\Delta q_{\mathrm{task}} + \Delta q_{\mathrm{null}}$ step is clipped to
   $[q_{\min}, q_{\max}]$ and accepted only if it lowers the task error;
   otherwise the step is halved, and if the combined step stalls the task-only
   step is tried, so the posture bias can never block convergence.
5. **Adaptive damping:** $\mu$ shrinks ($\times 0.7$) after a good step and grows
   ($\times 2$, up to a cap) when no step helps — the singularity-robust LM
   behaviour.

**Termination & verdict.** The solve succeeds once the worst-case raw error is
within tolerance (defaults `1e-3 m`, `3.5e-3 rad`), ignoring DOFs whose
stiffness is zero. Otherwise it stops on no-further-progress, the damping cap,
or `max_iters` (default 200) and reports *why*: `JOINT_LIMIT`, `SINGULAR`
(smallest singular value below threshold) or `TASK_CONFLICT`, along with the
**blocking joints**, manipulability and $\sigma_{\min}$. Seeding from the current
joint state yields minimal-change, continuous solutions.

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
