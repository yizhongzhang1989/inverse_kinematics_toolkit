"""High-level, ROS-free Python API for inverse kinematics.

This is the front door when using ``ikt_inverse_kinematics`` as a plain Python
library. Load a URDF (file, ``.xacro``, stdin or raw string) and solve IK with a
single call::

    from ikt_core import IK
    ik = IK.from_urdf_file("arm.urdf")
    sol = ik.solve("tool0", xyz=[0.4, -0.2, 0.5])
    print(sol.reachable, sol.joint_dict())

or the one-liner::

    from ikt_core import solve_ik
    sol = solve_ik("arm.urdf", "tool0", [0.4, -0.2, 0.5])

The class wraps :class:`~ikt_inverse_kinematics.robot_model.RobotModel` +
:func:`ikt_inverse_kinematics.ik_core.solve`, auto-deriving the moving joints for
the requested frame, defaulting the seed to the neutral pose, normalising
quaternions and wiring up tool-frame / arm-angle / relative-pose options. It
never imports ``rclpy``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Union

import numpy as np

from . import ik_core
from .ik_core import SolveParams
from .robot_model import RobotModel
from .tasks import Solution, Task
from .urdf_utils import read_urdf, run_xacro

SeedLike = Union[Sequence[float], Mapping[str, float], None]
_AUTO = "auto"


def _normalize_q(model: RobotModel, q: SeedLike) -> np.ndarray:
    """Turn a seed (None / array / {joint: value}) into a full nq vector."""
    if q is None:
        return model.neutral()
    if isinstance(q, Mapping):
        out = model.neutral()
        for jn, v in q.items():
            out[model.q_index(jn)] = float(v)
        return out
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.size != model.nq:
        raise ValueError(
            f"seed/config has {arr.size} entries but the model has {model.nq} "
            f"DOF; pass a length-{model.nq} array or a {{joint: value}} dict")
    return arr.copy()


def _as6_stiffness(stiffness: Optional[Sequence[float]],
                   position_only: bool) -> tuple:
    if stiffness is None:
        s = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    else:
        s = [float(v) for v in stiffness]
        if len(s) != 6:
            raise ValueError("stiffness must have 6 entries [x y z rx ry rz]")
    if position_only:
        s = [s[0], s[1], s[2], 0.0, 0.0, 0.0]
    return tuple(s)


class IK:
    """Reusable inverse-kinematics solver bound to one robot model.

    Parameters
    ----------
    urdf:
        A URDF source accepted by :func:`urdf_utils.read_urdf` — a file path, a
        ``.xacro`` path, ``"-"`` for stdin, or a raw URDF XML string.
    tool_frames:
        Optional list of virtual tool frames ``{name, parent, xyz, rpy}``
        attached to the model for every solve (R2). Per-call tool frames can
        also be passed to :meth:`solve`.
    base_frame:
        Informational only: all targets are expressed in the URDF root frame in
        this pure-Python API (TF reframing is a feature of the ROS node).
    params:
        Default :class:`~ikt_inverse_kinematics.ik_core.SolveParams` for solves
        (overridable per call).
    """

    def __init__(self, urdf: Union[str, Path], *,
                 tool_frames: Optional[Sequence[Mapping]] = None,
                 base_frame: Optional[str] = None,
                 params: Optional[SolveParams] = None) -> None:
        self._urdf_xml = read_urdf(urdf)
        self._tool_frames: List[Mapping] = list(tool_frames or [])
        self.model = RobotModel(
            self._urdf_xml,
            virtual_frames=self._tool_frames or None)
        self.base_frame = base_frame
        self.params = params or SolveParams()

    # -- constructors -----------------------------------------------------
    @classmethod
    def from_urdf_file(cls, path: Union[str, Path],
                       mappings: Optional[Mapping[str, str]] = None,
                       **kwargs) -> "IK":
        """Build from a ``.urdf`` or ``.xacro`` file path (xacro auto-detected)."""
        return cls(read_urdf(path, mappings), **kwargs)

    @classmethod
    def from_xacro(cls, path: Union[str, Path],
                   mappings: Optional[Mapping[str, str]] = None,
                   **kwargs) -> "IK":
        """Build by running ``xacro`` on ``path`` with optional ``name:=value``."""
        return cls(run_xacro(path, mappings), **kwargs)

    # -- introspection ----------------------------------------------------
    @property
    def urdf_xml(self) -> str:
        return self._urdf_xml

    @property
    def joint_names(self) -> List[str]:
        """Movable joint names in configuration-vector order."""
        return self.model.joint_names

    @property
    def link_names(self) -> List[str]:
        """Operable link/frame names (excludes the universe frame)."""
        return self.model.link_frame_names()

    def neutral(self) -> np.ndarray:
        return self.model.neutral()

    def supporting_joints(self, frame: str) -> List[str]:
        """Movable joints on the kinematic path root -> ``frame``."""
        return self.model.supporting_joints(frame)

    def fk(self, q: SeedLike, frame: str):
        """Forward kinematics: ``(xyz, quat_wxyz)`` of ``frame`` at config ``q``."""
        return self.model.fk(_normalize_q(self.model, q), frame)

    # -- solving ----------------------------------------------------------
    def solve(self, frame: str, xyz: Sequence[float],
              quat: Optional[Sequence[float]] = None, *,
              seed: SeedLike = None,
              stiffness: Optional[Sequence[float]] = None,
              position_only: bool = False,
              active_joints: Union[str, Sequence[str], None] = _AUTO,
              tool_frames: Optional[Sequence[Mapping]] = None,
              arm_angle: Optional[Mapping] = None,
              relative: Optional[Mapping] = None,
              params: Optional[SolveParams] = None) -> Solution:
        """Solve IK to place ``frame`` at ``xyz`` (and ``quat`` if given).

        Parameters
        ----------
        frame:
            Target frame name (link or tool frame).
        xyz:
            Target position (m) in the URDF root frame.
        quat:
            Optional target orientation ``(w, x, y, z)``. If omitted (or
            ``position_only=True``) orientation is left free.
        seed:
            Starting configuration — ``None`` (neutral), a length-nq array, or a
            ``{joint: value}`` dict. Seeding from the current pose gives the
            minimal-change solution.
        stiffness:
            Optional per-DOF weights ``[x y z rx ry rz]``.
        position_only:
            Zero the orientation stiffness (position-only task).
        active_joints:
            ``"auto"`` (default) freezes everything except the joints that move
            ``frame``; ``None`` allows all joints; or pass an explicit list.
        tool_frames:
            Optional per-call virtual tool frames (merged with construction-time
            ones).
        arm_angle:
            Optional S-R-S arm-angle task: ``{shoulder, elbow, wrist, psi,
            stiffness?, name?}``.
        relative:
            Optional dual-arm relative-pose hold: ``{frame_a, frame_b, xyz?,
            quat?, stiffness?}``.
        """
        model = self._model_with(tool_frames)
        if not model.has_frame(frame):
            raise ValueError(f"unknown frame '{frame}'")
        q_t = (1.0, 0.0, 0.0, 0.0)
        force_point = position_only or quat is None
        if not force_point:
            q_t = tuple(float(x) for x in quat)
            if len(q_t) != 4:
                raise ValueError("quat must have 4 entries (w, x, y, z)")
        stiff = _as6_stiffness(stiffness, force_point)
        task = Task(frame, tuple(float(v) for v in xyz), q_t, stiff)

        extras = self._build_extras(model, arm_angle, relative)
        aj = self._resolve_active(model, [frame], active_joints)
        seed_q = _normalize_q(model, seed)
        return ik_core.solve(model, seed_q, [task],
                             params=params or self.params,
                             active_joints=aj,
                             extra_tasks=extras or None)

    def solve_many(self, tasks: Sequence[Union[Task, Mapping]], *,
                   seed: SeedLike = None,
                   active_joints: Union[str, Sequence[str], None] = _AUTO,
                   tool_frames: Optional[Sequence[Mapping]] = None,
                   arm_angle: Optional[Mapping] = None,
                   relative: Optional[Mapping] = None,
                   params: Optional[SolveParams] = None) -> Solution:
        """Solve multiple simultaneous Cartesian tasks (multi-tip / dual-arm).

        Each task may be a :class:`~ikt_inverse_kinematics.tasks.Task` or a dict
        ``{frame, xyz, quat?, stiffness?, position_only?}``.
        """
        model = self._model_with(tool_frames)
        task_objs = [self._as_task(t) for t in tasks]
        frames = [t.frame for t in task_objs]
        for t in task_objs:
            if not model.has_frame(t.frame):
                raise ValueError(f"unknown frame '{t.frame}'")
        extras = self._build_extras(model, arm_angle, relative)
        aj = self._resolve_active(model, frames, active_joints)
        seed_q = _normalize_q(model, seed)
        return ik_core.solve(model, seed_q, task_objs,
                             params=params or self.params,
                             active_joints=aj,
                             extra_tasks=extras or None)

    # -- internals --------------------------------------------------------
    def _model_with(self, tool_frames: Optional[Sequence[Mapping]]) -> RobotModel:
        if not tool_frames:
            return self.model
        merged = self._tool_frames + list(tool_frames)
        return RobotModel(self._urdf_xml, virtual_frames=merged)

    def _resolve_active(self, model: RobotModel, frames: Sequence[str],
                        active_joints: Union[str, Sequence[str], None]
                        ) -> Optional[List[str]]:
        if active_joints is None:
            return None
        if isinstance(active_joints, str):
            if active_joints != _AUTO:
                raise ValueError("active_joints string must be 'auto'")
            out: List[str] = []
            seen = set()
            for f in frames:
                for jn in model.supporting_joints(f):
                    if jn not in seen:
                        seen.add(jn)
                        out.append(jn)
            return out or None
        return list(active_joints)

    @staticmethod
    def _as_task(t: Union[Task, Mapping]) -> Task:
        if isinstance(t, Task):
            return t
        frame = t["frame"]
        xyz = tuple(float(v) for v in t["xyz"])
        position_only = bool(t.get("position_only", False))
        quat = t.get("quat")
        force_point = position_only or quat is None
        q_t = (1.0, 0.0, 0.0, 0.0) if force_point \
            else tuple(float(x) for x in quat)
        stiff = _as6_stiffness(t.get("stiffness"), force_point)
        return Task(frame, xyz, q_t, stiff)

    @staticmethod
    def _build_extras(model: RobotModel, arm_angle: Optional[Mapping],
                      relative: Optional[Mapping]) -> list:
        extras = []
        if arm_angle is not None:
            from .arm_angle import SRSChain, make_arm_angle_extra_task
            a = arm_angle
            chain = SRSChain(
                name=str(a.get("name", "chain")),
                shoulder=a["shoulder"], elbow=a["elbow"], wrist=a["wrist"],
                base=a.get("base", ""), tip=a.get("tip", ""))
            extras.append(make_arm_angle_extra_task(
                model, chain, float(a["psi"]), float(a.get("stiffness", 0.5))))
        if relative is not None:
            from .relative import make_relative_extra_task
            r = relative
            extras.append(make_relative_extra_task(
                model, r["frame_a"], r["frame_b"],
                r.get("xyz", [0.0, 0.0, 0.0]),
                r.get("quat", [1.0, 0.0, 0.0, 0.0]),
                r.get("stiffness", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])))
        return extras


def solve_ik(urdf: Union[str, Path], frame: str, xyz: Sequence[float],
             quat: Optional[Sequence[float]] = None, *,
             tool_frames: Optional[Sequence[Mapping]] = None,
             params: Optional[SolveParams] = None, **solve_kwargs) -> Solution:
    """One-shot convenience: build an :class:`IK` from ``urdf`` and solve once.

    Extra keyword arguments are forwarded to :meth:`IK.solve` (``seed``,
    ``stiffness``, ``position_only``, ``active_joints``, ``arm_angle``,
    ``relative`` ...). For repeated solves on one robot, construct :class:`IK`
    once and reuse it instead.
    """
    ik = IK(urdf, tool_frames=tool_frames, params=params)
    return ik.solve(frame, xyz, quat, **solve_kwargs)
