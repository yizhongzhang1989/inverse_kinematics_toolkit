"""URDF string -> kinematic model (frames, Jacobians, limits) via Pinocchio.

A thin abstraction over Pinocchio so the rest of the package speaks in terms of
*frame names*, *joint names* and numpy arrays — never Pinocchio internals. This
is the only module that imports ``pinocchio``; swap it for a KDL-backed
implementation with the same surface if Pinocchio is ever unavailable.

Pure of rclpy: it takes a URDF *string* (from a live ``/robot_description`` or a
file) so it is fully unit-testable offline.

Key conventions
---------------
* Quaternions are ``(w, x, y, z)`` to match the rest of the toolkit and the
  reference IK script.
* Cartesian errors / Jacobians use the ``LOCAL_WORLD_ALIGNED`` frame: translation
  in world axes at the frame origin, rotation in world axes. This pairs with the
  log-map orientation error used by the solver.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - exercised only without the dep
    raise ImportError(
        "ikt_inverse_kinematics requires Pinocchio (python3-pinocchio). "
        "Install it (apt install python3-pinocchio, or pip install pin) and "
        "re-source your environment. Original error: %s" % exc
    ) from exc

try:
    # Reuse the toolkit's URDF aux-frame helper for virtual tool frames (R2).
    from ikt_common.urdf_loader import augment_urdf
except Exception:  # pragma: no cover - ikt_common always present in the toolkit
    augment_urdf = None


# Reference frame for all frame Jacobians / placements exposed by this module.
_REF = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED


def quat_wxyz_from_R(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    q_xyzw = pin.Quaternion(np.asarray(R, dtype=float))
    q_xyzw.normalize()
    return np.array([q_xyzw.w, q_xyzw.x, q_xyzw.y, q_xyzw.z], dtype=float)


def R_from_quat_wxyz(quat: Sequence[float]) -> np.ndarray:
    """Unit quaternion (w, x, y, z) -> rotation matrix."""
    w, x, y, z = (float(v) for v in quat)
    q = pin.Quaternion(w, x, y, z)
    q.normalize()
    return q.toRotationMatrix()


def se3_from_xyz_quat(xyz: Sequence[float], quat_wxyz: Sequence[float]) -> "pin.SE3":
    return pin.SE3(R_from_quat_wxyz(quat_wxyz),
                   np.asarray(xyz, dtype=float).reshape(3))


class RobotModel:
    """A Pinocchio kinematic model addressable by frame and joint *names*.

    Parameters
    ----------
    urdf_xml:
        Full URDF as a string.
    virtual_frames:
        Optional iterable of ``{name, parent, xyz, rpy}`` mappings appended as
        fixed joint+link pairs before the model is built (tool frames, R2).
    """

    def __init__(self, urdf_xml: str,
                 virtual_frames: Optional[Iterable[Mapping]] = None) -> None:
        frames = list(virtual_frames or [])
        if frames:
            if augment_urdf is None:
                raise RuntimeError(
                    "virtual_frames requested but ikt_common.urdf_loader."
                    "augment_urdf is unavailable")
            urdf_xml = augment_urdf(urdf_xml, frames)
        self._urdf_xml = urdf_xml
        self._virtual_frame_names = [f["name"] for f in frames]

        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()

        # Movable joints (skip the universe joint 0). Each 1-DOF revolute/
        # prismatic joint maps to one configuration index.
        self._joint_names: List[str] = []
        self._jname_to_qidx: Dict[str, int] = {}
        for jid in range(1, self.model.njoints):
            nq = self.model.nqs[jid]
            if nq != 1:
                # Non-1-DOF joints (free/spherical) are out of scope for this
                # solver's box-constrained revolute/prismatic assumption.
                continue
            name = self.model.names[jid]
            self._joint_names.append(name)
            self._jname_to_qidx[name] = self.model.idx_qs[jid]

        self.nq = self.model.nq

    # ------------------------------------------------------------------ #
    # Introspection
    # ------------------------------------------------------------------ #
    @property
    def joint_names(self) -> List[str]:
        """Movable 1-DOF joint names, in configuration-vector order."""
        return list(self._joint_names)

    @property
    def virtual_frame_names(self) -> List[str]:
        return list(self._virtual_frame_names)

    def has_frame(self, frame: str) -> bool:
        return self.model.existFrame(frame)

    def frame_names(self) -> List[str]:
        return [f.name for f in self.model.frames]

    def link_frame_names(self) -> List[str]:
        """Names of BODY frames (links) only, excluding the universe frame.

        These are the operable targets a user picks ("the link to control").
        Joint/sensor/fixed-op frames are excluded so a dashboard dropdown shows
        just the physical links plus any attached virtual tool frames.
        """
        try:
            body = int(pin.FrameType.BODY)
        except Exception:  # pragma: no cover
            body = None
        out: List[str] = []
        for f in self.model.frames:
            if f.name == "universe":
                continue
            if body is None or int(f.type) == body:
                out.append(f.name)
        return out

    def q_index(self, joint_name: str) -> int:
        if joint_name not in self._jname_to_qidx:
            raise KeyError(f"unknown joint '{joint_name}'")
        return self._jname_to_qidx[joint_name]

    def supporting_joints(self, frame: str) -> List[str]:
        """Movable 1-DOF joints on the kinematic path root -> ``frame``.

        This is what lets a caller specify ONLY the link to control: the joints
        that actually move that frame are derived from the model (Pinocchio
        ``model.supports`` of the frame's parent joint), returned in
        configuration order. Fixed/virtual frames resolve to their parent
        link's chain.
        """
        if not self.has_frame(frame):
            raise KeyError(f"unknown frame '{frame}'")
        fid = self.model.getFrameId(frame)
        parent_joint = self.model.frames[fid].parent
        movable = set(self._jname_to_qidx.keys())
        out: List[str] = []
        for jid in self.model.supports[parent_joint]:
            if jid == 0:
                continue  # universe
            name = self.model.names[jid]
            if name in movable:
                out.append(name)
        return out

    # ------------------------------------------------------------------ #
    # Limits
    # ------------------------------------------------------------------ #
    def joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """(q_min, q_max) over the full configuration vector.

        Continuous joints (URDF gives lower==upper==0, or +/-inf) are returned
        as +/-inf so the solver leaves them unclamped.
        """
        lo = np.array(self.model.lowerPositionLimit, dtype=float).copy()
        hi = np.array(self.model.upperPositionLimit, dtype=float).copy()
        bad = ~np.isfinite(lo) | ~np.isfinite(hi) | (hi <= lo)
        lo[bad] = -np.inf
        hi[bad] = np.inf
        return lo, hi

    def neutral(self) -> np.ndarray:
        return pin.neutral(self.model)

    # ------------------------------------------------------------------ #
    # Kinematics
    # ------------------------------------------------------------------ #
    def fk(self, q: np.ndarray, frame: str) -> Tuple[np.ndarray, np.ndarray]:
        """Forward kinematics: return (xyz, quat_wxyz) of ``frame`` for ``q``."""
        q = np.asarray(q, dtype=float).reshape(self.nq)
        fid = self.model.getFrameId(frame)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, fid)
        M = self.data.oMf[fid]
        return np.array(M.translation, dtype=float), quat_wxyz_from_R(M.rotation)

    def fk_se3(self, q: np.ndarray, frame: str) -> "pin.SE3":
        q = np.asarray(q, dtype=float).reshape(self.nq)
        fid = self.model.getFrameId(frame)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, fid)
        return self.data.oMf[fid].copy()

    def all_link_transforms(self, q: np.ndarray) -> Dict[str, list]:
        """Return {link_frame_name: 4x4 row-major nested list} for one config.

        Single FK + frame-placement pass, then read every BODY-frame placement.
        Pairs with the 3D viewer, which renders each visual at
        ``link_tf[link] * local_visual_origin``.
        """
        q = np.asarray(q, dtype=float).reshape(self.nq)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        out: Dict[str, list] = {}
        try:
            body = int(pin.FrameType.BODY)
        except Exception:  # pragma: no cover
            body = None
        for i, f in enumerate(self.model.frames):
            if f.name == "universe":
                continue
            if body is not None and int(f.type) != body:
                continue
            M = self.data.oMf[i]
            R = M.rotation
            p = M.translation
            out[f.name] = [
                [float(R[0, 0]), float(R[0, 1]), float(R[0, 2]), float(p[0])],
                [float(R[1, 0]), float(R[1, 1]), float(R[1, 2]), float(p[1])],
                [float(R[2, 0]), float(R[2, 1]), float(R[2, 2]), float(p[2])],
                [0.0, 0.0, 0.0, 1.0],
            ]
        return out

    def frame_jacobian(self, q: np.ndarray, frame: str) -> np.ndarray:
        """6xnq frame Jacobian in LOCAL_WORLD_ALIGNED (lin rows, then ang rows)."""
        q = np.asarray(q, dtype=float).reshape(self.nq)
        fid = self.model.getFrameId(frame)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.getFrameJacobian(self.model, self.data, fid, _REF)

    def pose_error(self, q: np.ndarray, frame: str,
                   target_xyz: np.ndarray, target_quat_wxyz: np.ndarray
                   ) -> np.ndarray:
        """6-vector [pos_err(3), ori_err(3)] = target - current.

        Position error is a plain difference in world axes; orientation error is
        the world-aligned rotation-vector (log-map) bringing current to target.
        Pairs with ``frame_jacobian`` (LOCAL_WORLD_ALIGNED).
        """
        p_cur, q_cur = self.fk(q, frame)
        e_pos = np.asarray(target_xyz, dtype=float).reshape(3) - p_cur
        R_cur = R_from_quat_wxyz(q_cur)
        R_tgt = R_from_quat_wxyz(target_quat_wxyz)
        # world-aligned orientation error: log(R_tgt * R_cur^T)
        R_err = R_tgt @ R_cur.T
        e_ori = pin.log3(R_err)
        return np.hstack([e_pos, e_ori])

    def manipulability(self, q: np.ndarray, frame: str,
                       rows: Optional[Sequence[int]] = None
                       ) -> Tuple[float, float]:
        """Yoshikawa manipulability sqrt(det(JJ^T)) and smallest singular value.

        ``rows`` optionally restricts to a subset of the 6 task rows (e.g. the
        3 position rows) — useful when only position is constrained.
        """
        J = self.frame_jacobian(q, frame)
        if rows is not None:
            J = J[list(rows), :]
        # use only columns of joints that actually move this frame (nonzero col)
        sv = np.linalg.svd(J, compute_uv=False)
        sigma_min = float(sv[-1]) if sv.size else 0.0
        w = float(np.sqrt(max(0.0, np.prod(sv[sv > 1e-12] ** 2))))
        return w, sigma_min

    # ------------------------------------------------------------------ #
    # Active-joint masking (named groups / arbitrary subsets)
    # ------------------------------------------------------------------ #
    def active_mask(self, active_joints: Optional[Sequence[str]]) -> np.ndarray:
        """Boolean length-nq mask; True where a joint is allowed to move.

        ``None`` or empty => all joints active.
        """
        mask = np.zeros(self.nq, dtype=bool)
        if not active_joints:
            mask[:] = True
            return mask
        for jn in active_joints:
            mask[self.q_index(jn)] = True
        return mask
