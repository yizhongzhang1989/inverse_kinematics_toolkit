"""Build the canonical ``robot_description`` URDF for a robot bringup package.

The wrapper runs ``xacro`` on the manufacturer's URDF at launch time and
optionally appends auxiliary fixed-joint+link pairs (for example
``ft_sensor_link`` or ``compliance_link``) declared in
``config/robot_config.yaml``. The single augmented URDF is then passed
as a parameter to ``robot_state_publisher`` and ``controller_manager``
so every downstream consumer (RSP TF, MoveIt, FZI cartesian controllers,
dashboards, RViz) sees the same canonical kinematic tree.

This module deliberately depends only on the Python stdlib so it can be
unit-tested without a ROS context.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, List, Mapping, NamedTuple, Optional, Sequence


def run_xacro(xacro_path: str, xacro_args: Mapping[str, str]) -> str:
    """Run ``xacro`` synchronously and return the resulting URDF string.

    Parameters
    ----------
    xacro_path:
        Absolute path to the manufacturer's ``.urdf.xacro`` file.
    xacro_args:
        Mapping of ``name`` -> ``value`` passed to xacro as ``name:=value``.

    Raises
    ------
    RuntimeError:
        if ``xacro`` is not on ``PATH``.
    subprocess.CalledProcessError:
        if xacro exits non-zero (its stderr is propagated).
    """
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise RuntimeError(
            "xacro executable not found on PATH "
            "(apt install ros-<distro>-xacro or source the ROS environment)")

    cmd: List[str] = [xacro_bin, str(xacro_path)]
    for k, v in xacro_args.items():
        cmd.append(f"{k}:={v}")

    proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return proc.stdout


def augment_urdf(urdf_xml: str,
                 aux_frames: Optional[Iterable[Mapping]]) -> str:
    """Append fixed ``<joint>``/``<link>`` pairs to a URDF string.

    Each entry in ``aux_frames`` is a mapping with::

        {
          "name":   "ft_sensor_link",
          "parent": "tool0",
          "xyz":    [0.0, 0.0, 0.0],   # meters
          "rpy":    [0.0, 0.0, 0.0],   # radians
        }

    ``xyz`` and ``rpy`` default to zero if omitted.

    If ``aux_frames`` is ``None`` or empty the function returns
    ``urdf_xml`` UNCHANGED (no XML round-trip), so callers get a
    byte-identical result to the manufacturer's URDF when no aux frames
    are configured.
    """
    frames = list(aux_frames or [])
    if not frames:
        return urdf_xml

    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(
            f"expected URDF root tag <robot>, got <{root.tag}>")

    existing_links = {
        link.get("name") for link in root.findall("link") if link.get("name")
    }

    for entry in frames:
        name = entry["name"]
        parent = entry["parent"]
        if name in existing_links:
            raise ValueError(
                f"aux_frame '{name}' already exists as a <link> in the URDF")
        xyz = _xyz_or_rpy(entry.get("xyz"))
        rpy = _xyz_or_rpy(entry.get("rpy"))

        ET.SubElement(root, "link", {"name": name})
        joint = ET.SubElement(root, "joint", {
            "name": f"{parent}_to_{name}",
            "type": "fixed",
        })
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": name})
        ET.SubElement(joint, "origin", {
            "xyz": " ".join(f"{v:g}" for v in xyz),
            "rpy": " ".join(f"{v:g}" for v in rpy),
        })
        existing_links.add(name)

    return ET.tostring(root, encoding="unicode")


def update_aux_frames(urdf_xml: str,
                      aux_frames: Optional[Iterable[Mapping]]
                      ) -> "UpdateResult":
    """Edit existing aux-frame ``<origin>`` elements in a URDF *in place*.

    Unlike :func:`augment_urdf`, this function does NOT add new joints
    or links: it walks an already-augmented URDF and rewrites the
    ``xyz`` / ``rpy`` attributes on the ``<origin>`` element of the
    fixed joint whose ``<child>`` link matches ``entry["name"]``.

    The use case is live tool-frame updates from the dashboard: the
    operator tweaks an offset, the dashboard rebuilds the URDF, and
    pushes it to ``robot_state_publisher`` via SetParameters so the
    static TF tree refreshes without restarting the launch.

    Returns an :class:`UpdateResult` describing which frames were
    rewritten (``updated``) and which were not found in the URDF
    (``missing``).  Frames that already match the requested values are
    still counted as ``updated`` (the rewrite is idempotent); the
    caller decides whether to send the new URDF based on its own
    bookkeeping.

    Parameters
    ----------
    urdf_xml:
        Current URDF string.  Must contain a ``<robot>`` root.
    aux_frames:
        Iterable of mappings ``{"name": str, "xyz": [3 floats],
        "rpy": [3 floats]}`` (``parent`` is ignored; lookup is by
        child link name only).  ``xyz`` / ``rpy`` default to zero.

    Raises
    ------
    ValueError:
        if the URDF root is not ``<robot>``.
    """
    frames = list(aux_frames or [])
    updated: List[str] = []
    missing: List[str] = []
    if not frames:
        return UpdateResult(urdf_xml, updated, missing)

    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(
            f"expected URDF root tag <robot>, got <{root.tag}>")

    # Index fixed joints by child-link name -- that is the schema
    # ``augment_urdf`` produced (``<joint type="fixed"><child link=name/>
    # <origin .../>``).  Non-fixed joints with the same child link are
    # not aux frames and are skipped.
    by_child: Dict[str, ET.Element] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        child = joint.find("child")
        if child is None:
            continue
        child_name = child.get("link")
        if child_name:
            by_child[child_name] = joint

    for entry in frames:
        name = str(entry["name"])
        joint = by_child.get(name)
        if joint is None:
            missing.append(name)
            continue
        xyz = _xyz_or_rpy(entry.get("xyz"))
        rpy = _xyz_or_rpy(entry.get("rpy"))
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", " ".join(f"{v:g}" for v in xyz))
        origin.set("rpy", " ".join(f"{v:g}" for v in rpy))
        updated.append(name)

    return UpdateResult(ET.tostring(root, encoding="unicode"),
                        updated, missing)


class UpdateResult(NamedTuple):
    urdf_xml: str
    updated: List[str]
    missing: List[str]


def _xyz_or_rpy(value: Optional[Sequence[float]]) -> List[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    vals = list(value)
    if len(vals) != 3:
        raise ValueError(f"expected 3 numbers, got {vals!r}")
    return [float(v) for v in vals]
