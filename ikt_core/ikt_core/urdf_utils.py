"""URDF loading + augmentation utilities (pure stdlib, ROS-free).

This module makes ``ikt_inverse_kinematics`` usable as a plain Python library:
it reads a URDF from a file, a ``.xacro`` file, stdin or a raw XML string, and
provides the virtual-tool-frame augmentation the solver needs — all without
importing ``rclpy`` or depending on a colcon workspace.

The :func:`augment_urdf` helper is vendored here (it was previously imported
from the separate ``ikt_common`` package) so the IK package is self-contained
when pip-installed. ``ikt_common`` remains the home of the launch-time config
loader used by the ROS layer.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Sequence, Union

__all__ = ["read_urdf", "run_xacro", "augment_urdf"]


def _looks_like_xml(text: str) -> bool:
    """True if ``text`` is a URDF/XML document rather than a path."""
    s = text.lstrip()
    return s.startswith("<")


def run_xacro(xacro_path: Union[str, Path],
              mappings: Optional[Mapping[str, str]] = None) -> str:
    """Run ``xacro`` synchronously and return the resulting URDF string.

    Parameters
    ----------
    xacro_path:
        Path to a ``.urdf.xacro`` / ``.xacro`` file.
    mappings:
        Optional ``name -> value`` pairs passed to xacro as ``name:=value``.

    Raises
    ------
    RuntimeError:
        if the ``xacro`` executable is not on ``PATH``.
    subprocess.CalledProcessError:
        if xacro exits non-zero (its stderr is propagated).
    """
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise RuntimeError(
            "xacro executable not found on PATH. Install it "
            "(apt install ros-<distro>-xacro) and source your ROS environment, "
            "or pass a plain .urdf file / URDF string instead.")
    cmd: List[str] = [xacro_bin, str(xacro_path)]
    for k, v in (mappings or {}).items():
        cmd.append(f"{k}:={v}")
    proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return proc.stdout


def read_urdf(source: Union[str, Path],
              mappings: Optional[Mapping[str, str]] = None) -> str:
    """Return a URDF XML string from a variety of sources.

    ``source`` may be:

    * a raw URDF/XML string (returned as-is);
    * ``"-"`` to read XML from stdin;
    * a path to a ``.xacro`` / ``.urdf.xacro`` file (run through ``xacro``);
    * a path to a ``.urdf`` (or any other) file (read verbatim). If the file
      content begins with ``<?xml`` and contains xacro directives but has no
      ``.xacro`` suffix it is still read verbatim — pass a ``.xacro`` path to
      force xacro processing.

    Parameters
    ----------
    source:
        File path, ``"-"`` for stdin, or a raw URDF string.
    mappings:
        Optional xacro ``name:=value`` arguments (only used when xacro runs).
    """
    if isinstance(source, Path):
        source = str(source)
    if source == "-":
        import sys
        return sys.stdin.read()
    # Raw XML string?  (Only treat as XML if it is clearly markup, so short
    # path-like strings are still treated as paths.)
    if _looks_like_xml(source):
        return source
    path = Path(source).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"URDF source not found: {source!r}")
    if path.suffix == ".xacro" or path.name.endswith(".urdf.xacro"):
        return run_xacro(path, mappings)
    return path.read_text()


def _xyz_or_rpy(value: Optional[Sequence[float]]) -> List[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    vals = list(value)
    if len(vals) != 3:
        raise ValueError(f"expected 3 numbers, got {vals!r}")
    return [float(v) for v in vals]


def augment_urdf(urdf_xml: str,
                 aux_frames: Optional[Iterable[Mapping]]) -> str:
    """Append fixed ``<joint>``/``<link>`` pairs to a URDF string.

    Each entry in ``aux_frames`` is a mapping::

        {"name": "tool0", "parent": "wrist_link",
         "xyz": [0, 0, 0.1], "rpy": [0, 0, 0]}

    ``xyz`` and ``rpy`` default to zero if omitted. If ``aux_frames`` is
    ``None`` or empty the URDF is returned UNCHANGED (no XML round-trip), so
    callers get a byte-identical result when no aux frames are configured.

    Raises
    ------
    ValueError:
        if the URDF root is not ``<robot>``, or an aux-frame name collides with
        an existing link.
    """
    frames = list(aux_frames or [])
    if not frames:
        return urdf_xml

    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected URDF root tag <robot>, got <{root.tag}>")

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
