"""Workspace location helpers for the cartesian_controllers_toolkit.

Lets any code in the workspace find the consuming project root (the
directory that contains ``src/``, ``config/``, ``tools/``, etc.)
regardless of whether it's running from the source tree or from a
colcon ``install/`` overlay.

The project root is found by, in order:

  1. The ``ROBOT_WORKSPACE_ROOT`` environment variable (explicit
     override).
  2. Walking up from a known installed share dir (``ament_index``).
  3. Walking up from this file's own location (development case).
  4. ``COLCON_PREFIX_PATH`` / ``ROS_WORKSPACE``.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


# Files/directories that, when present together at the same level, identify
# the consuming workspace root.
_ROOT_MARKERS = ("src", "config")


def _looks_like_root(path: Path) -> bool:
    return all((path / m).is_dir() for m in _ROOT_MARKERS)


def get_workspace_root() -> Optional[str]:
    """Return the absolute path of the consuming workspace root, or None."""

    # 1. explicit env var override.
    env = os.environ.get("ROBOT_WORKSPACE_ROOT")
    if env:
        p = Path(env).expanduser().resolve()
        if _looks_like_root(p):
            return str(p)

    # 2. walk up from an installed package's share dir
    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in ("ikt_common",
                    "cartesian_control_manager",
                    "ft_sensor_gravity_compensation",
                    "ft_sensor_dashboard"):
            try:
                share = Path(get_package_share_directory(pkg)).resolve()
            except Exception:
                continue
            for parent in (share, *share.parents):
                if _looks_like_root(parent):
                    return str(parent)
    except Exception:
        pass

    # 3. walk up from this file's location (running from source)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if _looks_like_root(parent):
            return str(parent)

    # 4. COLCON_PREFIX_PATH / ROS_WORKSPACE
    for env_name in ("COLCON_PREFIX_PATH", "ROS_WORKSPACE"):
        val = os.environ.get(env_name)
        if not val:
            continue
        for piece in val.split(os.pathsep):
            p = Path(piece).expanduser().resolve()
            for parent in (p, *p.parents):
                if _looks_like_root(parent):
                    return str(parent)

    return None


def get_config_dir() -> str:
    """Return the absolute path of the project's ``config/`` directory.

    Raises ``RuntimeError`` if the workspace root can't be located.
    """
    root = get_workspace_root()
    if root is None:
        raise RuntimeError(
            "Could not find the consuming workspace root. "
            "Set ROBOT_WORKSPACE_ROOT or run from inside the project tree.")
    return str(Path(root) / "config")


def get_config_path(filename: str) -> str:
    """Return the absolute path of ``config/<filename>`` in the project.

    Raises ``RuntimeError`` if the workspace root can't be located.
    Does NOT check whether the file actually exists -- callers that need
    that guarantee should ``os.path.isfile(...)`` the result themselves.
    """
    return str(Path(get_config_dir()) / filename)


def get_temp_dir(create: bool = True) -> str:
    """Return the absolute path of the project's ``temp/`` directory.

    The directory is created on demand if ``create`` is true.
    """
    root = get_workspace_root()
    if root is None:
        raise RuntimeError(
            "Could not find the consuming workspace root. "
            "Set ROBOT_WORKSPACE_ROOT or run from inside the project tree.")
    p = Path(root) / "temp"
    if create:
        p.mkdir(parents=True, exist_ok=True)
    return str(p)
