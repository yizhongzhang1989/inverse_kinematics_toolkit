"""Access to the sample URDFs bundled inside this package.

These small, mesh-free URDFs let you try the library, run the test-suite and
write examples with no external robot description::

    from ikt_core import IK, assets

    print(assets.list_sample_urdfs())          # ['arm_6dof', 'dual_arm', ...]
    ik = IK.from_urdf_file(assets.sample_urdf_path("srs_7dof"))

Resolution uses :mod:`importlib.resources`, so it works whether the package is
imported from source, pip-installed, or installed into a colcon/ament prefix.
"""

from __future__ import annotations

from pathlib import Path
from typing import List

try:  # Python 3.9+: files() API
    from importlib.resources import files as _files
except Exception:  # pragma: no cover
    _files = None

_URDF_SUBDIR = "urdf"


def _urdf_dir() -> Path:
    """Directory holding the bundled ``*.urdf`` files."""
    if _files is not None:
        try:
            return Path(str(_files("ikt_core").joinpath(_URDF_SUBDIR)))
        except Exception:  # pragma: no cover
            pass
    return Path(__file__).resolve().parent / _URDF_SUBDIR


def list_sample_urdfs() -> List[str]:
    """Sorted names (without ``.urdf``) of the bundled sample URDFs."""
    d = _urdf_dir()
    if not d.is_dir():
        return []
    return sorted(p.stem for p in d.glob("*.urdf"))


def sample_urdf_path(name: str) -> str:
    """Absolute path to a bundled URDF, by name with or without ``.urdf``.

    Raises
    ------
    FileNotFoundError:
        if no sample URDF matches ``name`` (the message lists the available
        names).
    """
    stem = name[:-5] if name.endswith(".urdf") else name
    path = _urdf_dir() / f"{stem}.urdf"
    if not path.is_file():
        raise FileNotFoundError(
            f"no bundled sample URDF named {name!r}; available: "
            f"{', '.join(list_sample_urdfs()) or '(none installed)'}")
    return str(path)


def load_sample_urdf(name: str) -> str:
    """Return the URDF XML string of a bundled sample by name."""
    return Path(sample_urdf_path(name)).read_text()
