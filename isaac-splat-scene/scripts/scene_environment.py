"""Deployment-only helpers for Isaac's Python and bundled USD libraries."""

import os
from pathlib import Path
import sys

_dll_handles: list[object] = []


def isaac_root() -> Path:
    value = os.environ.get("ISAAC_SIM_ROOT")
    if not value:
        raise RuntimeError("Set ISAAC_SIM_ROOT to the Isaac Sim installation directory")
    root = Path(value).expanduser().resolve()
    if not (root / "apps" / "isaacsim.exp.full.kit").is_file():
        raise RuntimeError(f"Not an Isaac Sim installation: {root}")
    return root


def add_usd_runtime() -> None:
    """Allow authoring scripts to run with Isaac's bundled pxr or a system pxr."""
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    root = isaac_root()
    candidates = sorted((root / "extscache").glob("omni.usd.libs-*/"))
    if not candidates:
        raise RuntimeError(f"No omni.usd.libs extension found under {root / 'extscache'}")
    usd_dir = candidates[-1]
    sys.path.insert(0, str(usd_dir))
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        _dll_handles.append(os.add_dll_directory(str(usd_dir / "bin")))
