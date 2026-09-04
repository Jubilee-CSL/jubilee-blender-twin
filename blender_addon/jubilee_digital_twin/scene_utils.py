"""Shared Blender scene helpers — twin_root, sys.path, jubilee_paths.json, axis positioning.

Import from here instead of repeating boilerplate in every addon script.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path
import bpy


def twin_root() -> Path:
    """Repository root derived from the currently open .blend file."""
    return Path(bpy.data.filepath).parent.parent


def ensure_pipeline_on_path() -> None:
    """Add twin_root to sys.path so jubilee_twin.* imports work inside Blender."""
    root = twin_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))


# keep private alias so internal callers don't need updating
_ensure_pipeline_on_path = ensure_pipeline_on_path


def read_paths_cache() -> dict:
    """Read pipeline_data/jubilee_paths.json; return {} if not found."""
    cache_path = twin_root() / "pipeline_data" / "jubilee_paths.json"
    if cache_path.is_file():
        with open(cache_path) as f:
            return json.load(f)
    return {}


def get_axis_objects() -> tuple[bpy.types.Object, bpy.types.Object, bpy.types.Object]:
    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    z_axis = bpy.data.objects.get("Z-axis")
    if x_axis is None or y_axis is None or z_axis is None:
        raise RuntimeError(
            "Scene is missing 'X-axis', 'Y-axis', or 'Z-axis'. "
            "Run 'jubilee-twin setup-scene' first."
        )
    return x_axis, y_axis, z_axis


def drive_to_mm(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    frame: int | None = None,
) -> None:
    """Move X/Y/Z axis objects to the given machine position (mm).

    Pass frame= to also insert keyframes (used by animate_path).
    """
    _ensure_pipeline_on_path()
    from jubilee_twin.pipeline.utils import get_axis_max  # type: ignore
    x_axis, y_axis, z_axis = get_axis_objects()
    x_axis.location.x = get_axis_max(x_axis, "X") - x_mm / 1000.0
    y_axis.location.y = get_axis_max(y_axis, "Y") - y_mm / 1000.0
    z_axis.location.z = get_axis_max(z_axis, "Z") - z_mm / 1000.0
    if frame is not None:
        x_axis.keyframe_insert(data_path="location", frame=frame)
        y_axis.keyframe_insert(data_path="location", frame=frame)
        z_axis.keyframe_insert(data_path="location", frame=frame)
    else:
        bpy.context.view_layer.update()


def read_mm() -> tuple[float, float, float]:
    """Inverse of drive_to_mm: machine (X, Y, Z) in mm from the current axis empties."""
    _ensure_pipeline_on_path()
    from jubilee_twin.pipeline.utils import get_axis_max  # type: ignore
    x_axis, y_axis, z_axis = get_axis_objects()
    return (
        (get_axis_max(x_axis, "X") - x_axis.location.x) * 1000.0,
        (get_axis_max(y_axis, "Y") - y_axis.location.y) * 1000.0,
        (get_axis_max(z_axis, "Z") - z_axis.location.z) * 1000.0,
    )
