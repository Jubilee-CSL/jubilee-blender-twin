"""
virtual_scanner.py
Generate a synthetic image dataset by driving the Jubilee digital twin over a
grid of (toolhead_x, toolhead_y, bed_z) machine positions and rendering one
image per point through a calibrated pinhole camera parented to the toolhead.

Inspired by https://github.com/romi/blender_virtual_scanner. Intended for
Marigold / 3DGS-MCMC style pipelines: images come out undistorted (Blender's
pinhole camera has no Brown-Conrady distortion), so the accompanying
camera.yaml records the calibration used *upstream* of the render, not applied.
"""

from __future__ import annotations

import json
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable

import bpy


# ── Reference-dataset defaults (matches images_justin/) ─────────────────────
# Filename convention: img_x{X}_y{Y}_z{Z}.jpg with integer millimetres, where
# (x, y) is the toolhead XY position and z is the bed height.
FILENAME_RE = re.compile(r"img_x(-?\d+)_y(-?\d+)_z(-?\d+)\.(?:jpg|jpeg|png)$", re.IGNORECASE)
FILENAME_TEMPLATE = "img_x{x}_y{y}_z{z}.jpg"

DEFAULT_X_MIN, DEFAULT_X_MAX, DEFAULT_X_STEPS = 110.0, 250.0, 5
DEFAULT_Y_MIN, DEFAULT_Y_MAX, DEFAULT_Y_STEPS = 80.0, 200.0, 5
DEFAULT_Z_MIN, DEFAULT_Z_MAX, DEFAULT_Z_STEPS = 280.0, 320.0, 3

DEFAULT_IMAGE_WIDTH = 1920
DEFAULT_IMAGE_HEIGHT = 1056

CAMERA_NAME = "Toolhead_Cam"


def _load_camera_params() -> dict:
    """Load intrinsics from jubilee_paths.json; fall back to defaults/camera_params.yaml."""
    addon_dir = Path(__file__).parent
    sys.path.insert(0, str(addon_dir))
    from camera_params import load_camera_params  # type: ignore
    try:
        return load_camera_params(_twin_root())
    except RuntimeError:
        # Before setup-scene has run, pull straight from the bundled fallback yaml.
        import importlib
        cp = importlib.import_module("camera_params")
        return cp._load_fallback_yaml()


# ── Small utilities ────────────────────────────────────────────────────────

def _twin_root() -> Path:
    from . import scene_utils
    return scene_utils.twin_root()


def _ensure_pipeline_on_path() -> None:
    from . import scene_utils
    scene_utils.ensure_pipeline_on_path()


def _drive_to_mm(x_mm: float, y_mm: float, z_mm: float) -> None:
    """Thin wrapper kept for callers that import this name directly."""
    _ensure_pipeline_on_path()
    from . import scene_utils
    scene_utils.drive_to_mm(x_mm, y_mm, z_mm)


def _infer_range(values: list[int]) -> tuple[float, float, int]:
    """Given a sorted unique list of ints, return (min, max, len)."""
    uniq = sorted(set(values))
    if not uniq:
        return 0.0, 0.0, 1
    return float(uniq[0]), float(uniq[-1]), len(uniq)


def scan_reference_folder(folder: Path) -> dict | None:
    """Parse img_x*_y*_z* filenames and derive per-axis (min, max, steps).

    Returns None if the folder is missing or no matching files are found.
    """
    if not folder.is_dir():
        return None
    xs, ys, zs = [], [], []
    for name in os.listdir(folder):
        m = FILENAME_RE.match(name)
        if not m:
            continue
        xs.append(int(m.group(1)))
        ys.append(int(m.group(2)))
        zs.append(int(m.group(3)))
    if not xs:
        return None
    return {
        "x": _infer_range(xs),
        "y": _infer_range(ys),
        "z": _infer_range(zs),
        "count": len(xs),
    }


def _linspace(a: float, b: float, n: int) -> list[float]:
    if n <= 1:
        return [a]
    step = (b - a) / (n - 1)
    return [a + i * step for i in range(n)]


# ── Scene wiring ───────────────────────────────────────────────────────────

def _get_axis_objects() -> tuple[bpy.types.Object, bpy.types.Object, bpy.types.Object]:
    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    z_axis = bpy.data.objects.get("Z-axis")
    if x_axis is None or y_axis is None or z_axis is None:
        raise RuntimeError(
            "Scene is missing one of 'X-axis', 'Y-axis', 'Z-axis'. "
            "Open jubilee.blend and run 'jubilee-twin setup-scene' first."
        )
    return x_axis, y_axis, z_axis


# ── Scan driver ────────────────────────────────────────────────────────────

@dataclass
class ScanConfig:
    x_min: float
    x_max: float
    x_steps: int
    y_min: float
    y_max: float
    y_steps: int
    z_min: float
    z_max: float
    z_steps: int
    width: int = DEFAULT_IMAGE_WIDTH
    height: int = DEFAULT_IMAGE_HEIGHT
    camera: dict = field(default_factory=_load_camera_params)
    output_root: Path | None = None

    def points(self) -> Iterable[tuple[float, float, float]]:
        xs = _linspace(self.x_min, self.x_max, self.x_steps)
        ys = _linspace(self.y_min, self.y_max, self.y_steps)
        zs = _linspace(self.z_min, self.z_max, self.z_steps)
        for x in xs:
            for y in ys:
                for z in zs:
                    yield x, y, z


def _timestamped_output_dir(root: Path | None) -> Path:
    root = root or (_twin_root() / "Scans")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = root / stamp
    out.mkdir(parents=True, exist_ok=True)
    return out


def _write_camera_yaml(out_dir: Path, cfg: ScanConfig) -> None:
    """Emit camera intrinsics next to the images for downstream pipelines."""
    cam = cfg.camera
    lines = [
        "camera:",
        f"  cx: {cam.get('cx', '')}",
        f"  cy: {cam.get('cy', '')}",
        "  dist:",
    ]
    for d in cam.get("dist", []):
        lines.append(f"  - {d}")
    lines += [
        f"  fx: {cam.get('fx', '')}",
        f"  fy: {cam.get('fy', '')}",
        "  offset:",
    ]
    for v in cam.get("offset", []):
        lines.append(f"  - {v}")
    lines += [
        "image:",
        f"  width: {cfg.width}",
        f"  height: {cfg.height}",
        "  distortion_applied: false",
    ]
    (out_dir / "camera.yaml").write_text("\n".join(lines) + "\n")


def _write_manifest(out_dir: Path, cfg: ScanConfig, rendered: list[dict]) -> None:
    manifest = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "grid": {
            "x": {"min": cfg.x_min, "max": cfg.x_max, "steps": cfg.x_steps},
            "y": {"min": cfg.y_min, "max": cfg.y_max, "steps": cfg.y_steps},
            "z": {"min": cfg.z_min, "max": cfg.z_max, "steps": cfg.z_steps},
        },
        "image": {"width": cfg.width, "height": cfg.height},
        "camera": cfg.camera,
        "frames": rendered,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))


def run_scan(cfg: ScanConfig) -> Path:
    """Drive the scene through cfg.points() and render one image per point.

    Expects Toolhead_Cam to already exist (run `jubilee-twin place-camera` first).
    The camera follows the carriage automatically via direct parenting to the plate.
    Returns the output directory (Scans/YYYYmmdd_HHMMSS/).
    """
    _ensure_pipeline_on_path()

    scene = bpy.context.scene
    _get_axis_objects()  # validate early — _drive_to_mm will use them per frame

    cam_obj = bpy.data.objects.get(CAMERA_NAME)
    if cam_obj is None:
        raise RuntimeError(
            f"{CAMERA_NAME!r} not found. Run `jubilee-twin place-camera` first."
        )
    scene.camera = cam_obj

    out_dir = _timestamped_output_dir(cfg.output_root)
    _write_camera_yaml(out_dir, cfg)

    scene.render.image_settings.file_format = 'JPEG'
    scene.render.image_settings.quality = 92

    rendered: list[dict] = []
    total = cfg.x_steps * cfg.y_steps * cfg.z_steps
    print(f"[virtual_scanner] Rendering {total} frames → {out_dir}")

    for i, (x_mm, y_mm, z_mm) in enumerate(cfg.points(), start=1):
        _drive_to_mm(x_mm, y_mm, z_mm)

        fname = FILENAME_TEMPLATE.format(x=int(round(x_mm)), y=int(round(y_mm)), z=int(round(z_mm)))
        scene.render.filepath = str(out_dir / fname)

        bpy.ops.render.render(write_still=True)

        depsgraph = bpy.context.evaluated_depsgraph_get()
        cam_eval = cam_obj.evaluated_get(depsgraph)
        mw = cam_eval.matrix_world
        rendered.append({
            "index": i,
            "file": fname,
            "machine_mm": {"x": x_mm, "y": y_mm, "z": z_mm},
            "camera_world_m": list(mw.translation),
        })
        print(f"[virtual_scanner] {i}/{total}: {fname}")

    _write_manifest(out_dir, cfg, rendered)
    print(f"[virtual_scanner] Done. Output: {out_dir}")
    return out_dir
