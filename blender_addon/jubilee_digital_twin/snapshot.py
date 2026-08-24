"""Render a single frame through Toolhead_Cam — equivalent of F12."""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import bpy


def _twin_root() -> Path:
    return Path(os.path.dirname(os.path.dirname(bpy.data.filepath)))


def _get_camera() -> bpy.types.Object:
    cam = bpy.data.objects.get("Toolhead_Cam")
    if cam is None:
        raise RuntimeError(
            "Toolhead_Cam not found. Click 'Place Camera' in the Setup panel first."
        )
    return cam


def take_snapshot(output: Path | str | None = None) -> Path:
    """Render the current frame through Toolhead_Cam and return the saved path."""
    cam = _get_camera()
    scene = bpy.context.scene
    scene.camera = cam

    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = _twin_root() / "Scans" / "snapshots"
        out_dir.mkdir(parents=True, exist_ok=True)
        output = out_dir / f"{stamp}_snapshot.jpg"

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.image_settings.file_format = 'JPEG'
    scene.render.image_settings.quality = 92
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    print(f"[snapshot] Saved: {output}")
    return output


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float, default=None)
    p.add_argument("--y", type=float, default=None)
    p.add_argument("--z", type=float, default=None)
    p.add_argument("--output", type=Path, default=None)
    args = p.parse_args(argv)

    if args.x is not None and args.y is not None and args.z is not None:
        addon_dir = str(Path(__file__).parent)
        if addon_dir not in sys.path:
            sys.path.insert(0, addon_dir)
        import virtual_scanner as vs  # shared axis-driving logic
        vs._drive_to_mm(args.x, args.y, args.z)

    take_snapshot(output=args.output)
