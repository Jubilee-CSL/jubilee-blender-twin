"""
workshop_utils.py
Plumbing for the digital-twin workshop notebook: launching headless Blender,
running the twin's collision check on a G-code plan, and reading/writing the
files the exercises exchange. The ideas the exercises teach live in the
notebook itself; this module only hides the subprocess and file handling.
"""

from __future__ import annotations

import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

WORKSHOP_DIR = Path(__file__).resolve().parent
TWIN_DIR = WORKSHOP_DIR.parent
PIPELINE_DIR = TWIN_DIR / "pipeline_data"
ADDON_DIR = TWIN_DIR / "blender_addon" / "jubilee_digital_twin"
BLENDER_OPS = WORKSHOP_DIR / "twin_ops_blender.py"
OUT_DIR = WORKSHOP_DIR / "out"

BLENDER_EXE = os.environ.get("JUBILEE_BLENDER_EXE", "blender")

PLANTS = {
    "basil": TWIN_DIR / "blender_models" / "basil_3D.blend",
    "monstera": TWIN_DIR / "blender_models" / "plant_3D.blend",
}


def _run_blender(blend: Path, args: list[str], verbose: bool = False) -> str:
    """Run Blender headless on `blend` with extra CLI args; return its stdout."""
    cmd = [BLENDER_EXE, str(blend), "--background", *args]
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if verbose or proc.returncode != 0:
        print(proc.stdout[-4000:])
        print(proc.stderr[-4000:])
    if proc.returncode != 0:
        raise RuntimeError(f"Blender exited with code {proc.returncode}")
    print(f"  (Blender finished in {time.time() - t0:.0f} s)")
    return proc.stdout


# ── scene ──────────────────────────────────────────────────────────────────

def build_scene(plant: str = "basil", force: bool = False) -> Path:
    """Copy of the working twin scene with a plant on the bed.

    Saved in pipeline_data/ because the twin's animation and collision scripts
    find pathout.csv relative to the .blend.
    """
    out = PIPELINE_DIR / f"workshop_{plant}.blend"
    if out.exists() and not force:
        print(f"Scene already built: {out.name}")
        return out
    working = PIPELINE_DIR / "jubilee_working.blend"
    if not working.exists():
        raise FileNotFoundError("Run `jubilee-twin setup-scene` first to create jubilee_working.blend")
    _run_blender(working, ["--python", str(BLENDER_OPS), "--",
                           "build", "--plant-blend", str(PLANTS[plant]), "--out", str(out)])
    return out


def open_in_blender(blend: Path) -> None:
    """Open a scene in the Blender GUI (does not block the notebook)."""
    subprocess.Popen([BLENDER_EXE, str(blend)])


# ── rendering ──────────────────────────────────────────────────────────────

def render(scene: Path, jobs: list[dict], out_dir: Path, percent: int = 25) -> list[dict]:
    """Render a list of jobs in one Blender session.

    Each job is a dict with at least name, x, y, z (machine mm). Optional keys:
    hdri, strength, rotation_deg, plain_background [r, g, b],
    fstop + focus_distance (depth of field), ground_truth (bool).
    Returns one metadata dict per job (image path, K, ...).
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    jobs_file = out_dir / "jobs.json"
    jobs_file.write_text(json.dumps({"out_dir": str(out_dir), "percent": percent, "jobs": jobs}, indent=2))
    print(f"Rendering {len(jobs)} image(s) -> {out_dir}")
    _run_blender(scene, ["--python", str(BLENDER_OPS), "--", "render", "--jobs", str(jobs_file)])
    return [json.loads((out_dir / f"{j['name']}.json").read_text()) for j in jobs]


def load_image(path) -> np.ndarray:
    from PIL import Image
    return np.asarray(Image.open(path).convert("RGB"))


# ── collision check (Exercise 1) ───────────────────────────────────────────

_HIT = re.compile(r"Collision between gantry and (.+) during frame (\d+)")


def _write_path(lines: list[str], step_mm: float) -> np.ndarray:
    """G-code -> pathout.csv, starting at the plan's first position.

    The twin's parser always starts at home (0, 0, 0). On the real machine the
    plant is loaded with the bed already lowered, so the move from home to the
    first commanded position is not part of the plan and is dropped.
    """
    from jubilee_twin.pipeline.path_follower import build_path

    with contextlib.redirect_stdout(io.StringIO()):  # the parser prints debug lines
        path = np.array(build_path(lines, step_mm))
        first = next(l for l in lines if l.strip().upper().startswith(("G0", "G1")))
        n_first = len(build_path([first], step_mm)) - 1
    path = path[n_first:]
    np.savetxt(PIPELINE_DIR / "pathout.csv", path, delimiter=",", fmt="%.4f")
    return path


def check_plan(gcode: str, scene: Path, step_mm: float = 10.0) -> list[dict]:
    """Animate a G-code plan in the twin and ray-trace it for collisions.

    Uses the twin's own animate_path.py and ray_tracing.py on a throw-away copy
    of the scene. Returns one dict per collision: frame, object, machine x/y/z.
    """
    lines = [l for l in gcode.splitlines() if l.strip() and not l.strip().startswith(";")]
    path = _write_path(lines, step_mm)
    print(f"Plan: {len(lines)} G-code lines -> {len(path)} animation frames")

    check_blend = PIPELINE_DIR / "_workshop_check.blend"
    shutil.copy2(scene, check_blend)
    stdout = _run_blender(check_blend, ["--python", str(ADDON_DIR / "animate_path.py"),
                                        "--python", str(ADDON_DIR / "ray_tracing.py")])
    hits = []
    for obj, frame in _HIT.findall(stdout):
        frame = int(frame)
        x, y, z = path[frame - 1][:3]
        hits.append({"frame": frame, "object": obj, "x": float(x), "y": float(y), "z": float(z)})
    return hits


# ── point clouds (Exercise 4) ──────────────────────────────────────────────

def read_ply(path) -> np.ndarray:
    """XYZ of every vertex in a .ply file (ascii or binary)."""
    from plyfile import PlyData
    v = PlyData.read(str(path))["vertex"]
    return np.stack([v["x"], v["y"], v["z"]], axis=-1).astype(np.float64)


def write_ply(path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    """Write points (N, 3) and optional uint8 colors (N, 3) as a binary .ply."""
    from plyfile import PlyData, PlyElement
    fields = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    if colors is not None:
        fields += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    data = np.empty(len(points), dtype=fields)
    data["x"], data["y"], data["z"] = points.T
    if colors is not None:
        data["red"], data["green"], data["blue"] = colors.T
    PlyData([PlyElement.describe(data, "vertex")]).write(str(path))
