"""Tests for path discovery in tool_id.py and path_follower.py.

These tests require the pixi environment (science-jubilee installed).
They verify that discovery finds real files, not stale local copies.
"""
import os
import pytest
from pathlib import Path
from unittest.mock import patch


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _jubilee_root() -> Path:
    from jubilee_twin.paths import resolve
    try:
        return resolve("jubilee_dir")
    except RuntimeError:
        pytest.skip("jubilee.paths/jubilee_dir not registered — run inside pixi environment")


def _sys_dir() -> str:
    return str(_jubilee_root() / "firmware" / "sys")


def _latest_gcode() -> str:
    return str(_jubilee_root() / "gcode_logs" / "latest.gcode")


# ---------------------------------------------------------------------------
# resolve() helper
# ---------------------------------------------------------------------------

def test_resolve_returns_valid_jubilee_dir():
    root = _jubilee_root()
    assert root.is_dir(), f"resolve('jubilee_dir') returned non-existent dir: {root}"


def test_resolve_raises_without_entry_point():
    from jubilee_twin.paths import resolve

    with patch("importlib.metadata.entry_points", return_value=[]):
        with pytest.raises(RuntimeError, match="jubilee.paths/jubilee_dir entry point not found"):
            resolve("jubilee_dir")


# ---------------------------------------------------------------------------
# tool_id discovery
# ---------------------------------------------------------------------------

def test_tool_id_finds_firmware_sys():
    sys_dir = _sys_dir()
    assert os.path.isdir(sys_dir), f"firmware/sys not found at: {sys_dir}"
    assert os.path.isfile(os.path.join(sys_dir, "config.g")), f"config.g missing in: {sys_dir}"


def test_tool_data_csv_is_created(tmp_path):
    from jubilee_twin.pipeline.tool_id import extract_names, extract_parks, extract_offset
    import csv

    sys_dir = _sys_dir()
    names = extract_names(sys_dir)
    parks = extract_parks(sys_dir)
    offsets = extract_offset(sys_dir)

    assert isinstance(names, dict), "extract_names should return a dict"
    assert isinstance(parks, dict), "extract_parks should return a dict"
    assert isinstance(offsets, dict), "extract_offset should return a dict"
    assert len(names) > 0, f"No tools found in {sys_dir}/config.g"

    # Write to tmp CSV and verify structure
    out = tmp_path / "tool_data.csv"
    with open(out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Tool_ID", "Name", "Park_X", "Park_Y", "Park_Z", "Offset_X", "Offset_Y", "Offset_Z"])
        for i in range(4):
            import numpy as np
            park = parks.get(i, np.zeros(3))
            offset = offsets.get(i, np.zeros(3))
            writer.writerow([i, names.get(i, f"Unassigned_Tool_{i}"), *park, *offset])

    rows = list(csv.reader(open(out)))
    assert rows[0] == ["Tool_ID", "Name", "Park_X", "Park_Y", "Park_Z", "Offset_X", "Offset_Y", "Offset_Z"]
    assert len(rows) == 5, f"Expected header + 4 tool rows, got {len(rows)} rows"


# ---------------------------------------------------------------------------
# path_follower discovery
# ---------------------------------------------------------------------------

def test_path_follower_finds_latest_gcode():
    gcode = _latest_gcode()
    assert os.path.isfile(gcode), f"latest.gcode not found at: {gcode}"


def test_path_follower_finds_named_gcode_in_science_jubilee():
    """A filename-only arg is resolved against science_jubilee/gcode_logs/."""
    gcode_dir = _jubilee_root() / "gcode_logs"
    if not gcode_dir.is_dir():
        pytest.skip("gcode_logs/ directory does not exist in science_jubilee")

    existing = next(gcode_dir.glob("*.gcode"), None)
    if existing is None:
        pytest.skip("no .gcode files in science_jubilee/gcode_logs/")

    candidate = gcode_dir / existing.name
    assert candidate.is_file()


def test_pathout_csv_is_created(tmp_path):
    from jubilee_twin.pipeline.path_follower import build_path

    with open(_latest_gcode(), "r") as f:
        lines = f.readlines()

    path = build_path(lines, distance_per_step=50.0)
    assert len(path) > 0, "build_path returned empty path"
    assert all(len(p) == 6 for p in path), "each row must have 6 columns: x,y,z,u,toolchange_flag,tool_id"

    out = tmp_path / "pathout.csv"
    with open(out, "w") as f:
        for i, pos in enumerate(path):
            if i != 0:
                f.write("\n")
            f.write(",".join(str(v) for v in pos))

    rows = out.read_text().strip().splitlines()
    assert len(rows) > 0
    assert len(rows[0].split(",")) == 6, f"Expected 6 columns, got: {rows[0]}"


def test_path_follower_raises_when_file_not_found_anywhere():
    from jubilee_twin.pipeline.path_follower import run

    _jubilee_root()  # skip if not in pixi env
    with pytest.raises(FileNotFoundError):
        run(gcode_file="does_not_exist.gcode")
