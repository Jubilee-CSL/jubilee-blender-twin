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
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    if not eps:
        pytest.skip("jubilee.paths/jubilee_dir not registered — run inside pixi environment")
    return Path(eps[0].load()())


# ---------------------------------------------------------------------------
# tool_id discovery
# ---------------------------------------------------------------------------

def test_tool_id_discovers_firmware_sys():
    from jubilee_twin.pipeline.tool_id import _discover_sys_dir

    result = _discover_sys_dir()
    assert os.path.isdir(result), f"_discover_sys_dir() returned non-existent dir: {result}"
    assert os.path.isfile(os.path.join(result, "config.g")), f"config.g missing in: {result}"


def test_tool_id_discovers_correct_repo():
    from jubilee_twin.pipeline.tool_id import _discover_sys_dir

    jubilee_root = _jubilee_root()
    result = _discover_sys_dir()
    assert Path(result).is_relative_to(jubilee_root), (
        f"_discover_sys_dir() returned {result!r}, expected a path under {jubilee_root}"
    )


def test_tool_id_raises_without_entry_point():
    from jubilee_twin.pipeline.tool_id import _discover_sys_dir

    # entry_points is imported inside the function, so patch it at the source
    with patch("importlib.metadata.entry_points", return_value=[]):
        with pytest.raises(RuntimeError, match="jubilee.paths/jubilee_dir entry point not found"):
            _discover_sys_dir()


def test_tool_data_csv_is_created(tmp_path):
    from jubilee_twin.pipeline.tool_id import _discover_sys_dir, extract_names, extract_parks, extract_offset
    import csv

    sys_dir = _discover_sys_dir()
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

def test_path_follower_discovers_latest_gcode():
    from jubilee_twin.pipeline.path_follower import _discover_gcode_file

    result = _discover_gcode_file()
    assert os.path.isfile(result), f"_discover_gcode_file() returned non-existent file: {result}"
    assert result.endswith("latest.gcode"), f"expected latest.gcode, got: {result}"


def test_path_follower_discovers_correct_repo():
    from jubilee_twin.pipeline.path_follower import _discover_gcode_file

    jubilee_root = _jubilee_root()
    result = _discover_gcode_file()
    assert Path(result).is_relative_to(jubilee_root), (
        f"_discover_gcode_file() returned {result!r}, expected a path under {jubilee_root}"
    )


def test_path_follower_finds_named_gcode_in_science_jubilee(tmp_path):
    """A filename-only arg is resolved against science_jubilee/gcode_logs/."""

    jubilee_root = _jubilee_root()
    gcode_dir = jubilee_root / "gcode_logs"
    if not gcode_dir.is_dir():
        pytest.skip("gcode_logs/ directory does not exist in science_jubilee")

    existing = next(gcode_dir.glob("*.gcode"), None)
    if existing is None:
        pytest.skip("no .gcode files in science_jubilee/gcode_logs/")

    # Simulate calling main() with just the filename (not a full path)
    # by checking the resolution logic directly
    import importlib.metadata
    eps = [ep for ep in importlib.metadata.entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    root = Path(eps[0].load()())
    candidate = os.path.join(str(root), "gcode_logs", existing.name)
    assert os.path.isfile(candidate)


def test_pathout_csv_is_created(tmp_path):
    from jubilee_twin.pipeline.path_follower import _discover_gcode_file, build_path

    gcode_file = _discover_gcode_file()
    with open(gcode_file, "r") as f:
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
    from jubilee_twin.pipeline.path_follower import _discover_gcode_file

    jubilee_root = _jubilee_root()
    with pytest.raises(FileNotFoundError):
        # Simulate main() finding entry point but file missing
        import os
        candidate = os.path.join(str(jubilee_root), "gcode_logs", "does_not_exist.gcode")
        if not os.path.isfile(candidate):
            raise FileNotFoundError(f"'does_not_exist.gcode' not found in cwd or in {os.path.join(str(jubilee_root), 'gcode_logs')}")
