"""CLI integration tests.

prepare: no Blender needed — fully verified.
animate/raytrace: require Blender on PATH — skipped if absent.
"""
import shutil
import subprocess
import sys
import csv
from pathlib import Path
import pytest


def _jubilee_root() -> Path:
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    if not eps:
        pytest.skip("jubilee.paths/jubilee_dir not registered — run inside virtual environment with science-jubilee installed")
    return Path(eps[0].load()())


def _latest_gcode() -> str:
    root = _jubilee_root()
    gcode = root / "gcode_logs" / "latest.gcode"
    if not gcode.exists():
        pytest.skip(f"latest.gcode not found at {gcode}")
    return str(gcode)


def _blender_available() -> bool:
    return shutil.which("blender") is not None


# ---------------------------------------------------------------------------
# prepare
# ---------------------------------------------------------------------------

def test_prepare_creates_csvs(tmp_path):
    """prepare writes tool_data.csv and pathout.csv to the given output dir."""
    from jubilee_twin.pipeline import tool_id, path_follower
    from jubilee_twin.paths import twin_dir

    gcode = _latest_gcode()
    out = str(tmp_path)

    tool_id.run(output_dir=out)
    path_follower.run(gcode_file=gcode, output_dir=out)

    tool_csv = tmp_path / "tool_data.csv"
    path_csv = tmp_path / "pathout.csv"

    assert tool_csv.exists(), "tool_data.csv not created"
    assert path_csv.exists(), "pathout.csv not created"

    rows = list(csv.reader(tool_csv.open()))
    assert rows[0] == ["Tool_ID", "Name", "Park_X", "Park_Y", "Park_Z", "Offset_X", "Offset_Y", "Offset_Z"]
    assert len(rows) == 5, f"Expected header + 4 rows, got {len(rows)}"

    path_rows = path_csv.read_text().strip().splitlines()
    assert len(path_rows) > 0, "pathout.csv is empty"
    assert len(path_rows[0].split(",")) == 6, "pathout.csv rows must have 6 columns"


def test_prepare_cli_exits_zero():
    """jubilee-twin prepare exits 0 and produces pipeline_data/ CSVs."""
    gcode = _latest_gcode()
    result = subprocess.run(
        [sys.executable, "-m", "jubilee_twin.cli", "prepare", gcode],
        capture_output=True, text=True
    )
    # cli.py has main() not a __main__ block — use the installed script instead
    result = subprocess.run(
        ["jubilee-twin", "prepare", gcode],
        capture_output=True, text=True
    )
    assert result.returncode == 0, f"prepare failed:\n{result.stderr}"
    assert "pipeline_data/ ready" in result.stdout


# ---------------------------------------------------------------------------
# animate  (requires Blender)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _blender_available(), reason="blender not on PATH")
def test_animate_cli_exits_zero():
    gcode = _latest_gcode()
    result = subprocess.run(
        ["jubilee-twin", "animate", gcode],
        capture_output=True, text=True
    )
    assert result.returncode == 0, f"animate failed:\n{result.stderr}\n{result.stdout}"
    combined = result.stdout + result.stderr
    assert "Animation complete" in combined, "Expected 'Animation complete' in Blender output"


@pytest.mark.skipif(not _blender_available(), reason="blender not on PATH")
def test_animate_produces_csvs():
    """animate regenerates pipeline_data/ CSVs as a side effect."""
    from jubilee_twin.paths import twin_dir
    gcode = _latest_gcode()
    subprocess.run(["jubilee-twin", "animate", gcode], capture_output=True)
    pipeline_data = twin_dir() / "pipeline_data"
    assert (pipeline_data / "tool_data.csv").exists()
    assert (pipeline_data / "pathout.csv").exists()


# ---------------------------------------------------------------------------
# raytrace  (requires Blender)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _blender_available(), reason="blender not on PATH")
def test_raytrace_cli_exits_zero():
    """raytrace uses existing pipeline_data/ CSVs (prepare must have run first)."""
    from jubilee_twin.paths import twin_dir
    pipeline_data = twin_dir() / "pipeline_data"
    if not (pipeline_data / "pathout.csv").exists():
        pytest.skip("pipeline_data/pathout.csv missing — run prepare first")

    result = subprocess.run(
        ["jubilee-twin", "raytrace"],
        capture_output=True, text=True
    )
    assert result.returncode == 0, f"raytrace failed:\n{result.stderr}\n{result.stdout}"
    combined = result.stdout + result.stderr
    assert "Animation complete" in combined, "Expected animation step before ray-tracing"


@pytest.mark.skipif(not _blender_available(), reason="blender not on PATH")
def test_raytrace_missing_csvs_raises():
    """raytrace without CSVs and without a gcode arg exits non-zero with a clear message."""
    from jubilee_twin.paths import twin_dir
    import os, shutil as sh

    pipeline_data = twin_dir() / "pipeline_data"
    backup = twin_dir() / "pipeline_data_backup_test"

    # temporarily hide pipeline_data/
    if pipeline_data.exists():
        pipeline_data.rename(backup)
    try:
        result = subprocess.run(
            ["jubilee-twin", "raytrace"],
            capture_output=True, text=True
        )
        assert result.returncode != 0
        assert "pipeline_data" in result.stdout + result.stderr
    finally:
        if backup.exists():
            backup.rename(pipeline_data)
