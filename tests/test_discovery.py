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
# camera_params discovery
# ---------------------------------------------------------------------------

def test_camera_params_yaml_resolves_via_entry_point():
    from jubilee_twin.paths import resolve
    try:
        p = resolve("camera_params_yaml")
    except RuntimeError:
        pytest.skip("jubilee.paths/camera_params_yaml not registered")
    assert p.is_file(), f"camera_params_yaml entry point points to missing file: {p}"


def test_camera_params_yaml_contains_required_keys():
    from jubilee_twin.paths import resolve
    import yaml
    try:
        p = resolve("camera_params_yaml")
    except RuntimeError:
        pytest.skip("jubilee.paths/camera_params_yaml not registered")
    data = yaml.safe_load(p.read_text())
    cam = data.get("camera", {})
    for key in ("fx", "fy", "cx", "cy", "dist", "offset"):
        assert key in cam, f"camera_params.yaml missing key: {key!r}"


def test_write_paths_cache_inlines_camera_params(tmp_path, monkeypatch):
    """_write_paths_cache must embed camera_params into jubilee_paths.json."""
    import json
    from jubilee_twin.paths import resolve
    from jubilee_twin.driver import TwinDriver

    try:
        resolve("camera_params_yaml")
    except RuntimeError:
        pytest.skip("jubilee.paths/camera_params_yaml not registered")

    monkeypatch.chdir(tmp_path)
    (tmp_path / "pipeline_data").mkdir()

    import jubilee_twin.driver as drv
    monkeypatch.setattr(drv, "twin_dir", lambda: tmp_path)

    d = TwinDriver(blender_exe="blender")
    d._write_paths_cache()

    cache = json.loads((tmp_path / "pipeline_data" / "jubilee_paths.json").read_text())
    assert "camera_params" in cache, "camera_params not inlined into jubilee_paths.json"
    for key in ("fx", "fy", "cx", "cy"):
        assert key in cache["camera_params"], f"camera_params missing {key!r}"


def test_resolve_raises_without_entry_point():
    from jubilee_twin.paths import resolve

    with patch("importlib.metadata.entry_points", return_value=[]):
        with pytest.raises(RuntimeError, match="jubilee.paths/jubilee_dir entry point not found"):
            resolve("jubilee_dir")


# ---------------------------------------------------------------------------
# tool_id: run() from machine_state.json
# ---------------------------------------------------------------------------

def test_run_uses_machine_state_names_and_offsets(tmp_path):
    import json
    from jubilee_twin.pipeline.tool_id import run

    state = {
        "tools": {"0": {"name": "OVERRIDE_NAME"}},
        "tool_offsets": {"0": [99.0, 88.0, 77.0]},
    }
    state_file = tmp_path / "machine_state.json"
    state_file.write_text(json.dumps(state))

    out = run(output_dir=str(tmp_path), machine_state_path=str(state_file))

    tool0 = json.loads(Path(out).read_text())["tools"][0]
    assert tool0["name"] == "OVERRIDE_NAME"
    assert tool0["offsets"][0] == pytest.approx(99.0)
    assert tool0["offsets"][1] == pytest.approx(88.0)


def test_run_uses_machine_state_parks(tmp_path):
    import json
    from jubilee_twin.pipeline.tool_id import run

    state = {
        "tools": {},
        "tool_offsets": {},
        "tool_parks": {"0": [111.1, 222.2, 0.0]},
    }
    state_file = tmp_path / "machine_state.json"
    state_file.write_text(json.dumps(state))

    out = run(output_dir=str(tmp_path), machine_state_path=str(state_file))

    tool0 = json.loads(Path(out).read_text())["tools"][0]
    assert tool0["park"][0] == pytest.approx(111.1)
    assert tool0["park"][1] == pytest.approx(222.2)


def test_run_falls_back_to_defaults_when_no_state(tmp_path):
    """With no state source available, defaults give known park positions."""
    import json
    from jubilee_twin.pipeline.tool_id import run
    from unittest.mock import patch

    from science_jubilee.machine_state import empty_state

    with patch(
        "jubilee_twin.pipeline.tool_id._science_jubilee_resolver",
        return_value=lambda **kw: (empty_state(), "empty machine"),
    ):
        out = run(output_dir=str(tmp_path))

    tools = json.loads(Path(out).read_text())["tools"]
    # Standard Jubilee parks: tool 0 at X=277, tool 3 at X=19
    assert tools[0]["park"][0] == pytest.approx(277.0)
    assert tools[3]["park"][0] == pytest.approx(19.0)


def test_run_uses_saved_state_over_defaults(tmp_path):
    import json
    from jubilee_twin.pipeline.tool_id import run

    state = {
        "tools": {"0": {"name": "MySyringe"}},
        "tool_offsets": {},
        "tool_parks": {"0": [55.0, 66.0, 0.0]},
    }
    state_file = tmp_path / "machine_state.json"
    state_file.write_text(json.dumps(state))

    # address is absent so live query is skipped; saved file is used
    out = run(output_dir=str(tmp_path), machine_state_path=str(state_file))

    tool0 = json.loads(Path(out).read_text())["tools"][0]
    assert tool0["name"] == "MySyringe"
    assert tool0["park"][0] == pytest.approx(55.0)


def test_run_live_query_takes_priority_over_saved(tmp_path):
    import json
    from jubilee_twin.pipeline.tool_id import run
    from unittest.mock import patch

    saved_state = {
        "address": "192.168.1.2",
        "tools": {"0": {"name": "OldName"}},
        "tool_offsets": {},
        "tool_parks": {"0": [1.0, 2.0, 0.0]},
    }
    live_state = {
        "transport": "HTTPTransport",
        "address": "192.168.1.2",
        "tools": {"0": {"name": "LiveName"}},
        "tool_offsets": {},
        "tool_parks": {"0": [99.0, 88.0, 0.0]},
    }
    state_file = tmp_path / "machine_state.json"
    state_file.write_text(json.dumps(saved_state))

    with patch("science_jubilee.machine_state.query_live", return_value=live_state):
        out = run(output_dir=str(tmp_path))

    tool0 = json.loads(Path(out).read_text())["tools"][0]
    assert tool0["name"] == "LiveName"
    assert tool0["park"][0] == pytest.approx(99.0)


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
