"""Hermetic coverage for CLI commands that launch Blender headlessly."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest


class RecordingDriver:
    """CLI replacement that records a single dispatched driver call."""

    instance: "RecordingDriver | None" = None

    def __init__(self, blender_exe: str | None = None):
        self.blender_exe = blender_exe
        self.calls: list[tuple[str, tuple, dict]] = []
        type(self).instance = self

    def _record(self, name: str, *args, **kwargs) -> int:
        self.calls.append((name, args, kwargs))
        return 0

    def setup_scene(self, base_blend=None) -> int:
        return self._record("setup_scene")

    def place_tools(self) -> int:
        return self._record("place_tools")

    def place_camera(self) -> int:
        return self._record("place_camera")

    def populate_deck(self) -> int:
        return self._record("populate_deck")

    def animate_from_gcode(self, *args, **kwargs) -> int:
        return self._record("animate_from_gcode", *args, **kwargs)

    def run_raytracing(self, *args, **kwargs) -> int:
        return self._record("run_raytracing", *args, **kwargs)

    def snapshot(self, *args, **kwargs) -> int:
        return self._record("snapshot", *args, **kwargs)

    def scan(self, *args, **kwargs) -> int:
        return self._record("scan", *args, **kwargs)


def _run_cli(monkeypatch: pytest.MonkeyPatch, args: list[str]) -> RecordingDriver:
    """Run the CLI with a recording driver and return its recorded call."""
    import jubilee_twin.cli as cli
    import jubilee_twin.driver as driver_module

    RecordingDriver.instance = None
    monkeypatch.setattr(driver_module, "TwinDriver", RecordingDriver)
    monkeypatch.setattr("sys.argv", ["jubilee-twin", *args])
    with pytest.raises(SystemExit) as exit_info:
        cli.main()
    assert exit_info.value.code == 0
    assert RecordingDriver.instance is not None
    return RecordingDriver.instance


@pytest.mark.parametrize(
    ("args", "method"),
    [
        (["setup-scene", "--blender", "test-blender"], "setup_scene"),
        (["place-tools", "--blender", "test-blender"], "place_tools"),
        (["place-camera", "--blender", "test-blender"], "place_camera"),
        (["populate-deck", "--blender", "test-blender"], "populate_deck"),
    ],
)
def test_cli_dispatches_headless_scene_commands(monkeypatch, args, method):
    driver = _run_cli(monkeypatch, args)

    assert driver.blender_exe == "test-blender"
    assert driver.calls == [(method, (), {})]


def test_cli_dispatches_headless_animate(monkeypatch):
    driver = _run_cli(monkeypatch, ["animate", "example.gcode", "--step", "12.5", "--blender", "test-blender"])

    assert driver.blender_exe == "test-blender"
    assert driver.calls == [
        ("animate_from_gcode", (), {
            "gcode_file": "example.gcode",
            "distance_per_step": 12.5,
            "interactive": False,
        })
    ]


def test_cli_dispatches_headless_raytrace_with_gcode(monkeypatch, tmp_path):
    import jubilee_twin.cli as cli
    import jubilee_twin.pipeline.path_follower as path_follower
    import jubilee_twin.pipeline.tool_id as tool_id
    import jubilee_twin.paths as paths

    calls: list[tuple[str, dict]] = []
    monkeypatch.setattr(paths, "twin_dir", lambda: tmp_path)
    monkeypatch.setattr(tool_id, "run", lambda **kwargs: calls.append(("tool_id", kwargs)))
    monkeypatch.setattr(path_follower, "run", lambda **kwargs: calls.append(("path_follower", kwargs)))

    driver = _run_cli(monkeypatch, ["raytrace", "example.gcode", "--step", "25", "--blender", "test-blender"])

    assert calls == [
        ("tool_id", {"output_dir": str(tmp_path / "pipeline_data")}),
        ("path_follower", {
            "gcode_file": "example.gcode",
            "distance_per_step": 25.0,
            "output_dir": str(tmp_path / "pipeline_data"),
        }),
    ]
    assert driver.calls == [("run_raytracing", (), {"interactive": False})]


def test_cli_dispatches_headless_snapshot(monkeypatch, tmp_path):
    output = tmp_path / "snapshot.jpg"
    driver = _run_cli(monkeypatch, [
        "snapshot", "--x", "1", "--y", "2", "--z", "3",
        "--output", str(output), "--blender", "test-blender",
    ])

    assert driver.calls == [("snapshot", (), {
        "x_mm": 1.0, "y_mm": 2.0, "z_mm": 3.0,
        "output": output, "pop": False,
    })]


def test_cli_dispatches_headless_scan_with_configured_grid(monkeypatch, tmp_path):
    output_root = tmp_path / "scans"
    driver = _run_cli(monkeypatch, [
        "scan", "--x-min", "1", "--x-max", "2", "--x-steps", "2",
        "--y-min", "3", "--y-max", "4", "--y-steps", "3",
        "--z-min", "5", "--z-max", "6", "--z-steps", "4",
        "--width", "640", "--height", "480", "--output-root", str(output_root),
        "--blender", "test-blender",
    ])

    assert driver.calls == [("scan", (), {
        "x_min": 1.0, "x_max": 2.0, "x_steps": 2,
        "y_min": 3.0, "y_max": 4.0, "y_steps": 3,
        "z_min": 5.0, "z_max": 6.0, "z_steps": 4,
        "width": 640, "height": 480, "output_root": output_root,
    })]


@pytest.fixture
def headless_driver(monkeypatch, tmp_path):
    """TwinDriver configured with a temporary working scene and fake Blender."""
    import jubilee_twin.driver as driver_module

    working = tmp_path / "pipeline_data" / "jubilee_working.blend"
    working.parent.mkdir()
    working.write_bytes(b"blend")
    monkeypatch.setattr(driver_module, "twin_dir", lambda: tmp_path)

    commands: list[list[str]] = []
    monkeypatch.setattr(
        driver_module.subprocess,
        "run",
        lambda command: commands.append(command) or SimpleNamespace(returncode=0),
    )
    return driver_module.TwinDriver(blender_exe="test-blender"), commands, tmp_path


@pytest.mark.parametrize(
    ("method", "script_name"),
    [
        ("place_tools", "tool_placement.py"),
        ("place_camera", "place_camera.py"),
    ],
)
def test_driver_runs_placement_commands_headlessly(headless_driver, method, script_name):
    driver, commands, _ = headless_driver

    assert getattr(driver, method)() == 0
    assert len(commands) == 1
    assert commands[0][0] == "test-blender"
    assert "--background" in commands[0]
    assert commands[0][-1].endswith(script_name)


def test_driver_runs_populate_deck_headlessly(headless_driver, monkeypatch):
    driver, commands, _ = headless_driver
    monkeypatch.setattr(driver, "_write_paths_cache", lambda: None)

    assert driver.populate_deck() == 0
    assert "--background" in commands[0]
    assert commands[0][-1].endswith("populate_deck.py")


def test_driver_runs_animate_headlessly(headless_driver, monkeypatch):
    driver, commands, tmp_path = headless_driver
    import jubilee_twin.pipeline.path_follower as path_follower
    import jubilee_twin.pipeline.tool_id as tool_id

    monkeypatch.setattr(driver, "_write_paths_cache", lambda: None)
    monkeypatch.setattr(tool_id, "run", lambda **kwargs: None)
    monkeypatch.setattr(path_follower, "run", lambda **kwargs: None)

    assert driver.animate_from_gcode(Path("example.gcode")) == 0
    assert "--background" in commands[0]
    assert commands[0][-1].endswith("animate_path.py")


def test_driver_runs_snapshot_headlessly(headless_driver):
    driver, commands, _ = headless_driver

    assert driver.snapshot(x_mm=1, y_mm=2, z_mm=3, output=Path("image.jpg")) == 0
    assert "--background" in commands[0]
    assert commands[0][commands[0].index("--python") + 1].endswith("snapshot.py")


def test_driver_runs_scan_headlessly(headless_driver, tmp_path):
    driver, commands, _ = headless_driver

    assert driver.scan(
        x_min=1, x_max=2, x_steps=2,
        y_min=3, y_max=4, y_steps=2,
        z_min=5, z_max=6, z_steps=2,
        width=640, height=480, output_root=tmp_path / "scans",
    ) == 0
    assert "--background" in commands[0]
    assert commands[0][commands[0].index("--python") + 1].endswith("scan.py")
    assert "--output-root" in commands[0]


def test_driver_runs_raytrace_headlessly(headless_driver):
    driver, commands, _ = headless_driver

    assert driver.run_raytracing() == 0
    assert "--background" in commands[0]
    assert commands[0].count("--python") == 2
    assert commands[0][-1].endswith("ray_tracing.py")
