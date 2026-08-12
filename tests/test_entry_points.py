from pathlib import Path
import pytest


def test_twin_dir_entry_point_registered():
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "twin_dir"]
    assert eps, "jubilee.paths/twin_dir not registered — run: pip install -e ."


def test_twin_dir_points_to_real_directory():
    from jubilee_twin.paths import twin_dir
    td = twin_dir()
    assert td.exists(), f"twin_dir() returned non-existent path: {td}"
    assert (td / "jubilee_twin" / "pipeline").exists()
    assert (td / "blender_addon" / "jubilee_digital_twin").exists()
    assert (td / "blender_models").exists()


def test_digital_twin_entry_point_registered():
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="science_jubilee.digital_twin") if ep.name == "default"]
    assert eps, "science_jubilee.digital_twin/default not registered — run: pip install -e ."
    assert callable(eps[0].load())


def test_driver_instantiates():
    from jubilee_twin.driver import TwinDriver
    d = TwinDriver(blender_exe="blender")
    assert d.blender_exe == "blender"


def test_driver_from_entry_point():
    from jubilee_twin.driver import TwinDriver
    d = TwinDriver.from_entry_point(blender_exe="blender")
    assert d.blender_exe == "blender"
