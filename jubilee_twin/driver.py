import os
import subprocess
from pathlib import Path

from jubilee_twin.paths import twin_dir


def _discover_jubilee_root() -> Path | None:
    """Find the science_jubilee repo root via entry point, then by import fallback."""
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    if eps:
        return Path(eps[0].load()())
    try:
        import science_jubilee
        # src/science_jubilee/__init__.py → parents[2] is the repo root
        return Path(science_jubilee.__file__).parents[2]
    except ImportError:
        return None


class TwinDriver:
    def __init__(self, blender_exe: str = None):
        self.blender_exe = blender_exe or os.environ.get("JUBILEE_BLENDER_EXE", "blender")

    @classmethod
    def from_entry_point(cls, blender_exe: str = None) -> "TwinDriver":
        """Instantiate by verifying the twin_dir entry point resolves correctly."""
        from importlib.metadata import entry_points
        eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "twin_dir"]
        if not eps:
            raise RuntimeError(
                "jubilee-twin is not installed. Run: pip install -e jubilee-blender-twin"
            )
        td = eps[0].load()()
        if not td.exists():
            raise RuntimeError(f"twin_dir() returned a path that does not exist: {td}")
        return cls(blender_exe=blender_exe)

    def animate_from_gcode(self, gcode_file: Path, science_jubilee_root: Path = None) -> int:
        """Run the full gcode→animation pipeline.

        Level 1: delegates to run_latest_gcode_animation.bat.
        Level 2: will call path_follower and Blender steps directly.
        """
        td = twin_dir()
        bat = td / "from_gcode" / "run_latest_gcode_animation.bat"

        if science_jubilee_root is None:
            science_jubilee_root = _discover_jubilee_root()
        if science_jubilee_root is None:
            raise RuntimeError(
                "science_jubilee root not found. "
                "Install science-jubilee or register jubilee.paths/jubilee_dir."
            )

        gcode_file = Path(gcode_file)
        cmd = [str(bat), str(science_jubilee_root), str(td), self.blender_exe, gcode_file.name]
        return subprocess.run(cmd, shell=True).returncode

    def open_interactive(self) -> None:
        """Open Blender with jubilee_belt.blend."""
        subprocess.Popen([self.blender_exe, str(twin_dir() / "jubilee_belt.blend")])

    def run_raytracing(self) -> None:
        """Run ray-tracing collision detection in-process. Requires bpy."""
        import sys
        sys.path.insert(0, str(twin_dir()))
        from ray_tracing import ray_tracing_cd
        ray_tracing_cd()


def run_twin(gcode_file, config_dir=None) -> int:
    """Entry point callable for science_jubilee.digital_twin."""
    return TwinDriver().animate_from_gcode(Path(gcode_file))
