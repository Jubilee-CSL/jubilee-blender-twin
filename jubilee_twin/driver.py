import os
import subprocess
from pathlib import Path

from jubilee_twin.paths import twin_dir


def _discover_jubilee_root() -> Path | None:
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    if eps:
        return Path(eps[0].load()())
    try:
        import science_jubilee
        return Path(science_jubilee.__file__).parents[2]
    except ImportError:
        return None


class TwinDriver:
    def __init__(self, blender_exe: str = None):
        self.blender_exe = blender_exe or os.environ.get("JUBILEE_BLENDER_EXE", "blender")

    @classmethod
    def from_entry_point(cls, blender_exe: str = None) -> "TwinDriver":
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

    def _write_paths_cache(self) -> None:
        """Write pipeline_data/jubilee_paths.json so Blender addon can locate gcode_logs/."""
        import json
        from importlib.metadata import entry_points

        td = twin_dir()
        cache: dict = {"twin_dir": str(td)}
        eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
        if eps:
            cache["jubilee_dir"] = str(eps[0].load()())
        cache_path = td / "pipeline_data" / "jubilee_paths.json"
        cache_path.parent.mkdir(exist_ok=True)
        cache_path.write_text(json.dumps(cache, indent=2))

    def _working_blend(self) -> Path:
        """Return the working blend file; fall back to jubilee_belt.blend if setup-scene hasn't run."""
        td = twin_dir()
        working = td / "pipeline_data" / "jubilee_working.blend"
        return working if working.exists() else td / "blender_models" / "jubilee_belt.blend"

    def setup_scene(self) -> int:
        """Copy jubilee_base.blend → pipeline_data/jubilee_working.blend and populate tools.

        Requires tool_data.csv in pipeline_data/ (run 'jubilee-twin prepare' first, or any
        command that generates tool_data.csv).
        """
        from jubilee_twin.pipeline import tool_id

        td = twin_dir()
        pipeline_data_dir = td / "pipeline_data"
        pipeline_data_dir.mkdir(exist_ok=True)

        self._write_paths_cache()
        # Generate tool_data.csv (does not need a gcode file)
        tool_id.run(output_dir=str(pipeline_data_dir))

        base = td / "blender_models" / "jubilee_base.blend"
        if not base.exists():
            raise FileNotFoundError(f"jubilee_base.blend not found at {base}")

        import shutil
        working = pipeline_data_dir / "jubilee_working.blend"
        print(f"Copying {base.name} -> {working} ...")
        shutil.copy2(base, working)

        tool_script = td / "blender_addon" / "jubilee_digital_twin" / "tool_placement.py"
        print("Populating tools (headless Blender)...")
        cmd = [self.blender_exe, str(working), "--background", "--python", str(tool_script)]
        return subprocess.run(cmd).returncode

    def animate_from_gcode(
        self,
        gcode_file: Path,
        distance_per_step: float = 50.0,
        interactive: bool = False,
    ) -> int:
        """Run the full gcode→animation pipeline.

        Steps:
          1. Extract tool config → pipeline_data/tool_data.csv
          2. Parse gcode        → pipeline_data/pathout.csv
          3. Launch Blender with animate_path.py

        interactive=False: Blender runs headless (--background), no window.
        interactive=True:  Blender opens in GUI mode so you can inspect the result.
        """
        from jubilee_twin.pipeline import tool_id, path_follower

        td = twin_dir()
        pipeline_data_dir = str(td / "pipeline_data")
        self._write_paths_cache()

        print("Step 1/3: extracting tool data...")
        tool_id.run(output_dir=pipeline_data_dir)

        print("Step 2/3: parsing gcode...")
        path_follower.run(
            gcode_file=str(gcode_file),
            distance_per_step=distance_per_step,
            output_dir=pipeline_data_dir,
        )

        print("Step 3/3: launching Blender...")
        blend = self._working_blend()
        script = td / "blender_addon" / "jubilee_digital_twin" / "animate_path.py"
        cmd = [self.blender_exe, str(blend)]
        if not interactive:
            cmd.append("--background")
        cmd += ["--python", str(script)]
        return subprocess.run(cmd).returncode

    def open_interactive(self) -> None:
        """Open the working blend file in the Blender GUI without running any script."""
        subprocess.Popen([self.blender_exe, str(self._working_blend())])

    def run_raytracing(self, interactive: bool = False) -> int:
        """Animate the scene then run ray-tracing, in a single Blender session."""
        td = twin_dir()
        blend = self._working_blend()
        animate_script = td / "blender_addon" / "jubilee_digital_twin" / "animate_path.py"
        raytrace_script = td / "blender_addon" / "jubilee_digital_twin" / "ray_tracing.py"
        cmd = [self.blender_exe, str(blend)]
        if not interactive:
            cmd.append("--background")
        # animate first so keyframes exist when ray_tracing evaluates them
        cmd += ["--python", str(animate_script), "--python", str(raytrace_script)]
        return subprocess.run(cmd).returncode


def run_twin(gcode_file, **kwargs) -> int:
    """Entry point callable for science_jubilee.digital_twin."""
    return TwinDriver().animate_from_gcode(Path(gcode_file), **kwargs)
