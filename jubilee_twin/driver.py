import os
import subprocess
from datetime import datetime
from pathlib import Path

from jubilee_twin.paths import resolve, twin_dir
from jubilee_twin.log import get_logger

log = get_logger(__name__)


class TwinDriver:
    def __init__(self, blender_exe: str = None):
        self.blender_exe = blender_exe or os.environ.get("JUBILEE_BLENDER_EXE", "blender")

    @classmethod
    def from_entry_point(cls, blender_exe: str = None) -> "TwinDriver":
        try:
            td = resolve("twin_dir")
        except RuntimeError as e:
            raise RuntimeError(
                "jubilee-twin is not installed. Run: pip install -e jubilee-blender-twin"
            ) from e
        if not td.exists():
            raise RuntimeError(f"twin_dir() returned a path that does not exist: {td}")
        return cls(blender_exe=blender_exe)

    def _write_paths_cache(self) -> None:
        """Write pipeline_data/jubilee_paths.json so Blender addon can locate gcode_logs/."""
        import json
        from jubilee_twin import trace

        td = twin_dir()
        pkg_sec = trace.session(td / "pipeline_data").section("Package paths", reset=True)
        cam_sec = trace.session().section("Camera calibration", reset=True)

        cache: dict = {"twin_dir": str(td)}
        pkg_sec.ok("twin_dir", str(td))
        for key in ("jubilee_dir", "interface_dir", "experiment_deck_dir", "camera_params_yaml"):
            try:
                cache[key] = str(resolve(key))
                pkg_sec.ok(key, cache[key])
            except RuntimeError:
                pkg_sec.failed(key, "entry point not registered")

        # Expose .env.hardware path so the Blender addon can reach the live machine.
        if "jubilee_dir" in cache:
            env_hw = Path(cache["jubilee_dir"]) / ".env.hardware"
            if env_hw.is_file():
                cache["env_hardware"] = str(env_hw)
                pkg_sec.ok("env_hardware", str(env_hw))
            else:
                pkg_sec.failed("env_hardware", "no .env.hardware in jubilee_dir")

        # Inline camera calibration so the Blender addon never has to touch YAML.
        # Preference: science_jubilee entry point → twin's own defaults/ fallback.
        fallback_yaml = Path(__file__).parent / "defaults" / "camera_params.yaml"
        from_entry_point = bool(cache.get("camera_params_yaml"))
        yaml_path = cache.get("camera_params_yaml") or str(fallback_yaml)
        if not from_entry_point:
            cam_sec.failed("science_jubilee calibration", "camera_params_yaml not registered")
        try:
            import yaml  # type: ignore
            with open(yaml_path) as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data.get("camera"), dict):
                cache["camera_params"] = data["camera"]
                cache["camera_params_source"] = yaml_path
                label = "science_jubilee calibration" if from_entry_point else "bundled defaults"
                cam_sec.ok(label, Path(yaml_path).name)
            else:
                cam_sec.partial(Path(yaml_path).name, "no 'camera' block in YAML")
        except FileNotFoundError:
            log.warning("camera_params yaml not found: %s", yaml_path)
            cam_sec.failed(Path(yaml_path).name, "file not found")
        except ImportError:
            log.warning("PyYAML not installed; skipping camera_params inlining")
            cam_sec.failed("PyYAML", "not installed — calibration not inlined")
        except Exception as e:
            log.warning("Failed to load %s: %s", yaml_path, e)
            cam_sec.failed(Path(yaml_path).name, str(e))

        cache_path = td / "pipeline_data" / "jubilee_paths.json"
        cache_path.parent.mkdir(exist_ok=True)
        cache_path.write_text(json.dumps(cache, indent=2))
        log.warning("paths cache  → %s", cache_path)

        trace.session().result("jubilee_paths.json", cache_path.read_text(), cache_path)
        trace_path = trace.flush(td / "pipeline_data")
        if trace_path:
            log.warning("trace recap  → %s", trace_path)

    def _working_blend(self) -> Path:
        """Return the working blend file; fall back to jubilee_belt.blend if setup-scene hasn't run."""
        td = twin_dir()
        working = td / "pipeline_data" / "jubilee_working.blend"
        return working if working.exists() else td / "blender_models" / "jubilee_belt.blend"

    def setup_scene(self, base_blend: Path | None = None) -> int:
        """Full one-shot setup: query machine state, write CSVs, build working.blend with tools/camera/deck.

        base_blend: use this .blend as the working-scene source instead of blender_models/jubilee_base.blend.
        """
        from jubilee_twin.pipeline import tool_id
        import shutil

        td = twin_dir()
        pipeline_data_dir = td / "pipeline_data"
        pipeline_data_dir.mkdir(exist_ok=True)

        log.info("Step 1/3: querying machine state and writing CSVs")
        self._write_paths_cache()
        tool_id.run(output_dir=str(pipeline_data_dir))

        base = Path(base_blend) if base_blend is not None else td / "blender_models" / "jubilee_base.blend"
        if not base.exists():
            raise FileNotFoundError(f"Base blend not found at {base}")
        working = pipeline_data_dir / "jubilee_working.blend"
        log.warning("base blend   → %s", working)
        shutil.copy2(base, working)

        log.info("Step 2/3: placing tools at live park positions")
        rc = self.place_tools()
        if rc != 0:
            return rc

        log.info("Step 3/3: placing camera")
        rc = self.place_camera()
        if rc != 0:
            return rc

        log.info("Done — open pipeline_data/jubilee_working.blend")
        log.info("Next steps (run 'jubilee-twin <command> --help' for all options):")
        log.info("  open                         Open the working scene in Blender.")
        log.info("  animate <gcode>              Bake a G-code path; --step MM, --interactive.")
        log.info("  snapshot [--x X --y Y --z Z] Render one camera frame; --output PATH, --pop.")
        log.info("  scan                         Render the virtual-camera grid; --x/y/z-*, --width/--height.")
        log.info("  raytrace [gcode]             Run collision detection; pass G-code to refresh CSV data.")
        log.info("  place-camera                 Recreate Toolhead_Cam after a scene/calibration change.")
        log.info("  place-tools                  Re-add tool models from pipeline_data/machine.json.")
        log.info("Optional deck: populate-deck requires science-jubilee-interface and an exported deck.blend.")
        log.info("All Blender commands accept --blender EXE (or use $JUBILEE_BLENDER_EXE / 'blender').")
        return 0

    def place_tools(self) -> int:
        """Load tool .blend files and parking posts into jubilee_working.blend (headless Blender)."""
        td = twin_dir()
        working = self._working_blend()
        tool_script = td / "blender_addon" / "jubilee_digital_twin" / "tool_placement.py"
        log.info("Placing tools (headless Blender)")
        log.warning("tool script  → %s", tool_script)
        cmd = [self.blender_exe, str(working), "--background", "--python", str(tool_script)]
        return subprocess.run(cmd).returncode

    def place_camera(self) -> int:
        """Create Toolhead_Cam in jubilee_working.blend using current jubilee_paths.json intrinsics."""
        td = twin_dir()
        working = self._working_blend()
        cam_script = td / "blender_addon" / "jubilee_digital_twin" / "place_camera.py"
        log.info("Placing camera (headless Blender)")
        log.warning("cam script   → %s", cam_script)
        cmd = [self.blender_exe, str(working), "--background", "--python", str(cam_script)]
        return subprocess.run(cmd).returncode

    def populate_deck(self) -> int:
        """Load labware from the latest interface experiment into jubilee_working.blend (headless Blender)."""
        td = twin_dir()
        working = self._working_blend()
        deck_script = td / "blender_addon" / "jubilee_digital_twin" / "populate_deck.py"
        # Refresh paths cache so the script can find interface_dir even if setup-scene wasn't re-run.
        self._write_paths_cache()
        log.info("Populating deck (headless Blender)")
        log.warning("deck script  → %s", deck_script)
        cmd = [self.blender_exe, str(working), "--background", "--python", str(deck_script)]
        return subprocess.run(cmd).returncode

    def animate_from_gcode(
        self,
        gcode_file: Path,
        distance_per_step: float = 50.0,
        interactive: bool = False,
    ) -> int:
        """Run the full gcode→animation pipeline.

        Steps:
          1. Extract tool config → pipeline_data/machine.json
          2. Parse gcode        → pipeline_data/pathout.csv
          3. Launch Blender with animate_path.py

        interactive=False: Blender runs headless (--background), no window.
        interactive=True:  Blender opens in GUI mode so you can inspect the result.
        """
        from jubilee_twin.pipeline import tool_id, path_follower

        td = twin_dir()
        pipeline_data_dir = str(td / "pipeline_data")
        self._write_paths_cache()

        log.info("Step 1/3: extracting tool data")
        tool_id.run(output_dir=pipeline_data_dir)

        log.info("Step 2/3: parsing gcode")
        path_follower.run(
            gcode_file=str(gcode_file),
            distance_per_step=distance_per_step,
            output_dir=pipeline_data_dir,
        )

        log.info("Step 3/3: launching Blender")
        blend = self._working_blend()
        script = td / "blender_addon" / "jubilee_digital_twin" / "animate_path.py"
        cmd = [self.blender_exe, str(blend)]
        if not interactive:
            cmd.append("--background")
        cmd += ["--python", str(script)]
        return subprocess.run(cmd).returncode

    def snapshot(
        self,
        x_mm: float | None = None,
        y_mm: float | None = None,
        z_mm: float | None = None,
        output: Path | None = None,
        pop: bool = False,
    ) -> int:
        """Render one frame through Toolhead_Cam at the current or specified position."""
        td = twin_dir()
        blend = self._working_blend()
        script = td / "blender_addon" / "jubilee_digital_twin" / "snapshot.py"

        # Fix the output path now so we know where to find it for --pop.
        if output is None and pop:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = td / "Scans" / "snapshots" / f"{stamp}_snapshot.jpg"

        cmd = [self.blender_exe, str(blend), "--background", "--python", str(script), "--"]
        if x_mm is not None:
            cmd += ["--x", str(x_mm)]
        if y_mm is not None:
            cmd += ["--y", str(y_mm)]
        if z_mm is not None:
            cmd += ["--z", str(z_mm)]
        if output is not None:
            cmd += ["--output", str(output)]
        rc = subprocess.run(cmd).returncode

        if pop and output is not None and Path(output).is_file():
            import sys
            if sys.platform == "win32":
                os.startfile(str(output))
            elif sys.platform == "darwin":
                subprocess.Popen(["open", str(output)])
            else:
                subprocess.Popen(["xdg-open", str(output)])

        return rc

    def scan(
        self,
        *,
        x_min: float, x_max: float, x_steps: int,
        y_min: float, y_max: float, y_steps: int,
        z_min: float, z_max: float, z_steps: int,
        width: int, height: int,
        output_root: Path | None = None,
    ) -> int:
        """Render a virtual-scanner grid in the working blend via headless Blender."""
        td = twin_dir()
        blend = self._working_blend()
        script = td / "blender_addon" / "jubilee_digital_twin" / "scan.py"
        cmd = [
            self.blender_exe, str(blend), "--background", "--python", str(script), "--",
            "--x-min", str(x_min), "--x-max", str(x_max), "--x-steps", str(x_steps),
            "--y-min", str(y_min), "--y-max", str(y_max), "--y-steps", str(y_steps),
            "--z-min", str(z_min), "--z-max", str(z_max), "--z-steps", str(z_steps),
            "--width", str(width), "--height", str(height),
        ]
        if output_root is not None:
            cmd += ["--output-root", str(output_root)]
        log.info("Running virtual scanner (headless Blender)")
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
