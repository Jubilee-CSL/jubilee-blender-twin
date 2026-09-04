import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        prog="jubilee-twin",
        description="Run the Jubilee digital twin pipeline.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # --- setup-scene ---
    setup_scene = subparsers.add_parser(
        "setup-scene",
        help="Write CSVs + paths cache and copy jubilee_base.blend → jubilee_working.blend.",
    )
    setup_scene.add_argument("--blender", default=None, metavar="EXE")
    setup_scene.add_argument("--blend-file", type=Path, default=None, metavar="PATH", help="Use this .blend as the working-scene source instead of blender_models/jubilee_base.blend.")

    # --- place-tools ---
    place_tools = subparsers.add_parser(
        "place-tools",
        help="Load tool .blend files and parking posts into the working blend (headless Blender).",
    )
    place_tools.add_argument("--blender", default=None, metavar="EXE")

    # --- place-camera ---
    place_camera = subparsers.add_parser(
        "place-camera",
        help="Create Toolhead_Cam in the working blend using calibration from jubilee_paths.json.",
    )
    place_camera.add_argument("--blender", default=None, metavar="EXE")

    # --- populate-deck ---
    populate_deck = subparsers.add_parser(
        "populate-deck",
        help="Load labware from the latest interface experiment into the working blend (headless Blender).",
    )
    populate_deck.add_argument("--blender", default=None, metavar="EXE")

    # --- prepare ---
    prepare = subparsers.add_parser("prepare", help="Generate pipeline_data/ CSVs only (no Blender).")
    prepare.add_argument("gcode", help="Path to gcode file.")
    prepare.add_argument("--step", type=float, default=50.0, metavar="MM")

    # --- animate ---
    animate = subparsers.add_parser("animate", help="Parse gcode and animate in Blender.")
    animate.add_argument("gcode", help="Path to gcode file (or filename to search in pipeline_data/gcode_logs/).")
    animate.add_argument("--interactive", action="store_true", help="Open Blender GUI instead of running headless.")
    animate.add_argument("--step", type=float, default=50.0, metavar="MM", help="mm between animation keyframes (default: 50).")
    animate.add_argument("--blender", default=None, metavar="EXE", help="Blender executable (default: $JUBILEE_BLENDER_EXE or 'blender').")

    # --- raytrace ---
    raytrace = subparsers.add_parser("raytrace", help="Run ray-tracing collision detection.")
    raytrace.add_argument(
        "gcode", nargs="?", default=None,
        help="Gcode file to (re)generate pipeline_data/ from. Omit to use existing pipeline_data/.",
    )
    raytrace.add_argument("--interactive", action="store_true", help="Open Blender GUI.")
    raytrace.add_argument("--step", type=float, default=50.0, metavar="MM")
    raytrace.add_argument("--blender", default=None, metavar="EXE")

    # --- snapshot ---
    snap = subparsers.add_parser("snapshot", help="Render one frame through Toolhead_Cam at the current or given position.")
    snap.add_argument("--x", type=float, default=None, metavar="MM")
    snap.add_argument("--y", type=float, default=None, metavar="MM")
    snap.add_argument("--z", type=float, default=None, metavar="MM")
    snap.add_argument("--output", type=Path, default=None, metavar="PATH", help="Output image path (default: Scans/snapshots/<timestamp>.jpg).")
    snap.add_argument("--pop", action="store_true", help="Show the rendered image with matplotlib after rendering.")
    snap.add_argument("--blender", default=None, metavar="EXE")

    # --- scan ---
    scan = subparsers.add_parser("scan", help="Render a virtual-scanner image grid in headless Blender.")
    scan.add_argument("--x-min", type=float, default=110.0, metavar="MM")
    scan.add_argument("--x-max", type=float, default=250.0, metavar="MM")
    scan.add_argument("--x-steps", type=int, default=5, metavar="N")
    scan.add_argument("--y-min", type=float, default=80.0, metavar="MM")
    scan.add_argument("--y-max", type=float, default=200.0, metavar="MM")
    scan.add_argument("--y-steps", type=int, default=5, metavar="N")
    scan.add_argument("--z-min", type=float, default=280.0, metavar="MM")
    scan.add_argument("--z-max", type=float, default=320.0, metavar="MM")
    scan.add_argument("--z-steps", type=int, default=3, metavar="N")
    scan.add_argument("--width", type=int, default=1920, metavar="PX")
    scan.add_argument("--height", type=int, default=1056, metavar="PX")
    scan.add_argument("--output-root", type=Path, default=None, metavar="PATH", help="Parent directory for the timestamped scan folder (default: Scans/).")
    scan.add_argument("--blender", default=None, metavar="EXE")

    # --- open ---
    open_cmd = subparsers.add_parser("open", help="Open the working blend file (or jubilee_belt.blend) in the Blender GUI.")
    open_cmd.add_argument("--blender", default=None, metavar="EXE")

    args = parser.parse_args()

    from jubilee_twin.driver import TwinDriver
    from jubilee_twin import trace
    from jubilee_twin.paths import twin_dir as _twin_dir

    # One recap image per command, titled with the command that produced it.
    try:
        trace.start_session(_twin_dir() / "pipeline_data", f"jubilee-twin {args.command}")
    except Exception:
        pass

    driver = TwinDriver(blender_exe=args.blender if hasattr(args, "blender") else None)

    if args.command == "setup-scene":
        rc = driver.setup_scene(base_blend=args.blend_file)
        raise SystemExit(rc)

    elif args.command == "place-tools":
        rc = driver.place_tools()
        raise SystemExit(rc)

    elif args.command == "place-camera":
        rc = driver.place_camera()
        raise SystemExit(rc)

    elif args.command == "populate-deck":
        rc = driver.populate_deck()
        raise SystemExit(rc)

    elif args.command == "prepare":
        from jubilee_twin.pipeline import tool_id, path_follower
        from jubilee_twin.paths import twin_dir
        pipeline_data_dir = str(twin_dir() / "pipeline_data")
        tool_id.run(output_dir=pipeline_data_dir)
        path_follower.run(gcode_file=args.gcode, distance_per_step=args.step, output_dir=pipeline_data_dir)
        print("pipeline_data/ ready — open Blender and click 'Animate from CSV'.")

    elif args.command == "animate":
        rc = driver.animate_from_gcode(
            gcode_file=args.gcode,
            distance_per_step=args.step,
            interactive=args.interactive,
        )
        raise SystemExit(rc)

    elif args.command == "raytrace":
        if args.gcode:
            # regenerate CSVs from the given gcode before ray-tracing
            from jubilee_twin.pipeline import tool_id, path_follower
            from jubilee_twin.paths import twin_dir
            pipeline_data_dir = str(twin_dir() / "pipeline_data")
            tool_id.run(output_dir=pipeline_data_dir)
            path_follower.run(gcode_file=args.gcode, distance_per_step=args.step, output_dir=pipeline_data_dir)
        else:
            from jubilee_twin.paths import twin_dir
            pathout = twin_dir() / "pipeline_data" / "pathout.csv"
            machine = twin_dir() / "pipeline_data" / "machine.json"
            if not pathout.exists() or not machine.exists():
                raise SystemExit(
                    "pipeline_data/ not populated. Run 'jubilee-twin animate <gcode>' first, "
                    "or pass a gcode file: 'jubilee-twin raytrace <gcode>'"
                )
        rc = driver.run_raytracing(interactive=args.interactive)
        raise SystemExit(rc)

    elif args.command == "snapshot":
        rc = driver.snapshot(x_mm=args.x, y_mm=args.y, z_mm=args.z, output=args.output, pop=args.pop)
        raise SystemExit(rc)

    elif args.command == "scan":
        if min(args.x_steps, args.y_steps, args.z_steps) < 1:
            parser.error("--x-steps, --y-steps, and --z-steps must each be at least 1")
        if min(args.width, args.height) < 16:
            parser.error("--width and --height must each be at least 16")
        rc = driver.scan(
            x_min=args.x_min, x_max=args.x_max, x_steps=args.x_steps,
            y_min=args.y_min, y_max=args.y_max, y_steps=args.y_steps,
            z_min=args.z_min, z_max=args.z_max, z_steps=args.z_steps,
            width=args.width, height=args.height, output_root=args.output_root,
        )
        raise SystemExit(rc)

    elif args.command == "open":
        driver.open_interactive()
