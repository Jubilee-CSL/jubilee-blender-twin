import argparse
import os
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

    # --- place-tools ---
    place_tools = subparsers.add_parser(
        "place-tools",
        help="Load tool .blend files and parking posts into the working blend (headless Blender).",
    )
    place_tools.add_argument("--blender", default=None, metavar="EXE")

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
    animate.add_argument("gcode", help="Path to gcode file (or filename to search in gcode_logs/).")
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

    # --- open ---
    open_cmd = subparsers.add_parser("open", help="Open the working blend file (or jubilee_belt.blend) in the Blender GUI.")
    open_cmd.add_argument("--blender", default=None, metavar="EXE")

    args = parser.parse_args()

    from jubilee_twin.driver import TwinDriver
    driver = TwinDriver(blender_exe=args.blender if hasattr(args, "blender") else None)

    if args.command == "setup-scene":
        rc = driver.setup_scene()
        raise SystemExit(rc)

    elif args.command == "place-tools":
        rc = driver.place_tools()
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
            tool_data = twin_dir() / "pipeline_data" / "tool_data.csv"
            if not pathout.exists() or not tool_data.exists():
                raise SystemExit(
                    "pipeline_data/ CSVs not found. Run 'jubilee-twin animate <gcode>' first, "
                    "or pass a gcode file: 'jubilee-twin raytrace <gcode>'"
                )
        rc = driver.run_raytracing(interactive=args.interactive)
        raise SystemExit(rc)

    elif args.command == "open":
        driver.open_interactive()
