import argparse
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        prog="jubilee-twin",
        description="Run the Jubilee digital twin pipeline.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

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
    subparsers.add_parser("open", help="Open jubilee_belt.blend in the Blender GUI.")

    args = parser.parse_args()

    from jubilee_twin.driver import TwinDriver
    driver = TwinDriver(blender_exe=args.blender if hasattr(args, "blender") else None)

    if args.command == "prepare":
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
