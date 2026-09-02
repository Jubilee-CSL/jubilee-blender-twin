"""Headless entry point for ``jubilee-twin scan``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description="Render a Jubilee virtual-scanner image grid.")
    parser.add_argument("--x-min", type=float, required=True)
    parser.add_argument("--x-max", type=float, required=True)
    parser.add_argument("--x-steps", type=int, required=True)
    parser.add_argument("--y-min", type=float, required=True)
    parser.add_argument("--y-max", type=float, required=True)
    parser.add_argument("--y-steps", type=int, required=True)
    parser.add_argument("--z-min", type=float, required=True)
    parser.add_argument("--z-max", type=float, required=True)
    parser.add_argument("--z-steps", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--output-root", type=Path, default=None)
    args = parser.parse_args(argv)

    addon_root = str(Path(__file__).resolve().parent.parent)
    if addon_root not in sys.path:
        sys.path.insert(0, addon_root)
    from jubilee_digital_twin.virtual_scanner import ScanConfig, run_scan

    output = run_scan(ScanConfig(
        x_min=args.x_min, x_max=args.x_max, x_steps=args.x_steps,
        y_min=args.y_min, y_max=args.y_max, y_steps=args.y_steps,
        z_min=args.z_min, z_max=args.z_max, z_steps=args.z_steps,
        width=args.width, height=args.height, output_root=args.output_root,
    ))
    print(f"[virtual_scanner] Output: {output}")


if __name__ == "__main__":
    main()