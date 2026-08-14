"""
path_follower.py
Adapted from: https://github.com/TanmayChhatbar/blender_3d_print_animation/blob/main/path_follower.py
Original author: Tanmay Chhatbar

Entry point for converting a G-code file into a CSV of interpolated 3D positions.
Run as:
    python path_follower.py <gcode_file> [distance_per_step]

Outputs pathout.csv in the current working directory, with one x,y,z row per step.
"""
import sys
import os

import numpy as np
from .gcode_handlers import parse_command, GCodeMachine
from jubilee_twin.paths import resolve

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Maximum mm between consecutive output positions. Larger values produce fewer
# rows (coarser animation); smaller values produce more rows (smoother animation).
DEFAULT_DISTANCE_PER_STEP = 50.0


def build_path(lines: list[str], distance_per_step: float) -> list[np.ndarray]:
    """
    Walk through gcode lines and build a list of interpolated [x, y, z, f] positions.

    Each registered command handler is responsible for producing the timestep
    positions for its segment.  Unknown commands are silently skipped.
    distance_per_step controls how far apart consecutive positions are along
    straight segments; smaller values produce denser paths.
    """
    # Machine starts at the origin; include it so frame 1 of the animation is at home.
    current_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0,-1.0])  # [x, y, z, u, toolchanging]
    path = [current_pos.copy()]
    machine = GCodeMachine()

    for line in lines:
        line = line.strip()

        # Returns None for blank lines and pure comment lines.
        command = parse_command(line)
        if command is None:
            continue

        # Commands without a registered handler (e.g. M-codes, T-codes) are skipped.
        handler = machine.get_handler(command)
        if handler is None:
            continue

        # Each handler returns a list of positions from current_pos (exclusive)
        # to the segment target (inclusive), spaced at most distance_per_step apart.
        # Non-motion handlers (G90, G91, ...) return [] and only update machine state.
        steps = handler(line, current_pos, distance_per_step)
        if steps :
            path.extend(steps)
            # Advance the position tracker to the end of this segment.
            current_pos = steps[-1].copy()

    return path


def run(
    gcode_file: str = None,
    distance_per_step: float = DEFAULT_DISTANCE_PER_STEP,
    output_dir: str = None,
) -> str:
    """Parse gcode and write pathout.csv. Returns the output path.

    ``gcode_file`` may be an absolute/relative path, a bare filename resolved
    against ``<jubilee_dir>/gcode_logs/``, or None to use ``latest.gcode``.
    """
    if gcode_file and os.path.isfile(gcode_file):
        fn = gcode_file
    else:
        gcode_logs = resolve("jubilee_dir") / "gcode_logs"
        candidate = gcode_logs / (gcode_file or "latest.gcode")
        if not candidate.is_file():
            raise FileNotFoundError(f"gcode file not found at: {candidate}")
        fn = str(candidate)
    with open(fn, 'r') as f:
        lines = f.readlines()
    path = build_path(lines, distance_per_step)

    out_dir = output_dir or os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "pipeline_data"))
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, "pathout.csv")
    with open(out_csv, 'w') as f2:
        for i, pos in enumerate(path):
            if i != 0:
                f2.write('\n')
            f2.write(f"{pos[0]},{pos[1]},{pos[2]},{pos[3]},{pos[4]},{pos[5]}")
    return out_csv


def main():
    fn = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        distance_per_step = float(sys.argv[2])
    except (IndexError, ValueError):
        distance_per_step = DEFAULT_DISTANCE_PER_STEP
    run(gcode_file=fn, distance_per_step=distance_per_step)


if __name__ == "__main__":
    main()
