"""
helper functions for path_follower
Adapted from: https://github.com/TanmayChhatbar/blender_3d_print_animation/blob/main/utils.py
Original author: Tanmay Chhatbar
"""
from math import sqrt
import numpy as np


def _extract_numeric_after(token: str) -> float | None:
    """Parse the first numeric token that follows an axis letter on a gcode line.

    Strips any inline comment (text after ';') before parsing so that values
    like 'X10.0 ; comment' are handled correctly.  Returns None if no valid
    float is found.
    """
    token = token.split(";", 1)[0]
    parts = [p for p in token.strip().split() if p]
    if not parts:
        return None
    try:
        return float(parts[0])
    except ValueError:
        return None


def find_coord(line: str, curcoord: np.ndarray, relative: bool = False) -> np.ndarray:
    """Extract target coordinates from a single gcode line.

    Scans for X, Y, Z, and F tokens and updates the corresponding element of
    curcoord.  Axes not mentioned in the line keep their current values.

    When relative is False (G90 absolute mode) each axis value replaces the
    current position directly.  When relative is True (G91 relative mode) each
    axis value is an offset added to the current position.  The feedrate (F) is
    always interpreted as an absolute value regardless of positioning mode,
    matching Duet firmware behaviour.

    Returns a new array; curcoord is not modified.
    """
    nl = curcoord.copy()
    checks = ["X", "Y", "Z", "F"]
    for i, axis in enumerate(checks):
        if axis in line:
            after = line.split(axis, 1)[1]
            val = _extract_numeric_after(after)
            if val is not None:
                if relative and axis != 'F':
                    nl[i] = curcoord[i] + val
                else:
                    nl[i] = val
    return nl


def dis(loc1, loc2) -> float:
    """Euclidean distance between two 3-D points (only the first three elements are used)."""
    return sqrt(sum((np.array(loc1)-np.array(loc2))[0:3]**2))


def get_axis_min(obj, axis: str) -> float:
    """Return the lower bound for the given axis from a Blender LIMIT_LOCATION constraint.

    Used by animate_path.py to find where each physical axis sits at its home
    position in the scene.  Returns 0.0 if no matching constraint is found.
    """
    for constraint in obj.constraints:
        if constraint.type == 'LIMIT_LOCATION':
            if axis == 'X':
                return constraint.min_x if constraint.use_min_x else 0.0
            elif axis == 'Y':
                return constraint.min_y if constraint.use_min_y else 0.0
            elif axis == 'Z':
                return constraint.min_z if constraint.use_min_z else 0.0
    return 0.0


def get_axis_max(obj, axis: str) -> float:
    """Return the upper bound for the given axis from a Blender LIMIT_LOCATION constraint.

    Used by animate_path.py to establish the Z home position (the bed is at its
    highest point when Z=0 in gcode).  Returns 0.0 if no matching constraint is found.
    """
    for constraint in obj.constraints:
        if constraint.type == 'LIMIT_LOCATION':
            if axis == 'X':
                return constraint.max_x if constraint.use_max_x else 0.0
            elif axis == 'Y':
                return constraint.max_y if constraint.use_max_y else 0.0
            elif axis == 'Z':
                return constraint.max_z if constraint.use_max_z else 0.0
    return 0.0