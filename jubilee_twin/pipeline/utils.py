"""
helper functions for path_follower
Adapted from: https://github.com/TanmayChhatbar/blender_3d_print_animation/blob/main/utils.py
Original author: Tanmay Chhatbar
"""
from math import sqrt
import numpy as np
import csv 
import os
import sys

from jubilee_twin import machine_data

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
pipeline_data_dir = os.path.join(SCRIPT_DIR, "..", "..", "pipeline_data")

def _identify_tool(x, y):
    """Return the tool_id whose park position is nearest (X,Y); returns None if no match within 30 mm."""
    tool_id = machine_data.nearest_tool_id(machine_data.load(pipeline_data_dir), x, y)
    return float(tool_id) if tool_id is not None else None


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


def find_coord(line: str, curcoord: np.ndarray, storage: np.ndarray = [], relative: bool = False) -> np.ndarray:
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
    nl[4] = 0.0

    restored_val=np.array([0.0,0.0,0.0,0.0,0.0,-1.0])

    split_line = line.split(";", 1)[0]

    checks = ["X", "Y", "Z", "U","H"]

    val = None

    for i, axis in enumerate(checks):
        
        if "S" in split_line: #save coord

            after = line.split("S", 1)[1]
            val = _extract_numeric_after(after)

            if axis != "F":
                storage[int(val),i] = curcoord[i]
                nl[i:4] = curcoord[i:4]

        if "R" in split_line:  #load saved coord

            after = line.split("R", 1)[1]
            val = _extract_numeric_after(after)

            restored_val = storage[int(val)]
            if axis in split_line:
                recurr_line = axis + after.split(axis, 1)[1]
                offset, storage = find_coord(recurr_line, restored_val, storage, relative=True)
                nl[i:4] = offset[i:4]

            continue


        if axis in split_line:
            after = line.split(axis, 1)[1]
            val = _extract_numeric_after(after)
            if val is not None:

                print(f"[Debug find_coord] Found axis {axis} with value {val}")

                if axis == "H":
                    print(f"[H branch] nl[3]={nl[3]}, curcoord[3]={curcoord[3]}, relative={relative}")
                    print(f"[H branch] nl[5] before={nl[5]}")
                    if nl[3] == 90.0:
                        nl[4] = 1.0
                        result = _identify_tool(curcoord[0], curcoord[1])
                        print(f"[H branch] _identify_tool returned: {result}")
                        nl[5] = result
                    elif nl[3] == -364.0:
                        nl[4] = -1.0
                        nl[5] = -1.0

 
                elif relative and axis != "T":
                    nl[i] = curcoord[i] + val

                else:
                    nl[i] = val
    
    return nl,storage

def dis(loc1, loc2) -> float:
    """Euclidean distance between two 3-D points (only the first three elements are used)."""
    return sqrt(sum((np.array(loc1)-np.array(loc2))[0:5]**2))


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