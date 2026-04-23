"""
gcode_handlers.py
Command handler registry for gcode path interpolation.

To add support for a new gcode command:
  1. Write a handler with the signature:
       handle_Gxx(line: str, current_pos: np.ndarray, distance_per_step: float) -> list[np.ndarray]
     The handler should return a list of [x, y, z, f] positions from current_pos (exclusive)
     to the target (inclusive), spaced at most distance_per_step apart along the path.
  2. Register it in COMMAND_REGISTRY.
"""
from __future__ import annotations

import numpy as np
from math import ceil
from utils import find_coord, dis


# ---------------------------------------------------------------------------
# Gcode line parsing
# ---------------------------------------------------------------------------

def parse_command(line: str) -> str | None:
    """Return the uppercase command word from a gcode line, or None for blank/comment lines."""
    line = line.split(';')[0].strip()
    if not line:
        return None
    return line.split()[0].upper()


# ---------------------------------------------------------------------------
# Interpolation helpers
# ---------------------------------------------------------------------------

def _interpolate_linear(
    start: np.ndarray,
    end: np.ndarray,
    distance_per_step: float,
) -> list[np.ndarray]:
    """
    Return evenly-spaced positions along a straight line from start to end.

    The returned list excludes start and includes end.  The number of steps is
    ceil(segment_length / distance_per_step), so every step is at most
    distance_per_step long.  Zero-length moves return [end] unchanged.
    """
    total_dist = dis(start[0:3], end[0:3])
    if total_dist == 0.0 or distance_per_step <= 0.0:
        return [end.copy()]
    n_steps = max(1, ceil(total_dist / distance_per_step))
    return [
        start + (end - start) * (i / n_steps)
        for i in range(1, n_steps + 1)
    ]


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def handle_G0(
    line: str,
    current_pos: np.ndarray,
    distance_per_step: float,
) -> list[np.ndarray]:
    """Rapid positioning move - straight-line interpolation."""
    target = find_coord(line, current_pos)
    return _interpolate_linear(current_pos, target, distance_per_step)


def handle_G1(
    line: str,
    current_pos: np.ndarray,
    distance_per_step: float,
) -> list[np.ndarray]:
    """Controlled linear move - straight-line interpolation."""
    target = find_coord(line, current_pos)
    return _interpolate_linear(current_pos, target, distance_per_step)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

COMMAND_REGISTRY: dict[str, callable] = {
    'G0': handle_G0,
    'G1': handle_G1,
}


def get_handler(command: str) -> callable | None:
    """Return the handler for a gcode command, or None if not registered."""
    return COMMAND_REGISTRY.get(command.upper())
