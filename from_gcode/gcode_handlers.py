"""
gcode_handlers.py
Stateful gcode command handler registry.

GCodeMachine mirrors the runtime state of a Duet controller.  Each handler
method processes one gcode line, updates internal machine state (e.g.
positioning mode), and returns a list of interpolated [x, y, z, f] positions
produced by that command.  Non-motion commands (G90, G91, ...) return an empty
list and only mutate state.

To add support for a new gcode command:
  1. Add a method  handle_Gxx(self, line, current_pos, distance_per_step)
     that returns list[np.ndarray].  Update state as needed before returning.
  2. Register it in _build_registry().
"""
from __future__ import annotations

import numpy as np
from math import ceil
from utils import find_coord, dis


# ---------------------------------------------------------------------------
# Gcode line parsing  (stateless - lives at module level)
# ---------------------------------------------------------------------------

def parse_command(line: str) -> str | None:
    """Return the uppercase command word from a gcode line, or None for blank/comment lines."""
    line = line.split(';')[0].strip()
    if not line:
        return None
    return line.split()[0].upper()


# ---------------------------------------------------------------------------
# Interpolation helper  (stateless - lives at module level)
# ---------------------------------------------------------------------------

def _interpolate_linear(
    start: np.ndarray,
    end: np.ndarray,
    distance_per_step: float,
) -> list[np.ndarray]:
    """
    Return evenly-spaced positions along a straight line from start to end.

    Excludes start, includes end.  Step count is ceil(length / distance_per_step)
    so each step is at most distance_per_step long.  Zero-length moves return
    [end] unchanged.
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
# Machine
# ---------------------------------------------------------------------------

class GCodeMachine:
    """
    Stateful handler registry that mirrors a Duet controller's runtime state.

    Handlers are bound methods so they can both produce interpolated path
    positions and update internal state (positioning mode, etc.) in one step,
    exactly as the real controller would.

    State fields
    ------------
    _absolute_mode : bool
        True  -> G90 absolute positioning (Duet power-on default).
        False -> G91 relative positioning.
    """

    def __init__(self) -> None:
        self._absolute_mode: bool = True  # G90 is the Duet power-on default
        self._registry: dict[str, callable] = self._build_registry()

    def _build_registry(self) -> dict[str, callable]:
        return {
            'G0':  self.handle_G0,
            'G1':  self.handle_G1,
            'G90': self.handle_G90,
            'G91': self.handle_G91,
        }

    def get_handler(self, command: str) -> callable | None:
        """Return the handler for a gcode command, or None if not registered."""
        return self._registry.get(command.upper())

    # --- internal helpers ---

    def _resolve_target(self, line: str, current_pos: np.ndarray) -> np.ndarray:
        """Resolve the target position for a move, respecting the current positioning mode."""
        return find_coord(line, current_pos, relative=not self._absolute_mode)

    # --- motion handlers ---

    def handle_G0(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Rapid positioning move - straight-line interpolation."""
        target = self._resolve_target(line, current_pos)
        return _interpolate_linear(current_pos, target, distance_per_step)

    def handle_G1(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Controlled linear move - straight-line interpolation."""
        target = self._resolve_target(line, current_pos)
        return _interpolate_linear(current_pos, target, distance_per_step)

    # --- positioning mode handlers ---

    def handle_G90(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Switch to absolute positioning mode (Duet power-on default)."""
        self._absolute_mode = True
        return []

    def handle_G91(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Switch to relative positioning mode."""
        self._absolute_mode = False
        return []
