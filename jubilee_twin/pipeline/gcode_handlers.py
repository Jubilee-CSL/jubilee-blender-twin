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
from .utils import find_coord, dis, pipeline_data_dir


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
    total_dist = dis(start[0:4], end[0:4])
    if total_dist == 0.0 or distance_per_step <= 0.0:
        return [end.copy()]
    n_steps = max(1, ceil(total_dist / distance_per_step))
    
    tail = end[4:]
    
    return [
        np.concatenate([
            start[0:4] + (end[0:4] - start[0:4]) * (i / n_steps),
            tail
        ])
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
    _offset : np.ndarray
        G10 X/Y/Z offset of the locked tool (zero with no tool).

    Positions are machine coordinates. As in RepRapFirmware, coordinates in
    a regular move are user coordinates (machine = user - tool offset) while
    G53 moves are machine coordinates, so tool pickup/park moves are never
    offset. Saved positions (G60) hold user coordinates, so restoring one
    after a tool change accounts for the new tool's offset.
    """

    def __init__(self, tool_offsets: dict | None = None) -> None:
        self._absolute_mode: bool = True  # G90 is the Duet power-on default
        self._registry: dict[str, callable] = self._build_registry()
        self.storage: np.ndarray = np.zeros((3,6))
        self._tool_offsets = (
            _load_tool_offsets() if tool_offsets is None else tool_offsets
        )
        self._offset: np.ndarray = np.zeros(3)
        self._machine_coords: bool = False  # inside a G53 line

    def _build_registry(self) -> dict[str, callable]:
        return {
            'G0':  self.handle_G0,
            'G1':  self.handle_G1,
            'G90': self.handle_G90,
            'G91': self.handle_G91,
            'G60': self.handle_G60,
            'G53': self.handle_G53,
        }

    def get_handler(self, command: str) -> callable | None:
        """Return the handler for a gcode command, or None if not registered."""
        return self._registry.get(command.upper())

    # --- internal helpers ---

    def _resolve_target(self, line: str, current_pos: np.ndarray) -> np.ndarray:
        """Resolve the target machine position for a move, respecting the
        positioning mode and the locked tool's offset."""
        offset = np.zeros(3) if self._machine_coords else self._offset
        user_pos = current_pos.copy()
        user_pos[0:3] += offset
        target, new_storage = find_coord(
            line, user_pos, self.storage, relative=not self._absolute_mode
        )
        target[0:3] -= offset

        # The tool's offset applies from its lock until its unlock.
        if target[4] > 0.0:
            self._offset = np.array(
                self._tool_offsets.get(int(target[5]), (0.0, 0.0, 0.0)), dtype=float
            )
        elif target[4] < 0.0:
            self._offset = np.zeros(3)

        return target, new_storage

    # --- motion handlers ---

    def handle_G0(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Rapid positioning move - straight-line interpolation."""
        target,_ = self._resolve_target(line, current_pos)
        return _interpolate_linear(current_pos, target, distance_per_step)

    def handle_G1(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Controlled linear move - straight-line interpolation."""
        target,self.storage = self._resolve_target(line, current_pos)
        if target[4] == 0.0:
            return _interpolate_linear(current_pos, target, distance_per_step)
        else:
            return [target]

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
    
    def handle_G60(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        target,self.storage = self._resolve_target(line, current_pos)
        return _interpolate_linear(current_pos, target, distance_per_step)

    def handle_G53(
        self
        ,line: str
        ,current_pos: np.ndarray
        ,distance_per_step: float
    ) -> list[np.ndarray]:
        """Run the rest of the line in machine coordinates (no tool offset)."""
        nl = str(line[3:]).strip()
        command = parse_command(nl)
        handler = self.get_handler(command) if command else None
        if handler is None:
            return []
        self._machine_coords = True
        try:
            return handler(nl, current_pos, distance_per_step)
        finally:
            self._machine_coords = False


def _load_tool_offsets() -> dict[int, list[float]]:
    """{tool id: [x, y, z]} G10 offsets (mm) from pipeline_data/machine.json."""
    from jubilee_twin import machine_data

    return {
        int(tool["id"]): [float(v) for v in tool.get("offsets", [0.0, 0.0, 0.0])[:3]]
        for tool in machine_data.load(pipeline_data_dir).get("tools", [])
    }
