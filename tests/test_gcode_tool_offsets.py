"""Tool offsets in the gcode parser follow RepRapFirmware semantics."""

import pytest

from jubilee_twin.pipeline import gcode_handlers, path_follower, utils

PEN = 1
PEN_OFFSET = [3.0, 38.0, -16.95]

# T1 pickup, draw start and park, as in the Duet's tpost1/tool_lock/tfree1.
# The lock's H1 move stops on the torque switch, so U ends anywhere.
GCODE = """
G90
G0 X270 Y270 F20000
G60 S0
G53 G1 X210 F6000
G53 G1 Y338 F6000
G91
G1 U10 F6000 H0
G1 U200 F6000 H1
G90
G1 R0 Y0
G1 R0 X0
G0 X178.670 Y42.170 F4000
G1 Z0.0 F800
G91
G1 Z2
G90
G53 G0 X210 Y270 F12000
G53 G1 Y338 F6000
G91
G1 U-4 F9000 H2
G1 U-360 F9000 H1
G90
G53 G1 Y305 F6000
G0 X100 Y100
"""


@pytest.fixture
def path(monkeypatch):
    monkeypatch.setattr(gcode_handlers, "_load_tool_offsets", lambda: {PEN: PEN_OFFSET})
    monkeypatch.setattr(utils, "_identify_tool", lambda x, y: float(PEN))
    rows = path_follower.build_path(GCODE.strip().splitlines(), distance_per_step=1e9)
    return [tuple(round(float(v), 3) for v in row) for row in rows]


def rows_where(path, predicate):
    return [row for row in path if predicate(row)]


def test_pickup_is_in_machine_coordinates(path):
    lock = rows_where(path, lambda r: r[4] == 1.0)
    assert len(lock) == 1
    assert lock[0][:2] == (210.0, 338.0)
    assert lock[0][5] == PEN


def test_restore_after_pickup_applies_the_new_offset(path):
    # G60 saved user (270, 270) with no tool; restoring it with the pen
    # loaded moves the carriage to 270 - offset.
    lock_index = path.index(rows_where(path, lambda r: r[4] == 1.0)[0])
    assert path[lock_index + 1][:2] == (210.0, 232.0)
    assert path[lock_index + 2][:2] == (267.0, 232.0)


def test_moves_with_the_pen_loaded_are_offset(path):
    assert (175.67, 4.17) in [row[:2] for row in path]
    # G1 Z0 with the pen: Z0 - (-16.95); the relative Z2 lift stays relative.
    z_values = [r[2] for r in rows_where(path, lambda r: r[:2] == (175.67, 4.17))]
    assert z_values[-2:] == pytest.approx([16.95, 18.95])


def test_park_is_in_machine_coordinates_and_clears_the_offset(path):
    unlock = rows_where(path, lambda r: r[4] == -1.0)
    assert len(unlock) == 1
    assert unlock[0][:2] == (210.0, 338.0)
    assert path[-1][:2] == (100.0, 100.0)


@pytest.mark.parametrize("lock, unlock", [
    ("G1 U200 F6000 H1", "G1 U-360 F6000 H1"),  # current Duet tool_lock.g
    ("G1 U80 F1500 H1", "G1 U-360 F9000 H1"),   # older macros (U ends at 90)
])
def test_lock_and_unlock_are_detected_by_direction(monkeypatch, lock, unlock):
    monkeypatch.setattr(gcode_handlers, "_load_tool_offsets", lambda: {PEN: PEN_OFFSET})
    monkeypatch.setattr(utils, "_identify_tool", lambda x, y: float(PEN))
    lines = ["G91", "G1 U10 H0", lock, "G1 U-4 H2", unlock, "G1 U5 H0", "G90"]
    flags = [row[4] for row in path_follower.build_path(lines, distance_per_step=1e9)]
    assert flags.count(1.0) == 1 and flags.count(-1.0) == 1
    assert flags.index(1.0) < flags.index(-1.0)
