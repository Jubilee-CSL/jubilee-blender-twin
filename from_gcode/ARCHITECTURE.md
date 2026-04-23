# from_gcode - Architecture Overview

This directory contains the pipeline that converts a G-code toolpath file into a
Blender animation of the Jubilee motion system.

---

## Pipeline Summary

```
path.gcode
    |
    v
path_follower.py   (entry point - driven by gcode_handlers.py + utils.py)
    |
    v
pathout.csv        (interpolated x,y,z positions, one row per animation frame)
    |
    v
animate_path.py    (Blender script - inserts keyframes on scene axis objects)
    |
    v
jubilee.blend      (animated Blender scene)
```

Running `run_latest_gcode_animation.bat` executes the full pipeline end-to-end.

---

## Files

### `path_follower.py`
The command-line entry point.  Reads a G-code file, delegates each line to the
appropriate command handler, and writes the resulting interpolated positions to
`pathout.csv`.

Usage:
```
python path_follower.py <gcode_file> [distance_per_step]
```

- `distance_per_step` (default `100.0` mm) controls how densely the path is
  sampled.  Smaller values produce more CSV rows and a smoother animation but
  take longer to process and animate.

### `gcode_handlers.py`
Defines a registry of per-command handler functions and the helpers used to
interpolate motion between positions.

- `parse_command(line)` - strips comments and returns the uppercase command word
  (e.g. `"G0"`, `"G1"`), or `None` for blank/comment-only lines.
- `_interpolate_linear(start, end, distance_per_step)` - returns evenly-spaced
  positions along a straight line segment.  The number of steps is
  `ceil(segment_length / distance_per_step)`, so no step exceeds
  `distance_per_step`.  Start is excluded; end is included.
- `handle_G0` / `handle_G1` - rapid and controlled linear moves respectively;
  both delegate to `_interpolate_linear`.
- `COMMAND_REGISTRY` - maps command strings to handler callables.  New commands
  are added here.

**Adding a new command:** write a handler with the signature
`handle_Gxx(line, current_pos, distance_per_step) -> list[np.ndarray]`
where each array is `[x, y, z, feedrate]`, then register it in
`COMMAND_REGISTRY`.

### `utils.py`
Low-level parsing and math helpers shared across the pipeline.

- `find_coord(line, curcoord)` - parses `X`, `Y`, `Z`, and `F` values out of a
  single G-code line and returns an updated `[x, y, z, f]` array.  Axes not
  present in the line retain their current values.
- `_extract_numeric_after(token)` - strips inline comments and converts the
  first whitespace-separated token to a float.
- `dis(loc1, loc2)` - Euclidean distance between two 3-D points.
- `get_axis_min` / `get_axis_max` - read the `LIMIT_LOCATION` constraint bounds
  from a Blender object; used by `animate_path.py` to find each axis's home
  position in the scene.

### `animate_path.py`
Blender Python script (run with `blender --python`) that reads `pathout.csv`
and inserts one keyframe per row on the `X-axis`, `Y-axis`, and `Z-axis` scene
objects.

Key details:
- CSV coordinates are in **millimetres**; they are divided by 1000 to convert to
  Blender's metre-based world units.
- The `x_min` / `y_min` offsets anchor frame 1 to the physical home position of
  each axis as encoded in the scene's `LIMIT_LOCATION` constraints.
- The Z axis is **inverted** (`z_max - z/1000`) because on the Jubilee the bed
  descends as the G-code Z value increases.
- The scene's end frame is set to `len(points)` so the timeline exactly covers
  the animation.

### `path.gcode`
Sample G-code file used for development and testing.  In production this is
replaced (by `run_latest_gcode_animation.bat`) with the latest file generated
by science-jubilee.

### `pathout.csv`
Intermediate output produced by `path_follower.py`.  Each row is
`x,y,z` in millimetres.  The file is consumed immediately by `animate_path.py`
and is not version-controlled.

---

## Data Flow in Detail

1. `path_follower.py` initialises `current_pos = [0, 0, 0, 0]` (machine origin).
2. Each G-code line is parsed for its command word.
3. If the command has a registered handler, the handler is called with the raw
   line, the current position, and `distance_per_step`.
4. The handler calls `find_coord` to extract the target position from the line,
   then `_interpolate_linear` to produce the intermediate steps.
5. All steps are appended to `path`; `current_pos` is advanced to the last step.
6. After all lines are processed, `path` is written as CSV.
7. `animate_path.py` reads the CSV inside Blender, converts units, and inserts
   keyframes so the scene plays back the full toolpath.
