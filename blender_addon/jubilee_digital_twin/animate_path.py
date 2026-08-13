"""
animate_path.py
Blender Python script that reads pathout.csv and inserts one keyframe per row
on the X-axis, Y-axis, and Z-axis scene objects.

Run via the Blender CLI:
    blender jubilee.blend --python animate_path.py

Or call run_animation() from the Digital Twin addon.
"""

import bpy
import csv
import sys
import os
import numpy as np


def _setup_paths():
    """Compute twin_root and CSV paths from the currently open .blend file."""
    twin_root = os.path.dirname(os.path.dirname(bpy.data.filepath))
    if twin_root not in sys.path:
        sys.path.insert(0, twin_root)
    csv_path = bpy.path.abspath("//../pipeline_data/pathout.csv")
    tool_data_path = bpy.path.abspath("//../pipeline_data/tool_data.csv")
    return csv_path, tool_data_path


def apply_offset(x, y, z, tool_id, tool_data_path):
    with open(tool_data_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            if row[0] == tool_id:
                x += float(row[5]) / 1000
                y += float(row[6]) / 1000
                z += float(row[7]) / 1000
    return x, y, z


def identify_tool(tool_id, tool_data_path):
    with open(tool_data_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            if float(row[0]) == tool_id:
                return row[1]
    return None


def run_animation():
    """Read pipeline_data/ CSVs and bake keyframes into the open .blend scene."""
    csv_path, tool_data_path = _setup_paths()
    from jubilee_twin.pipeline.utils import get_axis_min, get_axis_max

    gantry = bpy.data.collections["gantry"]

    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    z_axis = bpy.data.objects.get("Z-axis")
    if x_axis is None:
        raise Exception("No object named 'X-axis' in the scene!")
    if y_axis is None:
        raise Exception("No object named 'Y-axis' in the scene!")

    for obj in [x_axis, y_axis, z_axis]:
        if obj is not None:
            obj.animation_data_clear()

    points = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            points.append(tuple(map(float, row)))
    points = np.array(points)

    x_min = get_axis_min(x_axis, 'X')
    y_min = get_axis_min(y_axis, 'Y')
    x_max = get_axis_max(x_axis, 'X')
    y_max = get_axis_max(y_axis, 'Y')
    z_max = get_axis_max(z_axis, 'Z') if z_axis is not None else 0.0

    x_axis.location.x = x_max
    y_axis.location.y = y_max
    if z_axis is not None:
        z_axis.location.z = z_max

    print(f"Axis maxes: X={x_max}, Y={y_max}, Z={z_max}")
    print(f"Axis mins: X={x_min}, Y={y_min}")

    # Pass 1: insert position keyframes
    tool_flag = False
    for frame, (x, y, z, u, toolchange, tool_id) in enumerate(points, start=1):
        if toolchange > 0.0:
            tool_flag = True
        elif toolchange < 0.0:
            tool_flag = False

        if float(tool_id) != -1.0 and tool_flag is not None:
            x, y, z = apply_offset(x, y, z, float(tool_id), tool_data_path)

        x_axis.location.x = (x_max - x / 1000)
        x_axis.keyframe_insert(data_path="location", frame=frame)
        y_axis.location.y = (y_max - y / 1000)
        y_axis.keyframe_insert(data_path="location", frame=frame)
        if z_axis is not None:
            z_axis.location.z = z_max - z / 1000
            z_axis.keyframe_insert(data_path="location", frame=frame)

    # Pass 2: handle toolchanges now that all keyframes exist
    tool_flag = False
    tool = None
    tool_name = None

    for frame, (x, y, z, u, toolchange, tool_id) in enumerate(points, start=1):
        if toolchange > 0:
            tool_name = identify_tool(float(tool_id), tool_data_path)
            tool = bpy.data.collections.get(tool_name)
            if tool is None:
                print(f"Collection {tool_name} not found")
                continue

            print(tool.name)
            gantry_anchor = gantry.objects[0]
            bpy.context.scene.frame_set(frame)
            bpy.context.view_layer.update()
            # Read matrix_world from the evaluated depsgraph — required in --background mode,
            # where accessing obj.matrix_world directly returns the stale, pre-animation value.
            depsgraph = bpy.context.evaluated_depsgraph_get()
            gantry_anchor_eval = gantry_anchor.evaluated_get(depsgraph)
            gantry_world = gantry_anchor_eval.matrix_world.copy()

            print(f"Frame {frame}: gantry at {gantry_world.translation}")
            print(f"Frame {frame}: expected dock ~({(x_max - x/1000):.4f}, {(y_max - y/1000):.4f})")
            print(f"x_min={x_min}, y_min={y_min}")
            print(f"raw gcode at toolchange: x={x}mm, y={y}mm")
            print(f"expected blender pos: ({(x_max - x/1000):.4f}, {(y_max - y/1000):.4f})")
            print(f"gantry actual pos: {gantry_world.translation.xy}")

            for obj in tool.objects[:]:
                to_remove = [c for c in obj.constraints if c.type == 'CHILD_OF']
                for c in to_remove:
                    obj.constraints.remove(c)

                c = obj.constraints.new('CHILD_OF')
                c.target = gantry_anchor
                c.inverse_matrix = gantry_world.inverted()
                c.influence = 0.0
                c.keyframe_insert(data_path="influence", frame=frame - 1)
                c.influence = 1.0
                c.keyframe_insert(data_path="influence", frame=frame)

        elif toolchange < 0:
            tool = bpy.data.collections.get(tool_name) if tool_name else None
            if tool:
                for obj in tool.objects[:]:
                    c = obj.constraints.get('Child Of')
                    if c:
                        c.influence = 1.0
                        c.keyframe_insert(data_path="influence", frame=frame)
                        c.influence = 0.0
                        c.keyframe_insert(data_path="influence", frame=frame + 1)
                        tool_flag = False
                        tool = None
                        tool_name = None

    bpy.context.scene.frame_end = len(points)
    print(f"Animation complete: {len(points)} frames.")


if __name__ == "__main__":
    run_animation()
