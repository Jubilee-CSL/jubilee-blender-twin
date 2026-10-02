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
    """Return the toolpath CSV and the pipeline_data directory for the current .blend."""
    addon_dir = os.path.dirname(os.path.abspath(__file__))
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    import scene_utils
    scene_utils.ensure_pipeline_on_path()
    csv_path = bpy.path.abspath("//../pipeline_data/pathout.csv")
    pipeline_data = bpy.path.abspath("//../pipeline_data")
    return csv_path, pipeline_data


def identify_tool(tool_id, pipeline_data):
    from jubilee_twin import machine_data
    tool = machine_data.tool_by_id(machine_data.load(pipeline_data), tool_id)
    return tool["name"] if tool else None


def tool_roots(collection):
    """Objects that inherit no transform from another object in this tool."""
    objects = set(collection.all_objects)
    return [obj for obj in collection.all_objects if obj.parent not in objects]


def clear_tool_constraints(collection):
    """Remove previous mounts, including old constraints on child meshes."""
    for obj in collection.all_objects:
        for constraint in [c for c in obj.constraints if c.type == 'CHILD_OF']:
            obj.constraints.remove(constraint)


def carriage_anchor():
    """Use the named carriage, which inherits both X and Y axis motion."""
    anchor = bpy.data.objects.get("XY-carriage")
    if anchor is None:
        raise RuntimeError("XY-carriage not found — run setup-scene first")
    return anchor


def run_animation():
    """Read pipeline_data/ CSVs and bake keyframes into the open .blend scene."""
    csv_path, pipeline_data = _setup_paths()
    addon_dir = os.path.dirname(os.path.abspath(__file__))
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    import scene_utils

    gantry_anchor = carriage_anchor()

    tools_col = bpy.data.collections.get("Tools")
    if tools_col:
        for tool_col in tools_col.children:
            clear_tool_constraints(tool_col)

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

    scene_utils.drive_to_mm(0.0, 0.0, 0.0)  # park at home before keying

    # Pass 1: insert position keyframes. Rows are machine coordinates, with
    # tool offsets already applied by the gcode parser.
    for frame, (x, y, z, *_) in enumerate(points, start=1):
        scene_utils.drive_to_mm(x, y, z, frame=frame)

    # Pass 2: handle toolchanges now that all keyframes exist
    tool = None
    tool_name = None

    for frame, (_x, _y, _z, _u, toolchange, tool_id) in enumerate(points, start=1):
        if toolchange > 0:
            tool_name = identify_tool(float(tool_id), pipeline_data)
            if tool_name is None:
                print(f"Tool id {tool_id} not found in machine.json — skipping tool-change at frame {frame}")
                continue
            tool = bpy.data.collections.get(tool_name)
            if tool is None:
                print(f"Collection '{tool_name}' not found in scene — was Place Tools run?")
                continue

            print(tool.name)
            bpy.context.scene.frame_set(frame)
            bpy.context.view_layer.update()
            # Read matrix_world from the evaluated depsgraph — required in --background mode,
            # where accessing obj.matrix_world directly returns the stale, pre-animation value.
            depsgraph = bpy.context.evaluated_depsgraph_get()
            gantry_anchor_eval = gantry_anchor.evaluated_get(depsgraph)
            gantry_world = gantry_anchor_eval.matrix_world.copy()

            print(f"Frame {frame}: gantry at {gantry_world.translation}")

            clear_tool_constraints(tool)
            for obj in tool_roots(tool):
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
                for obj in tool_roots(tool):
                    c = obj.constraints.get('Child Of')
                    if c:
                        c.influence = 1.0
                        c.keyframe_insert(data_path="influence", frame=frame)
                        c.influence = 0.0
                        c.keyframe_insert(data_path="influence", frame=frame + 1)
                    tool = None
                    tool_name = None

    bpy.context.scene.frame_end = len(points)
    print(f"Animation complete: {len(points)} frames.")

    if "--background" in sys.argv:
        bpy.ops.wm.save_mainfile()
        print(f"Saved {bpy.data.filepath}")


if __name__ == "__main__":
    run_animation()
