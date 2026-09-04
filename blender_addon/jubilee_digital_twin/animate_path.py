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


def apply_offset(x, y, z, tool_id, pipeline_data):
    from jubilee_twin import machine_data
    tool = machine_data.tool_by_id(machine_data.load(pipeline_data), tool_id)
    if tool:
        ox, oy, oz = tool["offsets"]
        x += ox / 1000
        y += oy / 1000
        z += oz / 1000
    return x, y, z


def identify_tool(tool_id, pipeline_data):
    from jubilee_twin import machine_data
    tool = machine_data.tool_by_id(machine_data.load(pipeline_data), tool_id)
    return tool["name"] if tool else None


def apply_machine_state(positions: dict, active_tool_idx) -> None:
    """Position axes and mount the active tool without inserting keyframes."""
    _, pipeline_data = _setup_paths()
    from jubilee_twin.pipeline.utils import get_axis_max

    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    z_axis = bpy.data.objects.get("Z-axis")
    if x_axis is None or y_axis is None:
        raise RuntimeError("X-axis or Y-axis not found — run Place Tools first")

    x_max = get_axis_max(x_axis, 'X')
    y_max = get_axis_max(y_axis, 'Y')
    z_max = get_axis_max(z_axis, 'Z') if z_axis is not None else 0.0

    x = float(positions.get("X", 0.0))
    y = float(positions.get("Y", 0.0))
    z = float(positions.get("Z", 0.0))

    active_idx = int(active_tool_idx) if active_tool_idx is not None else -1
    if active_idx >= 0:
        x, y, z = apply_offset(x, y, z, float(active_idx), pipeline_data)

    x_axis.location.x = x_max - x / 1000
    y_axis.location.y = y_max - y / 1000
    if z_axis is not None:
        z_axis.location.z = z_max - z / 1000

    gantry = bpy.data.collections.get("gantry")
    if not gantry or not gantry.objects:
        return

    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    gantry_anchor = gantry.objects[0]
    gantry_world = gantry_anchor.evaluated_get(depsgraph).matrix_world.copy()

    active_name = identify_tool(float(active_idx), pipeline_data) if active_idx >= 0 else None
    # active_name may be None if the tool index has no entry in machine.json

    tools_col = bpy.data.collections.get("Tools")
    if tools_col:
        for tool_col in tools_col.children:
            is_active = tool_col.name == active_name
            for obj in tool_col.objects:
                for c in [c for c in obj.constraints if c.type == 'CHILD_OF']:
                    obj.constraints.remove(c)
                if is_active:
                    c = obj.constraints.new('CHILD_OF')
                    c.target = gantry_anchor
                    c.inverse_matrix = gantry_world.inverted()
                    c.influence = 1.0


def run_animation():
    """Read pipeline_data/ CSVs and bake keyframes into the open .blend scene."""
    csv_path, pipeline_data = _setup_paths()
    from jubilee_twin.pipeline.utils import get_axis_min, get_axis_max
    addon_dir = os.path.dirname(os.path.abspath(__file__))
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    import scene_utils

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
    print(f"Axis mins: X={x_min}, Y={y_min}")

    scene_utils.drive_to_mm(0.0, 0.0, 0.0)  # park at home before keying

    # Pass 1: insert position keyframes
    tool_flag = False
    for frame, (x, y, z, u, toolchange, tool_id) in enumerate(points, start=1):
        if toolchange > 0.0:
            tool_flag = True
        elif toolchange < 0.0:
            tool_flag = False

        if float(tool_id) != -1.0 and tool_flag is not None:
            x, y, z = apply_offset(x, y, z, float(tool_id), pipeline_data)

        scene_utils.drive_to_mm(x, y, z, frame=frame)

    # Pass 2: handle toolchanges now that all keyframes exist
    tool_flag = False
    tool = None
    tool_name = None

    for frame, (x, y, z, u, toolchange, tool_id) in enumerate(points, start=1):
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
            gantry_anchor = gantry.objects[0]
            bpy.context.scene.frame_set(frame)
            bpy.context.view_layer.update()
            # Read matrix_world from the evaluated depsgraph — required in --background mode,
            # where accessing obj.matrix_world directly returns the stale, pre-animation value.
            depsgraph = bpy.context.evaluated_depsgraph_get()
            gantry_anchor_eval = gantry_anchor.evaluated_get(depsgraph)
            gantry_world = gantry_anchor_eval.matrix_world.copy()

            print(f"Frame {frame}: gantry at {gantry_world.translation}")

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

    if "--background" in sys.argv:
        bpy.ops.wm.save_mainfile()
        print(f"Saved {bpy.data.filepath}")


if __name__ == "__main__":
    run_animation()
