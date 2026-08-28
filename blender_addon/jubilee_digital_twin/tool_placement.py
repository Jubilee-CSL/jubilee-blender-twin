import sys
import os
import logging
import bpy
import csv
import mathutils
from pathlib import Path

logger = logging.getLogger("jubilee_twin.tool_placement")


def _ensure_colorlog() -> None:
    try:
        from jubilee_twin.log import get_logger as _gl
        _gl("jubilee_twin.tool_placement")
    except Exception:
        pass


def _setup():
    addon_dir = str(Path(__file__).parent)
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    import scene_utils
    scene_utils.ensure_pipeline_on_path()
    return scene_utils.twin_root()


def get_or_create_collection(name):
    if name in bpy.data.collections:
        return bpy.data.collections[name]
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col

def remove_collection(name):
    col = bpy.data.collections.get(name)
    if col is None:
        return
    
    for parent in bpy.data.collections:
        if col.name in parent.children:
            parent.children.unlink(col)
    if col.name in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.unlink(col)

    for obj in col.objects[:]:
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(col)

def find_tool_blend(tool_name, blend_dir):
    candidate = blend_dir / f"{tool_name}.blend"
    return candidate if candidate.exists() else None

def move_to_pos(tool_name,x_pos,y_pos):
    collection = bpy.data.collections[tool_name]
    object_z_ref = bpy.data.objects['m5_locking_t_nut_20x20_generic026']
    object_xy_ref = bpy.data.objects['m3_6mm_buttonhead_screw_92095A179_92095A194']
    _,__,z_ref = object_z_ref.matrix_world.to_translation()
    x_ref,y_ref,___ = object_xy_ref.matrix_world.to_translation()
    for obj in collection.objects[:]:  
        obj.matrix_world.translation = mathutils.Vector(((x_ref-(x_pos-4)/1000), y_ref - ((y_pos + 27)/1000) , z_ref + 28/1000))
        

def place_tools():
    _ensure_colorlog()
    twin_root = _setup()
    logger.info("Placing tools from tool_data.csv")

    # Use live head position from machine_status.json if available; else park at home.
    import json as _json
    status_path = twin_root / "pipeline_data" / "machine_status.json"
    try:
        with open(status_path) as _f:
            _status = _json.load(_f)
        _pos = _status.get("head_position", {})
        _src = _status.get("source", "machine_status.json")
        x_mm = _pos.get("X", 0.0)
        y_mm = _pos.get("Y", 0.0)
        z_mm = _pos.get("Z", 0.0)
        scene_utils.drive_to_mm(x_mm, y_mm, z_mm)
        logger.warning("Head position from %s: X=%.1f Y=%.1f Z=%.1f", _src, x_mm, y_mm, z_mm)
    except Exception:
        scene_utils.drive_to_mm(0.0, 0.0, 0.0)
        logger.warning("No machine_status.json — gantry parked at home")

    from jubilee_twin.pipeline.utils import get_axis_max
    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    if x_axis is None:
        raise Exception("No object named 'X-axis' in the scene!")
    if y_axis is None:
        raise Exception("No object named 'Y-axis' in the scene!")

    blend_dir = twin_root / "Tools"
    data_csv = twin_root / "pipeline_data" / "tool_data.csv"
    logger.warning("tool_data.csv ← %s", data_csv)
    logger.warning("tool blends   ← %s", blend_dir)

    # Get or create target parent collections
    tools_col    = get_or_create_collection("Tools")
    toolpark_col = get_or_create_collection("Tool Park")

    with open(data_csv, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)

        for row in reader:
            tool_name, tool_id = row[1], row[0]
            park_name = f"park_post_{tool_id}"
            blend_file = find_tool_blend(tool_name, blend_dir)

            if blend_file:
                # Remove existing collections so we start fresh
                remove_collection(tool_name)
                remove_collection(park_name)

                logger.warning("Tool %s (%s)  ← %s", tool_id, tool_name, blend_file)
                logger.info(  "  park   X=%.1f  Y=%.1f  Z=%.1f", float(row[2]), float(row[3]), float(row[4]))
                logger.info(  "  offset X=%.3f  Y=%.3f  Z=%.3f", float(row[5]), float(row[6]), float(row[7]))

                # Append tool collection
                with bpy.data.libraries.load(str(blend_file), link=False) as (data_from, data_to):
                    if tool_name in data_from.collections:
                        data_to.collections = [tool_name]
                    else:
                        logger.warning("Collection '%s' not found in %s", tool_name, blend_file)
                        continue

                appended_tool = bpy.data.collections.get(tool_name)
                if appended_tool:
                    if tool_name in bpy.context.scene.collection.children:
                        bpy.context.scene.collection.children.unlink(appended_tool)
                    tools_col.children.link(appended_tool)

                # Append park post
                park_blend = twin_root / "Tool Post STL" / "park_post_47.blend"
                with bpy.data.libraries.load(str(park_blend), link=False) as (data_from, data_to):
                    if "park_post_47" in data_from.collections:
                        data_to.collections = ["park_post_47"]

                appended_park = bpy.data.collections.get("park_post_47")
                if appended_park:
                    appended_park.name = park_name
                    if appended_park.name in bpy.context.scene.collection.children:
                        bpy.context.scene.collection.children.unlink(appended_park)

                    for col in bpy.data.collections:
                        if appended_park.name in [c.name for c in col.children]:
                            col.children.unlink(appended_park)
                    toolpark_col.children.link(appended_park)


                move_to_pos(tool_name, float(row[2]), float(row[3]))
                move_to_pos(park_name,  float(row[2]), float(row[3]))

            else:
                logger.warning("Tool %s (%s): no .blend file found", tool_id, tool_name)


def main():
    place_tools()

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    bpy.ops.wm.quit_blender()

if __name__ == "__main__":
    main()