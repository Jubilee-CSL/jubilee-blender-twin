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


def asset_path(value):
    """Validate an optional model path recorded in machine.json."""
    if not value:
        return None
    path = Path(value)
    if not path.exists():
        logger.warning("machine.json references a missing file: %s", path)
        return None
    return path


def pick_collection(available, tool_name):
    """The collection named after the tool, else the only collection in the file."""
    if tool_name in available:
        return tool_name
    return available[0] if len(available) == 1 else None

def move_to_pos(tool_name, x_pos, y_pos):
    collection = bpy.data.collections[tool_name]
    z_ref = bpy.data.objects['m5_locking_t_nut_20x20_generic026'].matrix_world.translation.z
    xy_ref = bpy.data.objects['m3_6mm_buttonhead_screw_92095A179_92095A194'].matrix_world.translation
    target = mathutils.Vector((
        xy_ref.x - (x_pos - 4) / 1000,
        xy_ref.y - (y_pos + 27) / 1000,
        z_ref + 4 / 1000,
    ))
    # A tool hangs off one Empty, so moving it is enough; park posts are bare meshes.
    empties = [o for o in collection.objects if o.type == 'EMPTY']
    for obj in empties or collection.objects[:]:
        obj.matrix_world.translation = target

def place_tools():
    _ensure_colorlog()
    twin_root = _setup()
    import scene_utils
    from jubilee_twin import machine_data

    pipeline_data = twin_root / "pipeline_data"
    machine = machine_data.load(pipeline_data)
    logger.warning("machine.json ← %s", machine_data.path_for(pipeline_data))
    logger.info("Machine state: %s", machine.get("source") or "unknown")

    head = machine.get("head_position")
    if head:
        x_mm, y_mm, z_mm = head.get("X", 0.0), head.get("Y", 0.0), head.get("Z", 0.0)
        scene_utils.drive_to_mm(x_mm, y_mm, z_mm)
        logger.warning("Head position: X=%.1f Y=%.1f Z=%.1f", x_mm, y_mm, z_mm)
    else:
        scene_utils.drive_to_mm(0.0, 0.0, 0.0)
        logger.warning("No head position recorded — gantry parked at home")

    from jubilee_twin.pipeline.utils import get_axis_max
    x_axis = bpy.data.objects.get("X-axis")
    y_axis = bpy.data.objects.get("Y-axis")
    if x_axis is None:
        raise Exception("No object named 'X-axis' in the scene!")
    if y_axis is None:
        raise Exception("No object named 'Y-axis' in the scene!")

    blend_dir = twin_root / "Tools"
    default_park_blend = twin_root / "Tool Post STL" / "park_post_47.blend"

    # Get or create target parent collections
    tools_col    = get_or_create_collection("Tools")
    toolpark_col = get_or_create_collection("Tool Park")

    for tool in machine.get("tools", []):
        tool_id, tool_name = tool["id"], tool["name"]
        park_x, park_y, park_z = tool["park"]
        park_name = f"park_post_{tool_id}"
        blend_file = asset_path(tool.get("blend")) or find_tool_blend(tool_name, blend_dir)

        if not blend_file:
            logger.warning("Tool %s (%s): no .blend file found", tool_id, tool_name)
            continue

        # Remove existing collections so we start fresh
        remove_collection(tool_name)
        remove_collection(park_name)

        logger.warning("Tool %s (%s)  ← %s", tool_id, tool_name, blend_file)
        logger.info(  "  park   X=%.1f  Y=%.1f  Z=%.1f", park_x, park_y, park_z)
        logger.info(  "  offset X=%.3f  Y=%.3f  Z=%.3f", *tool["offsets"])

        # Append tool collection
        collection_name = None
        with bpy.data.libraries.load(str(blend_file), link=False) as (data_from, data_to):
            collection_name = pick_collection(list(data_from.collections), tool_name)
            if collection_name is None:
                logger.warning(
                    "No unambiguous collection for '%s' in %s (found %s)",
                    tool_name, blend_file, list(data_from.collections),
                )
            else:
                data_to.collections = [collection_name]
        if collection_name is None:
            continue

        appended_tool = bpy.data.collections.get(collection_name)
        if appended_tool:
            appended_tool.name = tool_name
            if tool_name in bpy.context.scene.collection.children:
                bpy.context.scene.collection.children.unlink(appended_tool)
            tools_col.children.link(appended_tool)

        # Append park post: plugin-provided if any, else the twin's default.
        park_blend = asset_path(tool.get("park_post")) or default_park_blend
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

        move_to_pos(tool_name, park_x, park_y)
        move_to_pos(park_name,  park_x, park_y)


def main():
    place_tools()

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    bpy.ops.wm.quit_blender()

if __name__ == "__main__":
    main()