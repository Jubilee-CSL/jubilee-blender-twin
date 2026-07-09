import sys
import os
import bpy
import csv
import numpy as np
import mathutils
from pathlib import Path

SCRIPT_DIR = Path(os.path.dirname(os.path.abspath(__file__)))

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import get_axis_min, get_axis_max


# Animate 'X-axis' in X, 'Y-axis' in Y, and 'Z-axis' in Z if present
x_axis = bpy.data.objects.get("X-axis")
y_axis = bpy.data.objects.get("Y-axis")
z_axis = bpy.data.objects.get("Z-axis")
if x_axis is None:
    raise Exception("No object named 'X-axis' in the scene!")
if y_axis is None:
    raise Exception("No object named 'Y-axis' in the scene!")


x_max = get_axis_max(x_axis, 'X')
y_max= get_axis_max(y_axis, 'Y')

x_axis.location.x = x_max
y_axis.location.y = y_max

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
    # unlink from all parents first
    for parent in bpy.data.collections:
        if col.name in parent.children:
            parent.children.unlink(col)
    if col.name in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.unlink(col)
    # remove objects
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
    blend_dir = SCRIPT_DIR.parent / "Tools"
    data_csv = os.path.join(SCRIPT_DIR, "tool_data.csv")

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

                # Append tool collection
                with bpy.data.libraries.load(str(blend_file), link=False) as (data_from, data_to):
                    if tool_name in data_from.collections:
                        data_to.collections = [tool_name]
                    else:
                        print(f"Collection '{tool_name}' not found in {blend_file}")
                        continue

                # Place into Tools collection instead of scene root
                appended_tool = bpy.data.collections.get(tool_name)
                if appended_tool:
                    # unlink from scene root if auto-linked
                    if tool_name in bpy.context.scene.collection.children:
                        bpy.context.scene.collection.children.unlink(appended_tool)
                    tools_col.children.link(appended_tool)

                # Append park post
                park_blend = SCRIPT_DIR.parent / "Tool Post STL" / "park_post_47.blend"
                with bpy.data.libraries.load(str(park_blend), link=False) as (data_from, data_to):
                    if "park_post_47" in data_from.collections:
                        data_to.collections = ["park_post_47"]

                # Rename and place into Tool Park collection
                appended_park = bpy.data.collections.get("park_post_47")
                if appended_park:
                    appended_park.name = park_name
                    if appended_park.name in bpy.context.scene.collection.children:
                        bpy.context.scene.collection.children.unlink(appended_park)
                    # unlink from any other parent collections
                    for col in bpy.data.collections:
                        if appended_park.name in [c.name for c in col.children]:
                            col.children.unlink(appended_park)
                    toolpark_col.children.link(appended_park)

                # Place both at correct gcode position
                move_to_pos(tool_name, float(row[2]), float(row[3]))
                move_to_pos(park_name,  float(row[2]), float(row[3]))

            else:
                print(f"Tool {tool_id} ({tool_name}): no .blend file found")


def main():
    place_tools()

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    bpy.ops.wm.quit_blender()

if __name__ == "__main__":
    main()