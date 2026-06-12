
"""
ray_tracing_test.py
Based on animate_path.py

Run via the Blender CLI:
    blender jubilee.blend --python ray_tracing_test.py

"""

import bpy
import csv
import sys
import os
import numpy as np
import blender_plots as bplt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import get_axis_min, get_axis_max

# Path to the CSV file (relative to the .blend file)
csv_path = bpy.path.abspath("//from_gcode/pathout.csv")
scene = bpy.context.scene
depsgraph = bpy.context.evaluated_depsgraph_get()

x_axis = bpy.data.objects.get("X-axis")
y_axis = bpy.data.objects.get("Y-axis")
z_axis = bpy.data.objects.get("Z-axis")
if x_axis is None:
    raise Exception("No object named 'X-axis' in the scene!")
if y_axis is None:
    raise Exception("No object named 'Y-axis' in the scene!")

z_axis = bpy.data.objects.get("Z-axis")

x_min = get_axis_min(x_axis, 'X')
y_min = get_axis_min(y_axis, 'Y')
z_max = get_axis_max(z_axis, 'Z') if z_axis is not None else 0.0

points = []
with open(csv_path, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        points.append(tuple(map(float, row)))
        
points = np.array(points)
print(points)

hit=False
directions = np.zeros_like(points)
points_norm = np.zeros_like(points)
coord_nozz = np.zeros_like(points)
nozzle = bpy.data.objects.get("D2HW_C201H001")

scene.frame_set(1)
bpy.context.view_layer.update()
depsgraph = bpy.context.evaluated_depsgraph_get()
nozzle_eval = nozzle.evaluated_get(depsgraph)
origin_offset = nozzle_eval.matrix_world.translation.copy()
#print(f"Origin offset: {origin_offset}")

for obj_name in ["X-axis", "Y-axis", "Z-axis", "D2HW_C201H001"]:
    obj = bpy.data.objects.get(obj_name)
    if obj:
        pos = obj.evaluated_get(depsgraph).matrix_world.translation
        #print(f"{obj_name} world pos at frame 1: {pos.x:.5f}, {pos.y:.5f}, {pos.z:.5f}")

for frame, (x, y, z), i in zip(range(1,len(points)+1), points, range(len(points))):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()

    nozzle_eval = nozzle.evaluated_get(depsgraph)
    world_pos = nozzle_eval.matrix_world.translation

    print(f"Frame {frame} | nozzle: {world_pos.x:.5f}, {world_pos.y:.5f}, {world_pos.z:.5f}")
    coord_nozz[i] = [(world_pos.x), world_pos.y,(world_pos.z)]

    points_norm[i] = [(x/1000 + origin_offset.x), 
                      (y/1000 + origin_offset.y), 
                      (origin_offset.z - z/1000)]
    
    print(origin_offset.x)
    print(origin_offset.y)
    print(frame)

for frame, i in zip(range(1,len(points)+1), range(len(points))):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()

    if frame < len(points)-1:
        directions[frame] = points_norm[i+1] - points_norm[i]
            
        direction = directions[frame]
        norm = np.linalg.norm(direction)
        
        if norm > 1e-6:
            direction_normalized = direction / norm
        else:
            direction_normalized = np.array([0.0, 0.0, -1.0]) 
        
        hit, loc, normal, index, object, mat = scene.ray_cast(
            depsgraph, coord_nozz[i], direction_normalized
        )
        if hit:
            print(f"Collision between gantry and {object.name} during frame {frame}")


#print(f"(DEBUG) Direction Vectors:{directions}")
#print(f"(DEBUG)Normalized coordinates : {points_norm}")
#print(f"(DEBUG) Nozzle_coordinates:{coord_nozz}")

bplt.Arrows(coord_nozz[0:9],directions[1:],color = (1,0,0),head_length=0.0001, radius=.0005,radius_ratio=2,end_trim_length=0)

print("DONE")