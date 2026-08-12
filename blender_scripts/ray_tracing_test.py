
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
import bmesh
import blender_plots as bplt

sys.path.append(os.path.dirname("collision_detection.py"))
from collision_detection import _build_hull_geometry

# Path to the CSV file (relative to the .blend file)
csv_path = bpy.path.abspath("//../pipeline_data/pathout.csv")
scene = bpy.context.scene
depsgraph = bpy.context.evaluated_depsgraph_get()

points = []
with open(csv_path, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        points.append(tuple(map(float, row)))
        
points = np.array(points)

hit=False
directions = np.zeros_like(points)
points_norm = np.zeros_like(points)

scene.frame_set(1)

gantry = bpy.data.collections["gantry"]
xy_carriage = bpy.data.collections["XY-carriage"]

gantry_parts = [o for o in gantry.all_objects if o.type == 'MESH']
gantry_parts.extend([o for o in xy_carriage.all_objects if o.type == 'MESH'])

anchor = gantry_parts[0]
hull_bvh, hull_verts = _build_hull_geometry(gantry_parts,
                                            anchor=anchor,
                                            depsgraph=depsgraph)

gantry_vert = np.zeros((len(points),np.array(hull_verts).shape[0],3))

mesh_hull=bpy.data.meshes.new("Gantry Hull")
obj_hull=bpy.data.objects.new("Gantry Hull", mesh_hull)
bm=bmesh.new()

for vert in hull_verts:
    bm.verts.new(vert)

bm.to_mesh(mesh_hull)
bm.free()

obj_hull.constraints.new('COPY_TRANSFORMS')
obj_hull.constraints['Copy Transforms'].target = gantry_parts[0]

gantry.objects.link(obj_hull)
bpy.ops.object.origin_set(type = 'ORIGIN_GEOMETRY')

origin_offset = obj_hull.matrix_world.translation.copy()

for frame, (x, y, z), i in zip(range(1,len(points)+1), points, range(len(points))):
    scene.frame_set(frame)
    view_layer = bpy.context.view_layer
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    obj_eval = obj_hull.evaluated_get(depsgraph)
    mesh_eval = obj_eval.to_mesh()
    
    gantry_vert[i] = np.array([obj_eval.matrix_world @ v.co for v in mesh_eval.vertices] )

    points_norm[i] = [(x/1000 + origin_offset.x), 
                      (y/1000 + origin_offset.y), 
                      (origin_offset.z)]

for o in gantry_parts:
    o.hide_set(True)

bpy.context.view_layer.update()

for frame, i in zip(range(1,len(points)+1), range(len(points))):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    if frame < len(points)-1:
        directions[frame] = points_norm[i+1] - points_norm[i]
            
        direction = directions[frame]
        
        for coord in gantry_vert[i]:
            hit, loc, normal, index, object, mat = scene.ray_cast(
                depsgraph, 
                coord,
                direction,
                distance=np.linalg.norm(direction)
            )
            if hit:
                print(f"Collision between gantry and {object.name} during frame {frame}")


for o in gantry_parts:
    o.hide_set(False)

bpy.context.view_layer.update()

colors = [c for c in np.linspace((0, 0, 1), (1, 0, 0), num=len(points), dtype=float)]

for frame,i in zip(range(1, len(points)),range(len(points))):
    print(colors[i])
    bplt.Arrows(gantry_vert[i],
                np.tile(directions[frame], (len(gantry_vert[i]), 1)),
                color=colors[i],
                name=f"frame_{frame}_directions",
                head_length=0.001,
                radius=.0005,
                radius_ratio=2,
                end_trim_length=0)
    
print("DONE")