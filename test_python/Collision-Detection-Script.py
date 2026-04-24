import bpy
import bmesh
from mathutils.bvhtree import BVHTree

####################################################################################################
#
###      Collision detection script written by Jeet for 5 Minutes Blender YouTube channel        ###
#
####################################################################################################

source_object_name = "greiner_24_wellplate_3300ul"
target_collection_name = "Z-axis"
collision_margin = 0.075
attribute_name = "Collision Frame"
enable_report = True
# Check every Nth vertex per target object. Higher values are faster but less precise.
# A value of 1 checks all vertices. Safe to increase for rigid, convex target objects.
vertex_sample_rate = 1

####################################################################################################

######################## No need to change anything below this line ################################


if source_object_name not in bpy.data.objects:
    print(f"Error: Source object '{source_object_name}' not found in scene.")
    raise KeyError(f"Source object '{source_object_name}' not found.")

if target_collection_name not in bpy.data.collections:
    print(f"Error: Target collection '{target_collection_name}' not found in scene.")
    raise KeyError(f"Target collection '{target_collection_name}' not found.")

source_object = bpy.data.objects[source_object_name]
target_coll = bpy.data.collections[target_collection_name]
target_objects = [obj for obj in target_coll.all_objects if obj.type == 'MESH' and obj != source_object]
num_of_objects = len(target_objects)

if num_of_objects == 0:
    print(f"Error: Target collection '{target_collection_name}' contains no eligible mesh objects.")
    raise ValueError(f"Target collection '{target_collection_name}' has no eligible mesh objects.")

print(f"Source object: '{source_object_name}'")
print(f"Target collection: '{target_collection_name}' ({num_of_objects} eligible mesh object(s))")

# Build source BVH once in local space — valid because source doesn't deform.
# Assumes source object scale is applied (i.e. scale is (1,1,1)); if not, queried
# distances will be in local space units rather than world-space units.
depsgraph = bpy.context.evaluated_depsgraph_get()
source_bm = bmesh.new()
source_bm.from_object(source_object.evaluated_get(depsgraph), depsgraph)
source_bvh = BVHTree.FromBMesh(source_bm)
source_bm.free()

frame_num = bpy.context.scene.frame_start
frame_curr = bpy.context.scene.frame_current
collision_detected = [False] * num_of_objects
collision_count = 0

if enable_report:
    print(f"Processing started under Collision Margin method...")

while frame_num <= bpy.context.scene.frame_end and collision_count < num_of_objects:

    bpy.context.scene.frame_set(frame_num)
    depsgraph = bpy.context.evaluated_depsgraph_get()

    # Recompute inverse each frame in case source translates/rotates
    source_matrix_inv = source_object.evaluated_get(depsgraph).matrix_world.inverted()

    for index, obj in enumerate(target_objects):
        if collision_detected[index]:
            continue

        target_eval = obj.evaluated_get(depsgraph)
        target_mesh = target_eval.to_mesh()
        matrix = target_eval.matrix_world
        vertices = target_mesh.vertices

        for i in range(0, len(vertices), vertex_sample_rate):
            local_pos = source_matrix_inv @ (matrix @ vertices[i].co)
            _, _, _, distance = source_bvh.find_nearest(local_pos)
            print(f"{distance=}")
            if distance is not None and distance <= collision_margin:
                collision_detected[index] = True
                obj[attribute_name] = frame_num
                collision_count += 1
                if enable_report:
                    print(f"Collision detected for {obj.name}")
                break

        target_eval.to_mesh_clear()

    if enable_report:
        print(f"Frame number {frame_num} is processed.")

    frame_num += 1

bpy.context.scene.frame_set(frame_curr)
print("Processing completed successfully.")
