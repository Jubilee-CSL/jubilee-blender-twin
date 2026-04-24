import bpy
import bmesh
import time
from mathutils.bvhtree import BVHTree

source_object_name = "greiner_24_wellplate_3300ul"
target_collection_name = "Z-axis"
collision_margin = .075
attribute_name = "Collision Frame"
enable_report = True

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

depsgraph = bpy.context.evaluated_depsgraph_get()

# Build source BVH once in local space — valid because source doesn't deform.
# Assumes source object scale is applied; if not, collision_margin should be
# expressed in source local-space units rather than world-space units.
source_eval = source_object.evaluated_get(depsgraph)
source_bm = bmesh.new()
source_bm.from_object(source_eval, depsgraph)
source_bvh = BVHTree.FromBMesh(source_bm)
source_bm.free()

# Convert collision_margin to source local space to account for uniform object scale.
source_scale = source_eval.scale
if not (abs(source_scale.x - source_scale.y) < 1e-6 and abs(source_scale.y - source_scale.z) < 1e-6):
    print(f"Warning: Source object has non-uniform scale {source_scale}; collision margin conversion will be approximate.")
local_collision_margin = collision_margin / source_scale.x
print(f"Collision margin: {collision_margin} (world) → {local_collision_margin:.6f} (source local, scale={source_scale.x:.4f})")

# Build combined convex hull of all target objects, stored in anchor-local space.
# This assumes all objects in the collection move together as a rigid body.
# The first object in the collection acts as the anchor for the shared local space.
anchor = target_objects[0]
anchor_matrix_inv = anchor.matrix_world.inverted()

combined_bm = bmesh.new()
for obj in target_objects:
    obj_eval = obj.evaluated_get(depsgraph)
    temp_bm = bmesh.new()
    temp_bm.from_object(obj_eval, depsgraph)
    transform = anchor_matrix_inv @ obj_eval.matrix_world
    for v in temp_bm.verts:
        combined_bm.verts.new(transform @ v.co)
    temp_bm.free()

combined_bm.verts.ensure_lookup_table()
result = bmesh.ops.convex_hull(combined_bm, input=combined_bm.verts[:])
hull_verts = [v.co.copy() for v in result['geom'] if isinstance(v, bmesh.types.BMVert)]
combined_bm.free()

print(f"Convex hull: {len(hull_verts)} vertices")

frame_num = bpy.context.scene.frame_start
frame_curr = bpy.context.scene.frame_current
frames_processed = 0

t_frame_set = 0.0
t_depsgraph = 0.0
t_matrices = 0.0
t_bvh_query = 0.0

if enable_report:
    print(f"Processing started under Collision Margin method...")

while frame_num <= bpy.context.scene.frame_end:

    t0 = time.perf_counter()
    bpy.context.scene.frame_set(frame_num)
    t1 = time.perf_counter()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    t2 = time.perf_counter()
    anchor_matrix = anchor.evaluated_get(depsgraph).matrix_world
    source_matrix_inv = source_object.evaluated_get(depsgraph).matrix_world.inverted()
    t3 = time.perf_counter()

    for v_co in hull_verts:
        local_pos = source_matrix_inv @ (anchor_matrix @ v_co)
        _, _, _, distance = source_bvh.find_nearest(local_pos)
        if distance is not None and distance <= local_collision_margin:
            for obj in target_objects:
                obj[attribute_name] = frame_num
            if enable_report:
                print(f"Collision detected at frame {frame_num}")
            break

    t4 = time.perf_counter()

    t_frame_set  += t1 - t0
    t_depsgraph  += t2 - t1
    t_matrices   += t3 - t2
    t_bvh_query  += t4 - t3
    frames_processed += 1

    if enable_report:
        print(f"Frame {frame_num} | frame_set={t1-t0:.4f}s  depsgraph={t2-t1:.4f}s  matrices={t3-t2:.4f}s  bvh={t4-t3:.4f}s")

    # Exit after timing the collision frame too
    if distance is not None and distance <= local_collision_margin:
        break

    frame_num += 1

if frames_processed:
    n = frames_processed
    print(f"\n--- Timing summary ({n} frame(s)) ---")
    print(f"  frame_set:  {t_frame_set:.3f}s total  ({t_frame_set/n*1000:.1f}ms avg)")
    print(f"  depsgraph:  {t_depsgraph:.3f}s total  ({t_depsgraph/n*1000:.1f}ms avg)")
    print(f"  matrices:   {t_matrices:.3f}s total  ({t_matrices/n*1000:.1f}ms avg)")
    print(f"  bvh query:  {t_bvh_query:.3f}s total  ({t_bvh_query/n*1000:.1f}ms avg)")
    print(f"  total:      {t_frame_set+t_depsgraph+t_matrices+t_bvh_query:.3f}s")

bpy.context.scene.frame_set(frame_curr)
print("Processing completed successfully.")
