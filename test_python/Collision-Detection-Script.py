import bpy
import bmesh
import time
from enum import Enum, auto
from dataclasses import dataclass
from mathutils.bvhtree import BVHTree


# ---------------------------------------------------------------------------
# Enums and dataclasses
# ---------------------------------------------------------------------------

class CandidateMode(Enum):
    SINGLE                = auto()
    COLLECTION_HULL       = auto()
    COLLECTION_INDIVIDUAL = auto()


@dataclass
class CollisionCandidate:
    name: str                  # key used in collision_pairs
    mode: CandidateMode
    object_name: str = ""      # SINGLE only
    collection_name: str = ""  # COLLECTION_HULL / COLLECTION_INDIVIDUAL only


@dataclass
class BVHEntry:
    obj: object        # bpy.types.Object whose local space the BVH lives in
    bvh: object        # BVHTree
    correction: object # Matrix — constraint-driven offset captured at ref_frame
    local_margin: float


@dataclass
class VertexEntry:
    anchor: object     # bpy.types.Object whose local space verts are stored in
    verts: list        # list[Vector] — convex hull verts in anchor-local space
    correction: object # Matrix


@dataclass
class PreparedCandidate:
    name: str
    bvh_entries: list    # list[BVHEntry]  — 1 for SINGLE/HULL, N for INDIVIDUAL
    vertex_entries: list # list[VertexEntry] — 1 for SINGLE/HULL, N for INDIVIDUAL


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

collision_margin               = 0.065
attribute_name                 = "Collision Frame"
enable_report                  = True
INDIVIDUAL_PAIR_WARN_THRESHOLD = 10

candidates = [
    CollisionCandidate(
        name="wellplate",
        mode=CandidateMode.SINGLE,
        object_name="greiner_24_wellplate_3300ul",
    ),
    CollisionCandidate(
        name="z-axis",
        mode=CandidateMode.COLLECTION_HULL,
        collection_name="Z-axis",
    ),
]

# First in each pair = BVH side; second = vertex side.
collision_pairs = [
    ("wellplate", "z-axis"),
]


# ---------------------------------------------------------------------------
# FCurve / matrix helpers (no frame_set)
# ---------------------------------------------------------------------------

def get_location_fcurves(obj):
    """
    Return location FCurves for obj.
    Handles both legacy actions (Blender < 4.4, action.fcurves) and
    layered/slotted actions (Blender 4.4+, action.layers → strips → channelbag).
    """
    if not (obj.animation_data and obj.animation_data.action):
        return []
    action = obj.animation_data.action
    if hasattr(action, 'layers'):
        slot = obj.animation_data.action_slot
        result = []
        for layer in action.layers:
            for strip in layer.strips:
                cb = strip.channelbag(slot)
                if cb:
                    result.extend(fc for fc in cb.fcurves if fc.data_path == 'location')
        return result
    return [fc for fc in action.fcurves if fc.data_path == 'location']


def eval_matrix_at_frame(obj, frame):
    """
    Compute an object's world matrix at a given frame using fcurve evaluation,
    without calling frame_set. Handles arbitrary parent chains.

    Assumes only location is animated; rotation and scale are taken from the
    object's current values (i.e. they don't change between frames).
    Does NOT evaluate constraints — use correction matrices to account for
    constraint-driven offsets that aren't visible via .parent.
    """
    loc = list(obj.location)
    for fc in get_location_fcurves(obj):
        loc[fc.array_index] = fc.evaluate(frame)

    mat = obj.matrix_basis.copy()
    mat.translation = loc

    if obj.parent:
        return eval_matrix_at_frame(obj.parent, frame) @ obj.matrix_parent_inverse @ mat
    return mat


# ---------------------------------------------------------------------------
# Preprocessing helpers
# ---------------------------------------------------------------------------

def compute_correction(obj, depsgraph, ref_frame):
    """
    Captures any constraint-driven world-space offset (e.g. Child Of) that is
    not visible through .parent. Computed once at ref_frame using Blender's
    fully-evaluated world matrix. Valid as long as constraint targets don't animate.
    """
    blender_world = obj.evaluated_get(depsgraph).matrix_world
    return blender_world @ eval_matrix_at_frame(obj, ref_frame).inverted()


def build_object_bvh(obj, depsgraph):
    """Builds a BVHTree from obj's full evaluated mesh in obj-local space."""
    obj_eval = obj.evaluated_get(depsgraph)
    bm = bmesh.new()
    bm.from_object(obj_eval, depsgraph)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()
    return bvh


def _build_hull_geometry(objects, anchor, depsgraph):
    """
    Builds a combined convex hull of all objects' meshes in anchor-local space.
    Returns (hull_bvh, hull_verts):
      hull_bvh   — BVHTree built from hull faces (for use as the BVH side of a pair)
      hull_verts — list[Vector] of hull vertex positions (for use as the vertex side)
    Falls back to all input vertices if the hull has fewer than 4 vertices.
    """
    anchor_matrix_inv = anchor.evaluated_get(depsgraph).matrix_world.inverted()
    combined_bm = bmesh.new()
    for obj in objects:
        obj_eval = obj.evaluated_get(depsgraph)
        temp_bm = bmesh.new()
        temp_bm.from_object(obj_eval, depsgraph)
        transform = anchor_matrix_inv @ obj_eval.matrix_world
        for v in temp_bm.verts:
            combined_bm.verts.new(transform @ v.co)
        temp_bm.free()

    combined_bm.verts.ensure_lookup_table()
    hull_result = bmesh.ops.convex_hull(combined_bm, input=combined_bm.verts[:])

    hull_geom_verts = [item for item in hull_result['geom'] if isinstance(item, bmesh.types.BMVert)]
    hull_verts = [v.co.copy() for v in hull_geom_verts]

    if len(hull_verts) < 4:
        print(f"Warning: Convex hull for '{anchor.name}' has only {len(hull_verts)} "
              f"vertices — falling back to all input vertices.")
        hull_verts = [v.co.copy() for v in combined_bm.verts]

    # Build a clean bmesh from hull faces for BVH construction.
    hull_bm = bmesh.new()
    old_to_new = {v: hull_bm.verts.new(v.co) for v in hull_geom_verts}
    for item in hull_result['geom']:
        if isinstance(item, bmesh.types.BMFace):
            face_verts = [old_to_new[v] for v in item.verts if v in old_to_new]
            if len(face_verts) >= 3:
                try:
                    hull_bm.faces.new(face_verts)
                except ValueError:
                    pass
    hull_bvh = BVHTree.FromBMesh(hull_bm)
    hull_bm.free()
    combined_bm.free()

    return hull_bvh, hull_verts


def compute_local_margin(obj, depsgraph, margin):
    """Converts world-space collision_margin to obj-local space (uniform scale assumed)."""
    scale = obj.evaluated_get(depsgraph).scale
    if not (abs(scale.x - scale.y) < 1e-6 and abs(scale.y - scale.z) < 1e-6):
        print(f"Warning: '{obj.name}' has non-uniform scale {scale}; "
              f"margin conversion will be approximate.")
    return margin / scale.x


def prepare_candidate(candidate, depsgraph, margin):
    """Builds all BVHEntry and VertexEntry instances for a CollisionCandidate."""
    ref_frame = bpy.context.scene.frame_start
    mode = candidate.mode

    if mode == CandidateMode.SINGLE:
        obj = bpy.data.objects[candidate.object_name]
        correction = compute_correction(obj, depsgraph, ref_frame)
        local_margin = compute_local_margin(obj, depsgraph, margin)
        bvh = build_object_bvh(obj, depsgraph)
        _, hull_verts = _build_hull_geometry([obj], anchor=obj, depsgraph=depsgraph)
        return PreparedCandidate(
            name=candidate.name,
            bvh_entries=[BVHEntry(obj=obj, bvh=bvh, correction=correction, local_margin=local_margin)],
            vertex_entries=[VertexEntry(anchor=obj, verts=hull_verts, correction=correction)],
        )

    elif mode == CandidateMode.COLLECTION_HULL:
        coll = bpy.data.collections[candidate.collection_name]
        objects = [o for o in coll.all_objects if o.type == 'MESH']
        anchor = objects[0]
        correction = compute_correction(anchor, depsgraph, ref_frame)
        local_margin = compute_local_margin(anchor, depsgraph, margin)
        hull_bvh, hull_verts = _build_hull_geometry(objects, anchor=anchor, depsgraph=depsgraph)
        return PreparedCandidate(
            name=candidate.name,
            bvh_entries=[BVHEntry(obj=anchor, bvh=hull_bvh, correction=correction, local_margin=local_margin)],
            vertex_entries=[VertexEntry(anchor=anchor, verts=hull_verts, correction=correction)],
        )

    elif mode == CandidateMode.COLLECTION_INDIVIDUAL:
        coll = bpy.data.collections[candidate.collection_name]
        objects = [o for o in coll.all_objects if o.type == 'MESH']
        bvh_entries = []
        vertex_entries = []
        for obj in objects:
            correction = compute_correction(obj, depsgraph, ref_frame)
            local_margin = compute_local_margin(obj, depsgraph, margin)
            bvh = build_object_bvh(obj, depsgraph)
            _, hull_verts = _build_hull_geometry([obj], anchor=obj, depsgraph=depsgraph)
            bvh_entries.append(BVHEntry(obj=obj, bvh=bvh, correction=correction, local_margin=local_margin))
            vertex_entries.append(VertexEntry(anchor=obj, verts=hull_verts, correction=correction))
        return PreparedCandidate(
            name=candidate.name,
            bvh_entries=bvh_entries,
            vertex_entries=vertex_entries,
        )

    else:
        raise ValueError(f"Unknown CandidateMode: {mode}")


# ---------------------------------------------------------------------------
# Per-frame collision check
# ---------------------------------------------------------------------------

def check_pair(a_prep, b_prep, frame):
    """
    Returns (collision, min_distance, t_matrices, t_bvh).
    a_prep provides BVH entries (BVH side); b_prep provides vertex entries (query side).
    Breaks early on first detected collision.
    """
    min_distance = float('inf')
    collision = False
    t_matrices = 0.0
    t_bvh = 0.0

    for bvh_e in a_prep.bvh_entries:
        t0 = time.perf_counter()
        obj_matrix_inv = (bvh_e.correction @ eval_matrix_at_frame(bvh_e.obj, frame)).inverted()
        t1 = time.perf_counter()
        t_matrices += t1 - t0

        for vert_e in b_prep.vertex_entries:
            t0 = time.perf_counter()
            anchor_matrix = vert_e.correction @ eval_matrix_at_frame(vert_e.anchor, frame)
            t1 = time.perf_counter()
            t_matrices += t1 - t0

            t0 = time.perf_counter()
            for v_co in vert_e.verts:
                local_pos = obj_matrix_inv @ (anchor_matrix @ v_co)
                _, _, _, dist = bvh_e.bvh.find_nearest(local_pos)
                if dist is not None:
                    if dist < min_distance:
                        min_distance = dist
                    if dist <= bvh_e.local_margin:
                        collision = True
                        break
            t1 = time.perf_counter()
            t_bvh += t1 - t0

            if collision:
                break
        if collision:
            break

    return collision, min_distance, t_matrices, t_bvh


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_and_collect_objects(candidates, collision_pairs):
    """
    Validates all referenced objects/collections exist and pair names resolve.
    Emits a performance warning for INDIVIDUAL×INDIVIDUAL pairs above the threshold.
    """
    candidate_map = {c.name: c for c in candidates}

    for c in candidates:
        if c.mode == CandidateMode.SINGLE:
            if c.object_name not in bpy.data.objects:
                raise KeyError(f"Candidate '{c.name}': object '{c.object_name}' not found.")
            if bpy.data.objects[c.object_name].type != 'MESH':
                raise ValueError(
                    f"Candidate '{c.name}': '{c.object_name}' is not a MESH "
                    f"(type={bpy.data.objects[c.object_name].type}).")
        else:
            if c.collection_name not in bpy.data.collections:
                raise KeyError(f"Candidate '{c.name}': collection '{c.collection_name}' not found.")
            meshes = [o for o in bpy.data.collections[c.collection_name].all_objects
                      if o.type == 'MESH']
            if not meshes:
                raise ValueError(
                    f"Candidate '{c.name}': collection '{c.collection_name}' has no MESH objects.")

    for a_name, b_name in collision_pairs:
        for name in (a_name, b_name):
            if name not in candidate_map:
                raise KeyError(f"Collision pair references unknown candidate '{name}'.")
        ca, cb = candidate_map[a_name], candidate_map[b_name]
        if (ca.mode == CandidateMode.COLLECTION_INDIVIDUAL
                and cb.mode == CandidateMode.COLLECTION_INDIVIDUAL):
            na = len([o for o in bpy.data.collections[ca.collection_name].all_objects
                      if o.type == 'MESH'])
            nb = len([o for o in bpy.data.collections[cb.collection_name].all_objects
                      if o.type == 'MESH'])
            if na * nb > INDIVIDUAL_PAIR_WARN_THRESHOLD:
                print(f"Warning: Pair ('{a_name}', '{b_name}') is INDIVIDUAL×INDIVIDUAL "
                      f"with {na}×{nb}={na*nb} BVH-entry combinations per frame — may be slow.")


# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

validate_and_collect_objects(candidates, collision_pairs)

frame_curr = bpy.context.scene.frame_current
bpy.context.scene.frame_set(bpy.context.scene.frame_start)
depsgraph = bpy.context.evaluated_depsgraph_get()

prepared_map = {}
for c in candidates:
    print(f"Preparing candidate '{c.name}' (mode={c.mode.name}) ...")
    prepared_map[c.name] = prepare_candidate(c, depsgraph, collision_margin)
    pc = prepared_map[c.name]
    total_hull_verts = sum(len(ve.verts) for ve in pc.vertex_entries)
    print(f"  BVH entries: {len(pc.bvh_entries)}, vertex entries: {len(pc.vertex_entries)}, "
          f"hull verts total: {total_hull_verts}")

resolved_pairs = [
    (prepared_map[a_name], prepared_map[b_name])
    for a_name, b_name in collision_pairs
]


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

frame_num = bpy.context.scene.frame_start
frames_processed = 0
closest_distance = float('inf')
closest_frame = None
collision_found_frame = None
collision_found_pair = None

t_matrices_total = 0.0
t_bvh_total = 0.0

if enable_report:
    print(f"\nProcessing {len(resolved_pairs)} collision pair(s), "
          f"frames {bpy.context.scene.frame_start}–{bpy.context.scene.frame_end} ...")

while frame_num <= bpy.context.scene.frame_end:
    any_collision = False
    frame_min_distance = float('inf')
    frame_t_matrices = 0.0
    frame_t_bvh = 0.0

    for a_prep, b_prep in resolved_pairs:
        collision, min_dist, t_mat, t_bvh = check_pair(a_prep, b_prep, frame_num)
        frame_t_matrices += t_mat
        frame_t_bvh += t_bvh

        if min_dist < frame_min_distance:
            frame_min_distance = min_dist

        if collision:
            any_collision = True
            collision_found_frame = frame_num
            collision_found_pair = (a_prep.name, b_prep.name)
            print(f"Collision detected at frame {frame_num} — '{a_prep.name}' vs '{b_prep.name}'")
            for bvh_e in a_prep.bvh_entries:
                bvh_e.obj[attribute_name] = frame_num
            for vert_e in b_prep.vertex_entries:
                vert_e.anchor[attribute_name] = frame_num
            break

    t_matrices_total += frame_t_matrices
    t_bvh_total += frame_t_bvh
    frames_processed += 1

    if frame_min_distance < closest_distance:
        closest_distance = frame_min_distance
        closest_frame = frame_num

    if enable_report:
        print(f"Frame {frame_num} | min_dist={frame_min_distance:.6f} | "
              f"matrices={frame_t_matrices:.4f}s  bvh={frame_t_bvh:.4f}s")

    if any_collision:
        break

    frame_num += 1


# ---------------------------------------------------------------------------
# Reporting and cleanup
# ---------------------------------------------------------------------------

if frames_processed:
    n = frames_processed
    print(f"\n--- Timing summary ({n} frame(s), {len(resolved_pairs)} pair(s)) ---")
    print(f"  matrices:   {t_matrices_total:.3f}s total  ({t_matrices_total/n*1000:.2f}ms avg)")
    print(f"  bvh query:  {t_bvh_total:.3f}s total  ({t_bvh_total/n*1000:.2f}ms avg)")
    print(f"  total:      {t_matrices_total+t_bvh_total:.3f}s")
    print(f"  closest measured distance: {closest_distance:.6f} (BVH-object local) at frame {closest_frame}")
    if collision_found_frame is not None:
        print(f"  first collision: frame {collision_found_frame}, pair {collision_found_pair}")
    else:
        print("  no collision detected across all pairs.")

bpy.context.scene.frame_set(frame_curr)
print("Processing completed successfully.")
