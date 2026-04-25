"""
collision_detection.py
======================
Blender Python library for proximity and collision detection between animated
scene objects or collections.

Overview
--------
Given two or more "candidates" (single objects or collections of objects) and a
list of pairs to check, this library scans an animation range frame by frame and
reports whether — and when — the candidates come within a configurable distance
of one another.

It is designed for use in lab-automation Blender scenes where detecting tool
collisions with the deck or labware is safety-critical.

Core concepts
-------------
**BVH tree (Bounding Volume Hierarchy)**
    A BVH tree is a spatial data structure that wraps a mesh so that "find the
    nearest point on this surface" queries run in O(log n) time rather than O(n).
    Imagine the geometry broken into a tree of nested bounding boxes — a query
    can quickly rule out large chunks of geometry before examining the details.

    In this library a BVH tree is built once for each candidate at startup and
    reused every frame. Because animated objects only *move* (they don't change
    shape), the BVH stays valid; only the query points need to be re-transformed
    each frame.

**Object-local space**
    Every Blender object has its own coordinate system ("local space"). A vertex
    at position (1, 0, 0) local may be far away in world space depending on the
    object's location, rotation, and scale. BVH queries are performed in the BVH
    object's local space so the tree never needs rebuilding. Per-frame work is
    just transforming query points into that local space.

**Convex hull approximation**
    Testing every vertex of a dense mesh against a BVH per frame is expensive.
    Instead, the library computes the *convex hull* of each candidate's geometry
    at startup — the smallest convex shell that contains all the geometry — and
    uses only those hull vertices as query points each frame. This is conservative:
    if no hull vertex is within the margin, nothing inside can be either.

**Correction matrices**
    Blender supports two ways to parent objects: explicit ``.parent`` relationships
    and *constraints* (e.g. "Child Of"). Constraints move an object in world space
    but leave ``.parent`` as ``None``, so a naive world-matrix reconstruction from
    animation curves alone gives the wrong result. The library captures this offset
    once at a reference frame as a correction matrix and applies it each frame.

**Avoiding frame_set**
    Advancing the timeline via ``bpy.context.scene.frame_set()`` triggers a full
    dependency-graph evaluation (~1 second per frame in typical scenes). This
    library reads animation FCurves directly and reconstructs world matrices in
    Python, allowing thousands of frames per second.

Quick-start example
-------------------
::

    import sys, os
    import bpy

    sys.path.append(bpy.path.abspath("//"))   # repo root where this file lives
    import collision_detection as cd

    result = cd.detect_collisions(
        candidates=[
            cd.CollisionCandidate(
                name="tool",
                mode=cd.CandidateMode.SINGLE,
                object_name="my_tool_object",
            ),
            cd.CollisionCandidate(
                name="deck",
                mode=cd.CandidateMode.COLLECTION_HULL,
                collection_name="Deck Objects",
            ),
        ],
        collision_pairs=[("tool", "deck")],
        collision_margin=0.001,          # 1 mm in world-space metres
        early_exit=True,
        log_level=cd.LogLevel.SUMMARY,
    )

    if result.collisions:
        print(f"First collision at frame {result.collisions[0].frame}")
    else:
        print(f"No collision. Closest: {result.closest_distance:.4f} "
              f"at frame {result.closest_frame}")
"""

import time
from dataclasses import dataclass
from enum import Enum, auto

import bpy
import bmesh
from mathutils.bvhtree import BVHTree


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_INDIVIDUAL_PAIR_WARN_THRESHOLD = 10
"""Emit a performance warning when an INDIVIDUAL×INDIVIDUAL pair would produce
more than this many BVH-entry combinations per frame."""


# ---------------------------------------------------------------------------
# Public enums
# ---------------------------------------------------------------------------

class CandidateMode(Enum):
    """
    Specifies how a :class:`CollisionCandidate` collects and represents geometry.

    SINGLE
        One named mesh object. Its full mesh is used to build the BVH tree (when
        this candidate is the surface side of a pair) and its convex hull is used
        as the set of query points (when it is the query side).

    COLLECTION_HULL
        A named collection of mesh objects treated as a single rigid body. All
        meshes are merged and their combined convex hull is computed. One BVH tree
        and one set of hull vertices are produced, both expressed in the local
        coordinate system of the first mesh in the collection (the "anchor").

        Use this when the collection moves as a unit and its overall shape matters
        more than individual piece boundaries. This is the fastest multi-object mode.

    COLLECTION_INDIVIDUAL
        A named collection of mesh objects where each mesh is treated separately.
        One BVH tree and one per-mesh convex hull vertex set are produced per mesh.

        Use this when you need to know *which* object in a collection is closest
        or colliding. Note that pairing two INDIVIDUAL candidates results in N×M
        checks per frame; a warning is printed when the product exceeds
        :data:`_INDIVIDUAL_PAIR_WARN_THRESHOLD`.
    """
    SINGLE                = auto()
    COLLECTION_HULL       = auto()
    COLLECTION_INDIVIDUAL = auto()


class LogLevel(Enum):
    """
    Controls how much output :func:`detect_collisions` prints to the console.

    NONE
        Silent. No output whatsoever.

    SUMMARY
        Prints a one-time timing and distance summary at the end of the run.

    COLLISIONS
        Prints one line per detected collision event, plus preparation info and
        the final summary.

    VERBOSE
        Prints one line per frame (frame number, minimum distance, timing),
        plus everything from COLLISIONS. Use this when debugging detection issues
        or tuning ``collision_margin``.
    """
    NONE       = 0
    SUMMARY    = 1
    COLLISIONS = 2
    VERBOSE    = 3


# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------

@dataclass
class CollisionCandidate:
    """
    Describes one participant in a collision check.

    Parameters
    ----------
    name : str
        A short label used to identify this candidate in log output and in
        ``collision_pairs``. Must be unique within the candidate list passed to
        :func:`detect_collisions`.
    mode : CandidateMode
        How this candidate's geometry is collected and represented. See
        :class:`CandidateMode` for the trade-offs between modes.
    object_name : str
        The exact name of the Blender mesh object as it appears in the Outliner.
        Required when ``mode`` is ``SINGLE``; ignored otherwise.
    collection_name : str
        The exact name of the Blender collection as it appears in the Outliner.
        Required when ``mode`` is ``COLLECTION_HULL`` or ``COLLECTION_INDIVIDUAL``;
        ignored otherwise. Non-mesh objects in the collection (lights, cameras,
        empties, etc.) are skipped automatically.
    """
    name: str
    mode: CandidateMode
    object_name: str = ""
    collection_name: str = ""


@dataclass
class CollisionEvent:
    """
    Records a single detected collision.

    Attributes
    ----------
    frame : int
        The animation frame at which the collision was detected.
    pair : tuple[str, str]
        Names of the two candidates involved as ``(bvh_side, vertex_side)``.
        The BVH side provides the surface; the vertex side provides query points.
        Both names match :attr:`CollisionCandidate.name` values from the call
        that produced this event.
    distance : float
        The minimum proximity measured for this pair at this frame, in the BVH
        object's local-space units. Will be ≤ ``collision_margin`` (converted
        to local units) for proximity events; for penetration events it is the
        distance from the hull vertex to the nearest interior face (positive,
        but the vertex has crossed the surface).
    kind : str
        ``"proximity"`` — a hull vertex came within ``collision_margin`` of the
        surface without crossing it.
        ``"penetration"`` — a hull vertex has crossed through the surface and
        is inside the mesh (detected via outward face-normal sign flip).
    """
    frame: int
    pair: tuple
    distance: float
    kind: str


@dataclass
class DetectionResult:
    """
    The return value of :func:`detect_collisions`.

    Attributes
    ----------
    collisions : list[CollisionEvent]
        All detected collision events, in frame order. Empty when no collision
        occurred. When ``early_exit=True`` this list contains at most one entry.
    closest_distance : float
        The minimum proximity measured across all checked frames and pairs,
        expressed in the BVH object's **local-space units**.

        .. note::
            If the BVH object has a non-unit scale — for example ``0.001`` when
            the model was authored in millimetres — this value is in millimetres,
            not metres. Multiply by ``obj.scale.x`` to convert to world-space
            metres. The ``collision_margin`` parameter is converted to local
            units automatically, so threshold comparisons are always consistent.

    closest_frame : int or None
        The frame at which ``closest_distance`` was measured. ``None`` when no
        frames were processed.
    furthest_distance : float
        The maximum proximity measured across all non-sphere-rejected frames and
        pairs, in the BVH object's local-space units. Large values here indicate
        frames where the bounding-sphere fast-reject did not fire but the objects
        were still far apart.
    furthest_frame : int or None
        The frame at which ``furthest_distance`` was measured. ``None`` when no
        non-rejected frames were processed.
    frames_processed : int
        How many frames were actually evaluated before the function returned.
    t_matrices : float
        Total wall-clock seconds spent computing object world matrices.
    t_bvh : float
        Total wall-clock seconds spent on BVH nearest-point queries.
    """
    collisions: list
    closest_distance: float
    closest_frame: int | None
    furthest_distance: float
    furthest_frame: int | None
    frames_processed: int
    t_matrices: float
    t_bvh: float


# ---------------------------------------------------------------------------
# Private internal dataclasses
# ---------------------------------------------------------------------------

@dataclass
class _BVHEntry:
    """One BVH tree plus the Blender object it lives in and associated metadata."""
    obj: object          # bpy.types.Object — defines the local coordinate space
    bvh: object          # BVHTree
    correction: object   # mathutils.Matrix — constraint-offset captured at ref frame
    local_margin: float  # collision_margin converted to obj-local units
    centroid_local: object  # mathutils.Vector — hull centroid in obj-local space
    world_radius: float     # bounding-sphere radius in world units
    world_margin: float     # collision_margin in world units


@dataclass
class _VertexEntry:
    """A set of query vertices stored in an anchor object's local coordinate space."""
    anchor: object   # bpy.types.Object — the coordinate frame for verts
    verts: list      # list[mathutils.Vector] — convex hull vertices, anchor-local
    correction: object  # mathutils.Matrix — constraint correction for anchor
    centroid_local: object  # mathutils.Vector — hull centroid in anchor-local space
    world_radius: float     # bounding-sphere radius in world units


@dataclass
class _PreparedCandidate:
    """Internal preprocessed form of a :class:`CollisionCandidate`."""
    name: str
    bvh_entries: list    # list[_BVHEntry]   — 1 for SINGLE/HULL, N for INDIVIDUAL
    vertex_entries: list # list[_VertexEntry] — 1 for SINGLE/HULL, N for INDIVIDUAL


# ---------------------------------------------------------------------------
# Private: FCurve / matrix helpers (no frame_set)
# ---------------------------------------------------------------------------

def _get_location_fcurves(obj):
    """
    Return all location FCurves for *obj*, handling both the legacy
    Blender < 4.4 flat ``action.fcurves`` API and the Blender 4.4+ layered /
    slotted action API. Returns an empty list when the object has no animation.
    """
    if not (obj.animation_data and obj.animation_data.action):
        return []
    action = obj.animation_data.action
    if hasattr(action, 'layers'):
        # Blender 4.4+: actions have layers → strips → channel bags
        slot = obj.animation_data.action_slot
        curves = []
        for layer in action.layers:
            for strip in layer.strips:
                cb = strip.channelbag(slot)
                if cb:
                    curves.extend(fc for fc in cb.fcurves
                                  if fc.data_path == 'location')
        return curves
    # Blender < 4.4: flat list of FCurves on the action
    return [fc for fc in action.fcurves if fc.data_path == 'location']


def _eval_matrix_at_frame(obj, frame):
    """
    Reconstruct *obj*'s world matrix at *frame* by reading FCurves directly,
    without calling ``frame_set``.

    How this works
    ~~~~~~~~~~~~~~
    A Blender object's world matrix is:

        world = parent_world @ matrix_parent_inverse @ matrix_basis

    ``matrix_basis`` encodes the object's own location, rotation, and scale in
    its parent's space. When location is animated, Blender stores it as three
    FCurves (X, Y, Z). This function reads those curves at *frame* and
    substitutes them into the current ``matrix_basis``, then recursively walks
    up the ``.parent`` chain.

    Limitations
    ~~~~~~~~~~~
    - Only **location** is read from FCurves. Rotation and scale are taken from
      the object's current (``frame_start``) values and assumed constant.
    - **Constraints** (e.g. "Child Of", "Copy Location") are *not* evaluated.
      Call :func:`_compute_correction` to capture constant constraint offsets.
    """
    loc = list(obj.location)
    for fc in _get_location_fcurves(obj):
        loc[fc.array_index] = fc.evaluate(frame)

    mat = obj.matrix_basis.copy()
    mat.translation = loc

    if obj.parent:
        return _eval_matrix_at_frame(obj.parent, frame) @ obj.matrix_parent_inverse @ mat
    return mat


# ---------------------------------------------------------------------------
# Private: Preprocessing helpers
# ---------------------------------------------------------------------------

def _compute_correction(obj, depsgraph, ref_frame):
    """
    Build a correction matrix that captures any constraint-driven world-space
    offset that :func:`_eval_matrix_at_frame` cannot see.

    Background
    ~~~~~~~~~~
    Constraints like "Child Of" or "Copy Location" position an object in world
    space without creating a Python ``.parent`` link. Since
    :func:`_eval_matrix_at_frame` only walks ``.parent`` chains, it returns the
    wrong world position for constrained objects.

    The fix: at a known *ref_frame* compare Blender's fully-evaluated world
    matrix (which includes constraint effects) against the FCurve-only
    reconstruction. Their ratio captures the residual offset as a constant
    matrix. Applied each frame as ``correction @ _eval_matrix_at_frame(…)``,
    it corrects the position.

    This is valid as long as constraint *targets* do not themselves animate
    (which is the common case for machine-assembly scenes).

    Returns
    -------
    mathutils.Matrix
        ``blender_world_at_ref @ fcurve_matrix_at_ref.inverted()``
    """
    blender_world = obj.evaluated_get(depsgraph).matrix_world
    return blender_world @ _eval_matrix_at_frame(obj, ref_frame).inverted()


def _build_object_bvh(obj, depsgraph):
    """
    Build a BVHTree from *obj*'s complete evaluated mesh in *obj*-local space.

    The tree is built in local space so it never needs to be rebuilt as the
    object moves. All movement is handled by transforming query points into
    this local space each frame instead.

    Returns
    -------
    BVHTree
    """
    obj_eval = obj.evaluated_get(depsgraph)
    bm = bmesh.new()
    bm.from_object(obj_eval, depsgraph)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()
    return bvh


def _build_hull_geometry(objects, anchor, depsgraph):
    """
    Compute the convex hull of all *objects* merged into *anchor*-local space
    and return both a BVH tree and the hull vertex positions.

    Why a convex hull?
    ~~~~~~~~~~~~~~~~~~
    Testing every vertex of a dense mesh against a BVH tree per frame would be
    slow. The convex hull dramatically reduces the number of query points while
    remaining conservative: if no hull vertex is within the collision margin,
    nothing inside the hull can be either.

    For collection modes, the hull collapses N separate meshes into one unified
    shape, further reducing per-frame work.

    Why two outputs?
    ~~~~~~~~~~~~~~~~
    When this candidate is the **BVH side** of a pair it provides the surface
    geometry — the returned ``hull_bvh`` (built from hull *faces*) is used.
    When it is the **vertex side** it provides the query points — the returned
    ``hull_verts`` list is transformed and queried each frame.

    Parameters
    ----------
    objects : list[bpy.types.Object]
        Mesh objects to include in the hull.
    anchor : bpy.types.Object
        Defines the local coordinate frame that all hull vertices are expressed
        in. Typically the first object in the collection.
    depsgraph : bpy.types.Depsgraph
        Must be evaluated at the reference frame.

    Returns
    -------
    hull_bvh : BVHTree
        BVH tree of the convex hull surface, in anchor-local space.
    hull_verts : list[mathutils.Vector]
        Convex hull vertex positions in anchor-local space. Falls back to all
        input vertices if the hull degenerates to fewer than 4 vertices.
    """
    anchor_matrix_inv = anchor.evaluated_get(depsgraph).matrix_world.inverted()
    combined_bm = bmesh.new()

    for obj in objects:
        obj_eval = obj.evaluated_get(depsgraph)
        temp_bm = bmesh.new()
        temp_bm.from_object(obj_eval, depsgraph)
        # Transform each vertex from obj-world space into anchor-local space:
        #   anchor_local = anchor_world⁻¹ @ obj_world @ vertex_local
        transform = anchor_matrix_inv @ obj_eval.matrix_world
        for v in temp_bm.verts:
            combined_bm.verts.new(transform @ v.co)
        temp_bm.free()

    combined_bm.verts.ensure_lookup_table()
    hull_result = bmesh.ops.convex_hull(combined_bm, input=combined_bm.verts[:])

    hull_geom_verts = [item for item in hull_result['geom']
                       if isinstance(item, bmesh.types.BMVert)]
    hull_verts = [v.co.copy() for v in hull_geom_verts]

    if len(hull_verts) < 4:
        print(f"Warning: convex hull for '{anchor.name}' has only {len(hull_verts)} "
              f"vertices (geometry may be flat or near-planar). "
              f"Falling back to all input vertices.")
        hull_verts = [v.co.copy() for v in combined_bm.verts]

    # Build a separate clean bmesh containing only the hull faces so that
    # the BVHTree has surface geometry to query against (BVH needs faces,
    # not just vertices, to answer nearest-point queries).
    hull_bm = bmesh.new()
    old_to_new = {v: hull_bm.verts.new(v.co) for v in hull_geom_verts}
    for item in hull_result['geom']:
        if isinstance(item, bmesh.types.BMFace):
            face_verts = [old_to_new[v] for v in item.verts if v in old_to_new]
            if len(face_verts) >= 3:
                try:
                    hull_bm.faces.new(face_verts)
                except ValueError:
                    pass  # skip degenerate or already-registered faces
    hull_bvh = BVHTree.FromBMesh(hull_bm)
    hull_bm.free()
    combined_bm.free()

    return hull_bvh, hull_verts


def _compute_local_margin(obj, depsgraph, margin):
    """
    Convert a world-space *margin* (metres) to *obj*-local-space units.

    BVH distance queries return values in the queried object's local coordinate
    system. If the object has a scale of 0.001 (typical when the model was
    authored in millimetres), one local unit equals 0.001 world metres. A
    1 mm world-space margin must become 1.0 in that local space.

    Assumes uniform scale on all axes. If the scale differs between axes, a
    warning is printed and the X component is used as an approximation.

    Returns
    -------
    float  — ``margin / scale.x``
    """
    scale = obj.evaluated_get(depsgraph).scale
    if not (abs(scale.x - scale.y) < 1e-6 and abs(scale.y - scale.z) < 1e-6):
        print(f"Warning: '{obj.name}' has non-uniform scale {scale}; "
              f"collision margin conversion will be approximate.")
    return margin / scale.x


def _compute_bounding_sphere(hull_verts, anchor, depsgraph):
    """
    Return ``(centroid_local, world_radius)`` for a set of hull vertices.

    centroid_local — mean of *hull_verts* expressed in *anchor*-local space.
    world_radius   — maximum world-space distance from the world centroid to any
                     hull vertex, evaluated at the reference frame. Constant across
                     frames because only translation is animated (scale/rotation are
                     fixed), so a point's distance from the centroid is invariant.
    """
    from mathutils import Vector
    centroid_local = sum(hull_verts, Vector()) / len(hull_verts)
    anchor_world = anchor.evaluated_get(depsgraph).matrix_world
    centroid_world = anchor_world @ centroid_local
    world_radius = max((anchor_world @ v - centroid_world).length for v in hull_verts)
    return centroid_local, world_radius


def _prepare_candidate(candidate, depsgraph, margin):
    """
    Build the internal :class:`_PreparedCandidate` for *candidate*.

    All expensive per-startup work — BVH construction, convex hull computation,
    correction matrices, and scale conversion — happens here, once, before the
    main frame loop begins.

    Parameters
    ----------
    candidate : CollisionCandidate
    depsgraph : bpy.types.Depsgraph
        Must be evaluated at ``bpy.context.scene.frame_start``.
    margin : float
        World-space collision margin in metres.

    Returns
    -------
    _PreparedCandidate
    """
    ref_frame = bpy.context.scene.frame_start
    mode = candidate.mode

    if mode == CandidateMode.SINGLE:
        obj = bpy.data.objects[candidate.object_name]
        correction = _compute_correction(obj, depsgraph, ref_frame)
        local_margin = _compute_local_margin(obj, depsgraph, margin)
        # Full mesh BVH for accurate surface representation on the BVH side.
        bvh = _build_object_bvh(obj, depsgraph)
        # Convex hull for the vertex side (fewer query points per frame).
        _, hull_verts = _build_hull_geometry([obj], anchor=obj, depsgraph=depsgraph)
        centroid_local, world_radius = _compute_bounding_sphere(hull_verts, obj, depsgraph)
        return _PreparedCandidate(
            name=candidate.name,
            bvh_entries=[_BVHEntry(obj=obj, bvh=bvh, correction=correction,
                                   local_margin=local_margin, centroid_local=centroid_local,
                                   world_radius=world_radius, world_margin=margin)],
            vertex_entries=[_VertexEntry(anchor=obj, verts=hull_verts, correction=correction,
                                         centroid_local=centroid_local,
                                         world_radius=world_radius)],
        )

    elif mode == CandidateMode.COLLECTION_HULL:
        coll = bpy.data.collections[candidate.collection_name]
        objects = [o for o in coll.all_objects if o.type == 'MESH']
        anchor = objects[0]
        correction = _compute_correction(anchor, depsgraph, ref_frame)
        local_margin = _compute_local_margin(anchor, depsgraph, margin)
        # Both the BVH and the vertex set come from the same combined hull.
        hull_bvh, hull_verts = _build_hull_geometry(objects, anchor=anchor,
                                                     depsgraph=depsgraph)
        centroid_local, world_radius = _compute_bounding_sphere(hull_verts, anchor, depsgraph)
        return _PreparedCandidate(
            name=candidate.name,
            bvh_entries=[_BVHEntry(obj=anchor, bvh=hull_bvh, correction=correction,
                                   local_margin=local_margin, centroid_local=centroid_local,
                                   world_radius=world_radius, world_margin=margin)],
            vertex_entries=[_VertexEntry(anchor=anchor, verts=hull_verts, correction=correction,
                                          centroid_local=centroid_local,
                                          world_radius=world_radius)],
        )

    elif mode == CandidateMode.COLLECTION_INDIVIDUAL:
        coll = bpy.data.collections[candidate.collection_name]
        objects = [o for o in coll.all_objects if o.type == 'MESH']
        bvh_entries = []
        vertex_entries = []
        for obj in objects:
            correction = _compute_correction(obj, depsgraph, ref_frame)
            local_margin = _compute_local_margin(obj, depsgraph, margin)
            bvh = _build_object_bvh(obj, depsgraph)
            _, hull_verts = _build_hull_geometry([obj], anchor=obj, depsgraph=depsgraph)
            centroid_local, world_radius = _compute_bounding_sphere(hull_verts, obj, depsgraph)
            bvh_entries.append(_BVHEntry(obj=obj, bvh=bvh, correction=correction,
                                          local_margin=local_margin,
                                          centroid_local=centroid_local,
                                          world_radius=world_radius, world_margin=margin))
            vertex_entries.append(_VertexEntry(anchor=obj, verts=hull_verts,
                                                correction=correction,
                                                centroid_local=centroid_local,
                                                world_radius=world_radius))
        return _PreparedCandidate(
            name=candidate.name,
            bvh_entries=bvh_entries,
            vertex_entries=vertex_entries,
        )

    else:
        raise ValueError(f"Unknown CandidateMode: {mode}")


# ---------------------------------------------------------------------------
# Private: Per-frame collision check
# ---------------------------------------------------------------------------

def _check_pair(a_prep, b_prep, frame):
    """
    Check one collision pair at a single animation frame without calling
    ``frame_set``.

    Algorithm
    ~~~~~~~~~
    For each BVH entry in *a_prep* (the surface side) and each vertex entry in
    *b_prep* (the query side):

    1. Reconstruct *a*'s world matrix at *frame* via FCurve evaluation plus the
       pre-computed correction matrix. Invert it to get the world→local transform
       for the BVH's coordinate system.
    2. Reconstruct *b*'s anchor world matrix the same way.
    3. For each hull vertex ``v`` stored in *b*'s anchor-local space:

       - Transform to world space:       ``anchor_matrix @ v``
       - Transform to *a*'s local space: ``a_matrix_inv @ world_v``
       - Query the BVH:                  ``a.bvh.find_nearest(local_v)`` →
         returns the nearest point on the surface and its distance.

    4. If any distance ≤ ``a.local_margin``, a collision is flagged and the
       remaining vertices are skipped (early-exit within the pair).

    Parameters
    ----------
    a_prep : _PreparedCandidate  — provides BVH surface(s)
    b_prep : _PreparedCandidate  — provides query vertex set(s)
    frame  : int

    Returns
    -------
    collision    : bool
    min_distance : float   — smallest distance found; ``inf`` if no geometry
    t_matrices   : float   — seconds spent in matrix evaluation
    t_bvh        : float   — seconds spent in BVH queries
    """
    min_distance = float('inf')
    collision = False
    kind = None
    t_matrices = 0.0
    t_bvh = 0.0

    for bvh_e in a_prep.bvh_entries:
        t0 = time.perf_counter()
        bvh_matrix = bvh_e.correction @ _eval_matrix_at_frame(bvh_e.obj, frame)
        bvh_center_world = bvh_matrix @ bvh_e.centroid_local
        obj_matrix_inv = bvh_matrix.inverted()
        t_matrices += time.perf_counter() - t0

        for vert_e in b_prep.vertex_entries:
            t0 = time.perf_counter()
            anchor_matrix = vert_e.correction @ _eval_matrix_at_frame(vert_e.anchor, frame)
            vert_center_world = anchor_matrix @ vert_e.centroid_local

            # Layer 1 — bounding sphere fast-reject.
            # If the closest the two spheres can be is already beyond the margin,
            # no hull vertex can be within the margin of any surface point.
            sphere_gap = (bvh_center_world - vert_center_world).length - \
                         bvh_e.world_radius - vert_e.world_radius
            t_matrices += time.perf_counter() - t0
            if sphere_gap > bvh_e.world_margin:
                continue  # objects too far apart — skip all BVH queries for this pair

            t0 = time.perf_counter()
            for v_co in vert_e.verts:
                # anchor-local → world → BVH-local
                local_pos = obj_matrix_inv @ (anchor_matrix @ v_co)
                # Limit the search to local_margin so find_nearest never returns a
                # face farther than the threshold. This prevents the normal-sign check
                # below from producing false positives on concave meshes, where a
                # distant back-facing face can yield a negative dot product even for
                # exterior points.
                #
                # TODO: deep penetration (hull vertex tunnels more than local_margin
                # through the surface in a single frame step) is not detected by this
                # approach. The correct fix is a ray-cast parity test: cast a ray from
                # local_pos in a fixed direction, count BVH crossings via a chain of
                # bvh.ray_cast() calls stepping past each hit, and flag odd counts as
                # inside. That approach is correct for arbitrary non-convex geometry but
                # requires multiple BVH calls per vertex.
                location, normal, index, dist = bvh_e.bvh.find_nearest(
                    local_pos, bvh_e.local_margin)
                if dist is not None:
                    if dist < min_distance:
                        min_distance = dist
                    collision = True
                    # dist <= local_margin is guaranteed by the search radius above.
                    # Use the face normal to classify: a negative dot product means the
                    # query point is on the interior side of the nearest face.
                    if normal is not None and (local_pos - location).dot(normal) < 0:
                        kind = "penetration"
                    else:
                        kind = "proximity"
                    break
            t_bvh += time.perf_counter() - t0

            if collision:
                break  # stop scanning b vertex entries
        if collision:
            break  # stop scanning a BVH entries

    return collision, kind, min_distance, t_matrices, t_bvh


# ---------------------------------------------------------------------------
# Private: Collision highlighting
# ---------------------------------------------------------------------------

def _apply_collision_highlights(collisions, resolved_pairs, f_start, f_end, collision_color):
    """
    Keyframe object colors to visually mark collision frames in the timeline.

    For each object involved in at least one collision event, inserts keyframes
    that set it to *collision_color* on collision frames and restore its
    original color one frame before and after each contiguous run, producing an
    instant step-function switch with no interpolation ramp.

    Limitation: for COLLECTION_HULL candidates only the anchor object (the
    first mesh in the collection) is highlighted, not every mesh in the
    collection.
    """
    if not collisions:
        return

    name_to_prep = {}
    for a_prep, b_prep in resolved_pairs:
        name_to_prep[a_prep.name] = a_prep
        name_to_prep[b_prep.name] = b_prep

    # Accumulate collision frames per object, deduplicating by id().
    frames_by_obj = {}  # id(obj) -> (obj, set[int])
    for event in collisions:
        for cand_name in event.pair:
            prep = name_to_prep[cand_name]
            objs = ([e.obj   for e in prep.bvh_entries]
                  + [e.anchor for e in prep.vertex_entries])
            for obj in objs:
                key = id(obj)
                if key not in frames_by_obj:
                    frames_by_obj[key] = (obj, set())
                frames_by_obj[key][1].add(event.frame)

    for obj, frames in frames_by_obj.values():
        normal_color  = tuple(obj.color)
        sorted_frames = sorted(frames)

        # Baseline: normal color at the start of the checked range.
        obj.color = normal_color
        obj.keyframe_insert(data_path="color", frame=f_start)

        for i, frame in enumerate(sorted_frames):
            is_run_start = i == 0 or sorted_frames[i - 1] != frame - 1
            is_run_end   = i == len(sorted_frames) - 1 or sorted_frames[i + 1] != frame + 1

            if is_run_start:
                # Normal keyframe one frame before the run so the color snaps
                # to red rather than interpolating from the previous keyframe.
                if frame - 1 >= f_start:
                    obj.color = normal_color
                    obj.keyframe_insert(data_path="color", frame=frame - 1)
                obj.color = collision_color
                obj.keyframe_insert(data_path="color", frame=frame)

            if is_run_end and frame + 1 <= f_end:
                obj.color = normal_color
                obj.keyframe_insert(data_path="color", frame=frame + 1)


# ---------------------------------------------------------------------------
# Private: Input validation
# ---------------------------------------------------------------------------

def _validate(candidates, collision_pairs):
    """
    Validate all candidate definitions and pair references before doing any
    geometry work. Raises a descriptive exception on the first problem found.

    Also emits a performance warning when two COLLECTION_INDIVIDUAL candidates
    are paired and the combined mesh count exceeds the threshold.
    """
    candidate_map = {c.name: c for c in candidates}

    for c in candidates:
        if c.mode == CandidateMode.SINGLE:
            if c.object_name not in bpy.data.objects:
                raise KeyError(
                    f"Candidate '{c.name}': object '{c.object_name}' was not "
                    f"found in the scene. Check spelling and that it is not "
                    f"hidden from the view layer.")
            obj = bpy.data.objects[c.object_name]
            if obj.type != 'MESH':
                raise ValueError(
                    f"Candidate '{c.name}': '{c.object_name}' is a {obj.type!r} "
                    f"object, not a MESH. Only mesh objects are supported.")
        else:
            if c.collection_name not in bpy.data.collections:
                raise KeyError(
                    f"Candidate '{c.name}': collection '{c.collection_name}' was "
                    f"not found in the scene.")
            meshes = [o for o in bpy.data.collections[c.collection_name].all_objects
                      if o.type == 'MESH']
            if not meshes:
                raise ValueError(
                    f"Candidate '{c.name}': collection '{c.collection_name}' "
                    f"contains no mesh objects. Non-mesh objects (lights, cameras, "
                    f"empties) are skipped automatically, so at least one mesh must "
                    f"be present.")

    for a_name, b_name in collision_pairs:
        for name in (a_name, b_name):
            if name not in candidate_map:
                raise KeyError(
                    f"collision_pairs references unknown candidate '{name}'. "
                    f"Available names: {sorted(candidate_map)}")
        ca, cb = candidate_map[a_name], candidate_map[b_name]
        if (ca.mode == CandidateMode.COLLECTION_INDIVIDUAL
                and cb.mode == CandidateMode.COLLECTION_INDIVIDUAL):
            na = len([o for o in bpy.data.collections[ca.collection_name].all_objects
                      if o.type == 'MESH'])
            nb = len([o for o in bpy.data.collections[cb.collection_name].all_objects
                      if o.type == 'MESH'])
            if na * nb > _INDIVIDUAL_PAIR_WARN_THRESHOLD:
                print(
                    f"Performance warning: pair ('{a_name}', '{b_name}') uses "
                    f"COLLECTION_INDIVIDUAL on both sides with {na}×{nb}={na*nb} "
                    f"BVH-entry combinations per frame. Consider using "
                    f"COLLECTION_HULL for one or both sides if speed is a concern.")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def detect_collisions(
    candidates,
    collision_pairs,
    collision_margin=0.001,
    frame_start=None,
    frame_end=None,
    early_exit=True,
    attribute_name="Collision Frame",
    collision_color=(1.0, 0.0, 0.0, 1.0),
    log_level=LogLevel.SUMMARY,
):
    """
    Detect proximity and collision between pairs of Blender objects or
    collections across an animation frame range.

    This function reads animation data directly from FCurves to reconstruct
    object positions frame by frame, avoiding the expensive ``frame_set`` call.
    On a typical scene it evaluates thousands of frames per second.

    Parameters
    ----------
    candidates : list[CollisionCandidate]
        All participants in the detection run. Each candidate must have a unique
        ``name``. Only candidates referenced in *collision_pairs* need to be
        listed, but you can include extras without harm.

    collision_pairs : list[tuple[str, str]]
        Which candidates to check against each other. Each entry is a tuple of
        two candidate names: ``(bvh_side, vertex_side)``.

        - The **first** name provides the surface geometry (a BVH tree is built
          from it). This is typically the geometrically richer object.
        - The **second** name provides the query points (its convex hull vertices
          are tested against the BVH each frame). This is typically the simpler
          or smaller object.

        Example — tool vs. deck::

            collision_pairs=[("tool", "deck")]

        Multiple pairs can be listed to check several interactions in one run::

            collision_pairs=[("tool_a", "deck"), ("tool_b", "deck")]

    collision_margin : float, optional
        Proximity threshold in Blender **world-space units** (metres by default
        in Blender). Two candidates are considered to be colliding when any
        query point is within this distance of the BVH surface.
        Default ``0.001`` (1 mm).

    frame_start : int or None, optional
        First frame to check. Defaults to ``bpy.context.scene.frame_start`` when
        ``None``.

    frame_end : int or None, optional
        Last frame to check, inclusive. Defaults to ``bpy.context.scene.frame_end``
        when ``None``.

    early_exit : bool, optional
        When ``True`` (default), detection stops immediately after the first
        collision is found across any pair and any frame. The returned
        :attr:`DetectionResult.collisions` list will contain exactly one entry.

        When ``False``, all frames are evaluated for all pairs. Every detected
        collision is recorded. Use this to build a complete collision map, e.g.
        when checking multiple independent pairs at once.

    attribute_name : str or None, optional
        When a collision is detected, write the frame number as a custom
        property with this name on every Blender object that participates in
        the colliding pair. This makes the result inspectable in the Outliner's
        Properties panel without writing additional code.

        Set to ``None`` to suppress all custom-property writes (pure library
        mode). Default ``"Collision Frame"``.

    collision_color : tuple[float, float, float, float] or None, optional
        RGBA color to keyframe on colliding objects for each collision frame.
        Each channel is in the range ``[0.0, 1.0]``. The object's original
        color is restored on the frame immediately before and after each
        contiguous run of collision frames, creating a step-function highlight
        visible in the viewport and timeline.

        Set to ``None`` to suppress all color keyframing. Default
        ``(1.0, 0.0, 0.0, 1.0)`` (opaque red).

        .. note::
            For ``COLLECTION_HULL`` candidates only the anchor object (the
            first mesh in the collection) is highlighted. For
            ``COLLECTION_INDIVIDUAL`` and ``SINGLE`` every mesh object
            involved in a collision event is highlighted individually.

    log_level : LogLevel, optional
        Controls console verbosity. See :class:`LogLevel`. Default ``SUMMARY``.

    Returns
    -------
    DetectionResult
        See :class:`DetectionResult` for full attribute documentation.

    Raises
    ------
    KeyError
        If a referenced object name, collection name, or candidate name does
        not exist in the current scene.
    ValueError
        If a referenced object is not a mesh, or a referenced collection
        contains no mesh objects.

    Notes
    -----
    **Scale and units:** ``closest_distance`` is returned in the BVH object's
    local-space units, not necessarily world metres. If the object has a scale
    of 0.001 (millimetre-scale models are common), divide by ``obj.scale.x``
    to get world metres.

    **Constraint support:** Only constraints whose targets do not animate are
    handled correctly. If a constraint target moves during the animation, the
    correction matrix (computed once at ``frame_start``) will not track those
    changes and results will be incorrect.

    **Deformation:** Objects whose mesh topology or vertex positions change over
    time (shape keys driven by animation, cloth simulation, particles) are not
    supported. BVH trees are built once at ``frame_start`` and remain fixed.
    """
    scene = bpy.context.scene
    f_start = frame_start if frame_start is not None else scene.frame_start
    f_end   = frame_end   if frame_end   is not None else scene.frame_end

    _validate(candidates, collision_pairs)

    # One frame_set to establish the depsgraph at the reference frame.
    # This is the only frame_set call in the entire detection run.
    frame_curr = scene.frame_current
    scene.frame_set(f_start)
    depsgraph = bpy.context.evaluated_depsgraph_get()

    if log_level.value >= LogLevel.COLLISIONS.value:
        print(f"[collision_detection] Preparing {len(candidates)} candidate(s) ...")

    prepared_map = {}
    for c in candidates:
        prepared_map[c.name] = _prepare_candidate(c, depsgraph, collision_margin)
        if log_level.value >= LogLevel.COLLISIONS.value:
            pc = prepared_map[c.name]
            n_bvh = len(pc.bvh_entries)
            total_verts = sum(len(ve.verts) for ve in pc.vertex_entries)
            print(f"  '{c.name}' ({c.mode.name}): "
                  f"{n_bvh} BVH entr{'y' if n_bvh == 1 else 'ies'}, "
                  f"{total_verts} hull vert(s) total")

    resolved_pairs = [
        (prepared_map[a_name], prepared_map[b_name])
        for a_name, b_name in collision_pairs
    ]

    if log_level.value >= LogLevel.SUMMARY.value:
        print(f"[collision_detection] Checking {len(resolved_pairs)} pair(s), "
              f"frames {f_start}–{f_end} ...")

    # Clear the attribute from all participating objects now so that calling
    # detect_collisions again always reflects the latest run, not a stale one.
    # Track written objects to avoid overwriting the first collision frame with
    # a later one during the same call.
    _written_objs = set()
    if attribute_name is not None:
        all_pair_objs = set()
        for a_prep, b_prep in resolved_pairs:
            for bvh_e in a_prep.bvh_entries:
                all_pair_objs.add(bvh_e.obj)
            for vert_e in b_prep.vertex_entries:
                all_pair_objs.add(vert_e.anchor)
        for obj in all_pair_objs:
            if attribute_name in obj:
                del obj[attribute_name]

    # ── Main frame loop ────────────────────────────────────────────────────
    frame_num        = f_start
    frames_processed = 0
    closest_distance = float('inf')
    closest_frame    = None
    furthest_distance = 0.0
    furthest_frame    = None
    collisions       = []
    t_matrices_total = 0.0
    t_bvh_total      = 0.0

    while frame_num <= f_end:
        frame_min_dist   = float('inf')
        frame_t_matrices = 0.0
        frame_t_bvh      = 0.0
        frame_collision  = False

        for a_prep, b_prep in resolved_pairs:
            hit, kind, min_dist, t_mat, t_bvh = _check_pair(a_prep, b_prep, frame_num)
            frame_t_matrices += t_mat
            frame_t_bvh      += t_bvh

            if min_dist < frame_min_dist:
                frame_min_dist = min_dist

            if hit:
                frame_collision = True
                event = CollisionEvent(frame=frame_num, pair=(a_prep.name, b_prep.name),
                                       distance=min_dist, kind=kind)
                collisions.append(event)

                if log_level.value >= LogLevel.COLLISIONS.value:
                    print(f"[collision_detection] Collision at frame {frame_num} — "
                          f"'{a_prep.name}' vs '{b_prep.name}' [{kind}]")

                if attribute_name is not None:
                    for bvh_e in a_prep.bvh_entries:
                        if id(bvh_e.obj) not in _written_objs:
                            bvh_e.obj[attribute_name] = frame_num
                            _written_objs.add(id(bvh_e.obj))
                    for vert_e in b_prep.vertex_entries:
                        if id(vert_e.anchor) not in _written_objs:
                            vert_e.anchor[attribute_name] = frame_num
                            _written_objs.add(id(vert_e.anchor))

                if early_exit:
                    break  # stop checking remaining pairs this frame

        t_matrices_total += frame_t_matrices
        t_bvh_total      += frame_t_bvh
        frames_processed += 1

        if frame_min_dist < closest_distance:
            closest_distance = frame_min_dist
            closest_frame    = frame_num

        if frame_min_dist != float('inf') and frame_min_dist > furthest_distance:
            furthest_distance = frame_min_dist
            furthest_frame    = frame_num

        if log_level.value >= LogLevel.VERBOSE.value:
            print(f"  frame {frame_num:>5} | min_dist={frame_min_dist:.6f} | "
                  f"matrices={frame_t_matrices:.4f}s  bvh={frame_t_bvh:.4f}s")

        if frame_collision and early_exit:
            break

        frame_num += 1

    # Restore timeline to where it was before this call.
    scene.frame_set(frame_curr)

    if collision_color is not None:
        _apply_collision_highlights(
            collisions, resolved_pairs, f_start, f_end, collision_color)

    # ── Summary output ─────────────────────────────────────────────────────
    if log_level.value >= LogLevel.SUMMARY.value and frames_processed:
        n = frames_processed
        print(f"[collision_detection] --- Summary ({n} frame(s), "
              f"{len(resolved_pairs)} pair(s)) ---")
        print(f"  matrices : {t_matrices_total:.3f}s total  "
              f"({t_matrices_total / n * 1000:.2f}ms avg)")
        print(f"  bvh      : {t_bvh_total:.3f}s total  "
              f"({t_bvh_total / n * 1000:.2f}ms avg)")
        print(f"  total    : {t_matrices_total + t_bvh_total:.3f}s")
        print(f"  closest  : {closest_distance:.6f} (BVH-local) at frame {closest_frame}")
        print(f"  furthest : {furthest_distance:.6f} (BVH-local) at frame {furthest_frame}")
        if collisions:
            proximity    = [e for e in collisions if e.kind == "proximity"]
            penetrations = [e for e in collisions if e.kind == "penetration"]
            kind_summary = (f"{len(proximity)} proximity"
                            + (f", {len(penetrations)} penetration"
                               if penetrations else ""))
            first   = collisions[0]
            last    = collisions[-1]
            nearest  = min(collisions, key=lambda e: e.distance)
            furthest_collision = max(collisions, key=lambda e: e.distance)
            avg_dist = sum(e.distance for e in collisions) / len(collisions)
            print(f"  collisions ({len(collisions)} frame(s) — {kind_summary}):")
            print(f"    first    : frame {first.frame}, dist={first.distance:.6f} [{first.kind}]")
            print(f"    last     : frame {last.frame}, dist={last.distance:.6f} [{last.kind}]")
            print(f"    closest  : frame {nearest.frame}, dist={nearest.distance:.6f} [{nearest.kind}]")
            print(f"    furthest : frame {furthest_collision.frame}, dist={furthest_collision.distance:.6f} [{furthest_collision.kind}]")
            print(f"    average distance: {avg_dist:.6f}")
        else:
            print("  no collisions detected.")

    return DetectionResult(
        collisions=collisions,
        closest_distance=closest_distance,
        closest_frame=closest_frame,
        furthest_distance=furthest_distance,
        furthest_frame=furthest_frame,
        frames_processed=frames_processed,
        t_matrices=t_matrices_total,
        t_bvh=t_bvh_total,
    )
