"""
Collision-Detection-Script.py
==============================
Blender script: detects the first animation frame at which the wellplate
collides with the Z-axis assembly and writes that frame number as a custom
property on the participating objects.

Run from the Blender scripting workspace or via:
    blender jubilee.blend --python test_python/Collision-Detection-Script.py

All detection logic lives in collision_detection.py at the repo root.
Edit the CONFIG section below to target different objects, collections,
or detection modes.
"""

import sys
import bpy

# Make the repo root importable so collision_detection.py can be found.
# bpy.path.abspath("//") resolves to the directory containing the .blend file,
# which is the repo root when jubilee.blend is open.
sys.path.append(bpy.path.abspath("//"))

from collision_detection import (
    CandidateMode,
    CollisionCandidate,
    LogLevel,
    detect_collisions,
)

# ---------------------------------------------------------------------------
# CONFIG — edit this section to match your scene
# ---------------------------------------------------------------------------

# Distance threshold in world-space metres at which two objects are considered
# to be colliding. Tune this using the "closest measured distance" value
# printed in the summary (divide by obj.scale.x to convert to world metres
# if the object has a non-unit scale).
collision_margin = 0.005

# Each CollisionCandidate defines one participant.
# mode options:
#   CandidateMode.SINGLE               — one named mesh object
#   CandidateMode.COLLECTION_HULL      — collection treated as a single convex hull
#   CandidateMode.COLLECTION_INDIVIDUAL — collection with each mesh checked separately
candidates = [
    CollisionCandidate(
        name="deck",
        mode=CandidateMode.COLLECTION_INDIVIDUAL,
        collection_name="deck",
    ),
#    CollisionCandidate(
#        name="tools",
#        mode=CandidateMode.COLLECTION_INDIVIDUAL,
#        collection_name="tools",
#    ),
    
    CollisionCandidate(
        name="base_plate",
        mode=CandidateMode.COLLECTION_INDIVIDUAL,
        collection_name="base_plate",
    ),
    CollisionCandidate(
        name="gantry",
        mode=CandidateMode.COLLECTION_INDIVIDUAL,
        collection_name="gantry",
    ),
]

# Pairs to check. Each entry is (bvh_side, vertex_side):
#   bvh_side   — provides the surface (richer geometry goes here)
#   vertex_side — provides the query points (simpler / smaller goes here)
collision_pairs = [
    ("gantry", "deck"),
    ("base_plate", "gantry"),
#    ("tools", "base_plate"),
#    ("tools", "deck"),
]

# ---------------------------------------------------------------------------
# Run detection
# ---------------------------------------------------------------------------

result = detect_collisions(
    candidates=candidates,
    collision_pairs=collision_pairs,
    collision_margin=collision_margin,
    early_exit=False,
    attribute_name="Collision Frame",
    log_level=LogLevel.VERBOSE,
)
