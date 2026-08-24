"""
place_camera.py
STEP 1 — mount a downward-facing pinhole camera on the Jubilee XY carriage.

Run inside Blender (Text Editor → Open → Run Script) with jubilee.blend open,
or headless:
    blender blender_models/jubilee.blend --python place_camera.py

Intrinsics are read from pipeline_data/jubilee_paths.json (written by
`jubilee-twin setup-scene`). Falls back to jubilee_twin/defaults/camera_params.yaml.
"""

import os
import sys
import bpy
from mathutils import Vector, Matrix


REF_NAME = "D2HW_C201H001"
PARENT_NAME = "carriage_Carriage Center Plate"
CAM_NAME = "Toolhead_Cam"

SENSOR_WIDTH_MM = 36.0


def _load_params() -> dict:
    twin_root_str = os.path.dirname(os.path.dirname(bpy.data.filepath))
    addon_dir = os.path.dirname(__file__)
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    from camera_params import load_camera_params
    from pathlib import Path
    return load_camera_params(Path(twin_root_str))


def main():
    ref = bpy.data.objects.get(REF_NAME)
    parent_obj = bpy.data.objects.get(PARENT_NAME)
    if ref is None:
        raise RuntimeError(f"Reference object {REF_NAME!r} not found in scene.")
    if parent_obj is None:
        raise RuntimeError(f"Parent object {PARENT_NAME!r} not found in scene.")

    p = _load_params()
    fx, fy = float(p["fx"]), float(p["fy"])
    cx, cy = float(p["cx"]), float(p["cy"])
    res_x = int(p.get("width") or p.get("image", {}).get("width", 1920))
    res_y = int(p.get("height") or p.get("image", {}).get("height", 1056))
    off = p.get("offset", [0.0, 0.0, 0.0])
    # YAML offset applied as-is, then -20mm on local Z to clear the plate's centre hole.
    offset_local_mm = Vector((float(off[0]), float(off[1]), float(off[2]) - 20.0))

    # Clean up any previous run so re-running is idempotent.
    if CAM_NAME in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[CAM_NAME], do_unlink=True)
    if CAM_NAME in bpy.data.cameras:
        bpy.data.cameras.remove(bpy.data.cameras[CAM_NAME])

    cam_data = bpy.data.cameras.new(CAM_NAME)
    cam_obj = bpy.data.objects.new(CAM_NAME, cam_data)
    bpy.context.scene.collection.objects.link(cam_obj)

    world_pos = ref.matrix_world @ (offset_local_mm / 1000.0)
    cam_obj.matrix_world = Matrix.Translation(world_pos)

    cam_obj.parent = parent_obj
    cam_obj.matrix_parent_inverse = parent_obj.matrix_world.inverted()

    cam_data.sensor_fit = 'HORIZONTAL'
    cam_data.sensor_width = SENSOR_WIDTH_MM
    cam_data.lens = fx * SENSOR_WIDTH_MM / res_x
    cam_data.shift_x = (cx - res_x / 2.0) / res_x
    cam_data.shift_y = (cy - res_y / 2.0) / res_x  # normalised by res_x (HORIZONTAL fit)
    cam_data.clip_start = 0.001
    cam_data.clip_end = 10.0

    scene = bpy.context.scene
    scene.render.resolution_x = res_x
    scene.render.resolution_y = res_y
    scene.render.pixel_aspect_x = 1.0
    scene.render.pixel_aspect_y = fx / fy
    scene.camera = cam_obj

    # ── Lighting ─────────────────────────────────────────────────────────
    world = scene.world
    if world and world.use_nodes:
        bg = world.node_tree.nodes.get("Background")
        if bg:
            bg.inputs["Strength"].default_value = 3.0

    print(f"Created {CAM_NAME!r} (source: {p.get('_source', 'jubilee_paths.json')}):")
    print(f"  world location : {list(cam_obj.matrix_world.translation)}")
    print(f"  lens (mm)      : {cam_data.lens:.4f}")
    print(f"  shift_x/y      : {cam_data.shift_x:+.4f}, {cam_data.shift_y:+.4f}")
    print(f"  resolution     : {res_x}x{res_y}  pixel_aspect_y={scene.render.pixel_aspect_y:.4f}")
    print(f"  parent         : {cam_obj.parent.name}")

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    print(f"Saved {bpy.data.filepath}")


if __name__ == "__main__":
    main()
