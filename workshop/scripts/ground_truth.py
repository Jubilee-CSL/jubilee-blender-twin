"""
Runs inside Blender: render the image for MoGe-2 and the twin's true point cloud of the plant.

    blender scene.blend --background --python ground_truth.py -- <x> <y> <z> <out_dir>

Writes <out_dir>/moge_input.png and <out_dir>/twin_plant.ply. The point cloud
is the plant surface the camera actually sees (one ray per pixel, half
resolution), in metres, in the same axes as the MoGe-2 demo's pointcloud.ply
(camera at the origin, x right, y up, z towards the viewer), so both files
open aligned in CloudCompare without any transform.
"""
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "blender_addon" / "jubilee_digital_twin"))
import scene_utils

x, y, z, out_dir = sys.argv[sys.argv.index("--") + 1:]
out_dir = Path(out_dir)
out_dir.mkdir(parents=True, exist_ok=True)
scene = bpy.context.scene
scene_utils.drive_to_mm(float(x), float(y), float(z))

image_path = out_dir / "moge_input.png"
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = str(image_path)
bpy.ops.render.render(write_still=True)

W = scene.render.resolution_x * scene.render.resolution_percentage // 200
H = scene.render.resolution_y * scene.render.resolution_percentage // 200
depsgraph = bpy.context.evaluated_depsgraph_get()
cam = scene.camera.evaluated_get(depsgraph)
to_cam = cam.matrix_world.inverted()
rotation, origin = cam.matrix_world.to_3x3(), cam.matrix_world.translation

# Corners of the camera's image plane (Blender handles lens shift and sensor fit).
tr, br, bl, tl = cam.data.view_frame(scene=scene)

# Pixel colours from the render, to colour the point cloud (Blender stores rows bottom-up).
render = bpy.data.images.load(str(image_path))
rgb = np.array(render.pixels[:]).reshape(render.size[1], render.size[0], 4)[::-1, :, :3]

plant = set(scene["plant"])
rows = []
for v in range(H):
    for u in range(W):
        local = Vector((bl.x + (u + 0.5) / W * (tr.x - bl.x),
                        tr.y - (v + 0.5) / H * (tr.y - br.y),
                        tr.z))
        hit, location, _, _, obj, _ = scene.ray_cast(depsgraph, origin, (rotation @ local).normalized())
        if hit and obj.original.name in plant:
            p = to_cam @ location                     # Blender camera axes = MoGe export axes
            r, g, b = (rgb[2 * v, 2 * u] * 255).astype(int)
            rows.append(f"{p.x:.5f} {p.y:.5f} {p.z:.5f} {r} {g} {b}")

ply = out_dir / "twin_plant.ply"
ply.write_text("\n".join([
    "ply", "format ascii 1.0", f"element vertex {len(rows)}",
    "property float x", "property float y", "property float z",
    "property uchar red", "property uchar green", "property uchar blue",
    "end_header", *rows]) + "\n")
print(f"twin point cloud: {len(rows)} points -> {ply}")
