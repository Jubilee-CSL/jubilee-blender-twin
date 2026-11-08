"""
twin_ops_blender.py
Blender-side helper for the digital-twin workshop notebook. Runs inside
Blender's own Python (headless), launched by workshop_utils.py:

    blender <scene.blend> --background --python twin_ops_blender.py -- build  --plant-blend P --out OUT
    blender <scene.blend> --background --python twin_ops_blender.py -- render --jobs jobs.json

`build`  appends a plant (everything parented to the bed plate in another
         .blend) into the working scene and saves it as a new .blend.
`render` drives the axes, sets lighting / depth of field, renders through
         Toolhead_Cam, and optionally writes ground truth (depth, mask,
         point map, intrinsics, pose) computed by ray casting the scene.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

ADDON_DIR = Path(__file__).resolve().parent.parent / "blender_addon" / "jubilee_digital_twin"
if str(ADDON_DIR) not in sys.path:
    sys.path.insert(0, str(ADDON_DIR))

PLATE_NAME = "z_plate_3_V5"
PLANT_COLLECTION = "Plant"
CAMERA_NAME = "Toolhead_Cam"


# ── build ──────────────────────────────────────────────────────────────────

def _reaches_plate(obj) -> bool:
    p = obj.parent
    while p is not None:
        if p.name.startswith(PLATE_NAME):
            return True
        p = p.parent
    return False


def build(plant_blend: Path, out: Path) -> None:
    """Append every object parented (directly or not) to the bed plate in plant_blend."""
    plate = bpy.data.objects.get(PLATE_NAME)
    if plate is None:
        raise RuntimeError(f"{PLATE_NAME!r} not in scene — is this a Jubilee twin .blend?")

    existing = set(bpy.data.objects.keys())
    with bpy.data.libraries.load(str(plant_blend), link=False) as (src, dst):
        dst.objects = [n for n in src.objects if n not in existing]

    coll = bpy.data.collections.get(PLANT_COLLECTION) or bpy.data.collections.new(PLANT_COLLECTION)
    if coll.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(coll)

    kept, dropped = [], []
    for obj in dst.objects:
        if obj is None:
            continue
        if obj.name.startswith(PLATE_NAME) or not _reaches_plate(obj):
            dropped.append(obj)
            continue
        if obj.parent.name.startswith(PLATE_NAME):
            # Same relative placement on this scene's plate, so the plant follows Z.
            mpi = obj.matrix_parent_inverse.copy()
            obj.parent = plate
            obj.matrix_parent_inverse = mpi
        coll.objects.link(obj)
        kept.append(obj.name)
    for obj in dropped:
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.orphans_purge(do_recursive=True)

    if not kept:
        raise RuntimeError(f"No plant objects found on {PLATE_NAME} in {plant_blend}")
    bpy.context.scene["plant_objects"] = kept
    print(f"[workshop] plant objects: {kept}")
    bpy.ops.wm.save_as_mainfile(filepath=str(out))
    print(f"[workshop] saved {out}")


# ── lighting ───────────────────────────────────────────────────────────────

_ORIGINAL_HDRI: str | None = None


def _original_hdri() -> str | None:
    """HDRI path of the scene's world when the file was opened."""
    global _ORIGINAL_HDRI
    if _ORIGINAL_HDRI is None:
        world = bpy.context.scene.world
        if world and world.node_tree:
            for n in world.node_tree.nodes:
                if n.type == "TEX_ENVIRONMENT" and n.image:
                    _ORIGINAL_HDRI = bpy.path.abspath(n.image.filepath)
    return _ORIGINAL_HDRI


def set_lighting(hdri: str | None = None, strength: float = 3.0,
                 rotation_deg: float = 0.0, plain_background: list | None = None) -> None:
    """Rebuild the world: HDRI image, its strength and rotation, and optionally a
    plain colour seen by the camera while the HDRI still lights the scene."""
    hdri = hdri or _original_hdri()
    world = bpy.data.worlds.get("Workshop World") or bpy.data.worlds.new("Workshop World")
    bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()

    coord = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    env = nt.nodes.new("ShaderNodeTexEnvironment")
    bg = nt.nodes.new("ShaderNodeBackground")
    out = nt.nodes.new("ShaderNodeOutputWorld")

    if hdri:
        env.image = bpy.data.images.load(hdri, check_existing=True)
    mapping.inputs["Rotation"].default_value[2] = math.radians(rotation_deg)
    bg.inputs["Strength"].default_value = strength

    nt.links.new(coord.outputs["Generated"], mapping.inputs["Vector"])
    nt.links.new(mapping.outputs["Vector"], env.inputs["Vector"])
    nt.links.new(env.outputs["Color"], bg.inputs["Color"])

    if plain_background is None:
        nt.links.new(bg.outputs["Background"], out.inputs["Surface"])
        return

    # Camera rays see the flat colour; every other ray (lighting) sees the HDRI.
    light_path = nt.nodes.new("ShaderNodeLightPath")
    flat = nt.nodes.new("ShaderNodeBackground")
    flat.inputs["Color"].default_value = (*plain_background[:3], 1.0)
    flat.inputs["Strength"].default_value = 1.0
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(light_path.outputs["Is Camera Ray"], mix.inputs["Fac"])
    nt.links.new(bg.outputs["Background"], mix.inputs[1])
    nt.links.new(flat.outputs["Background"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])


# ── ground truth ───────────────────────────────────────────────────────────

def camera_rays(cam, width: int, height: int):
    """Per-pixel ray directions in camera space and the matching OpenCV K.

    Uses Blender's own view frame, so lens shift and sensor fit are honoured.
    """
    frame = cam.data.view_frame(scene=bpy.context.scene)  # TR, BR, BL, TL
    x_l, x_r = frame[2].x, frame[0].x
    y_t, y_b = frame[0].y, frame[1].y
    zf = frame[0].z  # negative: Blender cameras look down -Z

    us = x_l + (np.arange(width) + 0.5) / width * (x_r - x_l)
    vs = y_t - (np.arange(height) + 0.5) / height * (y_t - y_b)
    gx, gy = np.meshgrid(us, vs)
    dirs = np.stack([gx, gy, np.full_like(gx, zf)], axis=-1)

    fx = width * (-zf) / (x_r - x_l)
    fy = height * (-zf) / (y_t - y_b)
    cx = -x_l * width / (x_r - x_l) - 0.5
    cy = y_t * height / (y_t - y_b) - 0.5
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    return dirs, K


def ground_truth(width: int, height: int) -> dict:
    """Ray-cast one ray per pixel through Toolhead_Cam.

    Returns depth (m, OpenCV z), plant mask, point map (OpenCV camera frame:
    x right, y down, z forward), K, and the camera-to-world matrix.
    """
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    cam = scene.camera.evaluated_get(depsgraph)
    mw = cam.matrix_world.copy()
    mw_inv = mw.inverted()
    rot = mw.to_3x3()
    origin = mw.translation

    plant = set(scene.get("plant_objects", []))
    dirs, K = camera_rays(cam, width, height)

    depth = np.zeros((height, width), np.float32)
    points = np.zeros((height, width, 3), np.float32)
    mask = np.zeros((height, width), bool)
    for v in range(height):
        for u in range(width):
            d = rot @ Vector(dirs[v, u])
            d.normalize()
            hit, loc, _n, _i, obj, _m = scene.ray_cast(depsgraph, origin, d, distance=10.0)
            if not hit:
                continue
            p = mw_inv @ loc
            points[v, u] = (p.x, -p.y, -p.z)
            depth[v, u] = -p.z
            mask[v, u] = obj.original.name in plant

    # Blender camera axes → OpenCV camera axes (flip Y and Z).
    cam_to_world = np.array(mw) @ np.diag([1.0, -1.0, -1.0, 1.0])
    return {"depth": depth, "mask": mask, "points": points, "K": K,
            "cam_to_world": cam_to_world}


def plant_vertices_in_camera() -> np.ndarray:
    """All plant mesh vertices (visible or hidden) in the OpenCV camera frame."""
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    cam = scene.camera.evaluated_get(depsgraph)
    w2c = np.array(cam.matrix_world.inverted())
    pts = []
    for name in scene.get("plant_objects", []):
        obj = bpy.data.objects[name].evaluated_get(depsgraph)
        if obj.type != "MESH":
            continue
        mesh = obj.to_mesh()
        co = np.empty(len(mesh.vertices) * 3)
        mesh.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3)
        obj.to_mesh_clear()
        hom = np.c_[co, np.ones(len(co))] @ (w2c @ np.array(obj.matrix_world)).T
        pts.append(hom[:, :3])
    if not pts:
        return np.zeros((0, 3))
    p = np.concatenate(pts)
    return p * np.array([1.0, -1.0, -1.0])


# ── render jobs ────────────────────────────────────────────────────────────

def render_jobs(jobs_file: Path) -> None:
    import scene_utils

    spec = json.loads(Path(jobs_file).read_text())
    out_dir = Path(spec["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    scene = bpy.context.scene
    cam = bpy.data.objects.get(CAMERA_NAME)
    if cam is None:
        raise RuntimeError(f"{CAMERA_NAME!r} not found — run setup-scene / place-camera first.")
    scene.camera = cam
    scene.render.resolution_percentage = int(spec.get("percent", 50))
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    width = scene.render.resolution_x * scene.render.resolution_percentage // 100
    height = scene.render.resolution_y * scene.render.resolution_percentage // 100

    for job in spec["jobs"]:
        name = job["name"]
        scene_utils.drive_to_mm(job["x"], job["y"], job["z"])
        set_lighting(job.get("hdri"), job.get("strength", 3.0),
                     job.get("rotation_deg", 0.0), job.get("plain_background"))

        dof = cam.data.dof
        dof.use_dof = job.get("fstop") is not None
        if dof.use_dof:
            dof.aperture_fstop = job["fstop"]
            dof.focus_distance = job["focus_distance"]

        image = out_dir / f"{name}.png"
        scene.render.filepath = str(image)
        bpy.ops.render.render(write_still=True)

        bpy.context.view_layer.update()
        depsgraph = bpy.context.evaluated_depsgraph_get()
        mw = scene.camera.evaluated_get(depsgraph).matrix_world
        meta = {**job, "image": str(image), "width": width, "height": height,
                "camera_world_m": list(mw.translation)}

        if job.get("ground_truth"):
            gt = ground_truth(width, height)
            gt["plant_vertices"] = plant_vertices_in_camera()
            np.savez_compressed(out_dir / f"{name}_gt.npz", **gt)
            meta["ground_truth"] = str(out_dir / f"{name}_gt.npz")
            meta["K"] = gt["K"].tolist()

        (out_dir / f"{name}.json").write_text(json.dumps(meta, indent=2))
        print(f"[workshop] rendered {name}")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--plant-blend", type=Path, required=True)
    b.add_argument("--out", type=Path, required=True)
    r = sub.add_parser("render")
    r.add_argument("--jobs", type=Path, required=True)
    args = p.parse_args(argv)

    if args.cmd == "build":
        build(args.plant_blend.resolve(), args.out.resolve())
    else:
        render_jobs(args.jobs.resolve())
