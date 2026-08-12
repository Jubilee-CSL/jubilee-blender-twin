import bpy
import os
import subprocess
import shutil
import json
from mathutils import Vector

# ---------------------------------------------------------
# CONFIG
# ---------------------------------------------------------
# Output folder for intermediate PNG frames
OUTPUT_DIR = os.path.join(bpy.path.abspath("//"), "render_gif")
# Final GIF path (relative to .blend folder)
GIF_PATH = os.path.join(bpy.path.abspath("//"), "docs", "jubilee_test.gif")

# Optional JSON config written by sacred_runner.py (same folder as .blend)
CONFIG_PATH = os.path.join(bpy.path.abspath("//"), "animation_config.json")

# Name of object to frame/zoom on (adjust if you prefer another)
TARGET_OBJECT_NAME = "XY-carriage"

# Default values (can be overridden by JSON config)
TEST_MODE = True
TEST_MAX_FRAMES = 5

FPS = 24  # playback frame rate
SCALE_WIDTH = 640  # GIF width in pixels (height auto-scaled)

# Render resolution (equivalent to Output Properties → Resolution X/Y and %)
RENDER_RES_X = 800   # pixels
RENDER_RES_Y = 800   # pixels
RENDER_RES_PERCENT = 100  # 1–100


def load_config_from_json():
    """If animation_config.json exists, override defaults from it."""
    global TEST_MODE, TEST_MAX_FRAMES, FPS, SCALE_WIDTH
    global RENDER_RES_X, RENDER_RES_Y, RENDER_RES_PERCENT, TARGET_OBJECT_NAME

    if not os.path.exists(CONFIG_PATH):
        print(f"[config] No JSON config at {CONFIG_PATH}, using defaults.")
        return

    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception as e:
        print(f"[config] Could not read JSON config: {e}. Using defaults.")
        return

    print(f"[config] Loaded JSON config from {CONFIG_PATH}.")

    TEST_MODE = bool(cfg.get("test_mode", TEST_MODE))
    TEST_MAX_FRAMES = int(cfg.get("test_max_frames", TEST_MAX_FRAMES))
    FPS = int(cfg.get("fps", FPS))
    SCALE_WIDTH = int(cfg.get("scale_width", SCALE_WIDTH))
    RENDER_RES_X = int(cfg.get("render_res_x", RENDER_RES_X))
    RENDER_RES_Y = int(cfg.get("render_res_y", RENDER_RES_Y))
    RENDER_RES_PERCENT = int(cfg.get("render_res_percent", RENDER_RES_PERCENT))
    TARGET_OBJECT_NAME = str(cfg.get("target_object_name", TARGET_OBJECT_NAME))


def setup_camera(scene, target_object_name, camera_offset=None, camera_lens=None):
    """Place and aim the active camera to nicely frame the target object.

    Runs in background (no UI) so we do manual math instead of using view3d ops.
    """
    obj = bpy.data.objects.get(target_object_name)
    if obj is None:
        print(f"[camera] Target object '{target_object_name}' not found; skipping camera framing.")
        return

    cam = scene.camera
    if cam is None:
        # Create a camera if the scene has none
        cam = bpy.data.cameras.new("Camera")
        cam_obj = bpy.data.objects.new("Camera", cam)
        scene.collection.objects.link(cam_obj)
        scene.camera = cam_obj
        cam = scene.camera

    cam_obj = scene.camera

    target_loc = obj.matrix_world.translation

    # Use config offset or default
    if camera_offset is None:
        camera_offset = (0.7, -1.2, 1.1)
    offset = Vector(camera_offset)
    cam_obj.location = target_loc + offset

    # Point camera at the target
    direction = target_loc - cam_obj.location
    if direction.length_squared != 0.0:
        cam_obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()

    # Use config lens or default
    if isinstance(cam_obj.data, bpy.types.Camera):
        cam_obj.data.lens = camera_lens if camera_lens is not None else 50

    print(f"[camera] Framed '{target_object_name}' from {cam_obj.location}.")


def setup_brightness(scene):
    """Make the render brighter and cleaner for GIF output."""
    # Use Eevee for fast, bright renders
    scene.render.engine = 'BLENDER_EEVEE'

    # Color management: bump exposure a bit
    view = scene.view_settings
    view.view_transform = 'Filmic'
    view.exposure = 1.5

    # Brighten the world background if using nodes
    world = scene.world
    if world and world.use_nodes and world.node_tree:
        for node in world.node_tree.nodes:
            if node.type == 'BACKGROUND':
                # Increase strength (default is often 1.0)
                node.inputs[1].default_value = 2.5
                break

    print("[brightness] Using Eevee with boosted exposure and brighter world.")


def main():
    # Load overrides from JSON if available
    load_config_from_json()

    scene = bpy.context.scene

    # Set render resolution explicitly
    scene.render.resolution_x = RENDER_RES_X
    scene.render.resolution_y = RENDER_RES_Y
    scene.render.resolution_percentage = RENDER_RES_PERCENT

    # Improve visibility/brightness and framing before rendering
    setup_brightness(scene)
    # Load camera config from JSON if available
    import os, json
    config_path = os.path.join(os.path.dirname(__file__), "animation_config.json")
    camera_offset = None
    camera_lens = None
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
                camera_offset = cfg.get("camera_offset", None)
                camera_lens = cfg.get("camera_lens", None)
        except Exception as e:
            print(f"[camera] Could not read camera config: {e}")
    setup_camera(scene, TARGET_OBJECT_NAME, camera_offset, camera_lens)

    # Use current timeline range; optionally clamp for quick tests
    orig_start = scene.frame_start
    orig_end = scene.frame_end
    if TEST_MODE:
        scene.frame_start = orig_start
        scene.frame_end = min(orig_end, orig_start + TEST_MAX_FRAMES - 1)
        print(f"[frames] TEST_MODE on: rendering {scene.frame_start}–{scene.frame_end} (was {orig_start}–{orig_end})")

    start = scene.frame_start
    end = scene.frame_end

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Clear any existing content in the render folder so we don't mix old/new frames
    for name in os.listdir(OUTPUT_DIR):
        path = os.path.join(OUTPUT_DIR, name)
        try:
            if os.path.isfile(path) or os.path.islink(path):
                os.unlink(path)
            elif os.path.isdir(path):
                shutil.rmtree(path)
        except Exception as e:
                print(f"[warn] Could not remove '{path}': {e}")
        os.makedirs(os.path.dirname(GIF_PATH), exist_ok=True)

    # Configure render settings for PNG sequence
    scene.render.image_settings.file_format = 'PNG'
    # Blender will append frame numbers: frame_0001.png, frame_0002.png, ...
    scene.render.filepath = os.path.join(OUTPUT_DIR, "frame_")

    print(f"Rendering frames {start}–{end} to {OUTPUT_DIR} ...")
    bpy.ops.render.render(animation=True)

    if shutil.which("ffmpeg") is None:
        print("WARNING: ffmpeg not found on PATH.")
        print("PNG frames are ready in:", OUTPUT_DIR)
        return

    input_pattern = os.path.join(OUTPUT_DIR, "frame_%04d.png")
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(FPS),
        "-i", input_pattern,
        "-vf", f"fps={FPS},scale={SCALE_WIDTH}:-1:flags=lanczos",
        GIF_PATH,
    ]

    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print("GIF written to:", GIF_PATH)


if __name__ == "__main__":
    main()