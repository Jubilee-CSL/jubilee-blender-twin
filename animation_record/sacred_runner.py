
import os
import json
import subprocess
from sacred import Experiment
from sacred.observers import MongoObserver

REPO_ROOT = os.path.dirname(__file__)
CONFIG_FILE = os.path.join(REPO_ROOT, "animation_config.json")

# Only read config, never write
with open(CONFIG_FILE, "r", encoding="utf-8") as f:
    _BASE = json.load(f)

_DEFAULT_EXPERIMENT_NAME = _BASE.get("experiment_name", "jubilee_blender_gif")
_DEFAULT_MONGO_URL = _BASE.get("mongo_url", "mongodb://localhost:27017")
_DEFAULT_MONGO_DB = _BASE.get("mongo_db_name", "animate_jubilee")

ex = Experiment(_DEFAULT_EXPERIMENT_NAME)

try:
    ex.observers.append(MongoObserver(url=_DEFAULT_MONGO_URL,
                                      db_name=_DEFAULT_MONGO_DB))
    print(f"[sacred] MongoObserver attached ({_DEFAULT_MONGO_URL}, db='{_DEFAULT_MONGO_DB}').")
except Exception as e:
    print(f"[sacred] Could not attach MongoObserver: {e}")



@ex.config
def cfg():
    # Blender invocation
    blender_exe = _BASE.get("blender_exe", r"C:\\Program Files\\Blender Foundation\\Blender 5.0\\blender.exe")
    blend_file = _BASE.get("blend_file", os.path.join(REPO_ROOT, "jubilee.blend"))
    script_file = _BASE.get("script_file", os.path.join(REPO_ROOT, "animation_to_gif.py"))

    # These mirror the constants in animation_to_gif.py
    test_mode = _BASE.get("test_mode", False)
    test_max_frames = _BASE.get("test_max_frames", 5)
    fps = _BASE.get("fps", 24)
    scale_width = _BASE.get("scale_width", 320)
    render_res_x = _BASE.get("render_res_x", 300)
    render_res_y = _BASE.get("render_res_y", 300)
    render_res_percent = _BASE.get("render_res_percent", 100)
    target_object_name = _BASE.get("target_object_name", "XY-carriage")
    camera_offset = _BASE.get("camera_offset", [0.7, -1.2, 1.1])
    camera_lens = _BASE.get("camera_lens", 50)


@ex.automain
def run(_run,
    blender_exe,
    blend_file,
    script_file,
        test_mode,
        test_max_frames,
        fps,
        scale_width,
        render_res_x,
        render_res_y,
        render_res_percent,
        target_object_name):
    """Sacred entry: write JSON config (including paths), then call Blender."""


    _run.info["config_file"] = CONFIG_FILE

    cmd = [
        blender_exe,
        "-b", blend_file,
        "-P", script_file,
    ]

    _run.info["blender_cmd"] = " ".join(cmd)
    _run.info["experiment_name"] = _DEFAULT_EXPERIMENT_NAME
    _run.info["mongo_url"] = _DEFAULT_MONGO_URL
    _run.info["mongo_db_name"] = _DEFAULT_MONGO_DB

    print("[sacred] Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)

    # The Blender script writes the GIF to docs/jubilee_test.gif by default
    gif_path = os.path.join(os.path.dirname(__file__), "docs", "jubilee_test.gif")
    if os.path.exists(gif_path):
        _run.add_artifact(gif_path, name="animation.gif")
        _run.info["gif_path"] = gif_path

    first_frame = os.path.join(os.path.dirname(__file__), "render_gif", "frame_0001.png")
    if os.path.exists(first_frame):
        _run.add_artifact(first_frame, name="first_frame.png")
        _run.info["first_frame_path"] = first_frame
