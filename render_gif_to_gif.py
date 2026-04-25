"""
render_gif_to_gif.py
--------------------
Standalone script (no Blender required) that assembles the PNG frames
inside the render_gif/ folder into an animated GIF.

Requirements:
    pip install Pillow

Usage:
    python render_gif_to_gif.py [--fps 24] [--width 640] [--output docs/jubilee.gif]
"""

import argparse
import glob
import logging
import os
import sys

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INPUT_DIR = os.path.join(SCRIPT_DIR, "render_gif")
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "docs", "jubilee_render.gif")
DEFAULT_FPS = 24
DEFAULT_WIDTH = 640  # resize width in pixels; 0 = keep original size


def parse_args():
    p = argparse.ArgumentParser(description="Convert render_gif PNG frames to an animated GIF.")
    p.add_argument("--input-dir", default=DEFAULT_INPUT_DIR,
                   help=f"Folder containing frame_NNNN.png files (default: {DEFAULT_INPUT_DIR})")
    p.add_argument("--output", default=DEFAULT_OUTPUT,
                   help=f"Output GIF path (default: {DEFAULT_OUTPUT})")
    p.add_argument("--fps", type=int, default=DEFAULT_FPS,
                   help=f"Playback frames per second (default: {DEFAULT_FPS})")
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH,
                   help="Resize frames to this width in pixels; 0 = keep original (default: 640)")
    p.add_argument("--loop", type=int, default=0,
                   help="Number of loops; 0 = infinite (default: 0)")
    return p.parse_args()


def collect_frames(input_dir: str) -> list[str]:
    """Return sorted list of PNG frame paths from input_dir."""
    pattern = os.path.join(input_dir, "frame_*.png")
    frames = sorted(glob.glob(pattern))
    log.debug("Pattern: %s", pattern)
    log.info("Found %d frame(s) in '%s'", len(frames), input_dir)
    return frames


def build_gif(frames: list[str], output_path: str, fps: int, width: int, loop: int) -> None:
    """Load frames with Pillow and save as an animated GIF."""
    try:
        from PIL import Image
    except ImportError:
        log.error("Pillow is not installed. Run: pip install Pillow")
        sys.exit(1)

    duration_ms = max(1, round(1000 / fps))
    log.info("Frame duration: %d ms  (%.1f fps)", duration_ms, fps)

    images = []
    for idx, path in enumerate(frames):
        log.debug("Loading frame %d/%d: %s", idx + 1, len(frames), os.path.basename(path))
        img = Image.open(path).convert("RGBA")

        if width > 0 and img.width != width:
            ratio = width / img.width
            new_h = round(img.height * ratio)
            img = img.resize((width, new_h), Image.LANCZOS)
            log.debug("  Resized to %dx%d", width, new_h)

        # Convert to palette mode for compact GIF output
        img = img.convert("P", palette=Image.ADAPTIVE, colors=256)
        images.append(img)

    if not images:
        log.error("No frames loaded — nothing to save.")
        sys.exit(1)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    log.info("Saving GIF → %s", output_path)
    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=loop,
        optimize=True,
    )
    size_kb = os.path.getsize(output_path) / 1024
    log.info("Done. GIF size: %.1f KB  (%d frames)", size_kb, len(images))


def main():
    args = parse_args()

    log.info("=== render_gif_to_gif ===")
    log.info("Input dir : %s", args.input_dir)
    log.info("Output    : %s", args.output)
    log.info("FPS       : %d", args.fps)
    log.info("Width     : %s px", args.width if args.width > 0 else "original")
    log.info("Loop      : %s", "infinite" if args.loop == 0 else args.loop)

    if not os.path.isdir(args.input_dir):
        log.error("Input directory not found: %s", args.input_dir)
        sys.exit(1)

    frames = collect_frames(args.input_dir)
    if not frames:
        log.error("No frame_*.png files found in '%s'", args.input_dir)
        sys.exit(1)

    build_gif(frames, args.output, args.fps, args.width, args.loop)


if __name__ == "__main__":
    main()
