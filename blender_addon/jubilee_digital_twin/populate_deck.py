import sys
import logging
import bpy
from pathlib import Path

# get_logger is imported lazily (jubilee_twin not on sys.path at Blender addon load time)
logger = logging.getLogger("jubilee_twin.populate_deck")


def _ensure_colorlog() -> None:
    try:
        from jubilee_twin.log import get_logger as _gl  # triggers colorlog setup
        _gl("jubilee_twin.populate_deck")
    except Exception:
        pass


def _read_paths_cache() -> dict:
    """Put the addon and pipeline on sys.path, then read jubilee_paths.json."""
    addon_dir = str(Path(__file__).parent)
    if addon_dir not in sys.path:
        sys.path.insert(0, addon_dir)
    import scene_utils
    scene_utils.ensure_pipeline_on_path()
    return scene_utils.read_paths_cache()


def _latest_experiment_dir(paths: dict) -> Path | None:
    """Locate the newest ``YYYY-MM-DD_<name>/`` experiment folder.

    Prefers ``experiment_deck_dir`` written by the science-jubilee-interface
    plugin (post-refactor).  Falls back to the legacy ``interface_dir /
    interface_graphique / experiment_deck`` layout for older installs.
    """
    candidates: list[Path] = []
    exp_dir = paths.get("experiment_deck_dir")
    if exp_dir:
        candidates.append(Path(exp_dir))
    interface_dir = paths.get("interface_dir")
    if interface_dir:
        # Legacy layout — pre-plugin refactor of science-jubilee-interface.
        candidates.append(Path(interface_dir) / "interface_graphique" / "experiment_deck")
        # New layout — the interface package writes to <repo_root>/experiment_deck/.
        candidates.append(Path(interface_dir) / "experiment_deck")

    for root in candidates:
        if not root.is_dir():
            continue
        subdirs = sorted(
            [d for d in root.iterdir() if d.is_dir()],
            reverse=True,
        )
        if subdirs:
            return subdirs[0]
    return None


def _remove_collection(name: str) -> None:
    col = bpy.data.collections.get(name)
    if col is None:
        return
    for obj in list(col.all_objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for parent in list(bpy.data.collections) + [bpy.context.scene.collection]:
        if col.name in [c.name for c in parent.children]:
            parent.children.unlink(col)
    bpy.data.collections.remove(col)


# Measured on the real machine: distance from the bed plate's edge on the
# machine-Y=0 side to the deck plate's Y=0 edge. In X the 305 mm deck spans
# the bed's full width.
DECK_Y_INSET_MM = 25.0


def _deck_xy_origin(z_plate):
    """World XY (metres) of machine X=Y=0: the deck plate's corner on the bed.

    The deck plate is the reference, so the origin comes from the bed
    plate's bounds, not from the carriage or tools. Machine +X/+Y run
    toward world -X/-Y, so X=Y=0 is on the bed's max-X / max-Y side.
    """
    from mathutils import Vector

    bpy.context.view_layer.update()
    bed = z_plate.evaluated_get(bpy.context.evaluated_depsgraph_get())
    corners = [bed.matrix_world @ Vector(c) for c in bed.bound_box]
    return (
        max(c.x for c in corners),
        max(c.y for c in corners) - DECK_Y_INSET_MM / 1000.0,
    )


def _file_world(obj):
    """World matrix as stored in deck.blend, rebuilt from local transforms.

    Objects freshly appended via bpy.data.libraries.load() are not in any
    view layer yet, so their matrix_world is identity until a depsgraph
    evaluates them (Blender 4+ no longer saves it in the file).
    """
    if obj.parent is None:
        return obj.matrix_basis.copy()
    return _file_world(obj.parent) @ obj.matrix_parent_inverse @ obj.matrix_basis


def _place_labware(labware_objs, deck_col, z_plate, z_shift):
    """Transform the complete exported geometry, then inherit bed motion.

    deck.blend is already in machine metres. A half-turn about Z maps
    increasing machine X/Y onto decreasing world X/Y, exactly once for
    each imported object hierarchy. Preserve the existing Z alignment.
    """
    from mathutils import Matrix

    origin_x, origin_y = _deck_xy_origin(z_plate)
    transform = Matrix.Diagonal((-1.0, -1.0, 1.0, 1.0))
    transform.translation = (origin_x, origin_y, z_shift)
    objects = set(labware_objs)
    original_world = {obj: _file_world(obj) for obj in labware_objs}

    root = bpy.data.objects.new("Deck Machine Frame", None)
    deck_col.objects.link(root)
    root.parent = z_plate
    root.matrix_parent_inverse = z_plate.matrix_world.inverted()
    root.matrix_basis = transform

    for obj in labware_objs:
        if obj.name in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(obj)
        if obj.name not in deck_col.objects:
            deck_col.objects.link(obj)
        if obj.parent not in objects:
            obj.parent = root
            obj.matrix_parent_inverse = Matrix.Identity(4)
            obj.matrix_basis = original_world[obj]

    bpy.context.view_layer.update()
    logger.info("Deck machine origin: X=%.6f Y=%.6f m", origin_x, origin_y)
    return root


def populate_deck() -> None:
    _ensure_colorlog()
    paths = _read_paths_cache()

    exp_dir = _latest_experiment_dir(paths)
    if exp_dir is None:
        raise RuntimeError(
            "No experiment folder found. Register science-jubilee-interface "
            "(exposes 'experiment_deck_dir' via the 'jubilee.paths' entry point) "
            "and generate a 3D export from the interface app first. "
            "If both packages are installed, run: jubilee-twin setup-scene"
        )

    logger.warning("deck.blend   → %s", exp_dir / "deck.blend")
    deck_blend = exp_dir / "deck.blend"
    if not deck_blend.exists():
        raise RuntimeError(
            f"deck.blend not found in {exp_dir.name}. "
            "Use the interface app to export a 3D deck first."
        )

    # The deck plate in the Jubilee model — labware will be parented to it
    z_plate = bpy.data.objects.get("z_plate_3_V5")
    if z_plate is None:
        raise RuntimeError("Object 'z_plate_3_V5' not found in scene.")

    logger.info("Loading deck from: %s", exp_dir.name)
    _remove_collection("Deck")
    deck_col = bpy.data.collections.new("Deck")
    bpy.context.scene.collection.children.link(deck_col)

    with bpy.data.libraries.load(str(deck_blend), link=False) as (data_from, data_to):
        data_to.objects = list(data_from.objects)

    import mathutils

    def _bbox(obj, world):
        return [world @ mathutils.Vector(c) for c in obj.bound_box]

    deck_plate_obj = None
    labware_objs = []
    for obj in data_to.objects:
        if obj is None:
            continue
        if obj.name.lower() == "deck" or obj.name.lower().startswith("deck."):
            deck_plate_obj = obj
        else:
            labware_objs.append(obj)

    if deck_plate_obj is None:
        raise RuntimeError("No 'deck' object found in deck.blend.")

    corners_zplate = _bbox(z_plate, z_plate.matrix_world)
    corners_deck   = _bbox(deck_plate_obj, _file_world(deck_plate_obj))

    zplate_min_z = min(c.z for c in corners_zplate)
    deck_min_z   = min(c.z for c in corners_deck)

    _place_labware(labware_objs, deck_col, z_plate, zplate_min_z - deck_min_z)
    bpy.data.objects.remove(deck_plate_obj, do_unlink=True)

    logger.info("%d labware placed on %s", len(labware_objs), z_plate.name)


def main():
    populate_deck()

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    bpy.ops.wm.quit_blender()


if __name__ == "__main__":
    main()
