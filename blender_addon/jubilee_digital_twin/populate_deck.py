import sys
import os
import json
import bpy
from pathlib import Path


def _setup() -> Path:
    # blender_models/ (or pipeline_data/) is one level below twin root
    twin_root = Path(os.path.dirname(os.path.dirname(bpy.data.filepath)))
    if str(twin_root) not in sys.path:
        sys.path.insert(0, str(twin_root))
    return twin_root


def _read_paths_cache(twin_root: Path) -> dict:
    cache_path = twin_root / "pipeline_data" / "jubilee_paths.json"
    if cache_path.is_file():
        with open(cache_path) as f:
            return json.load(f)
    return {}


def _latest_experiment_dir(interface_dir: str) -> Path | None:
    exp_root = Path(interface_dir) / "interface_graphique" / "experiment_deck"
    if not exp_root.is_dir():
        return None
    subdirs = sorted(
        [d for d in exp_root.iterdir() if d.is_dir()],
        reverse=True,
    )
    return subdirs[0] if subdirs else None


def _remove_collection(name: str) -> None:
    col = bpy.data.collections.get(name)
    if col is None:
        return
    for obj in list(col.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for parent in list(bpy.data.collections) + [bpy.context.scene.collection]:
        if col.name in [c.name for c in parent.children]:
            parent.children.unlink(col)
    bpy.data.collections.remove(col)


def populate_deck() -> None:
    twin_root = _setup()
    paths = _read_paths_cache(twin_root)

    interface_dir = paths.get("interface_dir")
    if not interface_dir:
        raise RuntimeError(
            "interface_dir not in jubilee_paths.json. Run: jubilee-twin setup-scene"
        )

    exp_dir = _latest_experiment_dir(interface_dir)
    if exp_dir is None:
        raise RuntimeError(
            f"No experiment folders found under {interface_dir}/interface_graphique/experiment_deck/. "
            "Generate a 3D export from the interface app first."
        )

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

    print(f"[populate_deck] Loading from: {exp_dir.name}")
    _remove_collection("Deck")
    deck_col = bpy.data.collections.new("Deck")
    bpy.context.scene.collection.children.link(deck_col)

    with bpy.data.libraries.load(str(deck_blend), link=False) as (data_from, data_to):
        data_to.objects = list(data_from.objects)

    import mathutils

    def _bbox(obj):
        corners = [obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box]
        return corners

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

    corners_zplate = _bbox(z_plate)
    corners_deck   = _bbox(deck_plate_obj)

    zplate_max_x = max(c.x for c in corners_zplate)
    zplate_max_y = max(c.y for c in corners_zplate)
    zplate_min_z = min(c.z for c in corners_zplate)
    deck_min_z   = min(c.z for c in corners_deck)

    bpy.data.objects.remove(deck_plate_obj, do_unlink=True)

    for obj in labware_objs:
        if obj.name in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(obj)
        # Jubilee X=0 is at max world-X; Y=0 is at max world-Y → reflect both axes
        obj.location.x = zplate_max_x - obj.location.x
        obj.location.y = zplate_max_y - obj.location.y
        obj.location.z += zplate_min_z - deck_min_z
        obj.parent = z_plate
        obj.matrix_parent_inverse = z_plate.matrix_world.inverted()
        deck_col.objects.link(obj)

    print(f"[populate_deck] {len(labware_objs)} labware placed on {z_plate.name}.")


def main():
    populate_deck()

    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
    bpy.ops.wm.quit_blender()


if __name__ == "__main__":
    main()
