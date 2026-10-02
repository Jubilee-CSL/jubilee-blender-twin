"""Deck alignment against the bed plate, the physical deck reference."""

import importlib.util
from math import pi
from pathlib import Path
from types import SimpleNamespace

import pytest

bpy = pytest.importorskip("bpy")


def load_addon_module(name):
    path = (Path(__file__).parents[1] / "blender_addon"
            / "jubilee_digital_twin" / f"{name}.py")
    spec = importlib.util.spec_from_file_location(f"deck_test_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def importer():
    return load_addon_module("populate_deck")


# Bed plate world bounds: X [-0.075, 0.23], Y [0.185, 0.525]. With the 25 mm
# Y inset, machine X=Y=0 sits at world (0.23, 0.5).
ORIGIN = (0.23, 0.5)


@pytest.fixture
def deck_scene():
    stage = bpy.data.collections.new("deck_alignment_stage")
    deck = bpy.data.collections.new("deck_alignment_import")
    deck_name = deck.name
    bpy.context.scene.collection.children.link(stage)
    bpy.context.scene.collection.children.link(deck)
    meshes = []

    def object_(name, parent=None, vertices=None):
        mesh = None
        if vertices is not None:
            mesh = bpy.data.meshes.new(name)
            mesh.from_pydata(vertices, [], [(0, 1, 2)])
            meshes.append(mesh)
        obj = bpy.data.objects.new(name, mesh)
        stage.objects.link(obj)
        obj.parent = parent
        return obj

    z = object_("deck_alignment_z")
    # Rotated bed: local X maps to world +Y, local Y to world -X.
    plate = object_("deck_alignment_bed", z, vertices=[
        (-0.115, 0.17, 0.0), (0.225, 0.17, 0.0), (-0.115, 0.475, 0.0),
    ])
    plate.location = (0.4, 0.3, 0.2)
    plate.rotation_euler.z = pi / 2
    bpy.context.view_layer.update()
    yield SimpleNamespace(deck=deck, z=z, plate=plate, object_=object_)
    current_deck = bpy.data.collections.get(deck_name)
    deck_objects = set(current_deck.all_objects) if current_deck else set()
    for obj in set(stage.all_objects) | deck_objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    for mesh in meshes:
        bpy.data.meshes.remove(mesh)
    if current_deck:
        bpy.data.collections.remove(current_deck)
    bpy.data.collections.remove(stage)


def world_matrix(obj):
    bpy.context.view_layer.update()
    return obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world.copy()


def test_origin_is_bed_corner_with_y_inset(importer, deck_scene):
    assert importer.DECK_Y_INSET_MM == 25.0
    assert importer._deck_xy_origin(deck_scene.plate) == pytest.approx(ORIGIN)


def test_origin_follows_bed_not_bed_height(importer, deck_scene):
    deck_scene.z.location.z += 0.05
    assert importer._deck_xy_origin(deck_scene.plate) == pytest.approx(ORIGIN)


def test_geometry_not_only_centres_is_rotated(importer, deck_scene):
    obj = deck_scene.object_(
        "deck_alignment_asymmetric", vertices=[(0, 0, 0), (0.04, 0, 0), (0, 0.01, 0)],
    )
    obj.location = (0.06, 0.08, 0.003)
    obj.rotation_euler.z = 0.3
    source = world_matrix(obj)
    original_points = [source @ vertex.co for vertex in obj.data.vertices]

    importer._place_labware([obj], deck_scene.deck, deck_scene.plate, 0.2)

    actual = world_matrix(obj)
    for vertex, point in zip(obj.data.vertices, original_points):
        expected = (ORIGIN[0] - point.x, ORIGIN[1] - point.y, 0.2 + point.z)
        assert tuple(actual @ vertex.co) == pytest.approx(expected)


def test_parented_geometry_is_transformed_once(importer, deck_scene):
    parent = deck_scene.object_("deck_alignment_parent")
    parent.location = (0.06, 0.08, 0.003)
    child = deck_scene.object_("deck_alignment_child", parent)
    child.location = (0.01, 0.02, 0.03)
    original = world_matrix(child).translation

    root = importer._place_labware(
        [parent, child], deck_scene.deck, deck_scene.plate, 0.2,
    )

    assert parent.parent == root
    assert child.parent == parent
    expected = (ORIGIN[0] - original.x, ORIGIN[1] - original.y, 0.2 + original.z)
    assert tuple(world_matrix(child).translation) == pytest.approx(expected)


def test_freshly_appended_geometry_keeps_file_position(importer, deck_scene):
    # Objects from bpy.data.libraries.load() are in no view layer, so their
    # matrix_world is identity; placement must use the stored transforms.
    obj = bpy.data.objects.new("deck_alignment_appended", None)
    obj.location = (0.06, 0.08, 0.003)
    assert obj.matrix_world.translation.length == 0

    importer._place_labware([obj], deck_scene.deck, deck_scene.plate, 0.2)

    expected = (ORIGIN[0] - 0.06, ORIGIN[1] - 0.08, 0.2 + 0.003)
    assert tuple(world_matrix(obj).translation) == pytest.approx(expected)


def test_z_alignment_and_bed_motion_are_preserved(importer, deck_scene):
    obj = deck_scene.object_("deck_alignment_height")
    obj.location = (0.06, 0.08, 0.003)
    importer._place_labware([obj], deck_scene.deck, deck_scene.plate, 0.2)
    before = world_matrix(obj).translation
    assert before.z == pytest.approx(0.203)

    deck_scene.z.location.z += 0.01

    assert tuple(world_matrix(obj).translation - before) == pytest.approx((0, 0, 0.01))


def test_reimport_cleanup_removes_transform_root(importer, deck_scene):
    obj = deck_scene.object_("deck_alignment_cleanup")
    root = importer._place_labware([obj], deck_scene.deck, deck_scene.plate, 0)
    root_name = root.name
    name = deck_scene.deck.name
    importer._remove_collection(name)
    assert bpy.data.objects.get(root_name) is None
    assert bpy.data.objects.get("deck_alignment_cleanup") is None
    # Restore an empty collection for fixture cleanup.
    deck_scene.deck = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(deck_scene.deck)
