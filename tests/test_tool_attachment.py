"""Blender transform regressions; skipped when bpy is not installed."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

bpy = pytest.importorskip("bpy")


@pytest.fixture
def animation():
    path = (Path(__file__).parents[1] / "blender_addon"
            / "jubilee_digital_twin" / "animate_path.py")
    spec = importlib.util.spec_from_file_location("attachment_animation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def tool_scene():
    objects = []
    collections = []

    def collection(name, parent=None):
        col = bpy.data.collections.new(name)
        (parent or bpy.context.scene.collection).children.link(col)
        collections.append(col)
        return col

    def object_(name, col, parent=None):
        obj = bpy.data.objects.new(name, None)
        col.objects.link(obj)
        obj.parent = parent
        objects.append(obj)
        return obj

    tools = collection("Tools")
    tool = collection("attachment_test_pen", tools)
    nested = collection("attachment_test_parts", tool)
    axes = collection("attachment_test_axes")
    y_axis = object_("Y-axis", axes)
    x_axis = object_("X-axis", axes, y_axis)
    z_axis = object_("Z-axis", axes)
    anchor = object_("XY-carriage", axes, x_axis)
    root = object_("attachment_test_root", tool)
    child = object_("attachment_test_child", nested, root)
    root.location = (0.1, 0.2, 0.3)
    child.location = (0.01, 0.02, 0.03)
    frame_start = bpy.context.scene.frame_current
    frame_end = bpy.context.scene.frame_end
    bpy.context.view_layer.update()
    yield SimpleNamespace(
        tool=tool, root=root, child=child, anchor=anchor,
        x=x_axis, y=y_axis, z=z_axis, object_=object_, nested=nested,
    )
    for obj in reversed(objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for col in reversed(collections):
        bpy.data.collections.remove(col)
    bpy.context.scene.frame_end = frame_end
    bpy.context.scene.frame_set(frame_start)


def world_position(obj):
    bpy.context.view_layer.update()
    graph = bpy.context.evaluated_depsgraph_get()
    return obj.evaluated_get(graph).matrix_world.translation.copy()


def configure_state(monkeypatch, animation, scene):
    import jubilee_twin.pipeline.utils as utils

    monkeypatch.setattr(animation, "_setup_paths", lambda: (None, None))
    monkeypatch.setattr(animation, "identify_tool", lambda *_: scene.tool.name)
    monkeypatch.setattr(utils, "get_axis_max", lambda *_: 0.0)


def test_roots_include_nested_and_independent_objects(animation, tool_scene):
    independent = tool_scene.object_("attachment_test_independent", tool_scene.nested)

    assert set(animation.tool_roots(tool_scene.tool)) == {tool_scene.root, independent}


def test_animation_moves_once_and_detaches(monkeypatch, tmp_path, animation, tool_scene):
    configure_state(monkeypatch, animation, tool_scene)
    import jubilee_twin.pipeline.utils as utils

    csv = tmp_path / "pathout.csv"
    csv.write_text("0,0,0,0,0,-1\n0,0,0,0,1,1\n10,20,0,0,0,1\n"
                   "0,0,0,0,-1,1\n10,20,0,0,0,-1\n")
    monkeypatch.setattr(animation, "_setup_paths", lambda: (str(csv), str(tmp_path)))
    monkeypatch.setattr(utils, "get_axis_min", lambda *_: 0.0)
    # Never permit the background script's save branch in a test.
    monkeypatch.setattr(sys, "argv", ["pytest"])

    def drive(x, y, z, frame=None):
        tool_scene.x.location.x = x / 1000
        tool_scene.y.location.y = y / 1000
        tool_scene.z.location.z = z / 1000
        for obj in (tool_scene.x, tool_scene.y, tool_scene.z):
            if frame is not None:
                obj.keyframe_insert(data_path="location", frame=frame)
        bpy.context.view_layer.update()

    monkeypatch.setitem(sys.modules, "scene_utils", SimpleNamespace(drive_to_mm=drive))
    tool_scene.child.constraints.new('CHILD_OF').target = tool_scene.anchor
    animation.run_animation()
    assert len(tool_scene.child.constraints) == 0
    assert len(tool_scene.root.constraints) == 1
    bpy.context.scene.frame_set(2)
    before = world_position(tool_scene.child)
    bpy.context.scene.frame_set(3)
    assert tuple(world_position(tool_scene.child) - before) == pytest.approx((0.01, 0.02, 0))
    bpy.context.scene.frame_set(5)
    assert tool_scene.root.constraints[0].influence == 0
    assert tuple(world_position(tool_scene.child)) == pytest.approx(tuple(before))


def test_missing_named_carriage_is_reported(animation):
    if bpy.data.objects.get("XY-carriage") is not None:
        pytest.skip("Existing scene has a carriage")
    with pytest.raises(RuntimeError, match="XY-carriage not found"):
        animation.carriage_anchor()