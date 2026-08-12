import bpy
import sys
import os
from bpy.props import StringProperty, IntProperty, CollectionProperty, BoolProperty, EnumProperty
from bpy.types import PropertyGroup, UIList, Operator, Panel

bl_info = {
    "name": "Digital Twin",
    "author": "Ines BOUIZEGARENE",
    "version": (1, 0),
    "blender": (5, 1, 2),
    "location": "View3D > Sidebar > Twin",
    "description": "Jubilee digital twin for simulation and collision detection",
    "category": "3D View",
}


# ── Gcode file discovery ───────────────────────────────────────────────────
# Module-level cache keeps the list alive between Blender redraws (Blender
# holds raw pointers to enum items, so they must not be garbage-collected).
_gcode_items_cache: list = []


def _jubilee_dir() -> str | None:
    """Read jubilee_dir written by the driver into pipeline_data/jubilee_paths.json."""
    import json
    twin_root = os.path.dirname(os.path.dirname(bpy.data.filepath))
    cache_path = os.path.join(twin_root, "pipeline_data", "jubilee_paths.json")
    if os.path.isfile(cache_path):
        with open(cache_path) as f:
            return json.load(f).get("jubilee_dir")
    return None


def _gcode_enum_items(self, context):
    global _gcode_items_cache
    items = []
    jdir = _jubilee_dir()
    if jdir:
        gcode_logs = os.path.join(jdir, "gcode_logs")
        if os.path.isdir(gcode_logs):
            for fn in sorted(os.listdir(gcode_logs)):
                if fn.endswith(".gcode"):
                    full = os.path.join(gcode_logs, fn)
                    items.append((full, fn, full))
    if not items:
        items = [("", "— run jubilee-twin setup-scene —", "")]
    _gcode_items_cache = items
    return _gcode_items_cache

class CollisionEvent(PropertyGroup):
    frame:       IntProperty(name="Frame", default=1)
    object_name: StringProperty(name="Collided Object")


# ── UIList ────────────────────────────────────────────────────────────────
class COLLISION_UL_list(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        row = layout.row(align=True)
        row.label(text=f"Frame {item.frame}", icon='KEYFRAME')
        row.label(text=item.object_name)


# ── Jump to frame operator ────────────────────────────────────────────────
class COLLISION_OT_goto_frame(Operator):
    bl_idname = "collision.goto_frame"
    bl_label = "Go To Frame"
    frame: IntProperty()

    def execute(self, context):
        context.scene.frame_set(self.frame)
        return {'FINISHED'}


# ── Animate operator ──────────────────────────────────────────────────────
class ANIM_OT_animate(Operator):
    bl_idname = "anim.animate_path"
    bl_label = "Animate from CSV"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        twin_root = os.path.dirname(os.path.dirname(bpy.data.filepath))
        if twin_root not in sys.path:
            sys.path.insert(0, twin_root)

        # If a gcode file is selected, regenerate pathout.csv first
        gcode_file = context.scene.jubilee_gcode_file
        if gcode_file and os.path.isfile(gcode_file):
            from jubilee_twin.pipeline import path_follower
            pipeline_data_dir = os.path.join(twin_root, "pipeline_data")
            path_follower.run(gcode_file=gcode_file, output_dir=pipeline_data_dir)

        # deferred import so bpy.data.filepath is set before animate_path runs
        from . import animate_path as ap
        ap.run_animation()
        return {'FINISHED'}


# ── Place tools operator ──────────────────────────────────────────────────
class ANIM_OT_place_tools(Operator):
    bl_idname = "anim.place_tools"
    bl_label = "Place Tools"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import tool_placement as tp
        tp.place_tools()
        return {'FINISHED'}


# ── Run ray tracing operator ──────────────────────────────────────────────
class ANIM_OT_raytracing(Operator):
    bl_idname = "anim.raytracing_sim"
    bl_label = "Launch Ray Tracing"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import ray_tracing as rt

        rt._collision_list.clear()
        rt.ray_tracing_CD()

        context.scene.collision_list.clear()
        for event in rt._collision_list:
            item = context.scene.collision_list.add()
            item.frame = event.frame
            item.object_name = event.collided_object
        context.scene.collision_list_index = 0

        return {'FINISHED'}


# ── Toggle rays operator ──────────────────────────────────────────────────
class ANIM_OT_toggle_rays(Operator):
    bl_idname = "anim.toggle_rays"
    bl_label = "Toggle Rays"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import ray_tracing as rt
        rt.set_show_rays(not rt._show_rays)
        context.scene.show_rays = rt._show_rays
        return {'FINISHED'}


# ── Toggle hulls operator ─────────────────────────────────────────────────
class ANIM_OT_toggle_hulls(Operator):
    bl_idname = "anim.toggle_hulls"
    bl_label = "Toggle Hulls"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import ray_tracing as rt
        rt.set_show_hulls(not rt._show_hulls)
        context.scene.show_hulls = rt._show_hulls
        return {'FINISHED'}


# ── Animation subpanel ────────────────────────────────────────────────────
class ANIM_PT_digital_twin(Panel):
    bl_label = "Digital Twin"
    bl_idname = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"

    def draw(self, context):
        pass


class ANIM_PT_animate(Panel):
    bl_label = "Animation"
    bl_idname = "ANIM_PT_animate"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"
    bl_order = 0

    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene, "jubilee_gcode_file", text="GCode")
        layout.operator("anim.animate_path", icon='PLAY')


# ── Setup subpanel ────────────────────────────────────────────────────────
class ANIM_PT_setup(Panel):
    bl_label = "Setup"
    bl_idname = "ANIM_PT_setup"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"
    bl_order = -1

    def draw(self, context):
        self.layout.operator("anim.place_tools", icon='TOOL_SETTINGS')


# ── Ray tracing subpanel ──────────────────────────────────────────────────
class ANIM_PT_raytracing(Panel):
    bl_label = "Ray Tracing"
    bl_idname = "ANIM_PT_raytracing"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        layout.operator("anim.raytracing_sim", icon='PLAY')
        layout.separator()

        count = len(scene.collision_list)
        if count == 0:
            layout.label(text="No collisions detected", icon='CHECKMARK')
        else:
            layout.label(text=f"{count} collision(s) detected", icon='ERROR')
            layout.template_list(
                "COLLISION_UL_list", "",
                scene, "collision_list",
                scene, "collision_list_index",
                rows=min(count, 5)
            )
            if scene.collision_list_index >= 0 and count > 0:
                selected = scene.collision_list[scene.collision_list_index]
                op = layout.operator("collision.goto_frame",
                                     text=f"Jump to Frame {selected.frame}",
                                     icon='NEXT_KEYFRAME')
                op.frame = selected.frame


# ── Display subpanel ──────────────────────────────────────────────────────
class ANIM_PT_display(Panel):
    bl_label = "Display"
    bl_idname = "ANIM_PT_display"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        row = layout.row()
        row.operator(
            "anim.toggle_rays",
            text="Rays: ON" if scene.show_rays else "Rays: OFF",
            icon='HIDE_OFF' if scene.show_rays else 'HIDE_ON'
        )
        row = layout.row()
        row.operator(
            "anim.toggle_hulls",
            text="Hulls: ON" if scene.show_hulls else "Hulls: OFF",
            icon='HIDE_OFF' if scene.show_hulls else 'HIDE_ON'
        )


classes = [
    CollisionEvent,
    COLLISION_UL_list,
    COLLISION_OT_goto_frame,
    ANIM_OT_place_tools,
    ANIM_OT_animate,
    ANIM_OT_raytracing,
    ANIM_OT_toggle_rays,
    ANIM_OT_toggle_hulls,
    ANIM_PT_digital_twin,
    ANIM_PT_setup,
    ANIM_PT_animate,
    ANIM_PT_raytracing,
    ANIM_PT_display,
]


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.collision_list = CollectionProperty(type=CollisionEvent)
    bpy.types.Scene.collision_list_index = IntProperty(default=0)
    bpy.types.Scene.show_rays = BoolProperty(name="Show Rays", default=False)
    bpy.types.Scene.show_hulls = BoolProperty(name="Show Hulls", default=False)
    bpy.types.Scene.jubilee_gcode_file = EnumProperty(
        name="GCode File",
        description="GCode file from science_jubilee/gcode_logs/ to animate",
        items=_gcode_enum_items,
    )


def unregister():
    for attr in ("collision_list", "collision_list_index", "show_rays", "show_hulls",
                 "jubilee_gcode_file"):
        if hasattr(bpy.types.Scene, attr):
            delattr(bpy.types.Scene, attr)
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass
