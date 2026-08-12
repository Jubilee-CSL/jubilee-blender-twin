import bpy
import sys
from bpy.props import StringProperty, IntProperty, CollectionProperty, BoolProperty
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


# ── Property group for one collision event ────────────────────────────────
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
        # deferred import so bpy.data.filepath is set before animate_path runs
        from . import animate_path as ap
        ap.run_animation()
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
        self.layout.operator("anim.animate_path", icon='PLAY')


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
    ANIM_OT_animate,
    ANIM_OT_raytracing,
    ANIM_OT_toggle_rays,
    ANIM_OT_toggle_hulls,
    ANIM_PT_digital_twin,
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


def unregister():
    for attr in ("collision_list", "collision_list_index", "show_rays", "show_hulls"):
        if hasattr(bpy.types.Scene, attr):
            delattr(bpy.types.Scene, attr)
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass
