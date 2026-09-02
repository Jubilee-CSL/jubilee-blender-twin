import bpy
import sys
import os
from bpy.props import (
    StringProperty, IntProperty, FloatProperty, FloatVectorProperty,
    CollectionProperty, BoolProperty, EnumProperty,
)
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
    bl_description = (
        "Load tool .blend files and parking posts into the scene.\n"
        "Requires pipeline_data/tool_data.csv — run 'jubilee-twin setup-scene' first."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import tool_placement as tp
        tp.place_tools()
        return {'FINISHED'}


# ── Place camera operator ──────────────────────────────────────────────────
class ANIM_OT_place_camera(Operator):
    bl_idname = "anim.place_camera"
    bl_label = "Place Camera"
    bl_description = (
        "Create Toolhead_Cam at D2HW_C201H001 with intrinsics from jubilee_paths.json.\n"
        "Run 'jubilee-twin setup-scene' first so the calibration is available."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import place_camera as pc
        try:
            pc.main()
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        self.report({'INFO'}, "Toolhead_Cam placed")
        return {'FINISHED'}


# ── Populate deck operator ───────────────────────────────────────────────
class ANIM_OT_populate_deck(Operator):
    bl_idname = "anim.populate_deck"
    bl_label = "Populate Deck"
    bl_description = (
        "Load labware from the latest experiment folder into the scene.\n"
        "Requires pipeline_data/jubilee_paths.json (run 'jubilee-twin setup-scene') "
        "and a deck.blend exported by the interface app."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import populate_deck as pd
        pd.populate_deck()
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


# ── Import .blend file operator ─────────────────────────────────────────
class IMPORT_OT_blend_file(Operator):
    bl_idname = "import.blend_file"
    bl_label = "Import .blend File"
    bl_description = "Append all objects from a .blend file into the current scene"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.blend", options={'HIDDEN'})

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not self.filepath.lower().endswith(".blend"):
            self.report({'ERROR'}, "Please select a .blend file")
            return {'CANCELLED'}
        if not os.path.isfile(self.filepath):
            self.report({'ERROR'}, f"File not found: {self.filepath}")
            return {'CANCELLED'}
        with bpy.data.libraries.load(self.filepath, link=False) as (data_from, data_to):
            data_to.objects = list(data_from.objects)
        linked = []
        for obj in data_to.objects:
            if obj is not None:
                bpy.context.scene.collection.objects.link(obj)
                linked.append(obj)
        self.report({'INFO'}, f"Imported {len(linked)} object(s) from {os.path.basename(self.filepath)}")
        return {'FINISHED'}


# ── Virtual scanner operators ────────────────────────────────────────────
class SCAN_OT_snapshot(Operator):
    bl_idname = "scan.snapshot"
    bl_label = "Take Snapshot"
    bl_description = "Render one frame through Toolhead_Cam at the current axis position."
    bl_options = {'REGISTER'}

    def execute(self, context):
        from . import snapshot as sn
        from . import virtual_scanner as vs
        scene = context.scene
        if scene.snap_use_pos:
            try:
                vs._drive_to_mm(scene.snap_x, scene.snap_y, scene.snap_z)
            except Exception as e:
                self.report({'ERROR'}, str(e))
                return {'CANCELLED'}
        try:
            out = sn.take_snapshot()
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        scene.scan_last_output = str(out)
        self.report({'INFO'}, f"Snapshot: {out.name}")
        bpy.ops.render.view_show('INVOKE_DEFAULT')
        return {'FINISHED'}


class SCAN_OT_run(Operator):
    bl_idname = "scan.run"
    bl_label = "Run Virtual Scan"
    bl_description = (
        "Render the (X, Y, bed-Z) grid to Scans/YYYYmmdd_HHMMSS/ with a "
        "camera.yaml + manifest.json alongside the images."
    )
    bl_options = {'REGISTER'}

    def execute(self, context):
        from . import virtual_scanner as vs
        scene = context.scene
        cfg = vs.ScanConfig(
            x_min=scene.scan_x_min, x_max=scene.scan_x_max, x_steps=scene.scan_x_steps,
            y_min=scene.scan_y_min, y_max=scene.scan_y_max, y_steps=scene.scan_y_steps,
            z_min=scene.scan_z_min, z_max=scene.scan_z_max, z_steps=scene.scan_z_steps,
            width=scene.scan_image_width, height=scene.scan_image_height,
            camera={
                "fx": scene.scan_fx, "fy": scene.scan_fy,
                "cx": scene.scan_cx, "cy": scene.scan_cy,
                "dist": scene.get("scan_dist", []),
                "offset": list(scene.scan_camera_offset),
            },
        )
        out_dir = vs.run_scan(cfg)
        scene.scan_last_output = str(out_dir)
        self.report({'INFO'}, f"Scan complete: {out_dir}")
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


class ANIM_PT_snapshot(Panel):
    bl_label = "Snapshot"
    bl_idname = "ANIM_PT_snapshot"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"
    bl_order = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        row = layout.row(align=True)
        row.prop(scene, "snap_use_pos", text="Move to")
        sub = row.row(align=True)
        sub.enabled = scene.snap_use_pos
        sub.prop(scene, "snap_x", text="x")
        sub.prop(scene, "snap_y", text="y")
        sub.prop(scene, "snap_z", text="z")
        layout.operator("scan.snapshot", icon='CAMERA_DATA')


class ANIM_PT_animate(Panel):
    bl_label = "Animation"
    bl_idname = "ANIM_PT_animate"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"
    bl_order = 1

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
        layout = self.layout
        twin_root = os.path.dirname(os.path.dirname(bpy.data.filepath))
        pipeline_data = os.path.join(twin_root, "pipeline_data")

        has_paths = os.path.isfile(os.path.join(pipeline_data, "jubilee_paths.json"))
        has_tools = os.path.isfile(os.path.join(pipeline_data, "tool_data.csv"))

        if not has_paths or not has_tools:
            col = layout.column(align=True)
            col.alert = True
            col.label(text="Run 'jubilee-twin setup-scene' first", icon='ERROR')
            layout.separator()

        layout.operator("anim.place_tools", icon='TOOL_SETTINGS')
        layout.operator("anim.place_camera", icon='OUTLINER_OB_CAMERA')
        layout.operator("anim.populate_deck", icon='OUTLINER_OB_SURFACE')
        layout.separator()
        layout.operator("import.blend_file", icon='APPEND_BLEND')


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


# ── Scanner subpanel ─────────────────────────────────────────────────────
class ANIM_PT_scanner(Panel):
    bl_label = "Virtual Scanner"
    bl_idname = "ANIM_PT_scanner"
    bl_parent_id = "ANIM_PT_digital_twin"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Twin"
    bl_order = 2

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        box = layout.box()
        box.label(text="Grid (machine mm)")
        for axis in ("x", "y", "z"):
            row = box.row(align=True)
            row.prop(scene, f"scan_{axis}_min", text=f"{axis.upper()} min")
            row.prop(scene, f"scan_{axis}_max", text="max")
            row.prop(scene, f"scan_{axis}_steps", text="steps")

        total = scene.scan_x_steps * scene.scan_y_steps * scene.scan_z_steps
        box.label(text=f"Total frames: {total}")

        row = box.row(align=True)
        row.prop(scene, "scan_image_width", text="W")
        row.prop(scene, "scan_image_height", text="H")

        layout.operator("scan.run", icon='RENDER_STILL')
        if scene.scan_last_output:
            layout.label(text=f"Last: {os.path.basename(scene.scan_last_output)}", icon='FILE_TICK')


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
    IMPORT_OT_blend_file,
    ANIM_OT_place_tools,
    ANIM_OT_place_camera,
    ANIM_OT_populate_deck,
    ANIM_OT_animate,
    ANIM_OT_raytracing,
    ANIM_OT_toggle_rays,
    ANIM_OT_toggle_hulls,
    SCAN_OT_snapshot,
    SCAN_OT_run,
    ANIM_PT_digital_twin,
    ANIM_PT_setup,
    ANIM_PT_snapshot,
    ANIM_PT_animate,
    ANIM_PT_raytracing,
    ANIM_PT_scanner,
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

    # ── Virtual scanner properties — defaults from bundled fallback yaml only. ──
    # bpy.data is restricted at register time so we never touch bpy.data.filepath here.
    from . import virtual_scanner as vs
    from . import camera_params as cp
    try:
        _cam = cp._load_fallback_yaml()
    except Exception:
        _cam = {}
    bpy.types.Scene.scan_x_min = FloatProperty(name="X min (mm)", default=vs.DEFAULT_X_MIN, unit='LENGTH')
    bpy.types.Scene.scan_x_max = FloatProperty(name="X max (mm)", default=vs.DEFAULT_X_MAX, unit='LENGTH')
    bpy.types.Scene.scan_x_steps = IntProperty(name="X steps", default=vs.DEFAULT_X_STEPS, min=1, soft_max=50)
    bpy.types.Scene.scan_y_min = FloatProperty(name="Y min (mm)", default=vs.DEFAULT_Y_MIN, unit='LENGTH')
    bpy.types.Scene.scan_y_max = FloatProperty(name="Y max (mm)", default=vs.DEFAULT_Y_MAX, unit='LENGTH')
    bpy.types.Scene.scan_y_steps = IntProperty(name="Y steps", default=vs.DEFAULT_Y_STEPS, min=1, soft_max=50)
    bpy.types.Scene.scan_z_min = FloatProperty(name="Z min (mm)", default=vs.DEFAULT_Z_MIN, unit='LENGTH')
    bpy.types.Scene.scan_z_max = FloatProperty(name="Z max (mm)", default=vs.DEFAULT_Z_MAX, unit='LENGTH')
    bpy.types.Scene.scan_z_steps = IntProperty(name="Z steps", default=vs.DEFAULT_Z_STEPS, min=1, soft_max=50)
    bpy.types.Scene.scan_image_width = IntProperty(name="Width", default=_cam.get("width", vs.DEFAULT_IMAGE_WIDTH), min=16)
    bpy.types.Scene.scan_image_height = IntProperty(name="Height", default=_cam.get("height", vs.DEFAULT_IMAGE_HEIGHT), min=16)
    bpy.types.Scene.scan_fx = FloatProperty(name="fx", default=float(_cam.get("fx", 1467.554)), precision=3)
    bpy.types.Scene.scan_fy = FloatProperty(name="fy", default=float(_cam.get("fy", 1476.769)), precision=3)
    bpy.types.Scene.scan_cx = FloatProperty(name="cx", default=float(_cam.get("cx", 961.213)), precision=3)
    bpy.types.Scene.scan_cy = FloatProperty(name="cy", default=float(_cam.get("cy", 538.565)), precision=3)
    _off = _cam.get("offset", [0.0, 0.0, -20.0])
    bpy.types.Scene.scan_camera_offset = FloatVectorProperty(
        name="Camera offset (mm)", size=3,
        default=(float(_off[0]), float(_off[1]), float(_off[2])),
        description="Offset in D2HW_C201H001 local frame, in millimetres",
    )
    bpy.types.Scene.scan_last_output = StringProperty(name="Last scan output", default="")

    bpy.types.Scene.snap_use_pos = BoolProperty(name="Move to position before snapshot", default=False)
    bpy.types.Scene.snap_x = FloatProperty(name="x (mm)", default=180.0, min=0.0, soft_max=300.0)
    bpy.types.Scene.snap_y = FloatProperty(name="y (mm)", default=140.0, min=0.0, soft_max=300.0)
    bpy.types.Scene.snap_z = FloatProperty(name="z (mm)", default=300.0, min=0.0, soft_max=400.0)


def unregister():
    for attr in ("collision_list", "collision_list_index", "show_rays", "show_hulls",
                 "jubilee_gcode_file",
                 "scan_x_min", "scan_x_max", "scan_x_steps",
                 "scan_y_min", "scan_y_max", "scan_y_steps",
                 "scan_z_min", "scan_z_max", "scan_z_steps",
                 "scan_image_width", "scan_image_height",
                 "scan_fx", "scan_fy", "scan_cx", "scan_cy",
                 "scan_camera_offset", "scan_last_output",
                 "snap_use_pos", "snap_x", "snap_y", "snap_z"):
        if hasattr(bpy.types.Scene, attr):
            delattr(bpy.types.Scene, attr)
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass
