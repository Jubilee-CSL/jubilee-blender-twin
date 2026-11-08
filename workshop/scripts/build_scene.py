"""
Runs inside Blender: put a plant on the bed and give the camera a real lens.

    blender jubilee_working.blend --background --python build_scene.py -- <plant.blend> <out.blend>

The plant is every object parented to the bed plate in <plant.blend>. It is
re-attached to this scene's bed plate, so it moves with Z like a real sample.
"""
import sys

import bpy

plant_blend, out = sys.argv[sys.argv.index("--") + 1:]
plate = bpy.data.objects["z_plate_3_V5"]            # the bed plate, moved by Z

existing = set(bpy.data.objects.keys())
with bpy.data.libraries.load(plant_blend) as (src, dst):
    dst.objects = [name for name in src.objects if name not in existing]


def on_bed(obj):
    parent = obj.parent
    while parent:
        if parent.name.startswith("z_plate_3_V5"):
            return True
        parent = parent.parent
    return False


plant = [o for o in dst.objects if o and on_bed(o) and not o.name.startswith("z_plate")]
for obj in plant:
    if obj.parent.name.startswith("z_plate"):       # re-attach to this scene's bed, same place
        inverse = obj.matrix_parent_inverse.copy()
        obj.parent = plate
        obj.matrix_parent_inverse = inverse
    bpy.context.scene.collection.objects.link(obj)
bpy.context.scene["plant"] = [o.name for o in plant]

lens = bpy.data.objects["Toolhead_Cam"].data        # fixed focus: moving Z changes sharpness
lens.dof.use_dof = True
lens.dof.aperture_fstop = 4.0
lens.dof.focus_distance = 0.14                      # metres

bpy.context.scene.render.resolution_percentage = 50
bpy.data.orphans_purge(do_recursive=True)
bpy.ops.wm.save_as_mainfile(filepath=out)
print("plant objects:", [o.name for o in plant])
