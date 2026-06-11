
"""
import bpy
import numpy as np

scene = bpy.context.scene
f_start = frame_start if frame_start is not None else scene.frame_start
f_end   = frame_end   if frame_end   is not None else scene.frame_end


frame_curr = scene.frame_current
scene.frame_set(f_start)
depsgraph = bpy.context.evaluated_depsgraph_get()

frame_num = f_start
while frame_num <= f_end:
    for obj in bpy.data.meshes:
        
        
        

scene.frame_set(frame_curr)
"""

"""
animate_path.py
Blender Python script that reads pathout.csv and inserts one keyframe per row
on the X-axis, Y-axis, and Z-axis scene objects.

Run via the Blender CLI:
    blender jubilee.blend --python animate_path.py

The script expects three objects named 'X-axis', 'Y-axis', and 'Z-axis' in the
scene.  Z-axis is optional; X-axis and Y-axis are required.  Each object must
have a LIMIT_LOCATION constraint whose bounds encode the physical travel range
of that axis in Blender world units (metres).
"""

import bpy
import csv
import sys
import os
import numpy as np
import blender_plots as bplt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import get_axis_min, get_axis_max

# Path to the CSV file (relative to the .blend file)
csv_path = bpy.path.abspath("//from_gcode/pathout.csv")
scene = bpy.context.scene
depsgraph = bpy.context.evaluated_depsgraph_get()

# Animate 'X-axis' in X, 'Y-axis' in Y, and 'Z-axis' in Z if present
x_axis = bpy.data.objects.get("X-axis")
y_axis = bpy.data.objects.get("Y-axis")
z_axis = bpy.data.objects.get("Z-axis")
if x_axis is None:
    raise Exception("No object named 'X-axis' in the scene!")
if y_axis is None:
    raise Exception("No object named 'Y-axis' in the scene!")

# Remove any existing keyframe data so the animation is rebuilt from scratch.
for obj in [x_axis, y_axis, z_axis]:
    if obj is not None:
        obj.animation_data_clear()

# Read interpolated positions produced by path_follower.py.
points = []
with open(csv_path, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        points.append(tuple(map(float, row)))
        
points = np.array(points)
print(points)

# Read the physical home position of each axis from its LIMIT_LOCATION constraint.
# These offsets anchor gcode coordinate (0, 0, 0) to the correct location in
# the Blender scene.
x_min = get_axis_min(x_axis, 'X')
y_min = get_axis_min(y_axis, 'Y')
# Z home is the maximum scene Z because the bed descends as the gcode Z value rises.
z_max = get_axis_max(z_axis, 'Z') if z_axis is not None else 0.0

# Snap axes to their home positions before inserting keyframes.
x_axis.location.x = x_min
y_axis.location.y = y_min
if z_axis is not None:
    z_axis.location.z = z_max

print(f"Axis minimums: X={x_min}, Y={y_min}, Z={z_max}")

directions = np.zeros_like(points)
for i in range(1,points.shape[0]):
    print(f"index {i}")
    print(f"test: next{points[i]} previous {points[i-1]}")
    print(points[i]-points[i-1])
    directions[i]=(points[i]-points[i-1])


# Insert one keyframe per CSV row.
# CSV values are in mm; divide by 1000 to convert to Blender's metre units.
# Z is subtracted from z_max to invert the axis direction.
hit=False
for frame, (x, y, z) in enumerate(points, start=1):
    x_axis.location.x = x/1000 + x_min
    x_axis.keyframe_insert(data_path="location", frame=frame)

    y_axis.location.y = y/1000 + y_min
    y_axis.keyframe_insert(data_path="location", frame=frame)

    if z_axis is not None:
        z_axis.location.z = z_max - z/1000
        z_axis.keyframe_insert(data_path="location", frame=frame)

    if frame != len(points):
        bplt.Arrows(np.array([x,y,z]).T,np.array(directions[frame]).T,radius=.3,radius_ratio=3)
        hit,loc,normal,index,object,mat=scene.ray_cast(depsgraph,(x,y,z),directions[frame])
        if hit:
            print(f"collision between gantry and {object.name} during frame {frame}")

    print(frame)

# Extend the timeline to exactly cover the animation.
scene = bpy.context.scene
scene.frame_end = len(points)