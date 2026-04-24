#!/bin/bash
# Workflow script: Copy latest G-code from science-jubilee to jubilee-blender-twin and run animation

# Set paths
SRC_GCODE="../../science_jubilee/gcode_logs/test_corners_path.gcode"
DST_GCODE="path.gcode"
ANIM_SCRIPT="path_follower.py"

# Copy latest G-code to animation folder
cp "$SRC_GCODE" "$DST_GCODE"

# Run the animation preparation script (uses local utils.py)
python "$ANIM_SCRIPT" "$DST_GCODE"

# Run Blender to animate the toolpath
BLEND_FILE="../jubilee.blend"
BLENDER_SCRIPT="animate_path.py"
# Adjust the path to Blender as needed
BLENDER_EXE="/Applications/Blender.app/Contents/MacOS/Blender"

"$BLENDER_EXE" "$BLEND_FILE" --python "$BLENDER_SCRIPT"

echo "G-code copied, animation path generated, and Blender animation complete."
