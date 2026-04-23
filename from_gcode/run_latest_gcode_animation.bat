@echo off
REM Workflow script: Copy latest G-code from science-jubilee to jubilee-blender-twin and run animation

REM Set paths
set "SRC_GCODE=./path.gcode"


set "DST_GCODE=path.gcode"
set "ANIM_SCRIPT=path_follower.py"


REM Copy latest G-code to animation folder
copy "%SRC_GCODE%" "%DST_GCODE%" /Y

REM Run the animation preparation script (uses local utils.py)
python "%ANIM_SCRIPT%" "%DST_GCODE%"

REM Run Blender to animate the toolpath
set "BLEND_FILE=..\jubilee.blend"
set "BLENDER_SCRIPT=animate_path.py"
REM Adjust the path to blender.exe as needed
set "BLENDER_EXE=C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"

"%BLENDER_EXE%" "%BLEND_FILE%" --python "%BLENDER_SCRIPT%"

echo G-code copied, animation path generated, and Blender animation complete.
pause
