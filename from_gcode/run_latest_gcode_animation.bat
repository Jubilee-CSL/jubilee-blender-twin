@echo off
REM Workflow script: Copy latest G-code from science-jubilee to jubilee-blender-twin and run animation
cd /d "%~dp0"

REM Set paths
set "SRC_GCODE=..\..\science_jubilee\gcode_logs\test_tool_change_from_no_active_tool_expands_tpre_and_tpost.gcode"
set "SRC_CONFIG=..\..\science_jubilee\firmware\sys"
set "DST_CONFIG=..\config"
set "DST_GCODE=path.gcode"
set "ANIM_SCRIPT=path_follower.py"
set "TOOL_SCRIPT=tool_id.py"
set "TOOL_SETUP_SCRIPT=tool_placement.py"

REM Create config folder if it doesn't exist
if not exist "%DST_CONFIG%" mkdir "%DST_CONFIG%"

REM Copy config.g and toffsets.g
copy "%SRC_CONFIG%\config.g"   "%DST_CONFIG%\config.g"   /Y
copy "%SRC_CONFIG%\toffsets.g" "%DST_CONFIG%\toffsets.g" /Y

REM Copy all tpost*.g files (handles tpost0.g, tpost1.g, tpost2.g, etc.)
for %%F in ("%SRC_CONFIG%\tpost*.g") do (
    copy "%%F" "%DST_CONFIG%\%%~nxF" /Y
)

REM Copy latest G-code to animation folder
copy "%SRC_GCODE%" "%DST_GCODE%" /Y

REM Fetch tool parking and offset info
python "%TOOL_SCRIPT%"

REM Run the animation preparation script (uses local utils.py)
python "%ANIM_SCRIPT%" "%DST_GCODE%"

set "BLEND_FILE=..\jubilee_belt.blend"
set "BLENDER_SCRIPT=animate_path.py"
set "BLENDER_EXE=C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"

REM Place tools on blend file
"%BLENDER_EXE%" "%BLEND_FILE%" --python "%TOOL_SETUP_SCRIPT%"


REM Run Blender to animate the toolpath
"%BLENDER_EXE%" "%BLEND_FILE%" --python "%BLENDER_SCRIPT%"

echo G-code copied, config files synced, animation complete.