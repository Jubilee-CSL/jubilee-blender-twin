@echo off
REM Workflow script: Copy latest G-code from science-jubilee to jubilee-blender-twin and run animation
cd /d "%~dp0"

set "SCIENCE_JUB=%~1"
set "JUB_TWIN=%~2"
set "BLENDER_EXE=%~3"
set "GCODE_FILE=%~4"

REM Arg check
if "%SCIENCE_JUB%"=="" (
    echo ERROR: Missing argument 1 - science_jubilee directory
    echo Usage: %~nx0 ^<science_jub_dir^> ^<jub_twin_dir^> ^<blender_exe^> ^<gcode_file^>
    exit /b 1
)
if "%JUB_TWIN%"=="" (
    echo ERROR: Missing argument 2 - jubilee-blender-twin directory
    exit /b 1
)
if "%BLENDER_EXE%"=="" (
    echo ERROR: Missing argument 3 - blender executable path
    exit /b 1
)
if "%GCODE_FILE%"=="" (
    echo ERROR: Missing argument 4 - gcode filename
    exit /b 1
)

REM Set paths
set "SRC_GCODE=%SCIENCE_JUB%\gcode_logs\%GCODE_FILE%"
set "SRC_CONFIG=%SCIENCE_JUB%\firmware\sys"
set "DST_CONFIG=%JUB_TWIN%\config"
set "DST_GCODE=path.gcode"
set "BLEND_FILE=%JUB_TWIN%\jubilee_belt.blend"

set "ANIM_SCRIPT=path_follower.py"
set "TOOL_SCRIPT=tool_id.py"
set "TOOL_SETUP_SCRIPT=tool_placement.py"
set "BLENDER_SCRIPT=animate_path.py"

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

REM Place tools on blend file
"%BLENDER_EXE%" --background "%BLEND_FILE%" --python "%TOOL_SETUP_SCRIPT%"

REM Run Blender to animate the toolpath
"%BLENDER_EXE%" "%BLEND_FILE%" --python "%BLENDER_SCRIPT%"

echo G-code copied, config files synced, animation complete.