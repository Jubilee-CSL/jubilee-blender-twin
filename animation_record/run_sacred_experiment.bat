@echo off
REM Run sacred_runner.py with Python
REM Adjust the path to Python if needed
set "PYTHON_EXE=python"
set "SACRED_SCRIPT=sacred_runner.py"

"%PYTHON_EXE%" "%SACRED_SCRIPT%" %*

echo Sacred experiment complete.
pause
