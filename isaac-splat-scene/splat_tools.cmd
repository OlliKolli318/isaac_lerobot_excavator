@echo off
setlocal
if exist "%~dp0splat.local.cmd" call "%~dp0splat.local.cmd"
if not defined PIXI_MANIFEST (echo Set PIXI_MANIFEST in splat.local.cmd or the environment. & exit /b 1)
if not exist "%PIXI_MANIFEST%" (echo Pixi manifest not found: "%PIXI_MANIFEST%". & exit /b 1)
if not defined ROS_LOCAL_SETUP goto run_tools
if not exist "%ROS_LOCAL_SETUP%" (echo Configured ROS_LOCAL_SETUP not found: "%ROS_LOCAL_SETUP%". Correct the path or unset it to use pip-installed packages. & exit /b 1)
call "%ROS_LOCAL_SETUP%"
if errorlevel 1 exit /b %errorlevel%
:run_tools
pixi run --frozen --manifest-path "%PIXI_MANIFEST%" python -X utf8 -B "%~dp0scripts\splat_tools.py" %*
exit /b %errorlevel%
