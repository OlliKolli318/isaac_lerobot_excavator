@echo off
setlocal
if exist "%~dp0splat.local.cmd" call "%~dp0splat.local.cmd"
if not defined ROS_LOCAL_SETUP (echo Set ROS_LOCAL_SETUP in splat.local.cmd or the environment. & exit /b 1)
if not defined PIXI_MANIFEST (echo Set PIXI_MANIFEST in splat.local.cmd or the environment. & exit /b 1)
if not exist "%ROS_LOCAL_SETUP%" (echo ROS 2 local_setup.bat not found: "%ROS_LOCAL_SETUP%". & exit /b 1)
if not exist "%PIXI_MANIFEST%" (echo Pixi manifest not found: "%PIXI_MANIFEST%". & exit /b 1)
call "%ROS_LOCAL_SETUP%"
if errorlevel 1 exit /b %errorlevel%
pixi run --frozen --manifest-path "%PIXI_MANIFEST%" python -X utf8 -B "%~dp0scripts\splat_tools.py" %*
exit /b %errorlevel%
