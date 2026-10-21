@echo off
setlocal
if exist "%~dp0splat.local.cmd" call "%~dp0splat.local.cmd"
if not defined ISAAC_SIM_ROOT (echo Set ISAAC_SIM_ROOT in splat.local.cmd or the environment. & exit /b 1)
if not exist "%ISAAC_SIM_ROOT%\python.bat" (echo Isaac Sim python.bat not found under "%ISAAC_SIM_ROOT%". & exit /b 1)
"%ISAAC_SIM_ROOT%\python.bat" "%~dp0scripts\launch_splat_simulation.py" %*
exit /b %errorlevel%
