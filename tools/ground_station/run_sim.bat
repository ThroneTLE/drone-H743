@echo off
setlocal
cd /d "%~dp0"

echo [INFO] Starting Drone Telemetry Simulator (50Hz TCP)...
start "Drone Telemetry Simulator" python "%~dp0drone_simulator.py"

ping -n 2 127.0.0.1 >nul

echo [INFO] Starting Ground Control Station...
call "%~dp0run_gcs.bat"
