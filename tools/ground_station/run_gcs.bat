@echo off
setlocal
cd /d "%~dp0"

set "APP_DIR=%~dp0Serial-Studio\build\app"
set "EXE_PATH=%APP_DIR%\Serial-Studio-GPL3.exe"
set "DEFAULT_PROJ=%~dp0Drone-H743-GCS.ssproj"
set "PATH=%APP_DIR%;D:\Qt\6.7.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;%PATH%"

if not exist "%EXE_PATH%" (
    echo [ERROR] Serial-Studio-GPL3.exe not found at:
    echo "%EXE_PATH%"
    pause
    exit /b 1
)

echo [INFO] Starting Drone-H743 Dedicated Ground Station (Serial Studio)...
if "%~1"=="" (
    start "" /d "%APP_DIR%" "%EXE_PATH%" "%DEFAULT_PROJ%"
) else (
    start "" /d "%APP_DIR%" "%EXE_PATH%" %*
)
exit /b 0
