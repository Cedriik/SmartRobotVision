@echo off
setlocal

set "PSCP=pscp"
where pscp >nul 2>nul
if errorlevel 1 (
    if exist "C:\Program Files\PuTTY\pscp.exe" (
        set "PSCP=C:\Program Files\PuTTY\pscp.exe"
    ) else (
        echo pscp.exe was not found.
        echo Install PuTTY or add pscp.exe to PATH first.
        pause
        exit /b 1
    )
)

set "SRC=C:\Users\Cedrick\Downloads\SmartRobotVision\robot.py"
set "HOST=admin@10.42.0.1"
set "HOSTKEY=SHA256:wWqBppVJYj3UsSAd/Pdgx2agSaXyvVncgeudLR89AY0"
set "PASSWORD=admin"

echo Uploading robot.py to /home/admin/robot.py ...
"%PSCP%" -pw "%PASSWORD%" -hostkey "%HOSTKEY%" "%SRC%" %HOST%:/home/admin/robot.py
if errorlevel 1 (
    echo Upload to /home/admin/robot.py failed.
    pause
    exit /b 1
)

echo Uploading robot.py to /home/admin/Robot/robot.py ...
"%PSCP%" -pw "%PASSWORD%" -hostkey "%HOSTKEY%" "%SRC%" %HOST%:/home/admin/Robot/robot.py
if errorlevel 1 (
    echo Upload to /home/admin/Robot/robot.py failed.
    pause
    exit /b 1
)

echo.
echo Upload complete.
pause
