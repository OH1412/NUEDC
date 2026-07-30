@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_receiver.ps1"
if errorlevel 1 (
    echo.
    echo Receiver startup failed. See the message above.
    pause
)
endlocal

