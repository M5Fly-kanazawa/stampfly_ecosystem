@echo off
REM StampFly Ecosystem Installer (Windows)
REM Usage: scripts\install.bat [options]

setlocal

set SCRIPT_DIR=%~dp0
set ROOT_DIR=%SCRIPT_DIR%..

REM Run Python installer
python "%SCRIPT_DIR%installer.py" %*

endlocal
