@echo off
REM StampFly CLI Wrapper (Windows)
REM This script activates the virtual environment and runs the CLI

setlocal

set SCRIPT_DIR=%~dp0
set ROOT_DIR=%SCRIPT_DIR%..
set VENV_DIR=%ROOT_DIR%\.venv

REM Check if virtual environment exists
if not exist "%VENV_DIR%" (
    echo [ERROR] Virtual environment not found: %VENV_DIR%
    echo         Run scripts\install.bat first
    exit /b 1
)

REM Activate virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

REM Set environment
set PYTHONPATH=%ROOT_DIR%\lib;%PYTHONPATH%
set STAMPFLY_ROOT=%ROOT_DIR%

REM Source ESP-IDF if available
if exist "%ROOT_DIR%\.esp-idf\export.bat" (
    call "%ROOT_DIR%\.esp-idf\export.bat" > nul 2>&1
) else if exist "%USERPROFILE%\esp\esp-idf\export.bat" (
    call "%USERPROFILE%\esp\esp-idf\export.bat" > nul 2>&1
)

REM Run CLI
python -m sfcli %*

endlocal
