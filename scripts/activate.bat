@echo off
REM StampFly Ecosystem Environment Activation (Windows)
REM Usage: scripts\activate.bat

setlocal EnableDelayedExpansion

set SCRIPT_DIR=%~dp0
set STAMPFLY_ROOT=%SCRIPT_DIR%..
set VENV_DIR=%STAMPFLY_ROOT%\.venv

REM Check if virtual environment exists
if not exist "%VENV_DIR%" (
    echo [ERROR] Virtual environment not found: %VENV_DIR%
    echo         Run scripts\install.bat first
    exit /b 1
)

REM Activate virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

REM Set environment variables (endlocal will lose these, so use different approach)
endlocal & (
    set "STAMPFLY_ROOT=%STAMPFLY_ROOT%"
    set "PYTHONPATH=%STAMPFLY_ROOT%\lib;%PYTHONPATH%"
    set "PATH=%STAMPFLY_ROOT%\bin;%PATH%"
)

REM Source ESP-IDF if available
if exist "%STAMPFLY_ROOT%\.esp-idf\export.bat" (
    call "%STAMPFLY_ROOT%\.esp-idf\export.bat" > nul 2>&1
) else if exist "%USERPROFILE%\esp\esp-idf\export.bat" (
    call "%USERPROFILE%\esp\esp-idf\export.bat" > nul 2>&1
)

echo StampFly Ecosystem activated
echo   Root: %STAMPFLY_ROOT%
echo.
echo Run 'sf --help' for available commands
