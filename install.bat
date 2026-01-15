@echo off
REM StampFly Ecosystem Installer (Windows)
REM Usage: install.bat [options]

setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"

echo.
echo ============================================================
echo  StampFly Ecosystem Installer
echo ============================================================
echo.

REM Check for Python
echo [INFO] Checking Python...

set "PYTHON_CMD="
set "PYTHON_VERSION="

REM Try different Python commands
for %%p in (python3 python py) do (
    if not defined PYTHON_CMD (
        where %%p >nul 2>&1
        if !errorlevel! equ 0 (
            for /f "tokens=*" %%v in ('%%p -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2^>nul') do (
                set "ver=%%v"
            )
            for /f "tokens=1,2 delims=." %%a in ("!ver!") do (
                if %%a geq 3 if %%b geq 10 (
                    set "PYTHON_CMD=%%p"
                    set "PYTHON_VERSION=!ver!"
                )
            )
        )
    )
)

if not defined PYTHON_CMD (
    echo [ERROR] Python 3.10+ is required but not found.
    echo.
    echo   Install Python from:
    echo     https://www.python.org/downloads/
    echo.
    echo   Or using winget:
    echo     winget install Python.Python.3.12
    echo.
    exit /b 1
)

echo [OK] Found Python %PYTHON_VERSION% (%PYTHON_CMD%)
echo.

REM Run Python installer
%PYTHON_CMD% "%SCRIPT_DIR%scripts\installer.py" %*
