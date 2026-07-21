@echo off
REM StampFly Ecosystem Installer (Windows)
REM Usage: install.bat [options]
REM
REM ASCII-ONLY comments in this file, and CRLF line endings (enforced via
REM .gitattributes). cmd.exe misparses LF-only .bat files, and under a
REM cp932 console it reads UTF-8 Japanese bytes as command separators
REM (& | < >), so a REM line with Japanese can execute part of itself.
REM Same ASCII-only rule the generated uninstall.cmd follows (spec 4-1).
REM
REM Options (forwarded to scripts\installer.py; see that file's docstring
REM for the full list):
REM   --help           Show installer.py's full option list and exit
REM   --force          Force reinstall all steps (skip probe checks)
REM   --uninstall      Remove sfcli from the ESP-IDF environment
REM   --clean          Clean install (remove config and sfcli, then reinstall)
REM   --no-flasher     Skip the optional GUI Flasher app install
REM   --minimal        Install minimal dependencies (skip simulator)
REM
REM Unlike install.sh, this script does not skip its git/Python checks for
REM --uninstall/--clean: both are required just to launch installer.py.

setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"

echo.
echo ============================================================
echo  StampFly Ecosystem Installer
echo ============================================================
echo.

REM Check for git
echo [INFO] Checking git...
where git >nul 2>&1
if errorlevel 1 (
    echo [ERROR] git is not installed.
    echo.
    echo   Install Git from:
    echo     https://git-scm.com/download/win
    echo.
    echo   Or using winget:
    echo     winget install Git.Git
    echo.
    exit /b 1
)
echo [OK] git found
echo.

REM --- Discover python.exe ---
set "PYTHON_DIR="

REM 1. pyenv-win: read configured version from version file
set "PYENV_ROOT=%USERPROFILE%\.pyenv\pyenv-win"
if exist "%PYENV_ROOT%\version" (
    set /p PYENV_VER=<"%PYENV_ROOT%\version"
    if exist "%PYENV_ROOT%\versions\!PYENV_VER!\python.exe" (
        set "PYTHON_DIR=%PYENV_ROOT%\versions\!PYENV_VER!"
    )
)

REM 2. Common install locations (only if pyenv-win not found)
REM Program Files / Program Files (x86) are python.org's default
REM "Install for all users" targets, distinct from the per-user
REM LOCALAPPDATA\Programs\Python default.
if not defined PYTHON_DIR (
    for %%d in (
        "%LOCALAPPDATA%\Programs\Python\Python313"
        "%LOCALAPPDATA%\Programs\Python\Python312"
        "%LOCALAPPDATA%\Programs\Python\Python311"
        "%LOCALAPPDATA%\Programs\Python\Python310"
        "C:\Python313"
        "C:\Python312"
        "C:\Python311"
        "C:\Python310"
        "C:\Program Files\Python313"
        "C:\Program Files\Python312"
        "C:\Program Files\Python311"
        "C:\Program Files\Python310"
        "C:\Program Files (x86)\Python313"
        "C:\Program Files (x86)\Python312"
        "C:\Program Files (x86)\Python311"
        "C:\Program Files (x86)\Python310"
        "%USERPROFILE%\scoop\apps\python\current"
        "%USERPROFILE%\anaconda3"
        "%USERPROFILE%\miniconda3"
    ) do (
        if not defined PYTHON_DIR (
            if exist "%%~d\python.exe" set "PYTHON_DIR=%%~d"
        )
    )
)

if defined PYTHON_DIR (
    set "PATH=!PYTHON_DIR!;!PATH!"
)

REM Check for Python
echo [INFO] Checking Python...

set "PYTHON_CMD="
set "PYTHON_VERSION="

for %%p in (python3 python py) do (
    if not defined PYTHON_CMD (
        for /f "tokens=*" %%v in ('%%p -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2^>nul') do (
            set "ver=%%v"
        )
        if defined ver (
            for /f "tokens=1,2 delims=." %%a in ("!ver!") do (
                if %%a geq 3 if %%b geq 8 (
                    set "PYTHON_CMD=%%p"
                    set "PYTHON_VERSION=!ver!"
                )
            )
        )
        set "ver="
    )
)

if not defined PYTHON_CMD (
    echo [ERROR] Python 3.8+ is required but not found.
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
%PYTHON_CMD% -u "%SCRIPT_DIR%scripts\installer.py" %*
