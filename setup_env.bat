@echo off
REM StampFly Ecosystem - Development Environment Setup (Windows)
REM Usage: setup_env.bat
REM
REM ASCII-ONLY comments in this file, and CRLF line endings (enforced via
REM .gitattributes). Two cmd.exe constraints, both learned the hard way:
REM  (1) cmd misparses LF-only .bat files (drops the first char of each
REM      line), and
REM  (2) under a cp932 console (Japanese Windows) it reads UTF-8 Japanese
REM      bytes as raw bytes -- some decode to command separators (& | < >),
REM      so a REM line with Japanese text can execute part of itself.
REM Same ASCII-only rule the generated uninstall.cmd follows (spec 4-1).

setlocal enabledelayedexpansion

echo.
echo [INFO] Setting up StampFly development environment...
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

REM 2. python.org / other installers (only if pyenv-win not found)
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

python.exe --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found. Install Python 3.8+ first.
    echo   https://www.python.org/downloads/
    exit /b 1
)

REM --- Determine IDF_PATH: .sf/config.toml > default ---
set "SF_IDF_PATH="
set "SF_CONFIG=%~dp0.sf\config.toml"
if exist "%SF_CONFIG%" (
    for /f "tokens=1,* delims==" %%a in ('findstr /b "path" "%SF_CONFIG%"') do (
        set "SF_RAW=%%b"
    )
    if defined SF_RAW (
        set "SF_RAW=!SF_RAW: =!"
        set "SF_RAW=!SF_RAW:"=!"
        set "SF_IDF_PATH=!SF_RAW!"
    )
)

if not defined SF_IDF_PATH set "SF_IDF_PATH=%USERPROFILE%\esp\esp-idf"

set "DISCOVERED_PATH=!PATH!"
endlocal & set "PATH=%DISCOVERED_PATH%" & set "IDF_PATH=%SF_IDF_PATH%"

if not exist "%IDF_PATH%\export.bat" (
    echo [ERROR] ESP-IDF not found at %IDF_PATH%
    echo   Run install.bat first.
    exit /b 1
)

echo [INFO] Loading ESP-IDF environment...
call "%IDF_PATH%\export.bat"

echo.
echo [OK] StampFly development environment ready.
echo.
echo   sf doctor          Check environment
echo   sf build vehicle   Build vehicle firmware
echo   sf --help          Show all commands
echo.
