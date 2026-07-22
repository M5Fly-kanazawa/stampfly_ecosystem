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
REM Supported range is Python 3.10-3.12 only (see PYTHON_PREFERRED_MIN/MAX
REM in scripts/installer.py). ESP-IDF's export.bat blindly uses whichever
REM python.exe is first on PATH to look up its matching venv
REM (idf5.x_pyX.Y_env). A python outside 3.10-3.12 has no matching venv
REM ("venv not found") or resolves to a stale one missing packages
REM ("No module named 'click'"). So every candidate below is version
REM checked via :sf_check_python before being accepted -- never trust a
REM discovered path without checking its actual version first.
set "PYTHON_DIR="

REM 1. pyenv-win: read configured version from version file, then verify
REM    it is actually within the supported range before trusting it.
set "PYENV_ROOT=%USERPROFILE%\.pyenv\pyenv-win"
if exist "%PYENV_ROOT%\version" (
    set /p PYENV_VER=<"%PYENV_ROOT%\version"
    if exist "%PYENV_ROOT%\versions\!PYENV_VER!\python.exe" (
        call :sf_check_python "%PYENV_ROOT%\versions\!PYENV_VER!\python.exe"
        if not errorlevel 1 (
            set "PYTHON_DIR=%PYENV_ROOT%\versions\!PYENV_VER!"
        )
    )
)
REM If the pyenv-win version file pointed at an out-of-range interpreter
REM (or none at all), PYTHON_DIR is still undefined here and step 2 below
REM falls through to the next search location.

REM 2. python.org / other installers (only if pyenv-win not found/valid)
REM Program Files / Program Files (x86) are python.org's default
REM "Install for all users" targets, distinct from the per-user
REM LOCALAPPDATA\Programs\Python default.
REM Search order is newest-supported-first: 3.12, then 3.11, then 3.10.
REM Python 3.13+ is intentionally NOT in this list -- it is unsupported
REM (see the header note above); listing it first was the original bug.
if not defined PYTHON_DIR (
    for %%d in (
        "%LOCALAPPDATA%\Programs\Python\Python312"
        "%LOCALAPPDATA%\Programs\Python\Python311"
        "%LOCALAPPDATA%\Programs\Python\Python310"
        "C:\Python312"
        "C:\Python311"
        "C:\Python310"
        "C:\Program Files\Python312"
        "C:\Program Files\Python311"
        "C:\Program Files\Python310"
        "C:\Program Files (x86)\Python312"
        "C:\Program Files (x86)\Python311"
        "C:\Program Files (x86)\Python310"
        "%USERPROFILE%\scoop\apps\python\current"
        "%USERPROFILE%\anaconda3"
        "%USERPROFILE%\miniconda3"
    ) do (
        if not defined PYTHON_DIR (
            if exist "%%~d\python.exe" (
                call :sf_check_python "%%~d\python.exe"
                if not errorlevel 1 set "PYTHON_DIR=%%~d"
            )
        )
    )
)

if defined PYTHON_DIR (
    set "PATH=!PYTHON_DIR!;!PATH!"
)

REM Final check: whatever python.exe is now first on PATH (either the
REM candidate found above, or whatever the user already had) must still be
REM verified -- a bare "python.exe --version" success does not mean the
REM version is in range, and an unverified out-of-range python left on
REM PATH is exactly what breaks export.bat's venv lookup later.
call :sf_check_python "python.exe"
if errorlevel 1 (
    echo [ERROR] Python 3.10-3.12 required, but no matching python.exe was found.
    echo   Install one with: winget install --id Python.Python.3.12
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
if errorlevel 1 (
    echo [ERROR] ESP-IDF export.bat failed. Environment not fully loaded.
    exit /b 1
)

echo.
echo [OK] StampFly development environment ready.
echo.
echo   sf doctor          Check environment
echo   sf build vehicle   Build vehicle firmware
echo   sf --help          Show all commands
echo.

exit /b 0

REM --- Subroutines ---

:sf_check_python
REM Verify that %1 (a path to a python.exe candidate, may be unquoted) is
REM usable and within the supported range 3.10-3.12. Returns via errorlevel:
REM   0 = in range
REM   1 = missing, unusable, or out of range
REM Silent by design (called many times while probing candidates) -- callers
REM decide what to print once the final outcome is known.
"%~1" -c "import sys; sys.exit(0 if (3,10) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>&1
exit /b %errorlevel%
