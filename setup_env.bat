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
REM
REM Dedicated vs legacy (docs\plans\dedicated-environment-plan.md section 2):
REM if .sf\config.toml has [env] kind = "dedicated", this script trusts the
REM paths written there (python_dir/tools_path/path) outright and skips ALL
REM of the probing below (IDF_TOOLS_PATH guessing, pyenv-win/python.org
REM discovery, version matching). A v1 config (no [env] section) or
REM kind = "legacy" runs the exact same probing this script always has.

setlocal enabledelayedexpansion

echo.
echo [INFO] Setting up StampFly development environment...
echo.

REM --- Read .sf\config.toml up front (kind / python_dir / tools_path /
REM     path) so dedicated mode can decide to skip all legacy probing below
REM     before any of it runs. /b anchors each findstr match to the start
REM     of the line, so "python_dir"/"tools_path" can never falsely match
REM     the shorter "python"/"path" searches (a line starting with
REM     "tools_path" does not start with "path", and vice versa).
REM     delims== (equals and space) makes token 2 the quoted value with
REM     its internal spaces intact -- stripping every space (the old
REM     approach) broke paths such as a LOCALAPPDATA fallback under a
REM     user name containing a space. Only the quotes are removed. ---
set "SF_CONFIG=%~dp0.sf\config.toml"
set "SF_KIND="
set "SF_PY_DIR_CFG="
set "SF_TOOLS_PATH_CFG="
set "SF_IDF_PATH="
if exist "%SF_CONFIG%" (
    set "SF_RAW="
    for /f "tokens=1,* delims== " %%a in ('findstr /b "kind" "%SF_CONFIG%"') do set "SF_RAW=%%b"
    if defined SF_RAW (
        set "SF_RAW=!SF_RAW:"=!"
        set "SF_KIND=!SF_RAW!"
    )

    set "SF_RAW="
    for /f "tokens=1,* delims== " %%a in ('findstr /b "python_dir" "%SF_CONFIG%"') do set "SF_RAW=%%b"
    if defined SF_RAW (
        set "SF_RAW=!SF_RAW:"=!"
        set "SF_PY_DIR_CFG=!SF_RAW!"
    )

    set "SF_RAW="
    for /f "tokens=1,* delims== " %%a in ('findstr /b "tools_path" "%SF_CONFIG%"') do set "SF_RAW=%%b"
    if defined SF_RAW (
        set "SF_RAW=!SF_RAW:"=!"
        set "SF_TOOLS_PATH_CFG=!SF_RAW!"
    )

    set "SF_RAW="
    for /f "tokens=1,* delims== " %%a in ('findstr /b "path" "%SF_CONFIG%"') do set "SF_RAW=%%b"
    if defined SF_RAW (
        set "SF_RAW=!SF_RAW:"=!"
        set "SF_IDF_PATH=!SF_RAW!"
    )
    set "SF_RAW="
)
if not defined SF_IDF_PATH set "SF_IDF_PATH=%USERPROFILE%\esp\esp-idf"

if "%SF_KIND%"=="dedicated" goto :sf_dedicated_env

REM ==========================================================================
REM Legacy mode: probe for IDF_TOOLS_PATH and a system Python, exactly as
REM this script did before dedicated mode existed.
REM ==========================================================================

REM --- Determine IDF_TOOLS_PATH: env var > C:\Espressif > %USERPROFILE%\.espressif ---
REM scripts\installer.py's _idf_tools_path_candidates() installs to
REM C:\Espressif when IDF_TOOLS_PATH is unset at install time (its Windows-
REM specific first choice, ahead of the ESP-IDF default of
REM %USERPROFILE%\.espressif). IDF_TOOLS_PATH is a plain process env var --
REM nothing persists it to the registry -- so a fresh shell that never had
REM it set falls through to export.bat's own default (%USERPROFILE%\
REM .espressif) and reports the venv "not found" even though it exists at
REM C:\Espressif (observed 2026-09-07). Probe both locations here and
REM export whichever one actually has a python_env, so export.bat gets the
REM same IDF_TOOLS_PATH the installer used, every time this script runs.
set "SF_TOOLS_PATH=%IDF_TOOLS_PATH%"
if not defined SF_TOOLS_PATH (
    if exist "C:\Espressif\python_env" (
        set "SF_TOOLS_PATH=C:\Espressif"
    ) else (
        set "SF_TOOLS_PATH=%USERPROFILE%\.espressif"
    )
)

REM --- Determine required Python minor from installed ESP-IDF venvs ---
REM export.bat derives the venv name (idf5.x_py3.Y_env) from the version of
REM the python.exe it finds on PATH. A merely in-range python is therefore
REM NOT enough: python 3.12 on PATH with only a py3.10 venv installed makes
REM export.bat fail with "python_env ... not found" -- and export.bat still
REM exits 0, so this script used to print [OK] right after that error
REM (observed 2026-07-22, pyenv-win global 3.12 + installer-created py3.10
REM venv). Collect the minor versions of installed venvs here and let
REM :sf_check_python accept only a matching python.
set "SF_VENV_MINORS="
if exist "%SF_TOOLS_PATH%\python_env" (
    for /d %%v in ("%SF_TOOLS_PATH%\python_env\idf*_env") do (
        for /f "tokens=3 delims=." %%m in ("%%~nxv") do (
            set "SF_M=%%m"
            set "SF_M=!SF_M:_env=!"
            if defined SF_VENV_MINORS (
                set "SF_VENV_MINORS=!SF_VENV_MINORS! !SF_M!"
            ) else (
                set "SF_VENV_MINORS=!SF_M!"
            )
        )
    )
)

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
REM (or none at all), PYTHON_DIR is still undefined here and the scan below
REM falls through to the next search location.

REM 1b. pyenv-win: the configured global version may be in range yet not
REM     match the installed ESP-IDF venv (e.g. global 3.12, venv py3.10),
REM     so also scan every installed version directory.
if not defined PYTHON_DIR if exist "%PYENV_ROOT%\versions" (
    for /f "delims=" %%v in ('dir /b /ad /o-n "%PYENV_ROOT%\versions" 2^>nul') do (
        if not defined PYTHON_DIR (
            if exist "%PYENV_ROOT%\versions\%%v\python.exe" (
                call :sf_check_python "%PYENV_ROOT%\versions\%%v\python.exe"
                if not errorlevel 1 set "PYTHON_DIR=%PYENV_ROOT%\versions\%%v"
            )
        )
    )
)

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
    if defined SF_VENV_MINORS (
        echo   The installed ESP-IDF env additionally requires Python 3.%SF_VENV_MINORS%.
        echo   Run install.bat again, or install that Python version.
    ) else (
        echo   Install one with: winget install Python.Python.3.12
        echo   https://www.python.org/downloads/
    )
    exit /b 1
)

set "DISCOVERED_PATH=!PATH!"
goto :sf_env_handoff

REM ==========================================================================
REM Dedicated mode: everything comes straight from .sf\config.toml -- no
REM probing, no version matching, no venv-minor scan. Just verify the two
REM paths the installer wrote are still there, then use them as-is.
REM ==========================================================================
:sf_dedicated_env
if not exist "%SF_PY_DIR_CFG%\python.exe" goto :sf_dedicated_incomplete
if not exist "%SF_IDF_PATH%\export.bat" goto :sf_dedicated_incomplete

set "DISCOVERED_PATH=%SF_PY_DIR_CFG%;%SF_PY_DIR_CFG%\Scripts;%PATH%"
set "SF_TOOLS_PATH=%SF_TOOLS_PATH_CFG%"
goto :sf_env_handoff

:sf_dedicated_incomplete
echo [ERROR] Dedicated environment is incomplete.
echo   Expected Python at: %SF_PY_DIR_CFG%\python.exe
echo   Expected ESP-IDF at: %SF_IDF_PATH%\export.bat
echo   Run install.bat to re-provision it.
exit /b 1

REM ==========================================================================
REM Shared tail: hand off PATH/IDF_PATH/IDF_TOOLS_PATH out of this script's
REM setlocal scope (so they persist in the caller's console), then load
REM ESP-IDF's own export.bat and verify it actually worked. Reached by
REM either branch above -- do not duplicate this in the dedicated branch.
REM ==========================================================================
:sf_env_handoff
endlocal & set "PATH=%DISCOVERED_PATH%" & set "IDF_PATH=%SF_IDF_PATH%" & set "IDF_TOOLS_PATH=%SF_TOOLS_PATH%"

if not exist "%IDF_PATH%\export.bat" (
    echo [ERROR] ESP-IDF not found at %IDF_PATH%
    echo   Run install.bat first.
    exit /b 1
)

echo [INFO] Loading ESP-IDF environment...
REM Clear any stale venv path from a previous run in this console so the
REM verification below reflects THIS run, not leftovers.
set "IDF_PYTHON_ENV_PATH="
call "%IDF_PATH%\export.bat"
if errorlevel 1 (
    echo [ERROR] ESP-IDF export.bat failed. Environment not fully loaded.
    exit /b 1
)

REM export.bat can exit 0 even when its activation failed (the error text is
REM printed but nothing gets exported), so a zero errorlevel is NOT proof
REM the environment loaded. Verify the exported venv actually exists before
REM declaring success -- a py3.12 console with only a py3.10 venv installed
REM used to print [OK] right after ESP-IDF's own "python_env ... not found"
REM error (observed 2026-07-22).
if not defined IDF_PYTHON_ENV_PATH (
    echo [ERROR] ESP-IDF environment did not load. Run install.bat again.
    exit /b 1
)
if not exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
    echo [ERROR] ESP-IDF Python venv is missing: %IDF_PYTHON_ENV_PATH%
    echo   Run install.bat again.
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
REM usable, within the supported range 3.10-3.12, AND -- when SF_VENV_MINORS
REM is non-empty -- matches the minor version of an installed ESP-IDF venv
REM (see the venv scan near the top). Returns via errorlevel:
REM   0 = acceptable
REM   1 = missing, unusable, out of range, or venv mismatch
REM Silent by design (called many times while probing candidates) -- callers
REM decide what to print once the final outcome is known. Legacy-mode-only
REM (dedicated mode goes straight to :sf_dedicated_env and never calls this).
"%~1" -c "import os,sys; v=sys.version_info[:2]; a=os.environ.get('SF_VENV_MINORS','').split(); sys.exit(0 if (3,10) <= v <= (3,12) and (not a or str(v[1]) in a) else 1)" >nul 2>&1
exit /b %errorlevel%
