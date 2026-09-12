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
REM   --help              Show installer.py's full option list and exit
REM   --force             Force reinstall all steps (skip probe checks)
REM   --uninstall         Remove sfcli from the ESP-IDF environment
REM   --clean             Clean install (remove config and sfcli, then
REM                       reinstall)
REM   --no-flasher        Skip the optional GUI Flasher app install
REM   --minimal           Install minimal dependencies (skip simulator)
REM   --use-existing-idf  Legacy mode: use a system Python + your own ESP-IDF
REM   --idf-path PATH     Legacy mode with an explicit ESP-IDF path
REM
REM Default (dedicated) mode fetches a private Python build into SF_HOME
REM (default C:\StampFly) and never touches any Python already on this
REM machine. It needs curl.exe and tar.exe, both included in Windows 10
REM version 1803 (April 2018 Update) and later. Passing --use-existing-idf
REM or --idf-path switches to legacy mode: find a system Python 3.8+ on
REM this machine and use it -- unchanged from before dedicated mode existed.
REM
REM Unlike install.sh, this script does not skip its git check for
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

REM --- Determine install mode: legacy (--use-existing-idf / --idf-path) or
REM     dedicated (default, private Python under SF_HOME) ---
set "SF_INSTALL_MODE=dedicated"
for %%A in (%*) do (
    if /i "%%~A"=="--use-existing-idf" set "SF_INSTALL_MODE=legacy"
    if /i "%%~A"=="--idf-path" set "SF_INSTALL_MODE=legacy"
)

if "%SF_INSTALL_MODE%"=="legacy" goto :sf_legacy_mode
goto :sf_dedicated_mode

REM ==========================================================================
REM Legacy mode: find a system Python and run installer.py exactly as before
REM dedicated mode existed. --idf-path's own value is still forwarded to
REM installer.py via %* below; this script only checks for the flag's name.
REM ==========================================================================
:sf_legacy_mode
echo [INFO] Mode: legacy (--use-existing-idf/--idf-path) -- using a system Python
echo.
echo [INFO] Checking Python...

call :sf_find_system_python

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
exit /b %errorlevel%

REM ==========================================================================
REM Dedicated mode (default): private Python + ESP-IDF under SF_HOME.
REM See docs\plans\dedicated-environment-plan.md sections 2-3.
REM ==========================================================================
:sf_dedicated_mode
echo [INFO] Mode: dedicated -- private Python + ESP-IDF under SF_HOME
echo.

REM --- Determine SF_HOME: env var > C:\StampFly > %LOCALAPPDATA%\StampFly ---
REM C:\StampFly is preferred because ESP-IDF does not support non-ASCII or
REM space-containing paths, and a Japanese Windows user name makes
REM LOCALAPPDATA non-ASCII. A standard user can create a top-level folder
REM on C:\ (Espressif's own installer does the same with C:\Espressif).
if not defined SF_HOME (
    set "SF_HOME=C:\StampFly"
    if not exist "C:\StampFly" mkdir "C:\StampFly" 2>nul
    if not exist "C:\StampFly" (
        set "SF_HOME=%LOCALAPPDATA%\StampFly"
        echo [WARN] Could not create C:\StampFly, using %LOCALAPPDATA%\StampFly instead.
        echo   ESP-IDF does not support non-ASCII paths or paths with spaces.
        echo   If this path is not plain ASCII, set SF_HOME to an ASCII-only,
        echo   space-free path and run install.bat again.
    )
)
if not exist "%SF_HOME%" mkdir "%SF_HOME%" 2>nul
echo   SF_HOME: %SF_HOME%
echo.

REM --- Required tools: curl.exe and tar.exe (Windows 10 1803+) ---
where curl.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] curl.exe not found.
    echo   Dedicated mode needs curl.exe and tar.exe, both included in
    echo   Windows 10 version 1803 ^(April 2018 Update^) and later.
    echo   Update Windows, or pass --use-existing-idf with your own
    echo   Python 3.10-3.12 and ESP-IDF instead.
    exit /b 1
)
where tar.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] tar.exe not found.
    echo   Dedicated mode needs curl.exe and tar.exe, both included in
    echo   Windows 10 version 1803 ^(April 2018 Update^) and later.
    echo   Update Windows, or pass --use-existing-idf with your own
    echo   Python 3.10-3.12 and ESP-IDF instead.
    exit /b 1
)

REM --- Fast path for --help/-h/--uninstall/--clean: these do not need the
REM     full bootstrap immediately -- reuse whatever Python is cheapest to
REM     reach, and only fetch the private Python as a last resort. ---
set "SF_SKIP_CHECKS=0"
for %%A in (%*) do (
    if /i "%%~A"=="--help" set "SF_SKIP_CHECKS=1"
    if /i "%%~A"=="-h" set "SF_SKIP_CHECKS=1"
    if /i "%%~A"=="--uninstall" set "SF_SKIP_CHECKS=1"
    if /i "%%~A"=="--clean" set "SF_SKIP_CHECKS=1"
)

if "%SF_SKIP_CHECKS%"=="1" goto :sf_dedicated_fast_path

call :sf_bootstrap_private_python
if errorlevel 1 exit /b 1
goto :sf_dedicated_run

:sf_dedicated_fast_path
if exist "%SF_HOME%\python\python.exe" (
    set "PYTHON_CMD=%SF_HOME%\python\python.exe"
    echo [INFO] Using existing private Python: !PYTHON_CMD!
    goto :sf_dedicated_run
)
call :sf_find_system_python
if defined PYTHON_CMD (
    echo [INFO] Using system Python on PATH: %PYTHON_CMD%
    goto :sf_dedicated_run
)
call :sf_bootstrap_private_python
if errorlevel 1 exit /b 1

:sf_dedicated_run
echo.
"%PYTHON_CMD%" -u "%SCRIPT_DIR%scripts\installer.py" --sf-home "%SF_HOME%" %*
exit /b %errorlevel%

REM ==========================================================================
REM Subroutines
REM All subroutines run in the same flat variable scope as the main script
REM (no nested setlocal/endlocal) -- delayed expansion (!VAR!) is already
REM enabled for the whole file, so results are visible to the caller
REM immediately after "call" returns, same as :sf_check_python below.
REM ==========================================================================

REM --- :sf_find_system_python ---
REM Discovers a system python.exe/py launcher, exactly as this script did
REM before dedicated mode existed (unchanged search order/logic; only moved
REM into a subroutine so both legacy mode and the dedicated fast path can
REM call it). Sets PYTHON_CMD/PYTHON_DIR/PYTHON_VERSION, or leaves
REM PYTHON_CMD undefined if nothing usable is found. Prepends PYTHON_DIR to
REM PATH when found, same as before.
:sf_find_system_python
set "PYTHON_DIR="
set "PYTHON_CMD="
set "PYTHON_VERSION="

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
exit /b 0

REM --- :sf_bootstrap_private_python ---
REM Fetches (if needed), verifies, and extracts the pinned private Python
REM into %SF_HOME%\python. Idempotent: does nothing but a version check if
REM the right version is already there. Sets PYTHON_CMD to
REM "%SF_HOME%\python\python.exe" on success. Returns via errorlevel:
REM   0 = ready, PYTHON_CMD set
REM   1 = failed (network, checksum, or extraction failure)
:sf_bootstrap_private_python
if exist "%SF_HOME%\python\python.exe" (
    set "SF_PBS_EXISTING="
    rem cmd /c strips the first and last quote of a command string that starts with a
    rem quote and contains more quotes, so '"...python.exe" -c "..."' breaks. Wrap the
    rem whole command in one extra pair of quotes and use -V (no inner quotes).
    rem cmd /c は、先頭が " で引用符が複数あるコマンド文字列の先頭と末尾の " を剥がすため、
    rem '"...python.exe" -c "..."' は壊れる。コマンド全体をもう一組の " で包み、内側に
    rem 引用符を含まない -V を使う（出力 "Python 3.12.14" の 2 番目のトークン）。
    for /f "tokens=2" %%v in ('""%SF_HOME%\python\python.exe" -V 2^>nul"') do set "SF_PBS_EXISTING=%%v"
    if "!SF_PBS_EXISTING!"=="3.12.14" (
        echo [OK] Private Python 3.12.14 already installed at %SF_HOME%\python
        set "PYTHON_CMD=%SF_HOME%\python\python.exe"
        exit /b 0
    )
)

REM CPython 3.12.14 (python-build-standalone release 20260901), install_only
REM builds. Filenames/SHA-256 must match
REM docs\plans\dedicated-environment-plan.md section 2 exactly -- that
REM table (also duplicated in install.sh) is the single source of truth.
set "SF_PBS_ARCH=%PROCESSOR_ARCHITEW6432%"
if not defined SF_PBS_ARCH set "SF_PBS_ARCH=%PROCESSOR_ARCHITECTURE%"

if /i "%SF_PBS_ARCH%"=="AMD64" (
    set "SF_PBS_ASSET=cpython-3.12.14+20260901-x86_64-pc-windows-msvc-install_only.tar.gz"
    set "SF_PBS_SHA256=e90c1b6419da3bd812dd73bb3de40287a21abf153438147639ec5e20375ea93f"
    set "SF_PBS_URL=https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%%2B20260901-x86_64-pc-windows-msvc-install_only.tar.gz"
) else if /i "%SF_PBS_ARCH%"=="ARM64" (
    set "SF_PBS_ASSET=cpython-3.12.14+20260901-aarch64-pc-windows-msvc-install_only.tar.gz"
    set "SF_PBS_SHA256=4e852236277eb8f7105cbe0f5adf45592f521af238bc0f700c351856e2c2e41a"
    set "SF_PBS_URL=https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%%2B20260901-aarch64-pc-windows-msvc-install_only.tar.gz"
) else (
    echo [ERROR] No private Python build for this CPU architecture: %SF_PBS_ARCH%
    echo   Supported: AMD64, ARM64
    echo   Use --use-existing-idf with a system Python 3.10-3.12 instead.
    exit /b 1
)

if not exist "%SF_HOME%\downloads" mkdir "%SF_HOME%\downloads" 2>nul
set "SF_PBS_ARCHIVE=%SF_HOME%\downloads\%SF_PBS_ASSET%"

set "SF_PBS_NEED_DOWNLOAD=1"
if exist "%SF_PBS_ARCHIVE%" (
    call :sf_sha256_check "%SF_PBS_ARCHIVE%" "%SF_PBS_SHA256%"
    if not errorlevel 1 (
        set "SF_PBS_NEED_DOWNLOAD=0"
        echo [INFO] Reusing downloaded %SF_PBS_ASSET% ^(checksum verified^)
    ) else (
        echo [WARN] Existing download has an unexpected checksum, re-downloading: %SF_PBS_ASSET%
        del /f /q "%SF_PBS_ARCHIVE%" >nul 2>&1
    )
)

if "%SF_PBS_NEED_DOWNLOAD%"=="1" (
    echo [INFO] Downloading private Python: %SF_PBS_ASSET%
    curl.exe -L --fail -o "%SF_PBS_ARCHIVE%.part" "%SF_PBS_URL%"
    if errorlevel 1 (
        echo [ERROR] Failed to download %SF_PBS_URL%
        del /f /q "%SF_PBS_ARCHIVE%.part" >nul 2>&1
        exit /b 1
    )
    move /y "%SF_PBS_ARCHIVE%.part" "%SF_PBS_ARCHIVE%" >nul
    call :sf_sha256_check "%SF_PBS_ARCHIVE%" "%SF_PBS_SHA256%"
    if errorlevel 1 (
        echo [ERROR] Checksum mismatch for %SF_PBS_ASSET%
        echo   expected: %SF_PBS_SHA256%
        del /f /q "%SF_PBS_ARCHIVE%" >nul 2>&1
        exit /b 1
    )
    echo [OK] Downloaded and verified %SF_PBS_ASSET%
)

echo [INFO] Extracting private Python...
set "SF_PBS_EXTRACT=%SF_HOME%\.python.extract.%RANDOM%"
if exist "%SF_PBS_EXTRACT%" rmdir /s /q "%SF_PBS_EXTRACT%"
mkdir "%SF_PBS_EXTRACT%"
tar.exe -xzf "%SF_PBS_ARCHIVE%" -C "%SF_PBS_EXTRACT%"
if errorlevel 1 (
    echo [ERROR] Failed to extract %SF_PBS_ARCHIVE%
    rmdir /s /q "%SF_PBS_EXTRACT%" >nul 2>&1
    exit /b 1
)
if exist "%SF_HOME%\python" rmdir /s /q "%SF_HOME%\python"
move "%SF_PBS_EXTRACT%\python" "%SF_HOME%\python" >nul
rmdir /s /q "%SF_PBS_EXTRACT%" >nul 2>&1

set "SF_PBS_NEWVER="
rem Same quoting rule as the existing-version probe above: outer quotes + -V.
rem 上の既存版の確認と同じ引用符の扱い: 外側の " で包み、-V を使う。
for /f "tokens=2" %%v in ('""%SF_HOME%\python\python.exe" -V 2^>nul"') do set "SF_PBS_NEWVER=%%v"
if not "%SF_PBS_NEWVER%"=="3.12.14" (
    echo [ERROR] Private Python bootstrap produced unexpected version: %SF_PBS_NEWVER%
    exit /b 1
)

echo [OK] Private Python 3.12.14 ready at %SF_HOME%\python
set "PYTHON_CMD=%SF_HOME%\python\python.exe"
exit /b 0

REM --- :sf_sha256_check ---
REM %1 = file path, %2 = expected SHA-256 (either may be quoted or not).
REM Verifies %1's SHA-256 against %2 using certutil (built into Windows,
REM no extra download needed). certutil's own output is:
REM   line 1: "SHA256 hash of <file>:"        (header, skip it)
REM   line 2: the hash, sometimes with spaces between byte pairs
REM   line 3: "CertUtil: -hashfile command completed successfully."
REM Returns via errorlevel: 0 = match, 1 = mismatch or hashing failed.
:sf_sha256_check
set "SF_HASH_ACTUAL="
for /f "skip=1 tokens=*" %%h in ('certutil -hashfile "%~1" SHA256 2^>nul') do (
    if not defined SF_HASH_ACTUAL set "SF_HASH_ACTUAL=%%h"
)
if not defined SF_HASH_ACTUAL exit /b 1
set "SF_HASH_ACTUAL=%SF_HASH_ACTUAL: =%"
if /i "%SF_HASH_ACTUAL%"=="%~2" exit /b 0
exit /b 1
