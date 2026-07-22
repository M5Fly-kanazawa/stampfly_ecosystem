#!/bin/bash
# StampFly Ecosystem - Development Environment Setup (Linux / macOS / WSL2)
# Usage: source setup_env.sh
#
# This script must be sourced, not executed:
#   source setup_env.sh
#
# StampFly開発環境セットアップスクリプト（source で読み込むこと）

# Guard: detect if executed instead of sourced
# 実行ではなく source されたか確認
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[ERROR] This script must be sourced, not executed."
    echo "  Usage: source setup_env.sh"
    exit 1
fi

# Colors
_sf_green='\033[0;32m'
_sf_blue='\033[0;34m'
_sf_red='\033[0;31m'
_sf_nc='\033[0m'

echo
echo -e "${_sf_blue}[INFO]${_sf_nc} Setting up StampFly development environment..."
echo

# Determine project root (directory containing this script)
# プロジェクトルートを特定（このスクリプトのあるディレクトリ）
_sf_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Determine IDF_PATH: .sf/config.toml > IDF_PATH env > default
# IDF_PATHの決定: 設定ファイル > 環境変数 > デフォルトパス
_sf_idf_path=""

# Try .sf/config.toml first
# まず設定ファイルを確認
_sf_config="${_sf_script_dir}/.sf/config.toml"
if [ -f "$_sf_config" ]; then
    _sf_idf_path="$(grep '^path = ' "$_sf_config" 2>/dev/null | head -1 | sed 's/^path = "//;s/"$//')"
fi

# Fallback to IDF_PATH env or default
# 環境変数またはデフォルトにフォールバック
if [ -z "$_sf_idf_path" ] || [ ! -d "$_sf_idf_path" ]; then
    if [ -n "$IDF_PATH" ] && [ -d "$IDF_PATH" ]; then
        _sf_idf_path="$IDF_PATH"
    else
        _sf_idf_path="$HOME/esp/esp-idf"
    fi
fi

# Verify ESP-IDF exists
# ESP-IDFの存在確認
if [ ! -f "$_sf_idf_path/export.sh" ]; then
    echo -e "${_sf_red}[ERROR]${_sf_nc} ESP-IDF not found at $_sf_idf_path"
    echo "  Run ./install.sh first."
    echo
    # Clean up temporary variables
    unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path
    return 1
fi

# WSL2: strip /mnt/ paths to avoid Windows executables with CRLF
# WSL2: CRLFのWindows実行ファイルを回避するため /mnt/ パスを除外
if [ -f /proc/version ] && grep -qi microsoft /proc/version 2>/dev/null; then
    echo -e "${_sf_blue}[INFO]${_sf_nc} WSL2 detected, filtering Windows paths..."
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | tr '\n' ':' | sed 's/:$//')
fi

# Homebrew (Apple Silicon): append its bin so build tools ESP-IDF expects
# from the system (cmake, ninja) resolve in shells that never sourced
# .zprofile -- e.g. the StampFly Terminal app's .command launcher, whose
# stock PATH has /usr/local/bin (Intel Homebrew) but not /opt/homebrew/bin.
# Appended (not prepended) so ESP-IDF-managed tools keep priority.
# Homebrew（Apple Silicon）: ESP-IDFがシステム側に期待するビルドツール
# （cmake, ninja）が、.zprofile を読まないシェル（StampFly Terminalアプリの
# .commandランチャー等）でも解決できるよう bin を追加する。素のPATHには
# /usr/local/bin（Intel Homebrew）はあるが /opt/homebrew/bin は無い。
# ESP-IDF管理ツールを優先するため先頭ではなく末尾に追加する。
if [ -d /opt/homebrew/bin ]; then
    case ":$PATH:" in
        *:/opt/homebrew/bin:*) ;;
        *) export PATH="$PATH:/opt/homebrew/bin" ;;
    esac
fi

# Ensure a supported Python (3.10-3.12) is first on PATH before running
# export.sh, since ESP-IDF's detect_python.sh blindly picks the first
# python3/python/python3.9/python3.10... found on PATH. Non-interactive
# shells (e.g. the StampFly Terminal app's .command launcher) don't source
# .zshrc/.zprofile, so pyenv/Homebrew shims are missing and the system
# /usr/bin/python3 (often 3.9 or 3.13+) gets picked instead, which has no
# matching idf5.x venv.
# export.sh を実行する前に、対応バージョン（3.10〜3.12）のPythonをPATHの
# 先頭に来るようにする。ESP-IDFのdetect_python.shはPATH上で最初に見つかった
# python3/python/python3.9/python3.10...を無条件に採用するため。非対話シェル
# （StampFly Terminalアプリの.commandランチャー等）では.zshrc/.zprofileが
# 読み込まれずpyenv/Homebrewのshimが無いため、システムの/usr/bin/python3
# （3.9や3.13+のことが多い）が使われてしまい、対応するidf5.x venvが見つからない。
#
# In addition, a merely "supported" Python is NOT enough: export.sh derives
# the venv directory name (idf5.x_py3.Y_env) from the DETECTED Python's
# version. If python3 on PATH is 3.12 but the installer created a py3.10
# venv, export.sh fails with "python_env ... not found" (observed
# 2026-07-22 on Windows with pyenv global 3.12 and an installer-created
# py3.10 venv; the same mismatch is possible here). So the minor version
# matching an already-installed ESP-IDF venv takes priority over the
# newest supported version.
# さらに「対応範囲内」というだけでは不十分: export.sh は検出したPythonの
# バージョンから venv ディレクトリ名（idf5.x_py3.Y_env）を組み立てる。
# PATH上の python3 が 3.12 でも、インストーラが作った venv が py3.10 なら
# 「python_env ... not found」で失敗する（2026-07-22, Windows/Git Bash、
# pyenv-win の global=3.12 で観測）。このため、インストール済み venv に
# 一致するマイナーバージョンを最新の対応バージョンより優先する。
_sf_py_min_major=3
_sf_py_min_minor=10
_sf_py_max_minor=12

# Get "X.Y" version string from a python executable, or empty on failure
# 指定したpython実行ファイルから"X.Y"形式のバージョン文字列を取得（失敗時は空）
_sf_py_version() {
    "$1" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null
}

# Return 0 if "X.Y" version string is within the supported band
# "X.Y"形式のバージョン文字列が対応範囲内なら0を返す
_sf_py_in_band() {
    local _major _minor
    _major="${1%%.*}"
    _minor="${1#*.}"
    [ "$_major" = "$_sf_py_min_major" ] || return 1
    [ "$_minor" -ge "$_sf_py_min_minor" ] 2>/dev/null || return 1
    [ "$_minor" -le "$_sf_py_max_minor" ] 2>/dev/null || return 1
    return 0
}

# Find a Python 3.<minor> interpreter in known locations and echo its path.
# Searched in order: versioned name on PATH, pyenv, Homebrew, python.org.
# 既知の場所から Python 3.<minor> のインタプリタを探してパスを出力する。
# 探索順: PATH上のバージョン付き名、pyenv、Homebrew、python.org。
_sf_py_find_minor() {
    local _minor="$1" _cand _dir _prefix

    # 1) Explicit versioned names already on PATH
    # 1) PATH上の明示的なバージョン名
    if command -v "python3.${_minor}" >/dev/null 2>&1; then
        _cand="$(command -v "python3.${_minor}")"
        if [ "$(_sf_py_version "$_cand")" = "3.${_minor}" ]; then
            echo "$_cand"
            return 0
        fi
    fi

    # 2) pyenv versions (newest patch first via reverse sort)
    # 2) pyenvのバージョン群（パッチバージョンの新しい順）
    if [ -d "$HOME/.pyenv/versions" ]; then
        for _dir in $(ls -d "$HOME/.pyenv/versions/3.${_minor}".* 2>/dev/null | sort -rV); do
            if [ -x "$_dir/bin/python3" ]; then
                echo "$_dir/bin/python3"
                return 0
            fi
        done
    fi

    # 3) Homebrew (Apple Silicon then Intel prefixes)
    # 3) Homebrew（Apple Siliconの後にIntel）
    for _prefix in /opt/homebrew/opt /usr/local/opt; do
        _dir="${_prefix}/python@3.${_minor}/bin"
        if [ -x "${_dir}/python3.${_minor}" ]; then
            echo "${_dir}/python3.${_minor}"
            return 0
        fi
    done

    # 4) python.org framework installs (macOS)
    # 4) python.org（macOS Frameworkインストール）
    _dir="/Library/Frameworks/Python.framework/Versions/3.${_minor}/bin"
    if [ -x "${_dir}/python3" ]; then
        echo "${_dir}/python3"
        return 0
    fi

    return 1
}

# Collect the minor versions of already-installed ESP-IDF venvs
# (idf*_py3.Y_env under IDF_TOOLS_PATH), newest first.
# インストール済み ESP-IDF venv（IDF_TOOLS_PATH配下のidf*_py3.Y_env）の
# マイナーバージョンを新しい順に収集する。
_sf_tools_path="${IDF_TOOLS_PATH:-$HOME/.espressif}"
_sf_venv_minors=""
for _sf_venv_dir in "$_sf_tools_path"/python_env/idf*_py3.*_env; do
    [ -d "$_sf_venv_dir" ] || continue
    _sf_venv_minor="${_sf_venv_dir##*_py3.}"
    _sf_venv_minor="${_sf_venv_minor%_env}"
    case "$_sf_venv_minor" in
        ''|*[!0-9]*) continue ;;
    esac
    case " $_sf_venv_minors " in
        *" $_sf_venv_minor "*) ;;
        *) _sf_venv_minors="$_sf_venv_minors $_sf_venv_minor" ;;
    esac
done
_sf_venv_minors="$(echo $_sf_venv_minors | tr ' ' '\n' | sort -rn | tr '\n' ' ')"

# Preferred search order: venv-matching minors first, then newest supported
# 探索の優先順位: venv一致のマイナーが先、その後は新しい対応バージョン順
_sf_preferred_minors=""
for _sf_py_minor in $_sf_venv_minors 12 11 10; do
    [ "$_sf_py_minor" -ge "$_sf_py_min_minor" ] 2>/dev/null || continue
    [ "$_sf_py_minor" -le "$_sf_py_max_minor" ] 2>/dev/null || continue
    case " $_sf_preferred_minors " in
        *" $_sf_py_minor "*) ;;
        *) _sf_preferred_minors="$_sf_preferred_minors $_sf_py_minor" ;;
    esac
done

# The current python3 can be kept as-is only if it is in the supported band
# AND (when a venv already exists) matches one of the venv minor versions.
# 現在のpython3をそのまま使えるのは、対応範囲内であり、かつ（venvが既に
# ある場合は）venvのマイナーバージョンのいずれかに一致する場合のみ。
_sf_py_current_version="$(_sf_py_version python3)"
_sf_py_current_ok=""
if [ -n "$_sf_py_current_version" ] && _sf_py_in_band "$_sf_py_current_version"; then
    if [ -z "${_sf_venv_minors// /}" ]; then
        _sf_py_current_ok=1
    else
        case " $_sf_venv_minors " in
            *" ${_sf_py_current_version#*.} "*) _sf_py_current_ok=1 ;;
        esac
    fi
fi

if [ -z "$_sf_py_current_ok" ]; then
    # Current python3 is missing, out of band, or does not match the
    # installed venv; search known locations in preferred-minor order.
    # 現在のpython3が無い・対応範囲外・インストール済みvenvと不一致の
    # いずれか。優先マイナー順で既知の場所から探す。
    _sf_py_found=""
    for _sf_py_minor in $_sf_preferred_minors; do
        _sf_py_found="$(_sf_py_find_minor "$_sf_py_minor")"
        [ -n "$_sf_py_found" ] && break
    done

    if [ -n "$_sf_py_found" ]; then
        _sf_py_found_dir="$(dirname "$_sf_py_found")"
        if [ -x "${_sf_py_found_dir}/python3" ] && _sf_py_in_band "$(_sf_py_version "${_sf_py_found_dir}/python3")"; then
            # Plain "python3" in that directory already resolves correctly
            # そのディレクトリの素の"python3"が既に適合している
            export PATH="${_sf_py_found_dir}:$PATH"
        else
            # No plain "python3" name (e.g. python3.12 only) -- create a
            # temporary shim directory so detect_python.sh's first
            # candidate ("python3") resolves to the right interpreter.
            # 素の"python3"という名前が無い（python3.12のみ等）ので、
            # detect_python.shが最初に試す"python3"が正しいインタプリタを
            # 指すよう、一時的なshimディレクトリを作成する。
            _sf_py_shim_dir="$(mktemp -d)"
            ln -sf "$_sf_py_found" "${_sf_py_shim_dir}/python3"
            export PATH="${_sf_py_shim_dir}:$PATH"
        fi
        echo -e "${_sf_blue}[INFO]${_sf_nc} Using Python $(_sf_py_version "$_sf_py_found") at $_sf_py_found for ESP-IDF (shell default did not match)."
        echo "  シェルのデフォルトPythonが不適合のため、上記のPythonを使用します。"
    else
        if [ -n "${_sf_venv_minors// /}" ]; then
            echo -e "${_sf_red}[ERROR]${_sf_nc} Installed ESP-IDF env needs Python 3.${_sf_venv_minors%% *}, but no matching interpreter was found."
            echo "  インストール済みESP-IDF環境はPython 3.${_sf_venv_minors%% *}を必要としますが、一致するインタプリタが見つかりません。"
            echo "  Run ./install.sh to reinstall, or install the matching Python."
            echo "  ./install.sh を再実行するか、一致するバージョンのPythonをインストールしてください。"
        else
            echo -e "${_sf_red}[ERROR]${_sf_nc} No supported Python (3.${_sf_py_min_minor}-3.${_sf_py_max_minor}) found."
            echo "  対応するPython (3.${_sf_py_min_minor}〜3.${_sf_py_max_minor}) が見つかりません。"
            echo "  Run ./install.sh to set one up, or open a normal interactive shell."
            echo "  ./install.sh を実行するか、通常の対話シェルで再試行してください。"
        fi
        echo
        unset -f _sf_py_version _sf_py_in_band _sf_py_find_minor
        unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path
        unset _sf_py_min_major _sf_py_min_minor _sf_py_max_minor _sf_py_current_version _sf_py_current_ok
        unset _sf_py_found _sf_py_found_dir _sf_py_minor _sf_py_shim_dir
        unset _sf_tools_path _sf_venv_minors _sf_venv_minor _sf_venv_dir _sf_preferred_minors
        return 1
    fi
fi

unset -f _sf_py_version _sf_py_in_band _sf_py_find_minor
unset _sf_py_min_major _sf_py_min_minor _sf_py_max_minor _sf_py_current_version _sf_py_current_ok
unset _sf_py_found _sf_py_found_dir _sf_py_minor _sf_py_shim_dir
unset _sf_tools_path _sf_venv_minors _sf_venv_minor _sf_venv_dir _sf_preferred_minors

# Source ESP-IDF environment
# ESP-IDF環境を読み込み
echo -e "${_sf_blue}[INFO]${_sf_nc} Loading ESP-IDF environment..."
# Drop any stale venv override left by a previous source in this shell so
# the verification below reflects THIS run, not leftovers.
# このシェルで以前sourceした際の古いvenv指定を破棄し、下の検証が今回の
# 実行結果を反映するようにする。
unset IDF_PYTHON_ENV_PATH
source "$_sf_idf_path/export.sh"
_sf_idf_export_status=$?

# export.sh (ESP-IDF v5.5) returns 0 even when its inner activate.py fails:
# the error is printed but the eval'd export list is simply empty. So a
# zero status is NOT proof the environment loaded -- verify the exported
# venv actually exists before declaring success (a py3.12 shell with only
# a py3.10 venv installed used to print [OK] right after ESP-IDF's own
# "python_env ... not found" error).
# export.sh（ESP-IDF v5.5）は内部のactivate.pyが失敗しても0を返す
# （エラーは表示されるがevalされるexport列が空になるだけ）。そのため
# ステータス0は環境ロード成功の証明にならない。成功を宣言する前に、
# エクスポートされたvenvが実在するか検証する（py3.12のシェルにpy3.10の
# venvしか無い場合、ESP-IDF自身の「python_env ... not found」エラーの
# 直後に[OK]と表示されていた）。
_sf_env_ok=1
if [ "$_sf_idf_export_status" -ne 0 ]; then
    _sf_env_ok=0
elif [ -z "${IDF_PYTHON_ENV_PATH-}" ]; then
    _sf_env_ok=0
elif [ ! -f "$IDF_PYTHON_ENV_PATH/bin/python" ] && [ ! -f "$IDF_PYTHON_ENV_PATH/Scripts/python.exe" ]; then
    _sf_env_ok=0
fi

if [ "$_sf_env_ok" -ne 1 ]; then
    echo
    echo -e "${_sf_red}[ERROR]${_sf_nc} Failed to load ESP-IDF environment."
    echo "  Re-run ./install.sh, or retry from a new terminal."
    echo "  ESP-IDF環境の読み込みに失敗しました。"
    echo "  ./install.sh を再実行するか、新しいターミナルで再試行してください。"
    echo
    unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path _sf_idf_export_status _sf_env_ok
    return 1
fi

echo
echo -e "${_sf_green}[OK]${_sf_nc} StampFly development environment ready."
echo
echo "  sf doctor          Check environment"
echo "  sf build vehicle   Build vehicle firmware"
echo "  sf --help          Show all commands"
echo

# Clean up temporary variables
# 一時変数をクリーンアップ
unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path _sf_idf_export_status _sf_env_ok
