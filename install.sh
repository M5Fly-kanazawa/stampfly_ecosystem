#!/bin/bash
# StampFly Ecosystem Installer
# Usage: ./install.sh [options]
#
# Options (forwarded to scripts/installer.py; see that file's docstring
# for the full list):
#   --help           Show installer.py's full option list and exit
#   --force          Force reinstall all steps (skip probe checks)
#   --uninstall      Remove sfcli from the ESP-IDF environment
#   --clean          Clean install (remove config and sfcli, then reinstall)
#   --no-flasher     Skip the optional GUI Flasher app install
#   --minimal        Install minimal dependencies (skip simulator)
#
# This script checks for Python 3.10-3.12 and then runs the Python installer.
# --help/--uninstall/--clean skip the system prerequisite checks below
# (cmake/ninja/etc. are only needed to actually build firmware, not to
# print help or remove an existing install).
# --help/--uninstall/--clean はシステム前提条件チェック(cmake/ninja等)を
# スキップする(ファームウェアのビルドに使うものであり、ヘルプ表示や
# アンインストールには不要なため)。

# Detect if script is being sourced (must be BEFORE set -e)
# sourceで実行された場合を検出（set -eより前に行うこと）
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo -e "\033[0;31m[ERROR]\033[0m Do not 'source' this script. Run it directly:"
    echo "    ./install.sh"
    echo "  or:"
    echo "    bash install.sh"
    return 1 2>/dev/null || true
fi

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

header() {
    echo
    echo -e "${CYAN}============================================================${NC}"
    echo -e "${BOLD} $1${NC}"
    echo -e "${CYAN}============================================================${NC}"
    echo
}

# WSL2 detection
# WSL2環境を検出
is_wsl() {
    [ -f /proc/version ] && grep -qi microsoft /proc/version
}

# Check system prerequisites for Linux
# Linux用システム前提条件チェック
check_prerequisites_linux() {
    # ESP-IDF required packages for Debian/Ubuntu
    # (per https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/linux-macos-setup.html)
    # In dedicated mode the private Python (bootstrap_private_python) replaces
    # the system python3/python3-pip/python3-venv packages, so they are only
    # required on the legacy path (LINUX_REQUIRED_PACKAGES is chosen by mode
    # further down, before this function is called).
    # 専用モードでは専用Python（bootstrap_private_python）がシステムの
    # python3/python3-pip/python3-venvを代替するため、これらはlegacy経路
    # でのみ必要（LINUX_REQUIRED_PACKAGESはこの関数が呼ばれる前にモードで
    # 選択される）。
    local required_packages="$LINUX_REQUIRED_PACKAGES"

    if [ -f /etc/debian_version ]; then
        local missing=""
        for pkg in $required_packages; do
            if ! dpkg -s "$pkg" > /dev/null 2>&1; then
                missing="$missing $pkg"
            fi
        done

        if [ -n "$missing" ]; then
            error "Missing required packages:$missing"
            echo
            echo "  Install with:"
            echo -e "    ${BOLD}sudo apt update && sudo apt install -y$missing${NC}"
            echo
            exit 1
        fi
        success "All prerequisite packages installed"
    else
        # Non-Debian (Fedora, Arch, etc.): check key commands only
        # 非Debian系: 主要コマンドのみ確認
        local missing_cmds=""
        for cmd_name in git cmake ninja python3 wget flex bison gperf ccache; do
            if ! command -v "$cmd_name" &> /dev/null; then
                missing_cmds="$missing_cmds $cmd_name"
            fi
        done

        if [ -n "$missing_cmds" ]; then
            error "Missing required commands:$missing_cmds"
            echo "  Please install these using your system package manager."
            echo
            exit 1
        fi
        success "All prerequisite commands found"
    fi
}

# Check system prerequisites for macOS
# macOS用システム前提条件チェック
check_prerequisites_macos() {
    local has_issues=false

    # Check XCode Command Line Tools
    # XCode CLTの確認
    if ! xcode-select -p > /dev/null 2>&1; then
        error "XCode Command Line Tools not installed"
        echo
        echo "  Install with:"
        echo -e "    ${BOLD}xcode-select --install${NC}"
        echo
        has_issues=true
    else
        success "XCode Command Line Tools installed"
    fi

    # Check Homebrew
    # Homebrewの確認
    if ! command -v brew &> /dev/null; then
        error "Homebrew not found"
        echo
        echo "  Install from:"
        echo -e "    ${BOLD}https://brew.sh${NC}"
        echo
        if [ "$has_issues" = true ]; then
            exit 1
        fi
        has_issues=true
    fi

    # Check required tools (cmake, ninja, dfu-util, ccache)
    # 必須ツールの確認
    local required_cmds="cmake ninja dfu-util ccache"
    local missing_cmds=""
    for cmd_name in $required_cmds; do
        if ! command -v "$cmd_name" &> /dev/null; then
            missing_cmds="$missing_cmds $cmd_name"
        fi
    done

    if [ -n "$missing_cmds" ]; then
        error "Missing required tools:$missing_cmds"
        echo
        if command -v brew &> /dev/null; then
            echo "  Install with:"
            echo -e "    ${BOLD}brew install$missing_cmds${NC}"
        else
            echo "  Install Homebrew first, then run:"
            echo -e "    ${BOLD}brew install$missing_cmds${NC}"
        fi
        echo
        has_issues=true
    else
        success "All required tools installed (cmake, ninja, dfu-util, ccache)"
    fi

    if [ "$has_issues" = true ]; then
        exit 1
    fi
}

# Check Python version
check_python() {
    local cmd=$1
    if command -v "$cmd" &> /dev/null; then
        local version
        version=$("$cmd" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null)
        local major
        major=$("$cmd" -c 'import sys; print(sys.version_info.major)' 2>/dev/null)
        local minor
        minor=$("$cmd" -c 'import sys; print(sys.version_info.minor)' 2>/dev/null)

        # Accept only the same 3.10-3.12 band scripts/installer.py itself
        # accepts (PYTHON_PREFERRED_MIN/MAX): older than 3.10 lacks features
        # ESP-IDF/sfcli rely on, and 3.13+ has caused real observed failures
        # (2026-07-22 policy change), so this script's own gate must match --
        # otherwise a 3.8/3.9/3.13+ interpreter passes here only to be
        # rejected by installer.py moments later.
        # scripts/installer.py 自身が受理する 3.10〜3.12 帯（PYTHON_PREFERRED_MIN/MAX）
        # とだけ一致させる: 3.10未満はESP-IDF/sfcliが必要とする機能を欠き、
        # 3.13以降は実際に動作しない事例が確認されている(2026-07-22の方針変更)
        # ため、本スクリプトのゲートもこれに合わせないと、3.8/3.9/3.13以降の
        # インタプリタがここを通過した直後に installer.py に弾かれることになる。
        if [ "$major" -eq 3 ] && [ "$minor" -ge 10 ] && [ "$minor" -le 12 ]; then
            echo "$cmd:$version"
            return 0
        fi
    fi
    return 1
}

# Find suitable Python
find_python() {
    for cmd in python3.12 python3.11 python3.10 python3 python; do
        result=$(check_python "$cmd") && {
            echo "$result"
            return 0
        }
    done
    return 1
}

# Install Python guidance
install_python_guidance() {
    echo
    error "Python 3.10-3.12 is required but not found."
    echo

    case "$(uname -s)" in
        Darwin)
            echo "  Install Python using Homebrew:"
            echo "    ${BOLD}brew install python@3.12${NC}"
            echo
            echo "  Or download from:"
            echo "    https://www.python.org/downloads/"
            ;;
        Linux)
            if [ -f /etc/debian_version ]; then
                echo "  Install Python using apt:"
                echo "    ${BOLD}sudo apt update && sudo apt install python3.12 python3.12-venv${NC}"
            elif [ -f /etc/fedora-release ]; then
                echo "  Install Python using dnf:"
                echo "    ${BOLD}sudo dnf install python3.12${NC}"
            else
                echo "  Install Python using your package manager."
            fi
            echo
            echo "  Or use pyenv:"
            echo "    https://github.com/pyenv/pyenv"
            ;;
        *)
            echo "  Download Python from:"
            echo "    https://www.python.org/downloads/"
            ;;
    esac
    echo
}

# --------------------------------------------------------------------------
# Dedicated environment: private Python bootstrap
# 専用環境: 専用Pythonの取得
#
# See docs/plans/dedicated-environment-plan.md sections 2-3. Default installs
# ("dedicated" mode) no longer look for a system Python at all -- they fetch
# a pinned python-build-standalone (PBS) "install_only" build into
# $SF_HOME/python and run scripts/installer.py with it. Passing
# --use-existing-idf or --idf-path keeps the old behavior unchanged (find a
# system Python 3.10-3.12, exactly as before this change).
# 既定のインストール（専用モード）は参加者のPythonを一切探さない。固定版の
# python-build-standalone（PBS）の install_only ビルドを $SF_HOME/python に
# 取得し、そのPythonで scripts/installer.py を起動する。--use-existing-idf
# または --idf-path を指定した場合は、この変更前と全く同じ挙動
# （参加者のPython 3.10-3.12を探す）を維持する。
# --------------------------------------------------------------------------

# CPython 3.12.14 (python-build-standalone release 20260901), install_only
# builds. Filenames/SHA-256 must match docs/plans/dedicated-environment-plan.md
# section 2 exactly -- this is the single source of truth for install.bat too.
# CPython 3.12.14（python-build-standalone リリース20260901）のinstall_only
# ビルド。ファイル名/SHA-256は docs/plans/dedicated-environment-plan.md の
# 2節と完全一致させること（install.batにとってもこれが正）。
PBS_PYTHON_VERSION="3.12.14"
PBS_BASE_URL="https://github.com/astral-sh/python-build-standalone/releases/download/20260901/"

# Fill PBS_ASSET / PBS_SHA256 / PBS_URL for the current OS+arch, or return 1
# listing supported targets if this platform has no pinned build.
# 現在のOS+アーキ向けにPBS_ASSET/PBS_SHA256/PBS_URLを設定する。対応する
# 固定ビルドが無ければ対応ターゲット一覧を表示して1を返す。
_sf_pbs_select_target() {
    local os_name arch_name
    os_name="$(uname -s)"
    arch_name="$(uname -m)"

    case "${os_name}:${arch_name}" in
        Darwin:arm64)
            asset="cpython-3.12.14+20260901-aarch64-apple-darwin-install_only.tar.gz"
            sha256="3ee3ee547cedfeb7c2b16b2b7156039f7b470bb8f857e226fd3d2eb11db83c76"
            url="https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%2B20260901-aarch64-apple-darwin-install_only.tar.gz"
            ;;
        Darwin:x86_64)
            asset="cpython-3.12.14+20260901-x86_64-apple-darwin-install_only.tar.gz"
            sha256="2e31b23f3f1319f707d0e620b48847a0046577541d357276821f9f1b5492e0ba"
            url="https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%2B20260901-x86_64-apple-darwin-install_only.tar.gz"
            ;;
        Linux:x86_64)
            asset="cpython-3.12.14+20260901-x86_64-unknown-linux-gnu-install_only.tar.gz"
            sha256="936c246dfdbbfa7cb22dd01814a21f582a892689fae96b06071a5e433baffa22"
            url="https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%2B20260901-x86_64-unknown-linux-gnu-install_only.tar.gz"
            ;;
        Linux:aarch64)
            asset="cpython-3.12.14+20260901-aarch64-unknown-linux-gnu-install_only.tar.gz"
            sha256="b61b856c3e1a4fc65b8f6e6b0495ef975dd0924f90c59f3ea61b38a079173b84"
            url="https://github.com/astral-sh/python-build-standalone/releases/download/20260901/cpython-3.12.14%2B20260901-aarch64-unknown-linux-gnu-install_only.tar.gz"
            ;;
        *)
            error "No private Python build for this platform: ${os_name} ${arch_name}"
            echo "  Supported targets: Darwin arm64, Darwin x86_64, Linux x86_64, Linux aarch64"
            echo "  Use --use-existing-idf (or --idf-path) with a system Python 3.10-3.12 instead."
            return 1
            ;;
    esac
    return 0
}

# Compute SHA-256 of a file, using whichever tool is available.
# 利用可能なツールでファイルのSHA-256を計算する。
_sf_sha256() {
    if command -v sha256sum > /dev/null 2>&1; then
        sha256sum "$1" | awk '{print $1}'
    else
        shasum -a 256 "$1" | awk '{print $1}'
    fi
}

# Warn if SF_HOME contains spaces or non-ASCII characters (ESP-IDF does not
# support either in its own path or in the Python interpreter's path).
# SF_HOMEに空白または非ASCII文字が含まれる場合に警告する（ESP-IDFは自身の
# パスにもPythonインタプリタのパスにも、どちらも非対応のため）。
_sf_warn_sf_home_chars() {
    case "$1" in
        *' '*)
            warn "SF_HOME contains spaces, which ESP-IDF does not support: $1"
            ;;
    esac
    case "$1" in
        *[![:print:]]*)
            warn "SF_HOME contains non-printable characters: $1"
            ;;
    esac
    # LC_ALL=C makes [:print:]/byte-range matches operate on raw bytes, so a
    # multi-byte UTF-8 (non-ASCII) character shows up as bytes outside the
    # 7-bit ASCII printable range here.
    # LC_ALL=Cにすることで[:print:]/バイト範囲マッチが生バイト単位になり、
    # マルチバイトUTF-8（非ASCII）文字はここで7bit ASCII可視範囲外のバイト
    # として検出される。
    if LC_ALL=C printf '%s' "$1" | grep -q '[^ -~]'; then
        warn "SF_HOME contains non-ASCII characters, which ESP-IDF does not support: $1"
    fi
}

# Fetch (if needed), verify, and extract the private Python into
# $SF_HOME/python. Idempotent: does nothing if the right version is already
# there. Returns 0 on success, 1 on any failure (network, checksum, extract).
# 専用Pythonを（必要なら）取得・検証・展開して $SF_HOME/python に配置する。
# 冪等: 既に正しい版があれば何もしない。成功時0、失敗時1を返す
# （ネットワーク・チェックサム・展開のいずれかの失敗）。
bootstrap_private_python() {
    # Reuse an already-provisioned private Python of the right version.
    # 既に正しい版の専用Pythonがあれば再利用する。
    if [ -x "$SF_HOME/python/bin/python3" ]; then
        local existing_version
        existing_version="$("$SF_HOME/python/bin/python3" -c 'import sys; print(sys.version.split()[0])' 2>/dev/null || true)"
        if [ "$existing_version" = "$PBS_PYTHON_VERSION" ]; then
            success "Private Python $PBS_PYTHON_VERSION already installed at $SF_HOME/python"
            return 0
        fi
    fi

    local asset sha256 url
    _sf_pbs_select_target || return 1

    mkdir -p "$SF_HOME/downloads"
    local archive="$SF_HOME/downloads/$asset"

    local need_download=true
    if [ -f "$archive" ]; then
        if [ "$(_sf_sha256 "$archive")" = "$sha256" ]; then
            need_download=false
            info "Reusing downloaded $asset (checksum verified)"
        else
            warn "Existing download has an unexpected checksum, re-downloading: $asset"
            rm -f "$archive"
        fi
    fi

    if [ "$need_download" = true ]; then
        info "Downloading private Python: $asset"
        if ! curl -L --fail --progress-bar -o "${archive}.part" "$url"; then
            error "Failed to download $url"
            rm -f "${archive}.part"
            return 1
        fi
        mv "${archive}.part" "$archive"

        local actual_sha256
        actual_sha256="$(_sf_sha256 "$archive")"
        if [ "$actual_sha256" != "$sha256" ]; then
            error "Checksum mismatch for $asset"
            echo "  expected: $sha256"
            echo "  actual:   $actual_sha256"
            rm -f "$archive"
            return 1
        fi
        success "Downloaded and verified $asset"
    fi

    info "Extracting private Python..."
    local extract_dir="$SF_HOME/.python.extract.$$"
    rm -rf "$extract_dir"
    mkdir -p "$extract_dir"
    if ! tar -xzf "$archive" -C "$extract_dir"; then
        error "Failed to extract $archive"
        rm -rf "$extract_dir"
        return 1
    fi
    rm -rf "$SF_HOME/python"
    mv "$extract_dir/python" "$SF_HOME/python"
    rm -rf "$extract_dir"

    local new_version
    new_version="$("$SF_HOME/python/bin/python3" -c 'import sys; print(sys.version.split()[0])' 2>/dev/null || true)"
    if [ "$new_version" != "$PBS_PYTHON_VERSION" ]; then
        error "Private Python bootstrap produced unexpected version: ${new_version:-<none>}"
        return 1
    fi

    local size_human
    size_human="$(du -sh "$SF_HOME/python" 2>/dev/null | awk '{print $1}')"
    success "Private Python $PBS_PYTHON_VERSION ready at $SF_HOME/python (${size_human:-size unknown})"
    return 0
}

# Skip the build-toolchain prerequisite checks (cmake/ninja/etc.) for
# argument combinations that never build firmware. E.g. `--uninstall` only
# needs to run pip uninstall + delete files -- ninja is irrelevant there
# (and requiring it would block uninstalling on a machine where the
# prerequisite install itself failed).
# ファームウェアをビルドしない引数の組み合わせでは、ビルドツールチェーン
# の前提条件チェック(cmake/ninja等)をスキップする。例えば`--uninstall`は
# pip uninstall + ファイル削除しか行わずninjaは無関係(前提条件インストール
# 自体が失敗した環境でアンインストールができなくなるのを防ぐ)。
SKIP_PREREQUISITE_CHECKS=false
for arg in "$@"; do
    case "$arg" in
        --help|-h|--uninstall|--clean)
            SKIP_PREREQUISITE_CHECKS=true
            break
            ;;
    esac
done

# Install mode: dedicated (default) fetches a private Python into $SF_HOME
# and never touches the participant's system Python. --use-existing-idf or
# --idf-path opts into the legacy behavior (find a system Python, use the
# participant's own ESP-IDF) -- unchanged from before this feature existed.
# インストールモード: dedicated（既定）は専用Pythonを$SF_HOMEに取得し、
# 参加者のシステムPythonには一切触れない。--use-existing-idf または
# --idf-path を指定するとlegacy挙動（システムPythonを探し、参加者自身の
# ESP-IDFを使う）を選べる -- この機能追加以前と変わらない。
INSTALL_MODE="dedicated"
for arg in "$@"; do
    case "$arg" in
        --use-existing-idf|--idf-path)
            INSTALL_MODE="legacy"
            break
            ;;
    esac
done

# Linux prerequisite package list depends on mode: dedicated mode's private
# Python replaces the system python3/python3-pip/python3-venv packages, so
# only the legacy path still requires them.
# Linuxの前提パッケージ一覧はモード依存: dedicatedモードは専用Pythonが
# システムのpython3/python3-pip/python3-venvを代替するため、legacy経路
# だけがそれらを引き続き必要とする。
if [ "$INSTALL_MODE" = "legacy" ]; then
    LINUX_REQUIRED_PACKAGES="git cmake ninja-build python3 python3-pip python3-venv wget flex bison gperf ccache libffi-dev libssl-dev dfu-util libusb-1.0-0"
else
    LINUX_REQUIRED_PACKAGES="git curl tar cmake ninja-build wget flex bison gperf ccache libffi-dev libssl-dev dfu-util libusb-1.0-0"
fi

# Main
header "StampFly Ecosystem Installer"

# WSL2 information
# WSL2環境の情報表示
if is_wsl; then
    info "WSL2 environment detected"
    echo
    echo "  Note: USB device access requires usbipd-win on Windows side."
    echo "  See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb"
    echo
fi

# Check system prerequisites
# システム前提条件チェック
if [ "$SKIP_PREREQUISITE_CHECKS" = true ]; then
    info "Skipping build-toolchain prerequisite checks (--help/--uninstall/--clean)"
    echo
else
    case "$(uname -s)" in
        Linux)
            info "Checking system prerequisites..."
            check_prerequisites_linux
            echo
            ;;
        Darwin)
            info "Checking system prerequisites..."
            check_prerequisites_macos
            echo
            ;;
    esac
fi

if [ "$INSTALL_MODE" = "legacy" ]; then
    info "Mode: legacy (--use-existing-idf/--idf-path) -- using a system Python"
    echo
    info "Checking Python..."

    PYTHON_RESULT=$(find_python) || {
        install_python_guidance
        exit 1
    }

    PYTHON_CMD="${PYTHON_RESULT%%:*}"
    PYTHON_VERSION="${PYTHON_RESULT##*:}"

    success "Found Python $PYTHON_VERSION ($PYTHON_CMD)"
    echo

    # Run Python installer
    # -u: unbuffered stdout to keep correct output ordering with subprocesses
    # -u: サブプロセスとの出力順序を正しく保つためバッファなし出力
    "$PYTHON_CMD" -u "$SCRIPT_DIR/scripts/installer.py" "$@"
    exit $?
fi

# Dedicated mode (default)
# 専用モード（既定）
SF_HOME="${SF_HOME:-$HOME/.stampfly}"
info "Mode: dedicated -- private Python + ESP-IDF under \$SF_HOME"
echo "  SF_HOME: $SF_HOME"
echo
_sf_warn_sf_home_chars "$SF_HOME"

if [ "$SKIP_PREREQUISITE_CHECKS" = true ]; then
    # --help/--uninstall/--clean: prefer an already-provisioned private
    # Python, then any python3 on PATH, and only bootstrap as a last resort
    # -- these subcommands do not need the full toolchain checks above, and
    # should stay fast/offline whenever possible.
    # --help/--uninstall/--clean: 既に取得済みの専用Pythonを優先し、次に
    # PATH上のpython3、最後の手段として取得する -- これらのサブコマンドは
    # 上記の完全なツールチェーン確認を必要とせず、可能な限り高速・
    # オフラインであるべき。
    if [ -x "$SF_HOME/python/bin/python3" ]; then
        PYTHON_CMD="$SF_HOME/python/bin/python3"
        info "Using existing private Python: $PYTHON_CMD"
    elif command -v python3 > /dev/null 2>&1; then
        PYTHON_CMD="python3"
        info "Using system Python on PATH: $PYTHON_CMD"
    else
        bootstrap_private_python || exit 1
        PYTHON_CMD="$SF_HOME/python/bin/python3"
    fi
    echo
    "$PYTHON_CMD" -u "$SCRIPT_DIR/scripts/installer.py" --sf-home "$SF_HOME" "$@"
    exit $?
fi

bootstrap_private_python || exit 1
echo
info "Running installer.py with the private Python"
echo

# Run Python installer
# -u: unbuffered stdout to keep correct output ordering with subprocesses
# -u: サブプロセスとの出力順序を正しく保つためバッファなし出力
"$SF_HOME/python/bin/python3" -u "$SCRIPT_DIR/scripts/installer.py" --sf-home "$SF_HOME" "$@"
exit $?
