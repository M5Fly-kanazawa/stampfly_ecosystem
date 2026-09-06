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
    local required_packages="git cmake ninja-build python3 python3-pip python3-venv wget flex bison gperf ccache libffi-dev libssl-dev dfu-util libusb-1.0-0"

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
