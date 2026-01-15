#!/bin/bash
# StampFly Ecosystem Installer
# Usage: ./install.sh [options]
#
# This script checks for Python 3.10+ and then runs the Python installer.

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

        if [ "$major" -ge 3 ] && [ "$minor" -ge 10 ]; then
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
    error "Python 3.10+ is required but not found."
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
                echo "    ${BOLD}sudo apt update && sudo apt install python3.12${NC}"
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

# Main
header "StampFly Ecosystem Installer"

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
exec "$PYTHON_CMD" "$SCRIPT_DIR/scripts/installer.py" "$@"
