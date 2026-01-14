#!/bin/bash
# StampFly Ecosystem Installer (Unix)
# Usage: ./scripts/install.sh [options]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Run Python installer
python3 "$SCRIPT_DIR/installer.py" "$@"
