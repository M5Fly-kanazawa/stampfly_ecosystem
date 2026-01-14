#!/bin/bash
# StampFly Ecosystem Environment Activation (Unix)
# Usage: source scripts/activate.sh

# Get script directory (works with source)
if [ -n "$BASH_SOURCE" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elif [ -n "$ZSH_VERSION" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"
else
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

STAMPFLY_ROOT="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$STAMPFLY_ROOT/.venv"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "[ERROR] Virtual environment not found: $VENV_DIR"
    echo "        Run ./scripts/install.sh first"
    return 1 2>/dev/null || exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Set environment variables
export STAMPFLY_ROOT="$STAMPFLY_ROOT"
export PYTHONPATH="$STAMPFLY_ROOT/lib:$PYTHONPATH"

# Add bin to PATH
export PATH="$STAMPFLY_ROOT/bin:$PATH"

# Source ESP-IDF if available
ESP_IDF_PATH=""
if [ -d "$STAMPFLY_ROOT/.esp-idf" ]; then
    ESP_IDF_PATH="$STAMPFLY_ROOT/.esp-idf"
elif [ -d "$HOME/esp/esp-idf" ]; then
    ESP_IDF_PATH="$HOME/esp/esp-idf"
fi

if [ -n "$ESP_IDF_PATH" ] && [ -f "$ESP_IDF_PATH/export.sh" ]; then
    source "$ESP_IDF_PATH/export.sh" > /dev/null 2>&1
fi

# Print activation message
echo "StampFly Ecosystem activated"
echo "  Root: $STAMPFLY_ROOT"
if [ -n "$ESP_IDF_PATH" ]; then
    echo "  ESP-IDF: $ESP_IDF_PATH"
fi
echo ""
echo "Run 'sf --help' for available commands"
