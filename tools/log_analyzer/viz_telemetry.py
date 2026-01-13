#!/usr/bin/env python3
"""Quick wrapper for visualize_telemetry.py --all"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from visualize_telemetry import main
if __name__ == '__main__':
    sys.argv.insert(1, '--all') if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else None
    main()
