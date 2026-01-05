#!/usr/bin/env python3
"""Show all ESKF data (sensors + states + biases)"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visualize_eskf import main
sys.argv = [sys.argv[0]] + sys.argv[1:] + ['--all']
main()
