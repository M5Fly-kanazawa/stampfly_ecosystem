"""
Allow running as: python -m sfcli
"""

import sys
from .cli import main

if __name__ == "__main__":
    sys.exit(main())
