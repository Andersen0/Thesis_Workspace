#!/usr/bin/env python3

import sys
sys.dont_write_bytecode = True
try:
    from .gamepad import main
except ImportError:
    from gamepad import main

if __name__ == '__main__':
    main()
