"""Run pyOCD using optional workspace-local dependencies; no operation is implicit.

Install with: python -m pip install --target .build/pyocd_deps -r tools/requirements-hardware.txt
"""
from pathlib import Path
import runpy
import sys


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '.build/pyocd_deps'))
    runpy.run_module('pyocd', run_name='__main__')
