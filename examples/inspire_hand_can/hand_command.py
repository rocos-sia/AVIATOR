#!/usr/bin/env python3
"""Compatibility entry point for nodes/aviator_hand/hand_command.py."""
import importlib.util
from pathlib import Path
import sys

_path = Path(__file__).resolve().parents[2] / "nodes/aviator_hand/hand_command.py"
_spec = importlib.util.spec_from_file_location("aviator_hand_command", _path)
_module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _module
_spec.loader.exec_module(_module)
globals().update({name: value for name, value in vars(_module).items() if not name.startswith("_")})
if __name__ == "__main__":
    sys.exit(_module.main())
