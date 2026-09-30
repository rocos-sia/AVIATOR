"""Compatibility runner for the maintained hand publisher regression tests."""
from pathlib import Path
import runpy
import sys

_directory = Path(__file__).resolve().parents[2] / "nodes/aviator_hand"
sys.path.insert(0, str(_directory))
if __name__ == "__main__":
    runpy.run_path(str(_directory / "hand_command_test.py"), run_name="__main__")
