#!/usr/bin/env python3
"""Example subscriber entry point for the shared camera node."""

from pathlib import Path
import runpy
import sys


CAMERA_NODE = Path(__file__).resolve().parents[2] / "nodes" / "camera"
sys.path.insert(0, str(CAMERA_NODE))
runpy.run_path(str(CAMERA_NODE / "detection_subscriber.py"), run_name="__main__")
