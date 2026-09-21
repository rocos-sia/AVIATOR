#!/usr/bin/env python3
"""Combine the 9 T0 videos into one 3x3 grid (column = profile, row = view).

Reads t0_<profile>_collision{,_topdown,_side}.mp4 and tiles them:
    column:        sine       roll_pull   random
    row (3/4)   |  sine 3/4 |  rp 3/4   | random 3/4
    row (top)   |  sine top |  rp top   | random top
    row (side)  |  sine side|  rp side  | random side
Each cell keeps full 720p, so the output is 3840x2160 (4K). Column/row labels are
overlaid. Run with the `mujo` env (for cv2) or any env with opencv + ffmpeg.
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import cv2

VIEWS = ["", "_topdown", "_side"]
PROFILES = ["sine", "roll_pull", "random"]
COL_LABEL = ["sine roll", "roll + pull", "irregular band-limited"]
ROW_LABEL = ["3/4 view", "top-down", "side"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="dir holding the 9 t0_*.mp4")
    ap.add_argument("--out", required=True, help="output 4K mp4 path")
    args = ap.parse_args()

    d = Path(args.dir)
    caps = [[cv2.VideoCapture(str(d / f"t0_{p}_collision{v}.mp4")) for v in VIEWS] for p in PROFILES]
    for row in caps:
        for c in row:
            if not c.isOpened():
                sys.exit("cannot open one of the input videos")

    W, H, fps = 1280, 720, 25
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{3 * W}x{3 * H}", "-r", str(fps), "-i", "-",
           "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
           str(out_path)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    font = cv2.FONT_HERSHEY_SIMPLEX

    n = 0
    while True:
        frames = []
        ok = True
        for row in caps:
            for c in row:
                ret, f = c.read()
                if not ret:
                    ok = False
                    break
                frames.append(f)
            if not ok:
                break
        if not ok:
            break

        grid = np.vstack([np.hstack([frames[p * 3 + i] for p in range(3)]) for i in range(3)])
        for vi, lab in enumerate(COL_LABEL):
            x = vi * W + 24
            cv2.putText(grid, lab, (x, 42), font, 1.3, (0, 0, 0), 6, cv2.LINE_AA)
            cv2.putText(grid, lab, (x, 42), font, 1.3, (255, 255, 255), 2, cv2.LINE_AA)
        for ri, lab in enumerate(ROW_LABEL):
            y = ri * H + 84
            cv2.putText(grid, lab, (24, y), font, 1.3, (0, 0, 0), 6, cv2.LINE_AA)
            cv2.putText(grid, lab, (24, y), font, 1.3, (255, 255, 255), 2, cv2.LINE_AA)

        proc.stdin.write(np.ascontiguousarray(cv2.cvtColor(grid, cv2.COLOR_BGR2RGB)).tobytes())
        n += 1

    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError("ffmpeg failed")
    for row in caps:
        for c in row:
            c.release()
    print(f"wrote {out_path} ({n} frames @ {fps} fps, {3 * W}x{3 * H})")


if __name__ == "__main__":
    main()
