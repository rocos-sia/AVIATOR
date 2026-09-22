#!/usr/bin/env python3
"""Render side-by-side T0 (pure TRAC-IK) vs T5 (safe-manifold + Newton) comparison videos.

Reads keyframes_t0_<profile>.csv / trajectory_t0_<profile>.csv (left panel) and
keyframes_t5_<profile>.csv / trajectory_t5_<profile>.csv (right panel) — written by the
aviator_clearance_trajectory tool's TASKS=record_t0 / record_t5 modes — and renders one
MP4 per profile with the two arm configurations side by side under the SAME camera, so the
only difference is the controller. Each panel's wall flashes red while that arm penetrates
it, and a live clearance readout is overlaid.

Run with the `mujo` conda env (mujoco 3.3.6); system ffmpeg does the H.264 encode.
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import mujoco
import cv2

D_SAFE = 0.005  # m
PROFILES = ["sine", "roll_pull", "random"]
TITLE = {"sine": "sine roll", "roll_pull": "roll + pull", "random": "irregular band-limited"}
LABEL_L = "T0  pure TRAC-IK"
LABEL_R = "T5  manifold + Newton"


def load_model(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    d.qvel[:] = 0.0
    return m, d


def joint_ids(m):
    def jid(name):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    return {
        "roll": jid("roll_input_joint"),
        "pitch": jid("pitch_input_joint"),
        "qL": [jid(f"AR5-5_07L-W4C4A2_joint_{j + 1}") for j in range(7)],
        "qR": [jid(f"AR5-5_07R-W4C4A2_joint_{j + 1}") for j in range(7)],
    }


def wall_geoms(m):
    return [g for g in range(m.ngeom)
            if "aviator_wall" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "")]


def read_keyframes(path):
    """(N, 16) float array: [theta, s, qL0..qL6, qR0..qR6]."""
    rows = np.genfromtxt(path, delimiter=",", names=True)
    cols = ["theta", "s"] + [f"qL{i}" for i in range(7)] + [f"qR{i}" for i in range(7)]
    return np.column_stack([rows[c] for c in cols])


def read_scalars(path, col):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    return np.atleast_1d(rows[col])


def set_config(m, d, ids, kf):
    d.qpos[m.jnt_qposadr[ids["roll"]]] = kf[0]
    d.qpos[m.jnt_qposadr[ids["pitch"]]] = kf[1]
    for j in range(7):
        d.qpos[m.jnt_qposadr[ids["qL"][j]]] = kf[2 + j]
        d.qpos[m.jnt_qposadr[ids["qR"][j]]] = kf[9 + j]
    mujoco.mj_forward(m, d)


def overlay_panel(frame, label, d_mm):
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if d_mm < 0.0:
        color = (60, 60, 240)
        text = f"d = {d_mm:+.1f} mm  COLLISION"
    elif d_mm * 1e3 < D_SAFE:
        color = (0, 200, 255)
        text = f"d = {d_mm:+.1f} mm"
    else:
        color = (60, 220, 60)
        text = f"d = {d_mm:+.1f} mm  safe"
    cv2.putText(bgr, label, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(bgr, label, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def render_side(m, d, ids, cam, renderer, kf, dvals, walls, wall_rgba, label):
    frame = None
    set_config(m, d, ids, kf[0])
    d_mm = float(dvals[0]) * 1e3
    rgba = (0.9, 0.1, 0.1, 0.7) if dvals[0] < 0.0 else wall_rgba
    for g in walls:
        m.geom_rgba[g] = rgba
    renderer.update_scene(d, camera=cam)
    return overlay_panel(renderer.render(), label, d_mm)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="walled aviator.xml")
    ap.add_argument("--dir", required=True, help="dir holding keyframes_t[05]_*.csv / trajectory_t[05]_*.csv")
    ap.add_argument("--out", required=True, help="output dir for the MP4s")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--azimuth", type=float, default=135.0)
    ap.add_argument("--elevation", type=float, default=20.0)
    ap.add_argument("--distance", type=float, default=2.1)
    ap.add_argument("--lookat", default="-0.8,0,-0.05")
    args = ap.parse_args()

    directory = Path(args.dir)
    m, d = load_model(args.model)
    m.vis.global_.offwidth = args.width
    m.vis.global_.offheight = args.height
    ids = joint_ids(m)
    walls = wall_geoms(m)
    wall_rgba = (0.35, 0.35, 0.42, 0.55)
    for g in walls:
        m.geom_rgba[g] = wall_rgba

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat = np.array([float(v) for v in args.lookat.split(",")])
    cam.distance = args.distance
    cam.azimuth = args.azimuth
    cam.elevation = args.elevation

    renderer = mujoco.Renderer(m, args.height, args.width)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    for profile in PROFILES:
        kf0 = read_keyframes(directory / f"keyframes_t0_{profile}.csv")
        d0 = read_scalars(directory / f"trajectory_t0_{profile}.csv", "d_base")
        kf5 = read_keyframes(directory / f"keyframes_t5_{profile}.csv")
        d5 = read_scalars(directory / f"trajectory_t5_{profile}.csv", "d_base")
        assert len(kf0) == len(d0) == len(kf5) == len(d5), f"row mismatch for {profile}"

        out_path = outdir / f"t0_vs_t5_{profile}.mp4"
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{2 * args.width}x{args.height}", "-r", str(args.fps), "-i", "-",
               "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
               str(out_path)]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

        n = 0
        for k in range(len(kf0)):
            left = render_side(m, d, ids, cam, renderer, kf0[k:k + 1], d0[k:k + 1],
                               walls, wall_rgba, LABEL_L)
            right = render_side(m, d, ids, cam, renderer, kf5[k:k + 1], d5[k:k + 1],
                                walls, wall_rgba, LABEL_R)
            grid = np.hstack([left, right])
            proc.stdin.write(np.ascontiguousarray(grid).tobytes())
            n += 1
        proc.stdin.close()
        if proc.wait() != 0:
            raise RuntimeError(f"ffmpeg failed for {out_path}")
        print(f"wrote {out_path} ({n} frames @ {args.fps} fps, {2 * args.width}x{args.height})")

    # optional global title strip is omitted; the per-panel labels carry the method names
    print(f"title rows: {', '.join(TITLE[p] for p in PROFILES)}")


if __name__ == "__main__":
    main()
