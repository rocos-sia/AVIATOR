#!/usr/bin/env python3
"""Render T0 (pure continuation TRAC-IK) trajectory videos showing wall penetration.

Reads keyframes_t0_<profile>.csv + trajectory_t0_<profile>.csv (written by the
aviator_clearance_trajectory tool's TASKS=record_t0 mode) and renders one MP4 per
profile with MuJoCo's offscreen renderer. The wall flashes red while the arm
penetrates it, and a live clearance readout (d_base) is overlaid.

Run with the `mujo` conda env (mujoco 3.3.6); system ffmpeg does the H.264 encode.

Example:
  render_t0_video.py --model reference/rocos-mujoco/model/aviator.xml \
      --dir build/out_g051 --out build/out_g051/videos
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
TITLE = {"sine": "T0 pure TRAC-IK  —  sine roll",
         "roll_pull": "T0 pure TRAC-IK  —  roll + pull",
         "random": "T0 pure TRAC-IK  —  irregular band-limited"}


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


def overlay(frame, title, d_mm):
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
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def render_frames(m, d, ids, cam, renderer, kf, dvals, walls, wall_rgba, title):
    frames = []
    for k in range(len(kf)):
        set_config(m, d, ids, kf[k])
        d_mm = float(dvals[k]) * 1e3
        rgba = (0.9, 0.1, 0.1, 0.7) if dvals[k] < 0.0 else wall_rgba
        for g in walls:
            m.geom_rgba[g] = rgba
        renderer.update_scene(d, camera=cam)
        frames.append(overlay(renderer.render(), title, d_mm))
    return frames


def encode(frames, out_path, fps, w, h):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
           "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
           str(out_path)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(f).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed for {out_path}")
    print(f"wrote {out_path} ({len(frames)} frames @ {fps} fps)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="walled aviator.xml")
    ap.add_argument("--dir", required=True, help="dir holding keyframes_t0_*.csv / trajectory_t0_*.csv")
    ap.add_argument("--out", required=True, help="output dir for the MP4s")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--azimuth", type=float, default=180.0)
    ap.add_argument("--elevation", type=float, default=15.0)
    ap.add_argument("--distance", type=float, default=1.8)
    ap.add_argument("--lookat", default="-0.8,0,-0.05")
    ap.add_argument("--suffix", default="", help="appended to each output filename before .mp4")
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

    for profile in PROFILES:
        kf_csv = directory / f"keyframes_t0_{profile}.csv"
        tr_csv = directory / f"trajectory_t0_{profile}.csv"
        if not (kf_csv.exists() and tr_csv.exists()):
            sys.exit(f"missing {kf_csv} or {tr_csv}")
        kf = read_keyframes(kf_csv)
        dvals = read_scalars(tr_csv, "d_base")
        assert len(kf) == len(dvals), f"row mismatch for {profile}"
        frames = render_frames(m, d, ids, cam, renderer, kf, dvals, walls, wall_rgba,
                               TITLE[profile])
        encode(frames, outdir / f"t0_{profile}_collision{args.suffix}.mp4", args.fps, args.width, args.height)


if __name__ == "__main__":
    main()
