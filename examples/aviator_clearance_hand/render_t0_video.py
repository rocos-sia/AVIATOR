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

# Named camera presets: (azimuth, elevation, distance, lookat). Wheel + hands sit at
# world x≈-0.44, y≈±0.13 (left/right handle), z≈0. Use --views to render a subset,
# or --views all for every preset below.
VIEWS = {
    "front": (180.0, 15.0, 1.8, "-0.8,0,-0.05"),     # original wide context
    "wheel": (0.0,   20.0, 1.1, "-0.44,0,-0.01"),    # driver view, both hands
    "left":  (25.0,  25.0, 0.5, "-0.44,0.13,-0.01"), # left-hand close-up
    "right": (25.0,  25.0, 0.5, "-0.44,-0.13,-0.01"),# right-hand close-up
    "top":   (0.0,   85.0, 0.9, "-0.44,0,-0.01"),    # top-down, see finger wrap
    "side":  (90.0,  15.0, 1.2, "-0.44,0,-0.01"),    # side profile
}


def load_model(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    d.qvel[:] = 0.0
    return m, d


def joint_ids(m):
    def jid(name):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    ids = {
        "roll": jid("roll_input_joint"),
        "pitch": jid("pitch_input_joint"),
        "qL": [jid(f"AR5-5_07L-W4C4A2_joint_{j + 1}") for j in range(7)],
        "qR": [jid(f"AR5-5_07R-W4C4A2_joint_{j + 1}") for j in range(7)],
    }
    # RH56E2 hand: 12 joints per side. 6 independent drivers (thumb_1/2, *_1) and 6
    # mimic joints driven by the `<equality>` polycoef multipliers below.
    for side in ("left", "right"):
        for f in ("index", "middle", "ring", "little"):
            ids[f"{side}_{f}_1"] = jid(f"{side}_{f}_1_joint")
            ids[f"{side}_{f}_2"] = jid(f"{side}_{f}_2_joint")
        for j in (1, 2, 3, 4):
            ids[f"{side}_thumb_{j}"] = jid(f"{side}_thumb_{j}_joint")
    return ids


# Grip pose (rad). Fingers curl around the steering-wheel rim; the thumb opposes
# across the rim. The mimic joints (thumb_3/4, *_2) are set explicitly to their
# equality multipliers; mj_forward computes constraints but does not project qpos.
GRIP = {
    "flex": 1.2,       # index/middle/ring/little _1
    "thumb_1": 0.0,    # upright thumb; 1.3 rad turns it sideways on this mounting
    "thumb_2": 0.55,   # thumb flexion
}
MIMIC = {
    "thumb_3": 0.8392, "thumb_4": 0.891,
    "index_2": 1.0843, "middle_2": 1.0843, "ring_2": 1.0843, "little_2": 1.0843,
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
    # close both hands into the grip pose (fingers wrap the rim, thumb opposes)
    for side in ("left", "right"):
        for f in ("index", "middle", "ring", "little"):
            d.qpos[m.jnt_qposadr[ids[f"{side}_{f}_1"]]] = GRIP["flex"]
            d.qpos[m.jnt_qposadr[ids[f"{side}_{f}_2"]]] = MIMIC[f"{f}_2"] * GRIP["flex"]
        d.qpos[m.jnt_qposadr[ids[f"{side}_thumb_1"]]] = GRIP["thumb_1"]
        d.qpos[m.jnt_qposadr[ids[f"{side}_thumb_2"]]] = GRIP["thumb_2"]
        d.qpos[m.jnt_qposadr[ids[f"{side}_thumb_3"]]] = MIMIC["thumb_3"] * GRIP["thumb_2"]
        d.qpos[m.jnt_qposadr[ids[f"{side}_thumb_4"]]] = MIMIC["thumb_4"] * MIMIC["thumb_3"] * GRIP["thumb_2"]
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
    ap.add_argument("--views", default="front",
                    help="comma-separated view names (or 'all'); see VIEWS dict")
    ap.add_argument("--suffix", default="", help="appended to each output filename before .mp4")
    args = ap.parse_args()

    views = [v.strip() for v in args.views.split(",") if v.strip()]
    if views == ["all"]:
        views = list(VIEWS.keys())
    for v in views:
        if v not in VIEWS:
            sys.exit(f"unknown view '{v}' (available: {', '.join(VIEWS)})")

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
        for vname in views:
            az, el, dist, lookat = VIEWS[vname]
            cam.lookat = np.array([float(x) for x in lookat.split(",")])
            cam.distance = dist
            cam.azimuth = az
            cam.elevation = el
            frames = render_frames(m, d, ids, cam, renderer, kf, dvals, walls, wall_rgba,
                                   TITLE[profile])
            encode(frames, outdir / f"t0_{profile}_collision_{vname}{args.suffix}.mp4",
                   args.fps, args.width, args.height)


if __name__ == "__main__":
    main()
