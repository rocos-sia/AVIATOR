#!/usr/bin/env python3
"""Render the three T0 (pure TRAC-IK continuation) random-stream trajectories
(raw / rate-limited / smooth) at gap 0.55, each from three camera viewpoints —
the T0 counterpart of render_dp_three_streams.py, same cameras and style, so the
DP and T0 videos are directly comparable (and comparable with the T5 videos).

Inputs (keyframes_t0_*.csv / trajectory_t0_*.csv written by TASKS=record_t0*):
  raw   /tmp/t0_g55/keyframes_t0_random.csv        + trajectory_t0_random.csv
  rl    /tmp/t0_rl_g55/keyframes_t0_rl15_random.csv + trajectory_t0_rl15_random.csv
  sm    /tmp/t0_sm_g55/keyframes_t0_sm_random.csv   + trajectory_t0_sm_random.csv
Run with the `mujo` conda env (mujoco 3.3.6); system ffmpeg does the H.264 encode.

Example:
  render_t0_three_streams.py --model reference/rocos-mujoco/model/aviator_gap55.xml \
      --rawdir /tmp/t0_g55 --rldir /tmp/t0_rl_g55 --smdir /tmp/t0_sm_g55 \
      --out outputs/t0_videos
"""
import argparse
import subprocess
from pathlib import Path

import numpy as np
import mujoco
import cv2

D_SAFE = 0.005  # m

VIEWS = {
    "view1_front": dict(azimuth=180.0, elevation=15.0, distance=1.8),
    "view2_side": dict(azimuth=90.0, elevation=10.0, distance=1.8),
    "view3_top": dict(azimuth=180.0, elevation=60.0, distance=2.2),
}


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


def read_keyframes(path, stride):
    """Downsample keyframes to (N,16): [theta, s, qL0..qL6, qR0..qR6]."""
    rows = np.genfromtxt(path, delimiter=",", names=True)
    cols = ["theta", "s"] + [f"qL{i}" for i in range(7)] + [f"qR{i}" for i in range(7)]
    return np.column_stack([rows[c] for c in cols])[::stride]


def read_scalars(path, col, stride):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    return np.atleast_1d(rows[col])[::stride]


def set_config(m, d, ids, kf):
    d.qpos[m.jnt_qposadr[ids["roll"]]] = kf[0]
    d.qpos[m.jnt_qposadr[ids["pitch"]]] = kf[1]
    for j in range(7):
        d.qpos[m.jnt_qposadr[ids["qL"][j]]] = kf[2 + j]
        d.qpos[m.jnt_qposadr[ids["qR"][j]]] = kf[9 + j]
    mujoco.mj_forward(m, d)


def overlay(frame, title, th_deg, s_mm, d_mm):
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if d_mm < 0.0:
        color = (60, 60, 240)
        dtext = f"d = {d_mm:+.1f} mm  COLLISION"
    elif d_mm < D_SAFE:
        color = (0, 200, 255)
        dtext = f"d = {d_mm:+.1f} mm"
    else:
        color = (60, 220, 60)
        dtext = f"d = {d_mm:+.1f} mm  safe"
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 1, cv2.LINE_AA)
    stext = f"theta = {th_deg:+6.1f} deg   s = {s_mm:+5.0f} mm"
    cv2.putText(bgr, stext, (24, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(bgr, stext, (24, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bgr, dtext, (24, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 3, cv2.LINE_AA)
    cv2.putText(bgr, dtext, (24, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 1, cv2.LINE_AA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


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
    print(f"wrote {out_path} ({len(frames)} frames @ {fps} fps)", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="walled aviator_gap55.xml")
    ap.add_argument("--rawdir", required=True, help="dir with keyframes_t0_random.csv")
    ap.add_argument("--rldir", required=True, help="dir with keyframes_t0_rl15_random.csv")
    ap.add_argument("--smdir", required=True, help="dir with keyframes_t0_sm_random.csv")
    ap.add_argument("--out", required=True, help="output dir for the MP4s")
    ap.add_argument("--gap-label", default="gap 0.55", help="gap text shown in titles")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--kf-stride", type=int, default=20, help="render every Nth keyframe")
    args = ap.parse_args()

    m, d = load_model(args.model)
    m.vis.global_.offwidth = args.width
    m.vis.global_.offheight = args.height
    ids = joint_ids(m)
    walls = wall_geoms(m)
    wall_rgba = (0.35, 0.35, 0.42, 0.55)
    for g in walls:
        m.geom_rgba[g] = wall_rgba

    renderer = mujoco.Renderer(m, args.height, args.width)
    outdir = Path(args.out)
    lookat = np.array([-0.8, 0.0, -0.05])

    streams = [
        ("t0_raw", "T0 pure TRAC-IK, random stream RAW",
         Path(args.rawdir) / "keyframes_t0_random.csv",
         Path(args.rawdir) / "trajectory_t0_random.csv"),
        ("t0_rl", "T0 pure TRAC-IK, random stream RATE-LIMITED 1.5 rad/s",
         Path(args.rldir) / "keyframes_t0_rl15_random.csv",
         Path(args.rldir) / "trajectory_t0_rl15_random.csv"),
        ("t0_sm", "T0 pure TRAC-IK, random stream SMOOTH C-inf",
         Path(args.smdir) / "keyframes_t0_sm_random.csv",
         Path(args.smdir) / "trajectory_t0_sm_random.csv"),
    ]

    for tag, base_title, kf_csv, tr_csv in streams:
        kf_full = read_keyframes(kf_csv, 1)
        qfull = kf_full[:, 2:]
        vmax = float(np.max(np.abs(np.diff(qfull, axis=0))) / 0.02)
        title = f"{base_title}  (max {vmax:.2f} rad/s, {args.gap_label})"
        kf = kf_full[:: args.kf_stride]
        dvals = read_scalars(tr_csv, "d_base", args.kf_stride)
        assert len(kf) == len(dvals), f"row mismatch for {tag}"
        print(f"{tag}: {len(kf)} keyframes, max|dq|/dt={vmax:.3f} rad/s", flush=True)
        for vname, vp in VIEWS.items():
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            cam.lookat = lookat
            cam.distance = vp["distance"]
            cam.azimuth = vp["azimuth"]
            cam.elevation = vp["elevation"]
            frames = []
            for k in range(len(kf)):
                set_config(m, d, ids, kf[k])
                d_mm = float(dvals[k]) * 1e3
                rgba = (0.9, 0.1, 0.1, 0.7) if dvals[k] < 0.0 else wall_rgba
                for g in walls:
                    m.geom_rgba[g] = rgba
                renderer.update_scene(d, camera=cam)
                frames.append(overlay(renderer.render(), title,
                                      float(kf[k][0]) * 180 / np.pi, float(kf[k][1]) * 1e3, d_mm))
            encode(frames, outdir / f"{tag}_{vname}.mp4", args.fps, args.width, args.height)


if __name__ == "__main__":
    main()
