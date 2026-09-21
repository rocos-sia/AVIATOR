#!/usr/bin/env python3
"""Render walled-trajectory videos from aviator_clearance_trajectory outputs.

Reads trajectory_<tag>.csv and keyframes_<tag>_{base,b3}.csv (written by the
aviator_clearance_trajectory tool) and renders, with MuJoCo's offscreen renderer,
three MP4s:

  <tag>_collision.mp4      B0 baseline continuation IK  -> elbow penetrates the wall
  <tag>_nullspace.mp4      B3 least-intervention null-space reshaping -> stays safe
  <tag>_side_by_side.mp4   the two regimes side by side

The wall flashes red while an arm penetrates it, and a live clearance readout
(d_base / d_b3 taken from the trajectory CSV) is overlaid. Run with the `mujo`
conda env (mujoco 3.3.6); system ffmpeg does the H.264 encoding.

Example:
  render_trajectory_video.py --model /tmp/aviator-video/model/aviator.xml \
      --dir /tmp/aviator-video/out --tag roll_neg --out /tmp/aviator-video
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import mujoco
import cv2

D_SAFE = 0.005  # m, matches the trajectory tool default


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
    """Draw title + clearance readout. frame is RGB uint8 (H,W,3)."""
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if d_mm < 0.0:
        color = (60, 60, 240)      # red (BGR)
        text = f"d = {d_mm:+.1f} mm  COLLISION"
    elif d_mm * 1e3 < D_SAFE:
        color = (0, 200, 255)      # amber
        text = f"d = {d_mm:+.1f} mm"
    else:
        color = (60, 220, 60)      # green
        text = f"d = {d_mm:+.1f} mm  safe"
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def render_frames(m, d, ids, cam, renderer, kf, dvals, walls, wall_rgba, title):
    """Render one frame per keyframe, flashing the wall red while penetrating."""
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
    ap.add_argument("--dir", required=True, help="dir holding trajectory_<tag>.csv and keyframes_<tag>_*.csv")
    ap.add_argument("--tag", default="roll_neg")
    ap.add_argument("--out", required=True, help="output dir for the MP4s")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--azimuth", type=float, default=180.0)
    ap.add_argument("--elevation", type=float, default=15.0)
    ap.add_argument("--distance", type=float, default=1.8)
    ap.add_argument("--lookat", default="-0.8,0,-0.05",
                    help="camera look-at point, comma-separated x,y,z")
    args = ap.parse_args()

    directory = Path(args.dir)
    traj_csv = directory / f"trajectory_{args.tag}.csv"
    base_csv = directory / f"keyframes_{args.tag}_base.csv"
    b3_csv = directory / f"keyframes_{args.tag}_b3.csv"
    for p in (traj_csv, base_csv, b3_csv):
        if not p.exists():
            sys.exit(f"missing {p}")

    m, d = load_model(args.model)
    m.vis.global_.offwidth = args.width
    m.vis.global_.offheight = args.height
    ids = joint_ids(m)
    walls = wall_geoms(m)
    wall_rgba = tuple(m.geom_rgba[walls[0]]) if walls else (0.35, 0.35, 0.40, 0.55)
    for g in walls:
        m.geom_rgba[g] = (0.35, 0.35, 0.42, 0.55)  # keep walls clearly visible

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat = np.array([float(v) for v in args.lookat.split(",")])
    cam.distance = args.distance
    cam.azimuth = args.azimuth
    cam.elevation = args.elevation

    renderer = mujoco.Renderer(m, args.height, args.width)

    kb = read_keyframes(base_csv)
    kv = read_keyframes(b3_csv)
    d_base = read_scalars(traj_csv, "d_base")
    d_b3 = read_scalars(traj_csv, "d_b3")
    assert len(kb) == len(kv) == len(d_base) == len(d_b3), "row-count mismatch across CSVs"

    frames_base = render_frames(m, d, ids, cam, renderer, kb, d_base, walls, wall_rgba,
                                "B0 baseline continuation IK")
    frames_b3 = render_frames(m, d, ids, cam, renderer, kv, d_b3, walls, wall_rgba,
                              "B3 least-intervention null-space")

    outdir = Path(args.out)
    encode(frames_base, outdir / f"{args.tag}_collision.mp4", args.fps, args.width, args.height)
    encode(frames_b3, outdir / f"{args.tag}_nullspace.mp4", args.fps, args.width, args.height)

    side = [np.hstack([a, b]) for a, b in zip(frames_base, frames_b3)]
    encode(side, outdir / f"{args.tag}_side_by_side.mp4", args.fps, 2 * args.width, args.height)


if __name__ == "__main__":
    main()
