#!/usr/bin/env python3
"""Render a t6 sine-trajectory MP4 from the aviatorT6Sine output CSV.

Reads sine.csv (written by AviatorRobot/src/t6_sine_main.cpp) and renders the
actual measured robot motion with MuJoCo's offscreen renderer, overlaying the
LUT-predicted clearance, the theta/s tracking error, and an INTERVENE badge on
the (rare) cycles where the policy command was scaled down.

Run with the `mujo` conda env (mujoco 3.3.6); system ffmpeg does the H.264 encode.

Example:
  render_t6_sine_video.py --csv /tmp/aviator_t6_video/sine.csv \
      --model reference/rocos-mujoco/build/aviator-visual/bin/model/aviator.xml \
      --out /tmp/aviator_t6_video/t6_sine.mp4
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import mujoco
import cv2

D_SAFE = 0.005  # m


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


def read_csv(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    cols = ["theta_meas", "s_meas"] + [f"q_meas_{i}" for i in range(14)]
    kf = np.column_stack([rows[c] for c in cols])          # (N, 16): theta,s,qL0..6,qR0..6
    dvals = np.atleast_1d(rows["lut_clearance"])
    theta_err = rows["theta_meas"] - rows["theta_ref"]
    s_err = rows["s_meas"] - rows["s_ref"]
    intervened = np.atleast_1d(rows["intervened"]).astype(bool)
    return kf, dvals, theta_err, s_err, intervened


def set_config(m, d, ids, kf):
    d.qpos[m.jnt_qposadr[ids["roll"]]] = kf[0]
    d.qpos[m.jnt_qposadr[ids["pitch"]]] = kf[1]
    for j in range(7):
        d.qpos[m.jnt_qposadr[ids["qL"][j]]] = kf[2 + j]
        d.qpos[m.jnt_qposadr[ids["qR"][j]]] = kf[9 + j]
    mujoco.mj_forward(m, d)


def overlay(frame, d_mm, theta_err, s_err, intervened):
    """frame is RGB uint8 (H,W,3)."""
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    title = "t6 (160k actor)  —  sine wheel trajectory"
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 1, cv2.LINE_AA)

    if d_mm * 1e3 < D_SAFE:
        color = (60, 60, 240)
        text = f"d = {d_mm * 1e3:+.1f} mm  COLLISION"
    elif d_mm * 1e3 < 2 * D_SAFE:
        color = (0, 200, 255)
        text = f"d = {d_mm * 1e3:+.1f} mm"
    else:
        color = (60, 220, 60)
        text = f"d = {d_mm * 1e3:+.1f} mm  safe"
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA)
    cv2.putText(bgr, text, (24, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 1, cv2.LINE_AA)

    track = (f"theta err {theta_err * 1e3:+.1f} mrad   "
             f"s err {s_err * 1e3:+.2f} mm")
    cv2.putText(bgr, track, (24, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(bgr, track, (24, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (220, 220, 220), 1, cv2.LINE_AA)

    if intervened:
        cv2.putText(bgr, "INTERVENE", (24, 176), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(bgr, "INTERVENE", (24, 176), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (60, 60, 240), 2, cv2.LINE_AA)
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
    print(f"wrote {out_path} ({len(frames)} frames @ {fps} fps)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="sine.csv from aviatorT6Sine")
    ap.add_argument("--model", required=True, help="aviator.xml")
    ap.add_argument("--out", required=True, help="output MP4 path")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--azimuth", type=float, default=180.0)
    ap.add_argument("--elevation", type=float, default=15.0)
    ap.add_argument("--distance", type=float, default=1.8)
    ap.add_argument("--lookat", default="-0.8,0,-0.05")
    ap.add_argument("--preview", default="", help="if set, write one PNG frame here and exit")
    args = ap.parse_args()

    m, d = load_model(args.model)
    m.vis.global_.offwidth = args.width
    m.vis.global_.offheight = args.height
    ids = joint_ids(m)
    walls = wall_geoms(m)
    for g in walls:
        m.geom_rgba[g] = (0.35, 0.35, 0.42, 0.55)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat = np.array([float(v) for v in args.lookat.split(",")])
    cam.distance = args.distance
    cam.azimuth = args.azimuth
    cam.elevation = args.elevation

    kf, dvals, theta_err, s_err, intervened = read_csv(args.csv)

    # CSV is 10 ms per sample (100 Hz); subsample to the target fps for real-time playback.
    stride = max(1, int(round(100 / args.fps)))
    idx = np.arange(0, len(kf), stride)

    renderer = mujoco.Renderer(m, args.height, args.width)

    def render_frame(i):
        set_config(m, d, ids, kf[i])
        renderer.update_scene(d, camera=cam)
        return overlay(renderer.render(), float(dvals[i]), float(theta_err[i]),
                       float(s_err[i]), bool(intervened[i]))

    if args.preview:
        cv2.imwrite(args.preview, cv2.cvtColor(render_frame(idx[len(idx) // 2]),
                                               cv2.COLOR_RGB2BGR))
        print(f"wrote preview {args.preview}")
        return

    frames = [render_frame(i) for i in idx]
    encode(frames, args.out, args.fps, args.width, args.height)


if __name__ == "__main__":
    main()
