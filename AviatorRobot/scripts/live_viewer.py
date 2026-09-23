#!/usr/bin/env python3
"""Live viewer for the AVIATOR RLPD training stream (ZMQ PUB/SUB, no files).

Subscribes to the actor's ZMQ PUB stream (see
``hil-serl/examples/experiments/aviator_manifold/live_stream.py``) and renders
each kinematic frame in MuJoCo.

Default backend is MuJoCo's interactive viewer (mouse orbit / zoom / pan, wall
colour tracks the live clearance: red = collision, cyan = below 5 mm, grey =
safe).  The latest d / theta / s / |qdot| is printed to the terminal at ~2 Hz.

    conda activate mujo
    python AviatorRobot/scripts/live_viewer.py

The actor enables the stream with::

    AVIATOR_LIVE_VIEWER=1 bash launch_rlpd.sh

``--backend cv2`` switches to a fixed-camera window with a numeric overlay
(1/2/3 = front/side/top, q = quit) for displays without a working GL context.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer  # noqa: F401  (launch_passive is a lazy submodule)
import numpy as np
import zmq

D_SAFE = 0.005  # m

# (label, azimuth, elevation) for the three fixed cv2 camera angles.
VIEWS = {
    ord("1"): ("front", 180.0, 15.0),
    ord("2"): ("side", 90.0, 10.0),
    ord("3"): ("top", 180.0, 60.0),
}


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


def wall_rgba(d_mm):
    if d_mm < 0.0:
        return (0.9, 0.1, 0.1, 0.7)
    if d_mm < D_SAFE * 1e3:
        return (0.0, 0.7, 0.8, 0.7)
    return (0.35, 0.35, 0.42, 0.55)


def set_config(m, d, ids, f):
    d.qpos[m.jnt_qposadr[ids["roll"]]] = f[0]
    d.qpos[m.jnt_qposadr[ids["pitch"]]] = f[1]
    for j in range(7):
        d.qpos[m.jnt_qposadr[ids["qL"][j]]] = f[2 + j]
        d.qpos[m.jnt_qposadr[ids["qR"][j]]] = f[9 + j]
    mujoco.mj_forward(m, d)


def status_line(f, d_mm, qdot):
    state = "COLLISION" if d_mm < 0.0 else ("<5mm" if d_mm < D_SAFE * 1e3 else "safe")
    return (f"\rtheta={f[0] * 180 / np.pi:+6.1f} deg   s={f[1] * 1e3:+6.1f} mm   "
            f"d={d_mm:+6.2f} mm   |qdot|={qdot:4.2f} rad/s   [{state}]  ")


def run_viewer_backend(args, m, d, ids, walls, sock):
    print("MuJoCo interactive viewer: left-drag rotate, right-drag pan, scroll zoom; "
          "close the window to exit", flush=True)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat = np.array([-0.8, 0.0, -0.05])
        viewer.cam.distance = 1.8
        viewer.cam.azimuth = 180.0
        viewer.cam.elevation = 15.0
        last_print = 0.0
        while viewer.is_running():
            try:
                raw = sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                raw = None
            if raw is not None:
                f = np.frombuffer(raw, dtype=np.float64)
                if f.size == 18:
                    set_config(m, d, ids, f)
                    d_mm = float(f[16]) * 1e3
                    qdot = float(f[17]) if f.size >= 18 else 0.0
                    for g in walls:
                        m.geom_rgba[g] = wall_rgba(d_mm)
                    now = time.monotonic()
                    if now - last_print > 0.5:
                        last_print = now
                        print(status_line(f, d_mm, qdot), end="", flush=True)
            viewer.sync()
            time.sleep(1 / 60)
    print()


def run_cv2_backend(args, m, d, ids, walls, sock):
    import cv2

    m.vis.global_.offwidth = args.width
    m.vis.global_.offheight = args.height
    renderer = mujoco.Renderer(m, args.height, args.width)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat = np.array([-0.8, 0.0, -0.05])
    cam.distance = 1.8
    view_key = ord("1")
    cam.azimuth, cam.elevation = VIEWS[view_key][1], VIEWS[view_key][2]

    def overlay(frame, d_mm, qdot, f0_deg, f1_mm):
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if d_mm < 0.0:
            color, dtext = (60, 60, 240), f"d = {d_mm:+.1f} mm  COLLISION"
        elif d_mm < D_SAFE * 1e3:
            color, dtext = (0, 200, 255), f"d = {d_mm:+.1f} mm  (< {D_SAFE * 1e3:.0f} mm)"
        else:
            color, dtext = (60, 220, 60), f"d = {d_mm:+.1f} mm  safe"
        title = f"AVIATOR live  [{VIEWS[view_key][0]}]"
        cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(bgr, title, (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 1, cv2.LINE_AA)
        stext = f"theta = {f0_deg:+6.1f} deg   s = {f1_mm:+5.0f} mm   |qdot| = {qdot:.2f} rad/s"
        cv2.putText(bgr, stext, (24, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(bgr, stext, (24, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(bgr, dtext, (24, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 3, cv2.LINE_AA)
        cv2.putText(bgr, dtext, (24, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 1, cv2.LINE_AA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    print("cv2 window: 1/2/3 = front/side/top, q = quit", flush=True)
    cv2.namedWindow("aviator live", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("aviator live", args.width, args.height)
    renderer.update_scene(d, camera=cam)
    cv2.imshow("aviator live", cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)

    while True:
        if not sock.poll(timeout=30):
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            continue
        raw = sock.recv()
        f = np.frombuffer(raw, dtype=np.float64)
        if f.size != 18:
            continue
        set_config(m, d, ids, f)
        d_mm = float(f[16]) * 1e3
        qdot = float(f[17]) if f.size >= 18 else 0.0
        f0_deg = float(f[0]) * 180 / np.pi
        f1_mm = float(f[1]) * 1e3
        for g in walls:
            m.geom_rgba[g] = wall_rgba(d_mm)
        renderer.update_scene(d, camera=cam)
        img = overlay(renderer.render(), d_mm, qdot, f0_deg, f1_mm)
        cv2.imshow("aviator live", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key in VIEWS:
            view_key = key
            cam.azimuth, cam.elevation = VIEWS[key][1], VIEWS[key][2]

    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    default_model = (Path(__file__).resolve().parents[2] /
                     "reference" / "rocos-mujoco" / "model" / "aviator.xml")
    ap.add_argument("--model", default=str(default_model),
                    help="walled aviator MJCF matching the trained gap (0.51 default)")
    ap.add_argument("--addr", default="tcp://127.0.0.1:5557")
    ap.add_argument("--backend", default="viewer", choices=["viewer", "cv2"],
                    help="viewer = MuJoCo interactive window; cv2 = fixed camera + text overlay")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    d.qvel[:] = 0.0
    ids = joint_ids(m)
    walls = wall_geoms(m)
    for g in walls:
        m.geom_rgba[g] = (0.35, 0.35, 0.42, 0.55)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.CONFLATE, 1)   # keep only the latest frame
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.connect(args.addr)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"listening on {args.addr}  (model {args.model})", flush=True)

    if args.backend == "viewer":
        try:
            run_viewer_backend(args, m, d, ids, walls, sock)
        except Exception as e:  # e.g. no GL context on a headless display
            print(f"MuJoCo viewer failed ({e}); retry with --backend cv2 "
                  f"or use an MJPEG/HTTP viewer", flush=True)
            raise
    else:
        run_cv2_backend(args, m, d, ids, walls, sock)

    sock.close(0)
    ctx.term()


if __name__ == "__main__":
    main()
