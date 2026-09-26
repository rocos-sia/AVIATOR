#!/usr/bin/env python3
"""Render a single cockpit + dual-arm grasp frame for the overview figure (Fig 1).

Reuses the camera / joint-id conventions of render_t0_vs_t5.py. Picks a mid
keyframe from keyframes_t5_roll_pull.csv (a moderate wheel roll + pull) and renders
one offscreen frame with the walls semi-transparent so the arms stay visible.

Run with the `mujo` env (mujoco 3.3.6):
  /home/rocos/miniconda3/envs/mujo/bin/python render_overview_frame.py
"""
import sys
from pathlib import Path
import numpy as np
import mujoco
import cv2

ROOT = Path('/home/rocos/sia/AVIATOR')
MODEL = ROOT / 'AviatorRobot' / 'build' / 'bin' / 'model' / 'aviator.xml'
KDIR = ROOT / 'AviatorRobot' / 'build' / 'out_g051'
OUT = ROOT / 'docs' / 'research' / 'figures'


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


def read_keyframes(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    cols = ["theta", "s"] + [f"qL{i}" for i in range(7)] + [f"qR{i}" for i in range(7)]
    return np.column_stack([rows[c] for c in cols])


def set_config(m, d, ids, kf):
    d.qpos[m.jnt_qposadr[ids["roll"]]] = kf[0]
    d.qpos[m.jnt_qposadr[ids["pitch"]]] = kf[1]
    for j in range(7):
        d.qpos[m.jnt_qposadr[ids["qL"][j]]] = kf[2 + j]
        d.qpos[m.jnt_qposadr[ids["qR"][j]]] = kf[9 + j]
    mujoco.mj_forward(m, d)


m, d = load_model(MODEL)
ids = joint_ids(m)

# semi-transparent walls so the arms read through them
for g in range(m.ngeom):
    body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
    if 'aviator_wall' in body:
        m.geom_rgba[g] = (0.45, 0.45, 0.52, 0.40)

kf = read_keyframes(KDIR / 'keyframes_t5_roll_pull.csv')
# pick a frame with a moderate roll (theta in [10, 30] deg) and mid pull
th = kf[:, 0] * 180 / np.pi
cand = np.where((th >= 10) & (th <= 30))[0]
i = cand[len(cand) // 2] if len(cand) else len(kf) // 2
print(f'[diag] picked kf {i}: theta={kf[i,0]*180/np.pi:+.1f} deg  s={kf[i,1]*1000:+.0f} mm')

cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.lookat = np.array([-0.8, 0.0, -0.05])
cam.distance = 2.1
cam.azimuth = 135.0
cam.elevation = 20.0

W, H = 1600, 900
m.vis.global_.offwidth = W
m.vis.global_.offheight = H
renderer = mujoco.Renderer(m, H, W)
set_config(m, d, ids, kf[i])
renderer.update_scene(d, camera=cam)
frame = renderer.render()  # RGB uint8 HxWx3

bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
out = OUT / 'fig1_render_cockpit.png'
cv2.imwrite(str(out), bgr)
print(f'WROTE {out}  ({W}x{H})')
