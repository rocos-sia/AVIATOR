#!/usr/bin/env python3
"""Publication figures for the AVIATOR safe-manifold approach.

Uses the scientific-visualization skill (style_presets / figure_export) and only
real on-disk data:
  - safe manifold Q(theta,s,phi)  (hil-serl/data/aviator/manifold_phi/)
  - orientation-slack velocity LP  (outputs/t5_pose_slack_200hz_20260922/final_v2/)
  - T0/T1/T4 online comparison     (AviatorRobot/build/out_g051/compare_*.csv)

Run with the serl_clean env:
  /home/rocos/miniconda3/envs/serl_clean/bin/python fig_manifold_and_results.py
"""
import sys, json
from pathlib import Path
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- scientific-visualization skill helpers ---------------------------------
SKILL = Path('/home/rocos/.claude/skills/scientific-visualization')
sys.path.insert(0, str(SKILL / 'scripts'))
from style_presets import apply_publication_style, set_color_palette
from figure_export import save_publication_figure

apply_publication_style('nature')
set_color_palette('okabe_ito')

OKABE = ['#E69F00', '#56B4E9', '#009E73', '#F0E442',
         '#0072B2', '#D55E00', '#CC79A7', '#000000']

ROOT = Path('/home/rocos/sia/AVIATOR')
OUT = ROOT / 'docs' / 'research' / 'figures'
MP = ROOT / 'hil-serl' / 'data' / 'aviator' / 'manifold_phi'

# =============================================================================
# 1. Load the safe manifold
# =============================================================================
mani = json.load(open(MP / 'manifest.json'))
nt, ns, nphi = mani['n_theta'], mani['n_s'], mani['n_phi']
th_min, th_max = mani['th_min'], mani['th_max']          # rad
s_min, s_max = mani['s_min'], mani['s_max']              # m
theta = np.linspace(th_min, th_max, nt) * 180 / np.pi    # deg
s_mm = np.linspace(s_min, s_max, ns) * 1000              # mm
phi = np.fromfile(MP / 'phi.bin', dtype=np.float32)

safe = np.fromfile(MP / 'safe.bin', dtype=np.float32).reshape(ns, nt, 4)  # [loL,hiL,loR,hiR]
dL = np.fromfile(MP / 'dL.bin', dtype=np.float32).reshape(ns, nt, nphi)
dR = np.fromfile(MP / 'dR.bin', dtype=np.float32).reshape(ns, nt, nphi)

# Simultaneously-safe phi interval (both arms): [max(lo), min(hi)]
lo_both = np.maximum(safe[..., 0], safe[..., 2])
hi_both = np.minimum(safe[..., 1], safe[..., 3])
width_both = hi_both - lo_both                     # <0 => no jointly-safe phi
width_L = safe[..., 1] - safe[..., 0]
width_R = safe[..., 3] - safe[..., 2]
Dg = np.minimum(dL, dR).max(axis=2) * 1000         # static max clearance [mm] over phi

d_safe_mm = mani['d_safe'] * 1000                  # 5 mm

# =============================================================================
# Fig 1 — the safe manifold Q(theta,s,phi)
# =============================================================================
fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.6),
                         gridspec_kw={'width_ratios': [1, 1, 1.15]})

# (a) clearance atlas D_g(theta,s)
ax = axes[0]
im = ax.imshow(Dg, origin='lower', aspect='auto', cmap='viridis',
               extent=[theta[0], theta[-1], s_mm[0], s_mm[-1]])
ax.contour(theta, s_mm, Dg, levels=[d_safe_mm], colors='white', linewidths=1.0)
ax.set_xlabel('Roll angle θ (°)')
ax.set_ylabel('Pull s (mm)')
ax.set_title('a  Clearance atlas D$_g$(θ,s)', fontsize=8)
cb = fig.colorbar(im, ax=ax, pad=0.02)
cb.set_label('D$_g$ (mm)', fontsize=7)

# (b) safe phi width (both arms) — redundancy availability
ax = axes[1]
w = np.ma.masked_where(width_both < 0, width_both) * 180 / np.pi   # to deg
im = ax.imshow(w, origin='lower', aspect='auto', cmap='plasma',
               extent=[theta[0], theta[-1], s_mm[0], s_mm[-1]])
# infeasible (no jointly-safe phi) -> hatched overlay
inf = (width_both < 0).astype(float)
ax.imshow(inf, origin='lower', aspect='auto', cmap='gray_r', vmin=0, vmax=1,
          alpha=0.0, extent=[theta[0], theta[-1], s_mm[0], s_mm[-1]])
ax.set_xlabel('Roll angle θ (°)')
ax.set_ylabel('Pull s (mm)')
ax.set_title('b  Jointly-safe φ range (°)', fontsize=8)
cb = fig.colorbar(im, ax=ax, pad=0.02)
cb.set_label('φ width (°)', fontsize=7)

# (c) self-motion arc: clearance vs phi at three representative points
ax = axes[2]
points = {'centre (0°, −80 mm)': (0, -80),
          'corner (+50°, −160 mm)': (50, -160),
          'corner (−50°, −160 mm)': (-50, -160)}
cols = [OKABE[2], OKABE[5], OKABE[0]]
for (label, (th, s)), c in zip(points.items(), cols):
    it = np.argmin(np.abs(theta - th)); is_ = np.argmin(np.abs(s_mm - s))
    d = np.minimum(dL[is_, it, :], dR[is_, it, :]) * 1000   # min over arms
    ax.plot(phi, d, color=c, lw=1.4, label=label)
ax.axhline(d_safe_mm, color='black', ls='--', lw=0.9)
ax.text(phi[0], d_safe_mm + 0.15, 'd$_{safe}$', fontsize=6, va='bottom')
ax.set_xlabel('Self-motion phase φ')
ax.set_ylabel('Clearance (mm)')
ax.set_title('c  Self-motion arc d(φ)', fontsize=8)
ax.legend(fontsize=6, loc='upper right')

for ax in axes:
    ax.tick_params(labelsize=6)

save_publication_figure(fig, OUT / 'fig1_safe_manifold', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

# =============================================================================
# Fig 2 — orientation slack reduces peak joint speed (pose-slack LP)
# =============================================================================
PS = ROOT / 'outputs' / 't5_pose_slack_200hz_20260922' / 'final_v2'
deg = [0, 1, 2, 3]
vL = [json.load(open(PS / f'pose_200hz_{d}deg_side0.json'))[-1]['vmax'] for d in deg]
vR = [json.load(open(PS / f'pose_200hz_{d}deg_side1.json'))[-1]['vmax'] for d in deg]

fig, ax = plt.subplots(figsize=(3.5, 2.6))
ax.plot(deg, vR, 'o-', color=OKABE[5], lw=1.8, ms=5, label='Right arm')
ax.plot(deg, vL, 's-', color=OKABE[2], lw=1.8, ms=5, label='Left arm')
ax.axhline(1.5, color='black', ls='--', lw=1.0)
ax.text(0.05, 1.55, 'real joint-speed limit (1.5 rad/s)', fontsize=6, va='bottom')
ax.set_xticks(deg)
ax.set_xlabel('Orientation tolerance α (°)')
ax.set_ylabel('Peak joint speed (rad/s)')
ax.set_ylim(0, 7)
for d, vr, vl in zip(deg, vR, vL):
    ax.annotate(f'{vr:.2f}', (d, vr), textcoords='offset points', xytext=(0, 5),
                ha='center', fontsize=6, color=OKABE[5])
    ax.annotate(f'{vl:.2f}', (d, vl), textcoords='offset points', xytext=(0, -12),
                ha='center', fontsize=6, color=OKABE[2])
ax.legend(fontsize=7, frameon=False)

save_publication_figure(fig, OUT / 'fig2_pose_slack_speed', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

# =============================================================================
# Fig 3 — T0/T1/T4 online comparison (safety + timing)
# =============================================================================
import csv
def read(fn):
    with open(ROOT / 'AviatorRobot' / 'build' / 'out_g051' / fn) as f:
        return list(csv.DictReader(f))

safety = read('compare_safety.csv')
timing = read('compare_timing.csv')

methods = ['T0_TRACIK', 'T1_TRACIK+coll', 'T4_atlas+reshape']
mlabels = ['T0\n(cont. IK)', 'T1\n(+collision)', 'T4\n(online reshape)']
profiles = ['sine', 'roll_pull', 'random']
plabels = ['sine', 'roll+push', 'random']

fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.6))

# (a) min clearance d_min
ax = axes[0]
x = np.arange(len(methods)); w = 0.25
for k, p in enumerate(profiles):
    dmin = [float(next(r['d_min_mm'] for r in safety if r['profile'] == p and r['method'] == m))
            for m in methods]
    ax.bar(x + (k - 1) * w, dmin, w, color=OKABE[k], label=plabels[k])
ax.axhline(5, color='black', ls='--', lw=1.0)
ax.text(0.02, 5.3, 'd$_{safe}$ = 5 mm', fontsize=6, va='bottom')
ax.set_xticks(x); ax.set_xticklabels(mlabels, fontsize=6)
ax.set_ylabel('Min clearance (mm)')
ax.set_title('a  Safety', fontsize=8)
ax.legend(fontsize=6, frameon=False)

# (b) timing P99
ax = axes[1]
for k, p in enumerate(profiles):
    t = [float(next(r['P99_ms'] for r in timing if r['profile'] == p and r['method'] == m))
         for m in ['T0_TRACIK', 'T1_TRACIK+coll', 'T4_atlas+reshape_active']]
    ax.plot(x, t, 'o-', color=OKABE[k], lw=1.5, ms=4, label=plabels[k])
ax.axhline(20, color='black', ls='--', lw=1.0)
ax.text(0.02, 21, '20 ms deadline', fontsize=6, va='bottom')
ax.set_xticks(x); ax.set_xticklabels(['T0', 'T1', 'T4'], fontsize=6)
ax.set_ylabel('P99 compute (ms)')
ax.set_yscale('log')
ax.set_title('b  Compute time', fontsize=8)
ax.legend(fontsize=6, frameon=False)

save_publication_figure(fig, OUT / 'fig3_methods_comparison', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

print('WROTE:')
for f in sorted(OUT.glob('fig*.pdf')):
    print(' ', f)
