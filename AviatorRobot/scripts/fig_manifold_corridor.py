#!/usr/bin/env python3
"""AVIATOR results figures 2 / 3 / 5 (all from real on-disk data).

  Fig 2  task-space feasibility map  — phi-redundant clearance atlas D_g(theta,s)
                                       + 5 mm boundary + 3 DP demo trajectories
  Fig 3  safe phi corridor           — along one combined trajectory: per-arm
                                       [phi - m_minus, phi + m_plus] bands + phi(t),
                                       plus d_min(t) vs 5 mm
  Fig 5  typical-trajectory quads     — roll / pull / combined, 4 rows:
                                       wheel pose, phases, clearance, peak joint speed

Data:
  hil-serl/data/aviator/manifold_phi/{dL,dR}.bin     (ns x nt x nphi)
  AviatorRobot/build/out_dp_v2/dp_demo/traj_*.csv    (100 Hz demos)

Run:  /home/rocos/miniconda3/envs/serl_clean/bin/python fig_manifold_corridor.py
"""
import sys, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SKILL = Path('/home/rocos/.claude/skills/scientific-visualization')
sys.path.insert(0, str(SKILL / 'scripts'))
from style_presets import apply_publication_style, set_color_palette
from figure_export import save_publication_figure

apply_publication_style('nature')
set_color_palette('okabe_ito')

# User colour scheme: blue = left arm, orange = right arm, purple = shared task,
# dark gray = safety boundary.
LEFT  = '#0072B2'
RIGHT = '#D55E00'
TASK  = '#CC79A7'
SAFE  = '#404040'
GRAY  = '#909090'

ROOT = Path('/home/rocos/sia/AVIATOR')
OUT  = ROOT / 'docs' / 'research' / 'figures'
MP   = ROOT / 'hil-serl' / 'data' / 'aviator' / 'manifold_phi'
DPD  = ROOT / 'AviatorRobot' / 'build' / 'out_dp_v2' / 'dp_demo'

# ----------------------------------------------------------------------------
# Manifold
# ----------------------------------------------------------------------------
mani = json.load(open(MP / 'manifest.json'))
nt, ns, nphi = mani['n_theta'], mani['n_s'], mani['n_phi']
theta = np.linspace(mani['th_min'], mani['th_max'], nt) * 180 / np.pi   # deg
s_mm   = np.linspace(mani['s_min'], mani['s_max'], ns) * 1000           # mm
dL = np.fromfile(MP / 'dL.bin', dtype=np.float32).reshape(ns, nt, nphi)
dR = np.fromfile(MP / 'dR.bin', dtype=np.float32).reshape(ns, nt, nphi)
Dg = np.minimum(dL, dR).max(axis=2) * 1000        # mm; rows=s(65), cols=theta(101)
d_safe = mani['d_safe'] * 1000                    # 5 mm

feasible_frac = (Dg >= d_safe).mean()
print(f'[diag] phi-redundant atlas: Dg in [{Dg.min():.2f}, {Dg.max():.2f}] mm, '
      f'feasible (>=5mm) = {feasible_frac*100:.1f}% of grid')

# ----------------------------------------------------------------------------
# DP demo trajectories
# ----------------------------------------------------------------------------
COLS = ['t', 'theta', 's', 'theta_dot', 's_dot',
        'sin_phi_L', 'cos_phi_L', 'sin_phi_R', 'cos_phi_R', 'phi_dot_L', 'phi_dot_R',
        *[f'qL{i}' for i in range(1, 8)], *[f'qR{i}' for i in range(1, 8)],
        *[f'qdotL{i}' for i in range(1, 8)], *[f'qdotR{i}' for i in range(1, 8)],
        'dL', 'dR', 'd_min', 'm_minus_L', 'm_minus_R', 'm_plus_L', 'm_plus_R', 'm_q']


def load_traj(name):
    data = {c: [] for c in COLS}
    with open(DPD / name) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split(',')
            if p[0] == 't':
                continue
            for c, v in zip(COLS, p):
                data[c].append(float(v))
    for c in COLS:
        data[c] = np.asarray(data[c])
    data['phi_L'] = np.arctan2(data['sin_phi_L'], data['cos_phi_L'])
    data['phi_R'] = np.arctan2(data['sin_phi_R'], data['cos_phi_R'])
    qd = np.column_stack([data[f'qdotL{i}'] for i in range(1, 8)]
                         + [data[f'qdotR{i}'] for i in range(1, 8)])
    data['max_qd'] = np.abs(qd).max(axis=1)
    return data


TRAJ = {'roll': 'traj_0025.csv', 'pull': 'traj_0031.csv', 'combine': 'traj_0011.csv'}
tr = {k: load_traj(f) for k, f in TRAJ.items()}

for k in tr:
    d = tr[k]
    span_th = (d['theta'].max() - d['theta'].min()) * 180 / np.pi
    span_s = (d['s'].max() - d['s'].min()) * 1000
    print(f'[diag] {k:7s} theta {d["theta"].min()*180/np.pi:+.0f}..'
          f'{d["theta"].max()*180/np.pi:+.0f} deg  s {d["s"].min()*1000:.0f}..'
          f'{d["s"].max()*1000:.0f} mm  d_min {d["d_min"].min()*1000:.2f} mm  '
          f'max|qd| {d["max_qd"].max():.2f} rad/s')

# ============================================================================
# Fig 2 — task-space feasibility map
# ============================================================================
fig, ax = plt.subplots(figsize=(3.5, 3.1))
im = ax.imshow(Dg, origin='lower', aspect='auto', cmap='viridis',
               extent=[theta[0], theta[-1], s_mm[0], s_mm[-1]])
ax.contour(theta, s_mm, Dg, levels=[d_safe], colors=SAFE,
           linewidths=1.1, linestyles='--')
cb = fig.colorbar(im, ax=ax, pad=0.02)
cb.set_label('D$_g$ (mm)', fontsize=7)
for k in TRAJ:
    ax.plot(tr[k]['theta'] * 180 / np.pi, tr[k]['s'] * 1000,
            color=TASK, lw=1.2, alpha=0.85)
    ax.annotate(k, (tr[k]['theta'][-1] * 180 / np.pi, tr[k]['s'][-1] * 1000),
                textcoords='offset points', xytext=(4, 0), fontsize=6, color=TASK)
ax.axhline(0, color='w', lw=0.4, alpha=0.5)
ax.set_xlabel('Roll angle θ (°)')
ax.set_ylabel('Pull s (mm)')
ax.set_title('Task-space feasibility (φ-redundant)', fontsize=8)
save_publication_figure(fig, OUT / 'fig2_feasibility', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

# ============================================================================
# Fig 3 — safe phi corridor (signature)
# ============================================================================
d = tr['combine']
t = d['t']
fig, axes = plt.subplots(2, 1, figsize=(3.6, 3.3), sharex=True,
                         gridspec_kw={'height_ratios': [1.7, 1]})
ax = axes[0]
ax.fill_between(t, d['phi_L'] - d['m_minus_L'], d['phi_L'] + d['m_plus_L'],
                color=LEFT, alpha=0.16, lw=0)
ax.plot(t, d['phi_L'], color=LEFT, lw=1.3, label='φ$_L$ (left arm)')
ax.fill_between(t, d['phi_R'] - d['m_minus_R'], d['phi_R'] + d['m_plus_R'],
                color=RIGHT, alpha=0.16, lw=0)
ax.plot(t, d['phi_R'], color=RIGHT, lw=1.3, label='φ$_R$ (right arm)')
ax.set_ylabel('self-motion φ (rad)')
ax.set_title('a  Safe φ corridor (combined demo)', fontsize=8)
ax.legend(fontsize=6, frameon=False, ncol=2, loc='upper right')
ax2 = axes[1]
ax2.plot(t, d['d_min'] * 1000, color=TASK, lw=1.2)
ax2.axhline(d_safe, color=SAFE, ls='--', lw=1.0)
ax2.set_ylabel('d$_{min}$ (mm)')
ax2.set_xlabel('time (s)')
ax2.set_title('b  Min clearance', fontsize=8)
for a in axes:
    a.tick_params(labelsize=6)
save_publication_figure(fig, OUT / 'fig3_phi_corridor', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

# ============================================================================
# Fig 5 — typical-trajectory quads
# ============================================================================
names = ['roll', 'pull', 'combine']
fig, axes = plt.subplots(4, 3, figsize=(7.2, 5.4), sharex='col')

for j, k in enumerate(names):
    dd = tr[k]
    tt = dd['t']

    # row 1: wheel pose  theta (left axis) + s (right axis)
    ax = axes[0, j]
    ax.plot(tt, dd['theta'] * 180 / np.pi, color=TASK, lw=1.2, label='θ')
    ax.set_ylabel('θ (°)', fontsize=7, color=TASK)
    ax.tick_params(axis='y', labelsize=6, colors=TASK)
    ax2 = ax.twinx()
    ax2.plot(tt, dd['s'] * 1000, color=GRAY, lw=1.1, ls=':', label='s')
    ax2.set_ylabel('s (mm)', fontsize=7, color=GRAY)
    ax2.tick_params(axis='y', labelsize=6, colors=GRAY)

    # row 2: phases
    ax = axes[1, j]
    ax.plot(tt, dd['phi_L'], color=LEFT, lw=1.1)
    ax.plot(tt, dd['phi_R'], color=RIGHT, lw=1.1)
    ax.set_ylabel('φ (rad)', fontsize=7)

    # row 3: clearance
    ax = axes[2, j]
    ax.plot(tt, dd['d_min'] * 1000, color=TASK, lw=1.1)
    ax.axhline(d_safe, color=SAFE, ls='--', lw=1.0)
    ax.set_ylabel('d$_{min}$ (mm)', fontsize=7)

    # row 4: peak joint speed
    ax = axes[3, j]
    ax.plot(tt, dd['max_qd'], color='#009E73', lw=1.1)
    ax.axhline(1.5, color=SAFE, ls='--', lw=1.0)
    ax.set_ylabel('max |q̇| (rad/s)', fontsize=7)
    ax.set_xlabel('time (s)', fontsize=7)

    axes[0, j].set_title(f'{k}', fontsize=8)

for ax in axes.flat:
    ax.tick_params(labelsize=6)

fig.tight_layout(h_pad=0.4, w_pad=0.6)
save_publication_figure(fig, OUT / 'fig5_trajectory_quads', formats=['pdf', 'png'], dpi=600)
plt.close(fig)

print('WROTE fig2/fig3/fig5')
