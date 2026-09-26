#!/usr/bin/env python3
"""Fig 1 overview — cockpit render + task/phase schematic + DP->policy->filter pipeline.

Composes the MuJoCo render (fig1_render_cockpit.png) with two matplotlib schematic
panels. Publication style via the scientific-visualization skill.

Run:  /home/rocos/miniconda3/envs/serl_clean/bin/python fig_overview.py
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle, FancyBboxPatch, Arc

SKILL = Path('/home/rocos/.claude/skills/scientific-visualization')
sys.path.insert(0, str(SKILL / 'scripts'))
from style_presets import apply_publication_style, set_color_palette
from figure_export import save_publication_figure

apply_publication_style('nature')
set_color_palette('okabe_ito')

LEFT  = '#0072B2'
RIGHT = '#D55E00'
TASK  = '#CC79A7'
SAFE  = '#404040'

ROOT = Path('/home/rocos/sia/AVIATOR')
OUT = ROOT / 'docs' / 'research' / 'figures'

render = plt.imread(OUT / 'fig1_render_cockpit.png')

fig = plt.figure(figsize=(7.2, 3.0))
gs = fig.add_gridspec(1, 3, width_ratios=[1.7, 1.05, 1.1], wspace=0.32)

# ---- (a) render ------------------------------------------------------------
ax = fig.add_subplot(gs[0, 0])
ax.imshow(render)
ax.set_xticks([]); ax.set_yticks([])
for s in ax.spines.values():
    s.set_visible(False)
ax.set_title('a  Cockpit: dual-arm grasp in the gap', fontsize=8)

# ---- (b) task coords + phases ---------------------------------------------
ax = fig.add_subplot(gs[0, 1])
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')

# steering yoke: a column (vertical bar) + wheel (rounded bar)
ax.add_patch(FancyBboxPatch((0.42, 0.10), 0.16, 0.42,
                            boxstyle='round,pad=0.01,rounding_size=0.03',
                            fc='#d9d9d9', ec='#404040', lw=1.2))          # column
ax.add_patch(FancyBboxPatch((0.24, 0.52), 0.52, 0.30,
                            boxstyle='round,pad=0.01,rounding_size=0.10',
                            fc='#f2f2f2', ec='#404040', lw=1.4))          # wheel
ax.text(0.5, 0.85, 'θ  roll', ha='center', va='bottom', fontsize=8, color=TASK)
# roll arrow (curved)
ax.add_patch(Arc((0.5, 0.67), 0.10, 0.10, theta1=-30, theta2=200, color=TASK, lw=1.6))
ax.annotate('', xy=(0.5, 0.82), xytext=(0.5, 0.53),
            arrowprops=dict(arrowstyle='-|>', color=TASK, lw=1.6))
# pull arrow
ax.annotate('', xy=(0.76, 0.32), xytext=(0.88, 0.32),
            arrowprops=dict(arrowstyle='-|>', color=SAFE, lw=1.6))
ax.text(0.90, 0.30, 's  pull', fontsize=8, color=SAFE, va='center')

# grips + self-motion phases
for x, c, lab in [(0.30, LEFT, 'φ$_L$'), (0.70, RIGHT, 'φ$_R$')]:
    ax.add_patch(Circle((x, 0.62), 0.05, fc=c, ec='k', lw=1.0, alpha=0.9))
    ax.add_patch(Arc((x, 0.62), 0.16, 0.16, theta1=10, theta2=300, color=c, lw=1.4))
    ax.text(x, 0.40, lab, ha='center', fontsize=8, color=c)
ax.text(0.5, 0.02, 'task (θ, s)  +  self-motion (φ$_L$, φ$_R$)',
        ha='center', va='bottom', fontsize=7, color='#404040')
ax.set_title('b  Task & redundancy', fontsize=8)

# ---- (c) pipeline ----------------------------------------------------------
ax = fig.add_subplot(gs[0, 2])
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')

boxes = [
    ('DP demonstrations', 'safe trajectories\n(θ, s, φ) from LUT', '#e3e3e3'),
    ('policy π', 'nominal action\na$_{nom}$', '#e3e3e3'),
    ('safety filter', 'manifold lookup\nleast-intervention', '#fde8d0'),
    ('dual-arm execution', '7-DOF redundant\nq$_L$, q$_R$', '#d7e8f7'),
]
y = 0.78
for i, (title, sub, fc) in enumerate(boxes):
    ax.add_patch(FancyBboxPatch((0.15, y), 0.70, 0.15,
                                boxstyle='round,pad=0.01,rounding_size=0.02',
                                fc=fc, ec='#404040', lw=1.1))
    ax.text(0.5, y + 0.105, title, ha='center', va='center', fontsize=8, weight='bold')
    ax.text(0.5, y + 0.05, sub, ha='center', va='center', fontsize=5.6, color='#333')
    if i < 3:
        ax.annotate('', xy=(0.5, y - 0.035), xytext=(0.5, y - 0.005),
                    arrowprops=dict(arrowstyle='-|>', color='#404040', lw=1.4))
    y -= 0.235
ax.set_title('c  Safe-manifold policy stack', fontsize=8)

save_publication_figure(fig, OUT / 'fig1_overview', formats=['pdf', 'png'], dpi=600)
plt.close(fig)
print('WROTE fig1_overview')
