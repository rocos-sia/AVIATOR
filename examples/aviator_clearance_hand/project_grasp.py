#!/usr/bin/env python3
"""Repair kinematic grasp closure in recorded trajectories without moving targets.

This is an offline IK projection, not a contact/dynamics simulation or the T0
baseline. Original recordings are retained; projected outputs use a separate dir.
"""
import argparse
import csv
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from render_t0_video import PROFILES, joint_ids, load_model, read_keyframes, set_config


def project_side(m, d, ids, side, previous):
    joints = ids['qL' if side == 'left' else 'qR']
    adr = m.jnt_qposadr[joints]
    tcp = m.site(side + '_tcp').id
    handle = m.site(side + '_handle').id
    target_p = d.site_xpos[handle].copy()
    target_R = d.site_xmat[handle].reshape(3, 3).copy()
    original = d.qpos[adr].copy()
    limits = m.jnt_range[joints]

    def residual(q):
        d.qpos[adr] = q
        # Only body/site kinematics are needed during IK; contact and constraint
        # assembly would dominate the cost of finite-difference iterations.
        mujoco.mj_kinematics(m, d)
        return np.r_[d.site_xpos[tcp] - target_p,
                     Rotation.from_matrix(target_R @ d.site_xmat[tcp].reshape(3, 3).T).as_rotvec()]

    def acceptable(e):
        return np.linalg.norm(e[:3]) <= 1e-6 and np.linalg.norm(e[3:]) <= 1e-5

    initial = residual(original)
    if acceptable(initial):
        return original, initial, False
    # Continue from the previous projected configuration to avoid freezing at a
    # failed baseline knot. Use the recorded configuration as a second seed.
    for seed in ([previous, original] if previous is not None else [original]):
        result = least_squares(residual, np.clip(seed, limits[:, 0] + 1e-10,
                                                limits[:, 1] - 1e-10),
                               bounds=(limits[:, 0], limits[:, 1]),
                               xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=1000)
        error = residual(result.x)
        if acceptable(error):
            return result.x, error, True
    d.qpos[adr] = original
    mujoco.mj_forward(m, d)
    raise RuntimeError(f'{side} grasp projection failed; position={np.linalg.norm(error[:3])*1000:.6f} mm, '
                       f'orientation={np.degrees(np.linalg.norm(error[3:])):.6f} deg')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', default='models/mjcf/aviator.xml')
    ap.add_argument('--dir', default='examples/aviator_clearance_hand/out')
    ap.add_argument('--out', default='outputs/hand_mount_check/projected')
    args = ap.parse_args()
    source, out = Path(args.dir).resolve(), Path(args.out).resolve()
    if source == out:
        ap.error('--out must differ from --dir to retain the baseline')
    out.mkdir(parents=True, exist_ok=True)
    m, d = load_model(args.model)
    ids = joint_ids(m)
    fields = ['k', 'theta', 's'] + [f'qL{i}' for i in range(7)] + [f'qR{i}' for i in range(7)]
    for profile in PROFILES:
        rows = read_keyframes(source / f'keyframes_t0_{profile}.csv')
        repaired, diagnostics = [], []
        previous = [None, None]
        for k, row in enumerate(rows):
            set_config(m, d, ids, row)
            row = row.copy()
            for i, side in enumerate(('left', 'right')):
                try:
                    q, e, changed = project_side(m, d, ids, side, previous[i])
                except RuntimeError as exc:
                    raise RuntimeError(f'{profile} frame {k}: {exc}') from exc
                row[2 + i*7:9 + i*7] = q
                previous[i] = q.copy()
                diagnostics.append([k, side, int(changed), np.linalg.norm(e[:3])*1000,
                                    np.degrees(np.linalg.norm(e[3:]))])
            repaired.append([k, *row])
        with (out / f'keyframes_projected_{profile}.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(fields); writer.writerows(repaired)
        with (out / f'closure_projected_{profile}.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['k', 'side', 'projected', 'position_mm', 'orientation_deg'])
            writer.writerows(diagnostics)
        print(f'{profile}: {len(rows)} frames, {sum(x[2] for x in diagnostics)} arm poses projected; '
              f'max closure {max(x[3] for x in diagnostics):.6f} mm, '
              f'{max(x[4] for x in diagnostics):.6f} deg', flush=True)


if __name__ == '__main__':
    main()
