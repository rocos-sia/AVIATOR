#!/usr/bin/env python
"""Cycle-consistency validator for the Q(x, phi) manifold (Task 1.2, BLOCKER #3).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.2)

The C++ producer (Task 0.2) already gates the manifold with a cycle-consistency
pass.  This tool re-checks the written ``.bin`` files on load, as a CI check
after ``TASKS=manifold_phi``.

Grid adjacency stand-in for ``||Q(x_{i+1}, phi_j) - IK_forward(Q(x_i, phi_j))||``:
two neighbouring grid points must hold the *same physical phase* at the same
``phi`` index, so their per-arm joint configs may only differ by the smooth
continuation step.  The tool reports the worst such jump over every pair of
adjacent (theta, s) grid points that both have ``branch == 0``.

Usage::

    python validate_manifold.py --dir data/aviator/manifold_phi/ [--eps 0.5]

Exit status is 0 when ``cycle_consistency_max_violation <= eps``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

# files written by build_manifold_phi (Task 0.2)
BIN_FILES = (
    "manifest.json", "phi.bin", "qL.bin", "qR.bin", "dL.bin", "dR.bin",
    "safe.bin", "branch.bin", "QxL.bin", "QxR.bin", "QphiL.bin", "QphiR.bin",
)


def _manifold_lookup_cls():
    """Import ManifoldLookup from the examples package (no PYTHONPATH needed)."""
    root = Path(__file__).resolve().parents[1]  # hil-serl/
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
    return ManifoldLookup


def _masked_max(vals: np.ndarray, mask: np.ndarray) -> float:
    """Max of ``vals`` (.., n_phi, 7) over entries whose (..)-pair is ``mask``-true."""
    if mask.size == 0 or not mask.any():
        return 0.0
    full = np.broadcast_to(mask[..., None, None], vals.shape)
    return float(vals[full].max())


def validate(manifold_dir: str, eps: float = 0.5) -> dict:
    """Re-check cycle consistency of an on-disk manifold.

    Returns a report dict with ``cycle_consistency_max_violation``,
    ``n_excluded_points`` and ``file_sizes`` (plus per-arm/direction detail).
    """
    ManifoldLookup = _manifold_lookup_cls()
    lk = ManifoldLookup(manifold_dir)

    n_theta, n_s, n_phi = lk.n_theta, lk.n_s, lk.n_phi
    # (i_s, i_theta, i_phi, joint) -- canonical s-outer / theta-inner layout
    qL = lk.qL.reshape(n_s, n_theta, n_phi, 7).astype(np.float64)
    qR = lk.qR.reshape(n_s, n_theta, n_phi, 7).astype(np.float64)
    branch = lk.branch.reshape(n_s, n_theta)

    ok_theta = (branch[:, :-1] == 0) & (branch[:, 1:] == 0)   # (n_s, n_theta-1)
    ok_s = (branch[:-1, :] == 0) & (branch[1:, :] == 0)       # (n_s-1, n_theta)

    def arm_report(q):
        v_th = _masked_max(np.abs(q[:, 1:, :, :] - q[:, :-1, :, :]), ok_theta)
        v_s = _masked_max(np.abs(q[1:, :, :, :] - q[:-1, :, :, :]), ok_s)
        return max(v_th, v_s), v_th, v_s

    vL, v_th_L, v_s_L = arm_report(qL)
    vR, v_th_R, v_s_R = arm_report(qR)
    max_violation = max(vL, vR)

    d = Path(manifold_dir)
    file_sizes = {name: os.path.getsize(d / name) for name in BIN_FILES if (d / name).is_file()}

    return {
        "manifold_dir": str(d),
        "n_grid_points": int(n_s * n_theta),
        "n_excluded_points": int((branch != 0).sum()),
        "cycle_consistency_max_violation": max_violation,
        "cycle_consistency_max_violation_qL": vL,
        "cycle_consistency_max_violation_qR": vR,
        "max_jump_theta_qL": v_th_L,
        "max_jump_s_qL": v_s_L,
        "max_jump_theta_qR": v_th_R,
        "max_jump_s_qR": v_s_R,
        "eps": float(eps),
        "passed": bool(max_violation <= eps),
        "file_sizes": file_sizes,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Validate a Q(x, phi) manifold directory.")
    ap.add_argument("--dir", required=True, help="manifold directory (manifest.json + *.bin)")
    ap.add_argument("--eps", type=float, default=0.5,
                    help="max tolerated adjacent-grid jump (default: 0.5)")
    ap.add_argument("--json", action="store_true", help="emit the full report as JSON")
    args = ap.parse_args(argv)

    rep = validate(args.dir, eps=args.eps)

    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        print(f"manifold dir                    : {rep['manifold_dir']}")
        print(f"grid points                     : {rep['n_grid_points']}")
        print(f"n_excluded_points               : {rep['n_excluded_points']}")
        print(f"cycle_consistency_max_violation : {rep['cycle_consistency_max_violation']:.6g}"
              f"  (qL {rep['cycle_consistency_max_violation_qL']:.6g},"
              f" qR {rep['cycle_consistency_max_violation_qR']:.6g}; eps {rep['eps']})")
        print("file_sizes:")
        for name, size in rep["file_sizes"].items():
            print(f"  {name:<14} {size:>14,} bytes")
        print(f"PASSED: {rep['passed']}")

    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
