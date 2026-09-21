#!/usr/bin/env python3
"""Acceptance gates for the rebuilt feasibility atlas.

  1. Reproducibility  max|Dg1 - Dg2| (and dL/dR) across two builds < 1e-6..1e-5 m
  2. IK-fail structure  isolated singletons (a no-IK cell with all valid 8-neighbours)
  3. Mirror symmetry   dL(θ,s) ≈ dR(-θ,s)  (MAE / P95 / max, mm)
  4. Trace completeness  fraction of reachable cells with trace_complete = 1

Run with the serl_clean env (numpy).
"""
import argparse
import json
from pathlib import Path

import numpy as np

SENTINEL = -0.5  # Dg <= this means baseline IK failed (stored as -1.0)


def load(dirpath):
    d = Path(dirpath)
    meta = json.loads((d / "atlas.json").read_text())
    g = meta["grid"]
    nt, ns = g["theta"]["n"], g["s"]["n"]
    shape = (ns, nt)
    Dg = np.fromfile(d / "atlas_Dg.bin", dtype=np.float32).reshape(shape)
    dL = np.fromfile(d / "atlas_dL.bin", dtype=np.float32).reshape(shape)
    dR = np.fromfile(d / "atlas_dR.bin", dtype=np.float32).reshape(shape)
    Feasible = np.fromfile(d / "atlas_feasible.bin", dtype=np.uint8).reshape(shape)
    Reachable = np.fromfile(d / "atlas_reachable.bin", dtype=np.uint8).reshape(shape)
    Complete = np.fromfile(d / "atlas_trace_complete.bin", dtype=np.uint8).reshape(shape)
    return meta, Dg, dL, dR, Feasible, Reachable, Complete


def ikfail_singletons(Reachable):
    """Count no-IK cells whose 8-neighbours are all valid (isolated false negatives)."""
    bad = Reachable == 0
    good = Reachable == 1
    singleton = 0
    ns, nt = bad.shape
    for j in range(ns):
        for i in range(nt):
            if not bad[j, i]:
                continue
            j0, j1 = max(0, j - 1), min(ns, j + 2)
            i0, i1 = max(0, i - 1), min(nt, i + 2)
            if good[j0:j1, i0:i1].all():
                singleton += 1
    return int(bad.sum()), singleton


def mirror_error(dL, dR):
    """dL(θ,s) ≈ dR(-θ,s) and dR(θ,s) ≈ dL(-θ,s); θ is the last axis, symmetric grid."""
    ns, nt = dL.shape
    dLr = dL[:, ::-1]
    dRr = dR[:, ::-1]
    errs = []
    mask = (dL > SENTINEL) & (dRr > SENTINEL)
    errs.append(np.abs(dL[mask] - dRr[mask]))
    mask = (dR > SENTINEL) & (dLr > SENTINEL)
    errs.append(np.abs(dR[mask] - dLr[mask]))
    e = np.concatenate(errs)
    return e.mean(), np.percentile(e, 95), e.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run1", required=True, help="first atlas dir")
    ap.add_argument("--run2", help="second atlas dir (for the reproducibility gate)")
    args = ap.parse_args()

    meta, Dg, dL, dR, Feasible, Reachable, Complete = load(args.run1)

    print(f"=== {args.run1} ===")
    b = meta.get("build", {})
    a = meta.get("audit", {})
    print(f"n_total        {meta['n_total']}")
    print(f"n_ik_fail      {b.get('n_ik_fail')}")
    print(f"n_feasible     {b.get('n_feasible')}  ({100*b.get('n_feasible',0)/meta['n_total']:.2f}%)")
    print(f"n_complete     {b.get('n_complete')}")

    # gate 2: IK-fail structure
    n_bad, n_single = ikfail_singletons(Reachable)
    print(f"\n[gate 2] IK-fail cells: {n_bad} total, {n_single} isolated singletons "
          f"({100*n_single/max(1,n_bad):.1f}% of fails)")

    # gate 3: mirror symmetry
    mae, p95, mx = mirror_error(dL, dR)
    print(f"[gate 3] mirror dL(θ,s)≈dR(-θ,s): MAE {mae*1e3:.4f} mm, P95 {p95*1e3:.4f} mm, "
          f"max {mx*1e3:.4f} mm   (C++ audit: {a.get('mirror_mae_mm')}/{a.get('mirror_p95_mm')}/{a.get('mirror_max_mm')})")

    # gate 4: trace completeness over reachable cells
    n_reach = int(Reachable.sum())
    n_comp = int(Complete.sum())
    print(f"[gate 4] trace completeness: {n_comp}/{n_reach} reachable "
          f"({100*n_comp/max(1,n_reach):.2f}%)")

    if args.run2:
        _, Dg2, dL2, dR2, F2, R2, C2 = load(args.run2)
        dDg = float(np.max(np.abs(Dg - Dg2)))
        ddL = float(np.max(np.abs(dL - dL2)))
        ddR = float(np.max(np.abs(dR - dR2)))
        same_f = bool((Feasible == F2).all())
        same_r = bool((Reachable == R2).all())
        print(f"\n=== [gate 1] reproducibility vs {args.run2} ===")
        print(f"max|ΔDg| = {dDg:.3e} m   max|ΔdL| = {ddL:.3e} m   max|ΔdR| = {ddR:.3e} m")
        print(f"feasible identical: {same_f}   reachable identical: {same_r}")
        ok = dDg < 1e-5 and same_f and same_r
        print(f"reproducibility {'PASS' if ok else 'FAIL'} (target < 1e-6..1e-5 m)")


if __name__ == "__main__":
    main()
