"""Cross-language consistency test: C++ ``lut_query`` trilinear == Python ``ManifoldLookup``.

Step 1 / Step 6 of Route A (LUT-Native DP).  The C++ ``lut_query`` subcommand was added to
``AviatorRobot/tools/clearance_trajectory.cpp`` to reproduce the trilinear/bilinear query of
``manifold_lookup.py`` bit-for-bit.  This test:

  1. draws a deterministic set of (theta, s, phi_L, phi_R) query points (interior + exact
     grid nodes + exact phi nodes + boundaries),
  2. answers them with the C++ binary (``TASKS=lut_query``) and with ``ManifoldLookup.query``,
  3. asserts every field (qL, qR, dL, dR, safe interval, branch) matches exactly.

The LUT data is float32; the interpolation is float64.  Both ends perform identical IEEE
double arithmetic in the same order, so the expected max deviation is 0.0 (the CSV round-trip
uses 17 significant digits, which round-trips a double exactly).

Run from ``hil-serl/`` with the ``serl_clean`` environment:
    PYTHONPATH=. python tools/test_lut_consistency.py
"""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup

# 0-indexed columns of lut_query.csv (matches the C++ header exactly).
COLS = {
    "theta": 0, "s": 1, "phi_L": 2, "phi_R": 3,
    "qL": slice(4, 11), "qR": slice(11, 18),
    "dL": 18, "dR": 19,
    "phi_safe_lo_L": 20, "phi_safe_hi_L": 21,
    "phi_safe_lo_R": 22, "phi_safe_hi_R": 23,
    "branch": 24,
}


def build_points(lookup: ManifoldLookup, n_random: int = 4000, seed: int = 0) -> np.ndarray:
    """Deterministic (theta, s, phi_L, phi_R) query points, incl. node/boundary edges."""
    rng = np.random.default_rng(seed)
    th = rng.uniform(lookup.th_min, lookup.th_max, n_random)
    s = rng.uniform(lookup.s_min, lookup.s_max, n_random)
    p_lo, p_hi = lookup.phi_axis[0], lookup.phi_axis[-1]
    pL = rng.uniform(p_lo, p_hi, n_random)
    pR = rng.uniform(p_lo, p_hi, n_random)

    # Exact nodes: theta/s on tabulated axes, phi on tabulated phi nodes, and grid boundaries.
    th_ax = lookup.theta_axis
    s_ax = lookup.s_axis
    p_ax = lookup.phi_axis
    nodes = []
    for thv in (th_ax[0], th_ax[len(th_ax) // 2], th_ax[-1]):
        for sv in (s_ax[0], s_ax[len(s_ax) // 2], s_ax[-1]):
            for pv in (p_ax[0], p_ax[len(p_ax) // 2], p_ax[-1]):
                nodes.append((thv, sv, pv, pv))
    nodes = np.asarray(nodes, dtype=np.float64)

    return np.vstack([np.column_stack([th, s, pL, pR]), nodes])


def run_cpp(binary: str, config: str, manifold_dir: str, points: np.ndarray) -> np.ndarray:
    """Answer ``points`` via the C++ ``lut_query`` subcommand, return the output rows."""
    tmp = tempfile.mkdtemp(prefix="lut_query_")
    try:
        os.symlink(os.path.abspath(manifold_dir), os.path.join(tmp, "manifold_phi"))
        input_path = os.path.join(tmp, "input.csv")
        np.savetxt(input_path, points, delimiter=",", fmt="%.17g")
        proc = subprocess.run(
            [binary, config, tmp, "0.005", "0.03", "10", "0.1", "lut_query", input_path],
            check=True, capture_output=True, text=True,
        )
        out_path = os.path.join(tmp, "lut_query.csv")
        rows = np.loadtxt(out_path, delimiter=",", skiprows=1)
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    if len(rows) != len(points):
        raise RuntimeError(f"C++ returned {len(rows)} rows for {len(points)} points: {proc.stdout}")
    return rows


def compare(rows_cpp: np.ndarray, points: np.ndarray, lookup: ManifoldLookup) -> dict:
    """Compare C++ lut_query output against ManifoldLookup.query at the same points."""
    th, s = points[:, 0], points[:, 1]
    phi = points[:, 2:4]
    py = lookup.query(np.column_stack([th, s]), phi, check_safe=False)

    def maxdiff(cpp: np.ndarray, ref: np.ndarray) -> float:
        return float(np.max(np.abs(cpp - ref)))

    fields = {}
    fields["qL"] = maxdiff(rows_cpp[:, COLS["qL"]], py["qL"])
    fields["qR"] = maxdiff(rows_cpp[:, COLS["qR"]], py["qR"])
    fields["dL"] = maxdiff(rows_cpp[:, COLS["dL"]], py["dL"])
    fields["dR"] = maxdiff(rows_cpp[:, COLS["dR"]], py["dR"])
    fields["phi_safe_lo_L"] = maxdiff(rows_cpp[:, COLS["phi_safe_lo_L"]], py["phi_safe_lo"][:, 0])
    fields["phi_safe_hi_L"] = maxdiff(rows_cpp[:, COLS["phi_safe_hi_L"]], py["phi_safe_hi"][:, 0])
    fields["phi_safe_lo_R"] = maxdiff(rows_cpp[:, COLS["phi_safe_lo_R"]], py["phi_safe_lo"][:, 1])
    fields["phi_safe_hi_R"] = maxdiff(rows_cpp[:, COLS["phi_safe_hi_R"]], py["phi_safe_hi"][:, 1])
    fields["branch"] = int(np.max(np.abs(rows_cpp[:, COLS["branch"]] - py["branch"])))
    return fields


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--binary", default="../AviatorRobot/build/bin/aviator_clearance_trajectory")
    parser.add_argument("--config", default="../AviatorRobot/config/aviator.yaml")
    parser.add_argument("--n-random", type=int, default=4000)
    args = parser.parse_args()

    lookup = ManifoldLookup(args.manifold_dir)
    points = build_points(lookup, n_random=args.n_random)
    print(f"cross-language consistency: {len(points)} query points "
          f"(theta [{lookup.th_min},{lookup.th_max}], s [{lookup.s_min},{lookup.s_max}], "
          f"phi [{lookup.phi_axis[0]:.6g},{lookup.phi_axis[-1]:.6g}])")

    rows_cpp = run_cpp(args.binary, args.config, args.manifold_dir, points)
    fields = compare(rows_cpp, points, lookup)

    print("field                     max |C++ - Python|")
    worst = 0.0
    for name, err in fields.items():
        print(f"  {name:<22} {err:.3e}")
        worst = max(worst, float(err))

    # float64 arithmetic is replicated exactly, so the target is bit-identity (0.0).  A
    # tolerance guards against any future axis/manifest rounding without masking a real bug.
    tol = 1e-12
    ok = worst <= tol
    print("PASS" if ok else "FAIL", f"(max deviation {worst:.3e} vs tolerance {tol:.0e})")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
