#!/usr/bin/env python3
"""RL-N12 stratified geometry and latency gate for the manifold decoder.

This node evaluates the existing trilinear decoder on an independently sampled
local domain.  It does not train a policy and does not modify N1--N11 data.
The MuJoCo FK closure and collision checker are independent of the stored LUT
clearance field.  The output directory is immutable once populated.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_differential_lift import AviatorClosure


DEFAULT_MANIFOLD = Path("/home/rocos/sia/AVIATOR/hil-serl/data/aviator/manifold_phi_stale_pitch-10deg")
DEFAULT_MODEL = Path("/home/rocos/sia/AVIATOR/AviatorRobot/build/bin/model/aviator.xml")
DEFAULT_OUTPUT = Path("/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261007/rl_n12_decoder_gate")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", type=Path, default=DEFAULT_MANIFOLD)
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--samples", type=int, default=10000)
    p.add_argument("--seed", type=int, default=1207)
    p.add_argument("--d-safe", type=float, default=0.005)
    p.add_argument("--closure-tol", type=float, default=2e-4)
    return p.parse_args()


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=np.float64), q))


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"n": 0, "median": None, "p05": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {
        "n": int(a.size),
        "median": float(np.median(a)),
        "p05": float(np.quantile(a, 0.05)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(np.max(a)),
    }


def sample_stratified(lk: ManifoldLookup, rng: np.random.Generator, index: int,
                      n_x_bins: int = 20, n_phi_bins: int = 10):
    """Sample a valid local point with deterministic x/phase strata."""
    # Leave the outer chart boundary out of this gate.  N12 is a local decoder
    # qualification; boundary stress and governor behavior belong to N15.
    th_lo, th_hi = lk.th_min + 0.04, lk.th_max - 0.04
    s_lo, s_hi = lk.s_min + 0.008, lk.s_max - 0.008
    for attempt in range(20000):
        xbin = (index + attempt) % (n_x_bins * n_x_bins)
        bth, bs = divmod(xbin, n_x_bins)
        theta = th_lo + (bth + rng.random()) / n_x_bins * (th_hi - th_lo)
        slide = s_lo + (bs + rng.random()) / n_x_bins * (s_hi - s_lo)
        x = np.array([theta, slide], dtype=np.float64)
        box = lk.safe_interval(x[None, :])
        lo, hi = box["phi_safe_lo"][0], box["phi_safe_hi"][0]
        width = hi - lo
        if np.any(width <= 1e-3):
            continue
        phibin = (index + attempt) % n_phi_bins
        frac = (phibin + rng.random()) / n_phi_bins
        # Keep a small interior margin so this gate does not confuse a chart
        # boundary query with a decoder interior failure.
        frac = 0.05 + 0.90 * frac
        phi = lo + frac * width
        result = lk.query(x[None, :], phi[None, :], check_safe=False)
        branch = int(result["branch"][0])
        q = np.concatenate([result["qL"][0], result["qR"][0]])
        if branch == 0 and np.all(np.isfinite(q)):
            return x, phi, result, bth, bs, phibin, attempt
    raise RuntimeError(f"unable to find valid stratified sample {index}")


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=False)

    lk = ManifoldLookup(str(args.manifold))
    fk = AviatorClosure(str(args.model))
    checker = CollisionChecker(str(args.model))
    rng = np.random.default_rng(args.seed)

    rows: list[dict] = []
    failures: dict[str, int] = {}
    for sample_id in range(args.samples):
        x, phi, result, bth, bs, phibin, attempts = sample_stratified(lk, rng, sample_id)
        q = np.concatenate([result["qL"][0], result["qR"][0]])

        t0 = time.perf_counter()
        decoder_result = lk.query(x[None, :], phi[None, :], check_safe=False)
        decode_ms = 1000.0 * (time.perf_counter() - t0)

        t0 = time.perf_counter()
        closure = fk.error(x, q)
        closure_inf = float(np.max(np.abs(closure)))
        closure_ms = 1000.0 * (time.perf_counter() - t0)

        t0 = time.perf_counter()
        d_independent, contact = checker.check(x, q)
        checker_ms = 1000.0 * (time.perf_counter() - t0)

        d_lut = float(decoder_result["d_min"][0])
        branch = int(decoder_result["branch"][0])
        m_q = float(decoder_result["m_q"][0])
        endpoint_pass = bool(np.isfinite(d_independent) and
                             d_independent >= args.d_safe - 1e-9 and
                             not contact)
        closure_pass = bool(closure_inf <= args.closure_tol)
        valid = bool(branch == 0 and closure_pass and endpoint_pass and m_q >= 0.0)
        if not closure_pass:
            failures["closure_above_tolerance"] = failures.get("closure_above_tolerance", 0) + 1
        if not endpoint_pass:
            failures["independent_endpoint_clearance"] = failures.get("independent_endpoint_clearance", 0) + 1
        if m_q < 0.0:
            failures["joint_limit_margin"] = failures.get("joint_limit_margin", 0) + 1
        if branch != 0:
            failures["branch_invalid"] = failures.get("branch_invalid", 0) + 1

        rows.append({
            "sample_id": sample_id,
            "theta_bin": bth,
            "s_bin": bs,
            "phi_bin": phibin,
            "sampling_attempts": attempts,
            "theta": float(x[0]),
            "s": float(x[1]),
            "phi_L": float(phi[0]),
            "phi_R": float(phi[1]),
            "branch": branch,
            "closure_residual_inf": closure_inf,
            "closure_pass": closure_pass,
            "d_lut_m": d_lut,
            "d_independent_m": float(d_independent),
            "clearance_difference_m": float(d_independent - d_lut),
            "endpoint_pass": endpoint_pass,
            "contact": bool(contact),
            "joint_limit_margin_rad": m_q,
            "decode_ms": decode_ms,
            "closure_ms": closure_ms,
            "checker_ms": checker_ms,
            "total_ms": decode_ms + closure_ms + checker_ms,
            "valid_sample": valid,
        })

    def vals(key: str) -> list[float]:
        return [float(row[key]) for row in rows if np.isfinite(row[key])]

    summary = {
        "node": "RL-N12",
        "status": "complete",
        "manifold": str(args.manifold.resolve()),
        "model": str(args.model.resolve()),
        "samples_requested": args.samples,
        "samples_recorded": len(rows),
        "seed": args.seed,
        "d_safe_m": args.d_safe,
        "closure_tolerance": args.closure_tol,
        "branch_counts": {
            str(k): int(v) for k, v in zip(*np.unique([row["branch"] for row in rows], return_counts=True))
        },
        "valid_samples": int(sum(row["valid_sample"] for row in rows)),
        "closure_pass_rate": float(np.mean([row["closure_pass"] for row in rows])),
        "endpoint_pass_rate": float(np.mean([row["endpoint_pass"] for row in rows])),
        "independent_contact_count": int(sum(row["contact"] for row in rows)),
        "closure_residual_inf": stats(vals("closure_residual_inf")),
        "d_lut_m": stats(vals("d_lut_m")),
        "d_independent_m": stats(vals("d_independent_m")),
        "clearance_difference_m": stats(vals("clearance_difference_m")),
        "joint_limit_margin_rad": stats(vals("joint_limit_margin_rad")),
        "decode_ms": stats(vals("decode_ms")),
        "closure_ms": stats(vals("closure_ms")),
        "checker_ms": stats(vals("checker_ms")),
        "total_ms": stats(vals("total_ms")),
        "failure_counts": failures,
        "interpretation": [
            "This is a local decoder qualification on the tested valid domain.",
            "Independent FK closure and clearance are reported separately from stored LUT fields.",
            "The result does not establish F(Q,x)=0 globally, global branch completeness, or continuous-time safety.",
            "No policy was trained and no N1-N11 raw result was modified.",
        ],
        "next_node": "RL-N13 policy training sanity gate",
    }
    (output / "n12_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    (output / "n12_raw_samples.json").write_text(json.dumps(rows, indent=2) + "\n")
    with (output / "n12_decoder_trace.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "README.md").write_text(
        "# RL-N12 decoder geometry gate\n\n"
        "The trace contains stratified samples from the local valid phase domain.\n"
        "Closure and clearance are recomputed independently with MuJoCo. This\n"
        "node did not train an RL policy and does not claim global exact closure.\n"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
