#!/usr/bin/env python3
"""Run the frozen N3 matrix with a predeclared linear clearance margin.

The margin is applied only to the linear candidate constraint.  The independent
nonlinear endpoint and swept checkers keep the frozen d_safe threshold.  All
source N3/v3 files are read-only; results are written to a new N4 directory.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
import subprocess
import sys
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--matrix", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--python", default=sys.executable)
    p.add_argument("--total-time", type=float, default=0.5)
    p.add_argument("--dts", default="0.0025,0.005,0.01")
    p.add_argument("--margins-mm", default="0,0.05,0.10,0.25,0.50")
    p.add_argument("--max-workers", type=int, default=4)
    return p.parse_args()


def main():
    args = parse_args()
    matrix_path = Path(args.matrix).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(matrix_path.read_text())
    root = Path(__file__).resolve().parents[1]
    dts = [float(v) for v in args.dts.split(",") if v.strip()]
    margins_mm = [float(v) for v in args.margins_mm.split(",") if v.strip()]
    if any(v < 0.0 for v in margins_mm):
        raise ValueError("curvature margins must be non-negative")
    manifest = {
        "kind": "n4_curvature_margin_manifest",
        "source_matrix": str(matrix_path),
        "total_time_s": args.total_time,
        "dts": dts,
        "margins_mm": margins_mm,
        "methods": ["lift_filtered", "full_joint_qp"],
        "d_safe_m": 0.005,
        "beta_stop": 0.125,
        "trace_version": "n4-v1",
        "cases": [],
    }

    jobs = []
    for margin_mm in margins_mm:
        margin_m = margin_mm * 1e-3
        margin_tag = f"{margin_mm:g}".replace(".", "p")
        for dt in dts:
            steps = int(round(args.total_time / dt))
            if abs(steps * dt - args.total_time) > 1e-12:
                raise ValueError(f"dt={dt} does not divide total time {args.total_time}")
            for cell in matrix["cells"]:
                source = Path(cell["report"]).resolve()
                report = json.loads(source.read_text())
                output = out_dir / (
                    f"margin_{margin_tag}mm_dt_{dt:g}_"
                    f"{cell['scene']}_{cell['speed']}.json"
                )
                cmd = [
                    args.python,
                    str(root / "tools/validate_paired_baselines.py"),
                    "--manifold", str(Path(report["manifold"]).resolve()),
                    "--model", str(Path(report["model"]).resolve()),
                    "--output", str(output),
                    "--trials", str(report["trials"]),
                    "--steps", str(steps),
                    "--dt", str(dt),
                    "--clearance-margin-m", str(margin_m),
                    "--seed", str(report["seed"]),
                    "--theta-dot", str(report["task_velocity"][0]),
                    "--slide-dot", str(report["task_velocity"][1]),
                    "--r-left", str(report["nominal_null_rate"][0]),
                    "--r-right", str(report["nominal_null_rate"][1]),
                    "--min-progress-scale", str(report["min_progress_scale"]),
                    "--scene-name", str(cell["scene"]),
                    "--methods", "lift_filtered,full_joint_qp",
                    "--trace-all-steps",
                ]
                if report.get("phase_governor", False):
                    cmd.append("--governor")
                env = os.environ.copy()
                env["PYTHONPATH"] = str(root)
                env["OMP_NUM_THREADS"] = "1"
                env["MKL_NUM_THREADS"] = "1"
                jobs.append({
                    "margin_mm": margin_mm,
                    "margin_m": margin_m,
                    "dt": dt,
                    "steps": steps,
                    "scene": cell["scene"],
                    "speed": cell["speed"],
                    "source_report": str(source),
                    "output": str(output),
                    "existing": output.exists(),
                    "command": cmd,
                    "env": env,
                })

    def run_job(job):
        if job.get("existing"):
            case = {k: v for k, v in job.items() if k not in ("env", "command")}
            case.update({"returncode": 0, "status": "existing"})
            return case
        result = subprocess.run(
            job["command"], cwd=root, env=job["env"], text=True,
            capture_output=True,
        )
        case = {k: v for k, v in job.items() if k not in ("env", "existing")}
        case.update({
            "returncode": result.returncode,
            "stdout": result.stdout[-1000:],
            "stderr": result.stderr[-3000:],
        })
        return case

    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {executor.submit(run_job, job): job for job in jobs}
        for future in as_completed(futures):
            case = future.result()
            manifest["cases"].append(case)
            if case["returncode"]:
                raise RuntimeError(json.dumps(case, indent=2))
            print(
                f"completed margin={case['margin_mm']:g}mm "
                f"dt={case['dt']:g} {case['scene']}/{case['speed']}",
                flush=True,
            )

    manifest["cases"].sort(
        key=lambda c: (c["margin_mm"], c["dt"], c["scene"], c["speed"])
    )
    manifest_path = out_dir / "n4_curvature_margin_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
