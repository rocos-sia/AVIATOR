#!/usr/bin/env python3
"""Run N3 finite-step clearance diagnostics on the frozen v3 state seeds.

This creates a new diagnostic directory and never edits the v3 matrix.  Each
cell reuses the v3 seed, scene, task velocity, null rates, and ten sampled
initial states.  The total task time is held at 0.5 s while dt changes.
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
    manifest = {
        "kind": "n3_dt_sensitivity_manifest",
        "source_matrix": str(matrix_path),
        "total_time_s": args.total_time,
        "dts": dts,
        "methods": ["lift_filtered", "full_joint_qp"],
        "trace_version": "n3-v1",
        "cases": [],
    }

    jobs = []
    for dt in dts:
        steps = int(round(args.total_time / dt))
        if abs(steps * dt - args.total_time) > 1e-12:
            raise ValueError(f"dt={dt} does not divide total time {args.total_time}")
        for cell in matrix["cells"]:
            source = Path(cell["report"]).resolve()
            report = json.loads(source.read_text())
            output = out_dir / f"dt_{dt:g}_{cell['scene']}_{cell['speed']}.json"
            cmd = [
                args.python,
                str(root / "tools/validate_paired_baselines.py"),
                "--manifold", str(Path(report["manifold"]).resolve()),
                "--model", str(Path(report["model"]).resolve()),
                "--output", str(output),
                "--trials", str(report["trials"]),
                "--steps", str(steps),
                "--dt", str(dt),
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
                "dt": dt,
                "steps": steps,
                "scene": cell["scene"],
                "speed": cell["speed"],
                "source_report": str(source),
                "output": str(output),
                "command": cmd,
                "env": env,
            })

    def run_job(job):
        result = subprocess.run(
            job["command"], cwd=root, env=job["env"], text=True,
            capture_output=True,
        )
        case = {k: v for k, v in job.items() if k != "env"}
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
                f"completed dt={case['dt']:g} {case['scene']}/{case['speed']}",
                flush=True,
            )

    manifest["cases"].sort(key=lambda c: (c["dt"], c["scene"], c["speed"]))

    manifest_path = out_dir / "n3_dt_sensitivity_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
