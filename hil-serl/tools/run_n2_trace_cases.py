#!/usr/bin/env python3
"""Re-run only frozen-matrix filtered stop cases with N2 trace logging.

The v3 matrix remains immutable.  This helper reads its cell reports, selects
the trial indices whose reduced or full-QP row stopped, and invokes the
baseline harness for those indices only.  The resulting JSON files are
diagnostic supplements; they never replace the formal v3 matrix.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


STOP_REASONS = {
    "endpoint_validation",
    "full_qp_endpoint_validation",
    "post_integrator_check",
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--matrix", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--python", default=sys.executable)
    return p.parse_args()


def main():
    args = parse_args()
    matrix_path = Path(args.matrix).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(matrix_path.read_text())
    root = Path(__file__).resolve().parents[1]
    manifest = {
        "kind": "n2_endpoint_stop_trace_manifest",
        "source_matrix": str(matrix_path),
        "methods": ["lift_filtered", "full_joint_qp"],
        "stop_reasons_selected": sorted(STOP_REASONS),
        "trace_version": "n2-v1",
        "cases": [],
    }

    for cell in matrix["cells"]:
        cell_path = Path(cell["report"]).resolve()
        report = json.loads(cell_path.read_text())
        selected = sorted({
            int(row["trial"])
            for row in report["trial_rows"]
            if row["method"] in {"lift_filtered", "full_joint_qp"}
            and row.get("stop_reason") in STOP_REASONS
        })
        if not selected:
            continue
        scene = str(cell["scene"])
        model = Path(report["model"]).resolve()
        output = out_dir / f"trace_{scene}_{cell['speed']}.json"
        cmd = [
            args.python,
            str(root / "tools/validate_paired_baselines.py"),
            "--manifold", str(Path(report["manifold"]).resolve()),
            "--model", str(model),
            "--output", str(output),
            "--trials", str(report["trials"]),
            "--steps", str(report["steps_per_trial"]),
            "--dt", str(report["dt"]),
            "--seed", str(report["seed"]),
            "--theta-dot", str(report["task_velocity"][0]),
            "--slide-dot", str(report["task_velocity"][1]),
            "--r-left", str(report["nominal_null_rate"][0]),
            "--r-right", str(report["nominal_null_rate"][1]),
            "--min-progress-scale", str(report["min_progress_scale"]),
            "--scene-name", scene,
            "--methods", "lift_filtered,full_joint_qp",
            "--trial-indices", ",".join(map(str, selected)),
            "--trace-stops",
        ]
        if report.get("phase_governor", False):
            cmd.append("--governor")
        env = os.environ.copy()
        env["PYTHONPATH"] = str(root)
        result = subprocess.run(
            cmd, cwd=root, env=env, text=True, capture_output=True
        )
        case = {
            "scene": scene,
            "speed": cell["speed"],
            "source_report": str(cell_path),
            "trace_report": str(output),
            "selected_trial_indices": selected,
            "command": cmd,
            "returncode": result.returncode,
            "stdout": result.stdout[-2000:],
            "stderr": result.stderr[-4000:],
        }
        manifest["cases"].append(case)
        if result.returncode:
            raise RuntimeError(json.dumps(case, indent=2))
        print(f"traced {scene}/{cell['speed']} trials={selected}")

    manifest_path = out_dir / "n2_trace_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
