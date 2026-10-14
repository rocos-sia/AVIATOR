#!/usr/bin/env python3
"""Freeze the RL-N11 protocol without running training or modifying old results.

The script writes an immutable protocol manifest and an input hash ledger.  It
intentionally refuses to overwrite an existing output directory so that a
later run cannot silently change the frozen RL experiment contract.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path


AVIATOR_ROOT = Path(__file__).resolve().parents[2]
HIL_SERL = AVIATOR_ROOT / "hil-serl"
TSR_ROOT = AVIATOR_ROOT.parent / "tsr"
OUTPUT_ROOT = AVIATOR_ROOT / "outputs" / "differential_lift_feasibility_20261007" / "rl_n11_protocol"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def add_file(files: dict[str, str], missing: list[str], path: Path) -> None:
    key = str(path)
    if path.is_file():
        files[key] = sha256_file(path)
    else:
        missing.append(key)


def add_tree(files: dict[str, str], missing: list[str], root: Path) -> None:
    if not root.is_dir():
        missing.append(str(root))
        return
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        add_file(files, missing, path)


def load_json(path: Path) -> dict:
    with path.open() as stream:
        return json.load(stream)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=False)

    python_exe = Path("/home/rocos/miniconda3/envs/mujo/bin/python")
    frozen = {
        "action": ["r_L", "r_R"],
        "task_state": "x=(theta,s) follows fixed nominal trajectory",
        "beta_owner": "endpoint-aware governor",
        "d_safe_m": 0.005,
        "m_curv_m": 0.0005,
        "training_dt_s": 0.01,
        "evaluation_dt_s": [0.005, 0.01],
        "phi_dot_scale_rad_s": 1.5,
        "methods": ["fixed_phase", "greedy_phase", "RL_raw", "RL_governed"],
        "training_seeds": [0, 1, 2],
        "pilot_trials_per_cell": 10,
        "formal_trials_per_cell": "30-50 after pilot go/no-go",
        "no_global_branch_claim": True,
        "no_continuous_time_safety_claim": True,
    }

    files: dict[str, str] = {}
    missing: list[str] = []
    add_file(files, missing, python_exe)
    add_file(files, missing, HIL_SERL / "data/aviator/manifold_phi_stale_pitch-10deg/manifest.json")
    add_tree(files, missing, HIL_SERL / "data/aviator/manifold_phi_stale_pitch-10deg")
    add_file(files, missing, AVIATOR_ROOT / "AviatorRobot/build/bin/model/aviator.xml")
    add_file(files, missing, HIL_SERL / "examples/experiments/aviator_manifold/env.py")
    add_file(files, missing, HIL_SERL / "examples/experiments/aviator_manifold/manifold_lookup.py")
    add_file(files, missing, HIL_SERL / "examples/experiments/aviator_manifold/config.py")
    add_file(files, missing, HIL_SERL / "examples/experiments/aviator_manifold/reward.py")
    add_file(files, missing, HIL_SERL / "examples/experiments/aviator_manifold/safety_filter.py")
    add_file(files, missing, HIL_SERL / "data/aviator/trajectory_source/split_manifest.json")
    add_file(files, missing, HIL_SERL / "data/aviator/trajectory_source/generate.py")
    add_file(files, missing, HIL_SERL / "tools/run_n7_endpoint_governor.py")
    add_file(files, missing, HIL_SERL / "tools/run_n8_dt_heldout.py")

    immutable_results = [
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_dt_sensitivity_summary.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_curvature_margin_summary.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_qualification_summary_final_v2.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261006/n7_formal_v1/n7_qualification_summary_final.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_dt_summary.json",
        AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261007/n8_dt_heldout/n8_raw_trials.json",
        TSR_ROOT / "aviator_branch_preserving_differential_lift_draft.tex",
    ]
    for path in immutable_results:
        add_file(files, missing, path)

    split_path = HIL_SERL / "data/aviator/trajectory_source/split_manifest.json"
    split_summary = {}
    if split_path.is_file():
        split = load_json(split_path)
        if isinstance(split, dict):
            for name, value in split.items():
                if isinstance(value, (list, dict, int, float, str, bool)):
                    split_summary[name] = value if not isinstance(value, list) else len(value)

    protocol = {
        "node": "RL-N11",
        "status": "protocol_frozen",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repository": str(AVIATOR_ROOT),
        "paper": str(TSR_ROOT / "aviator_branch_preserving_differential_lift_draft.tex"),
        "python": {
            "executable": str(python_exe),
            "version": platform.python_version(),
            "runtime": sys.version,
        },
        "frozen": frozen,
        "trajectory_split_summary": split_summary,
        "input_hash_count": len(files),
        "missing_inputs": sorted(set(missing)),
        "old_results_read_only": True,
        "training_not_run": True,
        "next_node": "RL-N12 decoder geometry gate",
    }
    (output / "n11_protocol.json").write_text(json.dumps(protocol, indent=2, sort_keys=True) + "\n")
    with (output / "input_hashes.txt").open("w") as stream:
        for path, digest in sorted(files.items()):
            stream.write(f"{digest}  {path}\n")
        if missing:
            stream.write("\n# Missing inputs\n")
            for path in sorted(set(missing)):
                stream.write(f"MISSING  {path}\n")
    (output / "README.md").write_text(
        "# RL-N11 protocol freeze\n\n"
        "This directory freezes the RL extension contract. No training or new\n"
        "safety result was run in N11. `n11_protocol.json` is immutable for\n"
        "the following nodes; `input_hashes.txt` records the exact inputs.\n"
        "The existing N1--N10 result files remain read-only.\n"
    )
    print(json.dumps({
        "output": str(output),
        "input_hash_count": len(files),
        "missing_inputs": sorted(set(missing)),
        "training_not_run": True,
        "next_node": "RL-N12 decoder geometry gate",
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
