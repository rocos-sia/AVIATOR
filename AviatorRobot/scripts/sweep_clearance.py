#!/usr/bin/env python3
"""Driver for the inner-gap sweep and the roll_neg joint-margin ablation.

Phase 1 — inner-gap sweep: regenerate the wall model at each inner-face gap in
GAPS and run all four tasks through aviator_clearance_trajectory (d_safe=5 mm,
q_margin=0.03). Phase 2 — joint-margin ablation: at the reference gap, rerun
roll_neg only with q_margin_target in MARGINS to separate "conservative margin
active" from "true redundancy exhaustion".

The generated models land in separate directories (sharing the mesh assets via a
symlink) so the trajectory runs execute in parallel. Aggregates every per-run
summary.csv into <out>/sweep_summary.csv.
"""
import argparse
import csv
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TOOL = REPO / "AviatorRobot" / "build" / "bin" / "aviator_clearance_trajectory"
GEN = REPO / "reference" / "rocos-mujoco" / "scripts" / "generate_aviator.py"
MESHES = REPO / "reference" / "rocos-mujoco" / "model" / "aviator_meshes"
CONFIG_DIR = REPO / "AviatorRobot" / "config"

GAPS = [0.54, 0.53, 0.52, 0.51, 0.50, 0.49, 0.48, 0.47]
MARGINS = [0.0, 0.01, 0.02, 0.03]
D_SAFE = 0.005
REF_GAP = 0.54  # reference gap for the joint-margin ablation


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def write_config(path: Path, model: Path):
    path.write_text(
        f"urdf: {CONFIG_DIR / 'aviator_control.urdf'}\n"
        f"model: {model}\n"
        f"grasp: {CONFIG_DIR / 'grasp.json'}\n"
        f"posture: {CONFIG_DIR / 'posture.json'}\n"
    )


def generate_model(outdir: Path, gap: float):
    model_dir = outdir / "models" / f"gap_{gap:.2f}"
    model_dir.mkdir(parents=True, exist_ok=True)
    model = model_dir / "aviator.xml"
    r = run([sys.executable, str(GEN), "--gap", f"{gap:.2f}", "--out", str(model)])
    if r.returncode != 0:
        raise RuntimeError(f"generate gap={gap} failed:\n{r.stderr}")
    link = model_dir / "aviator_meshes"
    if not link.exists():
        os.symlink(MESHES, link)
    return model


def run_tool(outdir: Path, model: Path, d_safe: float, q_margin: float, tasks: str, name: str):
    cfg = outdir / "configs" / f"{name}.yaml"
    cfg.parent.mkdir(parents=True, exist_ok=True)
    write_config(cfg, model)
    out = outdir / "out" / name
    cmd = [str(TOOL), str(cfg), str(out), f"{d_safe:.4f}", f"{q_margin:.3f}", "10", "0.1"]
    if tasks:
        cmd.append(tasks)
    r = run(cmd)
    if r.returncode != 0:
        raise RuntimeError(f"tool {name} failed:\n{r.stderr}\n{r.stdout}")
    return name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("/tmp/aviator-sweep"))
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--only", help="comma list: gap|ablation")
    args = ap.parse_args()
    outdir = args.out
    outdir.mkdir(parents=True, exist_ok=True)

    jobs = []  # (name, fn)

    if args.only is None or "gap" in args.only:
        models = {}
        for g in GAPS:
            models[g] = generate_model(outdir, g)
            print(f"generated gap={g:.2f} -> {models[g]}", flush=True)
        for g in GAPS:
            name = f"gap_{g:.2f}"
            jobs.append((name, lambda g=g: run_tool(
                outdir, models[g], D_SAFE, 0.03, "", f"gap_{g:.2f}")))

    if args.only is None or "ablation" in args.only:
        model = generate_model(outdir, REF_GAP)
        for m in MARGINS:
            name = f"marg_{m:.2f}"
            jobs.append((name, lambda m=m: run_tool(
                outdir, model, D_SAFE, m, "roll_neg", f"marg_{m:.2f}")))

    print(f"running {len(jobs)} jobs with {args.workers} workers", flush=True)
    results = {}
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(fn): name for name, fn in jobs}
        for fut in as_completed(futs):
            name = futs[fut]
            try:
                fut.result()
                results[name] = "ok"
                print(f"[ok] {name}", flush=True)
            except Exception as e:
                results[name] = f"FAIL: {e}"
                print(f"[FAIL] {name}: {e}", flush=True)

    # aggregate every summary.csv into one combined file
    rows, header = [], None
    for name in sorted(results):
        p = outdir / "out" / name / "summary.csv"
        if not p.exists():
            continue
        with open(p) as fh:
            r = csv.DictReader(fh)
            header = header or r.fieldnames
            for row in r:
                row["run"] = name
                rows.append(row)
    if rows:
        header = ["run"] + list(header)
        with open(outdir / "sweep_summary.csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=header)
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {outdir / 'sweep_summary.csv'} ({len(rows)} rows)")
    print("done:", {k: v for k, v in results.items()})


if __name__ == "__main__":
    main()
