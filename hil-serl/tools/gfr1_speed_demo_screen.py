#!/usr/bin/env python3
"""Screen speed-stratified nominal successful windows for the next GFR run.

This is a new, immutable data-collection utility.  It reuses the frozen N17
endpoint/swept governor and only writes a new output directory.
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.run_n8_dt_heldout import run_endpoint_governor
from tools.validate_differential_lift import AviatorClosure

SPEED_BINS = {
    "v015_050": (0.020, 0.0005),
    "v050_150": (0.050, 0.00125),
    "v150_300": (0.150, 0.00375),
    "v300_450": (0.300, 0.00750),
    "v450_700": (0.450, 0.01125),
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n16-raw", required=True)
    ap.add_argument("--manifold", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--states-per-bin", type=int, default=5)
    args = ap.parse_args()
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()): raise RuntimeError(f"non-empty output: {out}")
    out.mkdir(parents=True, exist_ok=True)
    raw = json.loads(Path(args.n16_raw).read_text())
    variants = {v["model_sha256"]: v["model"] for v in raw["variants"]}
    # One nominal, successful state per trial, keeping the source model nominal.
    states = {}
    for r in raw["trial_rows"]:
        if r.get("method") != "online_endpoint_governor" or not r.get("complete"): continue
        if r.get("family") != "fixture_translation" or float(r.get("error_level", 1.0)) != 0.0: continue
        states.setdefault(int(r["trial"]), r)
    states = list(states.values())
    states = states[: max(args.states_per_bin, 1)]
    lk = ManifoldLookup(str(Path(args.manifold).resolve()))
    lower, upper = lk.joint_lower.reshape(14), lk.joint_upper.reshape(14)
    rows=[]
    for b,(theta,slide) in SPEED_BINS.items():
        for i,r in enumerate(states):
            model = variants[r["model_sha256"]]
            fk, checker = AviatorClosure(model), CollisionChecker(model)
            result = run_endpoint_governor(fk, checker, np.asarray(r["initial_x"]), np.asarray(r["initial_q"]), lower, upper, 0.005, 100, theta, slide)
            rows.append({"bin":b,"theta_dot":theta,"slide_dot":slide,"source_trial":int(r["trial"]),"model":model,"model_sha256":r["model_sha256"],"initial_x":r["initial_x"],"initial_q":r["initial_q"],"initial_phi":r["initial_phi"],**result})
            print(b, i+1, result.get("complete"), result.get("task_progress_fraction"), result.get("stop_reason"), flush=True)
    summary={}
    for b in SPEED_BINS:
        rr=[x for x in rows if x["bin"]==b]
        summary[b]={"n":len(rr),"complete":sum(bool(x.get("complete")) for x in rr),"completion_rate":float(np.mean([bool(x.get("complete")) for x in rr])) if rr else 0.0,"mean_progress":float(np.mean([x.get("task_progress_fraction",0) for x in rr])) if rr else 0.0,"stop_reasons":{k:sum(x.get("stop_reason")==k for x in rr) for k in sorted(set(x.get("stop_reason") for x in rr))}}
    report={"kind":"gfr1_speed_demo_screen","source_n16_raw":str(Path(args.n16_raw).resolve()),"speed_bins":SPEED_BINS,"states_per_bin":len(states),"d_safe_m":0.005,"dt":0.005,"steps":100,"summary":summary,"rows":rows}
    (out/"speed_demo_screen_raw.json").write_text(json.dumps(report,indent=2)+"\n")
    (out/"speed_demo_screen_summary.json").write_text(json.dumps({"summary":summary,"states":len(states)},indent=2)+"\n")
    print(json.dumps(summary,indent=2))
if __name__=="__main__": main()
