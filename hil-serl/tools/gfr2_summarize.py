"""Summarize completed GPU distillation gates without treating assisted runs as successes."""
import argparse
import json
from pathlib import Path
import numpy as np
from tools.gfr1_remount_evaluate import dump, sha


def summarize(root):
    root=Path(root).resolve()
    report=dict(scope="local exact-calibration kinematic pilot; not residual robustness or hardware validation",
                teacher=json.loads((root/"teacher/gate.json").read_text()),
                initialization=json.loads((root/"dataset/gate.json").read_text()),
                gpu=json.loads((root/"gpu_preflight/result.json").read_text()),
                rounds=[],dagger=[],latency_claim=False,rl_started=False)
    for folder in sorted((root/"shared_bc").glob("evaluation_*")):
        if not (folder/"gate.json").exists():
            continue
        rows=json.loads((folder/"episodes.json").read_text())
        splits={}
        reasons={}
        for split in ("train","development"):
            sub=[r for r in rows if r["case"].startswith(split+"_")]
            if sub:
                splits[split]=dict(completed=sum(r["completed"] for r in sub),cases=len(sub),
                    median_progress=float(np.median([r["progress"] for r in sub])),
                    max_progress=max(r["progress"] for r in sub))
        for file in folder.glob("*.jsonl"):
            last=None
            with file.open() as stream:
                for line in stream:
                    last=json.loads(line)
            if not last or last.get("accepted",False):
                continue
            trial=last["direct_trial"]
            labels=[]
            if trial.get("joint_limit_margin_rad",0)<-1e-9: labels.append("joint_limit")
            if trial.get("d_actual_m",1)<.005-1e-9: labels.append("clearance")
            if trial.get("qdot_max_rad_s",0)>1.50002: labels.append("joint_speed")
            if trial.get("reason")=="closure_tolerance": labels.append("closure_geometry")
            if trial.get("endpoint_contact") or trial.get("swept_contact"): labels.append("contact")
            for label in labels or [trial.get("reason","unknown")]:
                reasons[label]=reasons.get(label,0)+1
        i=int(folder.name.split("_")[-1])
        training=json.loads((root/f"shared_bc/actor_round_{i}.training.json").read_text())
        report["rounds"].append(dict(round=i,splits=splits,first_rejection_categories_overlapping=reasons,
            training=training,checkpoint_unchanged=json.loads((folder/"gate.json").read_text())["checkpoint_unchanged"]))
    for folder in sorted((root/"shared_bc").glob("dagger_*")):
        if not (folder/"gate.json").exists():
            continue
        rows=json.loads((folder/"episodes.json").read_text())
        report["dagger"].append(dict(round=folder.name,cases=len(rows),
            teacher_assisted_reached_end=sum(r.get("rollout_reached_end",False) for r in rows),
            student_success_claim=False,labels=sum(r.get("labels",0) for r in rows),
            teacher_interventions=sum(r.get("teacher_interventions",0) for r in rows)))
    final=root/"shared_bc/gate.json"
    report["final_gate"]=json.loads(final.read_text()) if final.exists() else {"pending":True}
    protocol=json.loads((root/"protocol.json").read_text())
    report["frozen_sources_unchanged"]=all(sha(p)==h for p,h in protocol["source_sha256"].items())
    report["formal_gpu_updates"]=sum(r["training"]["updates"] for r in report["rounds"])
    report["formal_gpu_training_s"]=sum(r["training"]["elapsed_s"] for r in report["rounds"])
    dump(root/"run_summary.json",report)
    print(json.dumps({k:report[k] for k in ("final_gate","formal_gpu_updates","formal_gpu_training_s","frozen_sources_unchanged","rounds")},indent=2))
    return report


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",required=True)
    summarize(p.parse_args().root)
