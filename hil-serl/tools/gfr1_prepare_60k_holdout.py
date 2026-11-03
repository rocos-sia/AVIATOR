#!/usr/bin/env python3
"""Prospective new-trajectory selection after freezing final 60k, without policy rollouts."""
import argparse
import json
import re
from pathlib import Path
import numpy as np
from tools.gfr1_prepare_unseen_regression import trajectory_signature
from tools.gfr1_same_state_diagnosis import digest, write_json
from tools.gfr1_prepare_teacher_buffer import sha256


def scan_history(root):
    names=set(); references=[]; scanned=[]; skipped=[]
    pattern=re.compile(r'(?:/[^\s"<>]*?/)?trajs/test/traj_\d+\.npz')
    for date in sorted((root/'outputs').glob('differential_lift_feasibility_*')):
        for task in sorted(date.iterdir()):
            if not task.is_dir() or not any(k in task.name.lower() for k in ('gfr','rl_n')):continue
            for p in sorted(task.rglob('*.json')):
                if not any(k in p.name for k in ('manifest','summary','protocol','initial_state','frozen','checkpoint_','selection','result')):continue
                if p.stat().st_size>5_000_000:
                    skipped.append(str(p));continue
                scanned.append(str(p))
                found=sorted({Path(x).name for x in pattern.findall(p.read_text(errors='replace'))})
                if found:
                    names.update(found);references.append(dict(path=str(p),sha256=sha256(p),test_names=found))
    return names,dict(scanned_metadata=scanned,large_metadata_skipped=skipped,references=references,
        scope='GFR and rl_n lineage metadata <=5 MB (manifest, summary, protocol, initial_state, frozen, checkpoint, selection, result). Not a claim about every unrelated project or unlogged use.')


def choose(tests, used_names, excluded_signatures, count):
    selected=[];rejected=[];seen=set(excluded_signatures)
    for path in sorted(tests):
        sig=trajectory_signature(path)
        if path.name in used_names or sig in seen:
            rejected.append(dict(path=str(path),reason='historical_test_use' if path.name in used_names else 'content_overlap'));continue
        selected.append(path);seen.add(sig)
        if len(selected)==count:break
    if len(selected)!=count:raise RuntimeError('Insufficient new test trajectories; do not reuse or replace based on outcome')
    return selected,rejected


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[2]
    a.output.mkdir(parents=True,exist_ok=False)
    frozen=json.loads((a.run/'frozen_config.json').read_text())
    for path,h in frozen['code_sha256'].items():assert sha256(path)==h,path
    checkpoint=a.run/'training/b4_seed11/checkpoint_60000'
    assert checkpoint.exists()
    names,history=scan_history(root)
    trajectory_root=Path(frozen['environment']['AVIATOR_GFR_TRAJECTORY_DIR'])
    non_test=sorted(p.resolve() for split in ('dp_train','rl_train','val') for p in (trajectory_root/'trajs'/split).glob('*.npz'))
    exclusion={str(p):trajectory_signature(p) for p in non_test}
    previous=[trajectory_root/'trajs/test'/name for name in names]
    previous_hashes={str(p):trajectory_signature(p) for p in previous if p.exists()}
    selected,rejected=choose(list((trajectory_root/'trajs/test').glob('*.npz')),names,set(exclusion.values())|set(previous_hashes.values()),20)
    old=json.loads(Path(frozen['eval_manifest']).read_text())
    assert len(old['models'])==4
    protected={str(checkpoint):digest(checkpoint),str(a.run/'frozen_config.json'):digest(a.run/'frozen_config.json'),
               str(Path(frozen['train_manifest'])):digest(frozen['train_manifest']),str(Path(frozen['eval_manifest'])):digest(frozen['eval_manifest']),
               frozen['environment']['AVIATOR_GFR_MANIFOLD']:digest(frozen['environment']['AVIATOR_GFR_MANIFOLD'])}
    protected.update({str(p):digest(p) for p in selected})
    protected.update({m:digest(m) for m in old['models']})
    protected.update({path:digest(path) for path in frozen['code_sha256']})
    protocol=dict(kind='gfr_final60k_prospective_trajectory_test',checkpoint=str(checkpoint.resolve()),
        checkpoint_sha256=digest(checkpoint),training_frozen_config=str((a.run/'frozen_config.json').resolve()),
        selection_rule='First 20 sorted test trajectories not referenced in audited prior metadata, content-disjoint from all dp_train/rl_train/val and previously referenced tests; frozen before policy rollout',
        selected=[dict(path=str(p.resolve()),content_sha256=trajectory_signature(p),sha256=digest(p)) for p in selected],
        exclusion_signatures=exclusion,previous_test_signatures=previous_hashes,rejected=rejected,
        history_audit=history,models=old['models'],model_assignment='round-robin, exactly 5 trajectories/model',
        initialization='Existing LUT midpoint followed by unchanged 12-iteration Newton repair; no retiming; invalid starts retained as failures, no replacement',
        batches='Two sequential CPU batches of 10; episode JSONL immediately flushed; no parameter updates',
        policy='Final checkpoint_60000 only, deterministic argmax=True; no checkpoint selection on test',
        seeds='Within each batch: reset=20262000+local_index, policy=500000+local_index*10000+step; recovery uses frozen executor reset/cycle rule',
        interpretation='New-trajectory performance on four previously development-tested perturbation models; not unseen-model or arbitrary-disturbance generalization',
        decision='Report all selected cases and failure reasons; no outcome-dependent replacement, tuning, or retraining',
        protected_input_hashes=protected,training_started=False)
    write_json(a.output/'selection_protocol.json',protocol)
    from experiments.aviator_manifold.manifold_lookup import ManifoldLookup
    from tools.audit_static_collision import CollisionChecker
    from tools.validate_differential_lift import AviatorClosure
    from tools.gfr1_build_consistent_model_manifest import repair_q
    lookup=ManifoldLookup(frozen['environment']['AVIATOR_GFR_MANIFOLD'])
    lo,hi=np.concatenate(lookup.joint_lower),np.concatenate(lookup.joint_upper)
    cache={};records=[]
    for i,path in enumerate(selected):
        model=old['models'][i%4]
        if model not in cache:cache[model]=(AviatorClosure(model),CollisionChecker(model))
        fk,checker=cache[model]
        with np.load(path) as z:
            x=np.asarray(z['x'][0],dtype=float);assert np.allclose(np.diff(z['t']),.01,atol=1e-10,rtol=0)
        box=lookup.safe_interval(x[None,:]);phi=.5*(box['phi_safe_lo'][0]+box['phi_safe_hi'][0])
        start=lookup.query(x[None,:],phi[None,:],check_safe=False);q=np.r_[start['qL'][0],start['qR'][0]].astype(float)
        q,hist=repair_q(fk,x,q,lo,hi);e=fk.error(x,q);d,contact=checker.check(x,q)
        margin=float(min(np.min(q-lo),np.min(hi-q)))
        reasons=[]
        if not np.isfinite(q).all() or not np.isfinite(e).all():reasons.append('nonfinite_initial_state')
        if np.max(np.abs(e))>1e-7:reasons.append('initial_closure_not_repaired')
        if d<frozen['d_safe']-1e-9 or contact:reasons.append('initial_clearance_or_contact')
        if margin < -1e-9:reasons.append('initial_joint_limit')
        records.append(dict(global_episode=i,trajectory_path=str(path.resolve()),model=model,initial_q=q.tolist(),initial_x=x.tolist(),
            initial_closure_inf=float(np.max(np.abs(e))),initial_clearance_m=float(d),initial_joint_margin_rad=margin,
            initialization_valid=not reasons,initialization_failures=reasons,repair_history=hist))
        print(json.dumps({k:records[-1][k] for k in ('global_episode','trajectory_path','initialization_valid','initialization_failures')}),flush=True)
    payload=dict(kind='gfr1_consistent_model_manifest',split='test',models=old['models'],records=records,record_count=20,trajectory_count=20,
                 selection_protocol=str((a.output/'selection_protocol.json').resolve()))
    write_json(a.output/'heldout_manifest.json',payload)
    for b in range(2):write_json(a.output/f'batch_{b}_manifest.json',dict(payload,records=records[b*10:(b+1)*10],record_count=10,trajectory_count=10))
    write_json(a.output/'initialization_audit.json',dict(records=20,valid=sum(r['initialization_valid'] for r in records),unique_trajectories=20,
        content_overlap=0,no_policy_rollout_for_selection=True,initialization_failures=[r for r in records if not r['initialization_valid']]))


if __name__=='__main__':main()
