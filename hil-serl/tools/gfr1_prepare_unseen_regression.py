#!/usr/bin/env python3
"""Freeze new trajectory selection and model-consistent starts before policy evaluation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.gfr1_same_state_diagnosis import digest, write_json


def trajectory_signature(path):
    """Content identity, ignoring archive metadata and filename/split."""
    h = hashlib.sha256()
    with np.load(path) as arrays:
        for key in ('t', 'x', 'xdot', 'xddot'):
            a = np.asarray(arrays[key], dtype='<f8')
            if not np.isfinite(a).all():
                raise ValueError(f'Non-finite trajectory: {path}')
            h.update(key.encode()); h.update(str(a.shape).encode()); h.update(a.tobytes())
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    prior=json.loads((args.source/'protocol.json').read_text())
    frozen=prior['frozen']
    trajroot=Path(frozen['environment']['AVIATOR_GFR_TRAJECTORY_DIR'])
    old=json.loads(Path(frozen['eval_manifest']).read_text())
    excluded_paths=sorted(p.resolve() for split in ('dp_train','rl_train','val')
                          for p in (trajroot/'trajs'/split).glob('*.npz'))
    excluded_signatures={trajectory_signature(p) for p in excluded_paths}
    tests=sorted(p.resolve() for p in (trajroot/'trajs/test').glob('*.npz'))
    selected=[]; excluded_duplicates=[]; used=set(excluded_signatures)
    for path in tests:
        sig=trajectory_signature(path)
        if sig in used:
            excluded_duplicates.append(str(path)); continue
        selected.append(path); used.add(sig)
        if len(selected)==10: break
    if len(selected)!=10: raise RuntimeError('Fewer than 10 disjoint test trajectories')
    source_artifacts=[Path(frozen['train_manifest']),Path(frozen['eval_manifest']),
                      args.source/'protocol.json',args.source/'summary.json',
                      Path(__file__),Path('hil-serl/tools/gfr1_build_consistent_model_manifest.py')]
    write_json(args.output/'selection_protocol.json',dict(
        selected=[dict(trajectory_path=str(p),content_sha256=trajectory_signature(p),file_sha256=digest(p)) for p in selected],
        selection_rule='First 10 sorted test trajectories after exact content deduplication against ALL dp_train, rl_train and val trajectories; no policy-outcome filtering',
        models=old['models'], model_assignment='round-robin by selected trajectory index: counts 3/3/2/2',
        excluded_path_count=len(excluded_paths), excluded_duplicates=excluded_duplicates,
        excluded_trajectory_hashes={str(p):trajectory_signature(p) for p in excluded_paths},
        source_hashes={str(p):digest(p) for p in source_artifacts},
        initialization='Existing LUT safe interval midpoint and existing 12-iteration Newton repair; no trajectory retiming',
        invalid_initialization_rule='Retain as initialization_failure for all methods, never replace by a successful case',
        methods=['original','alternative_only','combined'], policy='Frozen 40k, deterministic, no training',
        interpretation='New task trajectories within the same four perturbation models; not unseen-model or global safety evidence',
        historical_usage_audit_scope='Current 40k lineage, all source non-test splits, and 2026-11-01 repair/full-regression cases. No claim of never used by any other historical project.'))
    from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
    from tools.audit_static_collision import CollisionChecker
    from tools.validate_differential_lift import AviatorClosure
    from tools.gfr1_build_consistent_model_manifest import repair_q
    lookup=ManifoldLookup(frozen['environment']['AVIATOR_GFR_MANIFOLD'])
    lo,hi=np.concatenate(lookup.joint_lower),np.concatenate(lookup.joint_upper)
    cache={};records=[]
    for i,tp in enumerate(selected):
        model=old['models'][i%len(old['models'])]
        if model not in cache: cache[model]=(AviatorClosure(model),CollisionChecker(model))
        fk,checker=cache[model]
        with np.load(tp) as z:
            x=np.asarray(z['x'][0],dtype=float)
            duration=float(z['t'][-1]-z['t'][0]); samples=len(z['x'])
            max_xdot=np.max(np.abs(z['xdot']),axis=0)
            assert np.allclose(np.diff(z['t']),.01,atol=1e-10,rtol=0)
        box=lookup.safe_interval(x[None,:])
        phi=.5*(box['phi_safe_lo'][0]+box['phi_safe_hi'][0])
        start=lookup.query(x[None,:],phi[None,:],check_safe=False)
        q=np.r_[start['qL'][0],start['qR'][0]].astype(float)
        q,hist=repair_q(fk,x,q,lo,hi)
        e=fk.error(x,q);d,contact=checker.check(x,q)
        margin=float(min(np.min(q-lo),np.min(hi-q)))
        reasons=[]
        if not np.isfinite(q).all() or not np.isfinite(e).all(): reasons.append('nonfinite_initial_state')
        if np.max(np.abs(e))>1e-7: reasons.append('initial_closure_not_repaired')
        if d<frozen['d_safe']-1e-9 or contact: reasons.append('initial_clearance_or_contact')
        if margin < -1e-9: reasons.append('initial_joint_limit')
        rec=dict(trajectory_path=str(tp),model=model,initial_q=q.tolist(),initial_x=x.tolist(),
                 initial_closure_inf=float(np.max(np.abs(e))),initial_clearance_m=float(d),
                 initial_contact=bool(contact),initial_joint_margin_rad=margin,repair_history=hist,
                 initialization_valid=not reasons,initialization_failures=reasons,
                 trajectory_content_sha256=trajectory_signature(tp),trajectory_file_sha256=digest(tp),
                 model_sha256=digest(model),samples=samples,duration_s=duration,max_abs_xdot=max_xdot.tolist())
        records.append(rec)
        print(json.dumps({k:rec[k] for k in ('trajectory_path','initialization_valid','initialization_failures','initial_clearance_m') }),flush=True)
    write_json(args.output/'heldout_manifest.json',dict(kind='gfr1_consistent_model_manifest',split='test',
        models=old['models'],records=records,trajectory_count=10,record_count=10,
        frozen={k:frozen[k] for k in ('dt','d_safe','beta_stop','r_max','qdot_max')},
        selection_protocol=str(args.output/'selection_protocol.json')))
    write_json(args.output/'initialization_audit.json',dict(records=10,
        valid=sum(r['initialization_valid'] for r in records),
        non_test_content_overlap=0,unique_test_trajectories=len(set(r['trajectory_content_sha256'] for r in records)),
        no_policy_rollout_used_for_selection=True,training_started=False))


if __name__=='__main__':
    main()
