#!/usr/bin/env python3
"""Freeze G1--G3 before outcomes; retain invalid starts and audit prior use."""
import argparse,hashlib,json,re
from pathlib import Path
import numpy as np
from tools.gfr1_same_state_diagnosis import digest,write_json
from tools.gfr1_prepare_60k_holdout import scan_history,choose
from tools.gfr1_prepare_unseen_regression import trajectory_signature

ROOT=Path(__file__).resolve().parents[2]
RUN=ROOT/'outputs/differential_lift_feasibility_20261102/gfr_repaired_scratch/formal_seed11'

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    frozen=json.loads((RUN/'frozen_config.json').read_text());train=json.loads(Path(frozen['train_manifest']).read_text())
    trajectories=Path(frozen['environment']['AVIATOR_GFR_TRAJECTORY_DIR'])
    names,history=scan_history(ROOT)
    excluded={str(f):trajectory_signature(f) for split in ('dp_train','rl_train','val') for f in sorted((trajectories/'trajs'/split).glob('*.npz'))}
    previous={str(trajectories/'trajs/test'/name):trajectory_signature(trajectories/'trajs/test'/name) for name in names if (trajectories/'trajs/test'/name).exists()}
    selected,rejected=choose(list((trajectories/'trajs/test').glob('*.npz')),names,set(excluded.values())|set(previous.values()),60)
    from tools.run_n16_installation_error import make_variant
    base=Path(frozen['environment']['AVIATOR_GFR_MODEL']);models=a.output/'models';models.mkdir()
    specs=[('fixture_translation',.0004),('fixture_translation',.0007),('base_translation',.0007),
           ('base_translation',.0013),('fixture_rotation',.15),('base_rotation',.15)]
    generated=[]
    for family,level in specs:
        model=models/f'{family}_{level:g}.xml'
        spec=make_variant(base,model,family,level)
        # Relocating XML must not relocate mesh content.
        text=model.read_text().replace('meshdir="aviator_meshes"',f'meshdir="{(base.parent/"aviator_meshes").resolve()}"')
        model.write_text(text)
        spec.update(model=str(model),model_sha256=digest(model),
                    stratum='within_training_family_new_magnitude' if family=='fixture_translation' else 'untrained_error_family')
        generated.append(spec)
    # Compare compiled geometry transforms: insensitive to XML whitespace,
    # filenames and equivalent quaternion normalization. Mesh contents are
    # shared with the frozen source; topology and geometric arrays are included.
    import mujoco
    def geometry_signature(path):
        model=mujoco.MjModel.from_xml_path(str(path))
        keys=('body_parentid','body_pos','body_quat','geom_type','geom_bodyid','geom_pos',
              'geom_quat','geom_size','geom_dataid','site_bodyid','site_pos','site_quat',
              'jnt_type','jnt_bodyid','jnt_pos','jnt_axis','jnt_range','mesh_vert','mesh_face')
        h=hashlib.sha256()
        for key in keys:
            arr=np.asarray(getattr(model,key))
            if arr.dtype.kind=='f':
                arr=np.round(arr,10);arr[arr==0]=0.
            h.update(key.encode());h.update(str(arr.shape).encode());h.update(arr.tobytes())
        return h.hexdigest()
    prior_models=sorted((ROOT/'AviatorRobot/build/bin/model').glob('*.xml'))
    prior={str(x):geometry_signature(x) for x in prior_models}
    for spec in generated:
        g=geometry_signature(spec['model'])
        matches=[path for path,sig in prior.items() if g==sig]
        if matches:raise RuntimeError(f'new model duplicates old geometry: {matches}')
    known=list(dict.fromkeys(r['model'] for r in train['records']))
    known_tasks=list(dict.fromkeys(r['trajectory_path'] for r in train['records']))[:30]
    assert len(known_tasks)==30
    assignments={
        'G1':[(str(t),known[i%len(known)],'seen_training_model') for i,t in enumerate(selected[:30])],
        'G2':[(t,generated[i%6]['model'],generated[i%6]['stratum']) for i,t in enumerate(known_tasks)],
        'G3':[(str(t),generated[i%6]['model'],generated[i%6]['stratum']) for i,t in enumerate(selected[30:])],
    }
    protected={str(RUN/'training/b4_seed11/checkpoint_60000'):digest(RUN/'training/b4_seed11/checkpoint_60000'),
               str(RUN/'frozen_config.json'):digest(RUN/'frozen_config.json'),frozen['train_manifest']:digest(frozen['train_manifest']),
               frozen['environment']['AVIATOR_GFR_MANIFOLD']:digest(frozen['environment']['AVIATOR_GFR_MANIFOLD'])}
    for group,items in assignments.items():
        for t,m,_ in items:protected[t]=digest(t);protected[m]=digest(m)
    write_json(a.output/'selection_protocol.json',dict(kind='gfr_scheduler_G1_G3_frozen_selection',
        checkpoint=str(RUN/'training/b4_seed11/checkpoint_60000'),training_seed=11,
        selection_rule='60 first sorted content-disjoint unreferenced test trajectories; first30 G1, next30 G3. G2 first30 unique successful-demo trajectories. Round-robin model assignments before rollouts.',
        history_audit=history,excluded_signatures=excluded,previous_test_signatures=previous,rejected=rejected,
        assignments=assignments,generated_models=generated,known_training_models=known,old_model_scan_count=len(prior_models),
        model_novelty_scope='Compiled geometric arrays differ from all existing top-level AviatorRobot/build/bin/model XMLs (round10 decimals); no claim against unlogged or external use.',
        frozen=dict(dt=.005,d_safe=.005,beta_stop=.125,r_max=1.5,qdot_max=1.5),
        initialization='LUT midpoint and existing 12-iteration Newton correction, no retiming, invalid starts counted separately and retained; paired q identical across methods.',
        information='Calibrated model supplied equally to FK/checker of all methods; actor sees existing40D observation. Not hidden-parameter estimation.',
        primary_metrics=['completion','accepted_validation_violations','progress','interventions','alternative_recoveries','total_compute_s','cycle_P95_P99','distance_and_swept_calls_per_progress'],
        inference='Descriptive paired results and paired bootstrap intervals; cases independent at trajectory level, model-stratified results required. Single policy seed, not training-seed robustness.',
        decisions='All cases and methods reported regardless of outcome. No tuning, test selection, or automatic retraining. Teacher/fixed/greedy share combined executor.',
        protected_input_hashes=protected))
    from experiments.aviator_manifold.manifold_lookup import ManifoldLookup
    from tools.audit_static_collision import CollisionChecker
    from tools.validate_differential_lift import AviatorClosure
    from tools.gfr1_build_consistent_model_manifest import repair_q
    lookup=ManifoldLookup(frozen['environment']['AVIATOR_GFR_MANIFOLD']);lo=np.concatenate(lookup.joint_lower);hi=np.concatenate(lookup.joint_upper)
    cache={};audit={}
    for group,items in assignments.items():
        records=[]
        for i,(t,m,stratum) in enumerate(items):
            if m not in cache:cache[m]=(AviatorClosure(m),CollisionChecker(m))
            fk,checker=cache[m]
            with np.load(t) as z:
                x=np.asarray(z['x'][0],dtype=float)
                assert np.allclose(np.diff(z['t']),.01,atol=1e-10,rtol=0)
            box=lookup.safe_interval(x[None,:]);phi=.5*(box['phi_safe_lo'][0]+box['phi_safe_hi'][0])
            start=lookup.query(x[None,:],phi[None,:],check_safe=False);q=np.r_[start['qL'][0],start['qR'][0]].astype(float)
            q,hist=repair_q(fk,x,q,lo,hi);e=fk.error(x,q);d,contact=checker.check(x,q)
            margin=float(min(np.min(q-lo),np.min(hi-q)));reasons=[]
            if not np.isfinite(q).all() or not np.isfinite(e).all():raise RuntimeError('nonfinite initialization')
            if np.max(np.abs(e))>1e-7:reasons.append('initial_closure')
            if d<.005-1e-9 or contact:reasons.append('initial_clearance_or_contact')
            if margin< -1e-9:reasons.append('initial_joint_limit')
            records.append(dict(case_id=f'{group}_{i:02d}',group=group,stratum=stratum,trajectory_path=t,model=m,
                initial_q=q.tolist(),initial_x=x.tolist(),initialization_valid=not reasons,initialization_failures=reasons,
                initial_closure_inf=float(np.max(np.abs(e))),initial_clearance_m=float(d),initial_joint_margin_rad=margin,repair_history=hist))
        write_json(a.output/f'{group}_manifest.json',dict(records=records,models=list(dict.fromkeys(r['model'] for r in records)),split='test',group=group))
        audit[group]=dict(records=len(records),valid=sum(r['initialization_valid'] for r in records),invalid=[r['case_id'] for r in records if not r['initialization_valid']])
    write_json(a.output/'initialization_audit.json',audit)
    print(json.dumps(audit),flush=True)

if __name__=='__main__':main()
