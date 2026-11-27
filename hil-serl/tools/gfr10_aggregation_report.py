"""Paired J3 interpretation and final C-branch failure witness checks only."""
import argparse
import json
from pathlib import Path

import numpy as np

from tools.gfr7_n0_qualify import FrozenActor
from tools.gfr7_n0i_feasibility import assess,search
from tools.gfr2_teacher import make_env
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr1_train_direct_only import install_guards
from tools.gfr1_remount_evaluate import ROOT,dump,sha
from tools.gfr4_common import require_memory_limit


def run(root):
    require_memory_limit();install_guards()
    result=json.loads((root/'results.json').read_text())
    jobs={s['id']:s for s in json.loads((ROOT/'outputs/gfr_foresight_n0_20261126/states.json').read_text()) if s['split']=='development'}
    models={(m['seed'],m['group']):m for m in result['models']}
    paired=[]
    for seed in (11,23,37):
        base=models[(seed,'A')]
        for group in ('B','C'):
            m=models[(seed,group)]
            paired.append(dict(seed=seed,group=group,completion_difference=m['completed']-base['completed'],
                hard_completion_difference=m['difficult_completed']-base['difficult_completed'],
                old_retention_difference=m['retained_original_successes']-base['retained_original_successes'],
                progress_differences={r['state']:r['accepted_steps']-next(x['accepted_steps'] for x in base['rows'] if x['state']==r['state']) for r in m['rows']}))
    directory=root/'C_failure_diagnostics';directory.mkdir(exist_ok=False)
    dump(directory/'protocol.json',dict(scope='All new C-branch s008/s009 failures; no training or aggregation of diagnostic labels',
        search='unchanged N0-I dense finite search',source_sha256=sha(__file__)))
    probes=[]
    for m in result['models']:
        if m['group']!='C':continue
        actor=FrozenActor(m['checkpoint'])
        assert sha(m['checkpoint'])==m['checkpoint_sha256']
        for row in m['rows']:
            sid=row['state']
            if sid not in ('s008','s009') or row['completed']:continue
            data=json.loads((root/'evaluations'/m['model']/f'{sid}.json').read_text())
            last=data['rows'][-1]
            env=make_env(jobs[sid]['record'])
            try:
                obs=unpack_state(env,last['state_before']);raw=actor(obs)
                np.testing.assert_array_equal(raw,np.asarray(last['action'],np.float32))
                trial=assess(env,raw);assert not trial['feasible']
                scan=search(env,raw,dense=True)
                dump(directory/f"seed{m['seed']}_{sid}.json",dict(state=last['state_before'],observation=obs.tolist(),**scan))
                probes.append(dict(seed=m['seed'],state=sid,accepted_steps=row['accepted_steps'],
                    reasons=trial['reasons'],evaluations=scan['evaluations'],feasible_witnesses=scan['feasible_count']))
            finally:env.close()
        assert actor.unchanged()
    audit=json.loads((root/'old_label_audit.json').read_text())
    report=dict(summary=result['summary'],paired_comparisons=paired,
        models=[dict(seed=m['seed'],group=m['group'],completed=m['completed'],retained=m['retained_original_successes'],
            old_fitting=m['old_fitting'],new_fitting=m['new_fitting'],added_samples=m['added_samples'],
            added_repairs=m['added_repairs'],failures=[dict(state=r['state'],steps=r['accepted_steps']) for r in m['rows'] if not r['completed']]) for m in result['models']],
        final_C_failure_witnesses=probes,
        old_label_audit=dict(gradients=[dict(seed=m['seed'],**m['gradient']) for m in audit['model_audits']],
            feasible_interpolation_trials=sum(m['interpolation_trials'] for m in audit['model_audits']),
            infeasible_interiors=sum(m['infeasible_interior'] for m in audit['model_audits']),
            close_large_target_jumps=audit['close_large_jump_diagnostic']),
        training_rounds=1,no_second_round=True,no_heldout_queries=True,
        limitation='Development-state aggregation only; progress gains are not completed trajectories, and nonempty finite witness sets do not prove global viability')
    tie_path=root/'teacher_tie_diagnostic.json'
    if tie_path.exists():
        tie=json.loads(tie_path.read_text());rows=tie['rows'];limiters=tie['score_limiters']
        report['posthoc_teacher_tie_audit']=dict(samples=len(rows),
            tied_labels=sum(r['ties']>1 for r in rows),
            tied_count_range=[min(r['ties'] for r in rows),max(r['ties'] for r in rows)],
            clearance_limited=sum(r['limiter']=='clearance' for r in limiters),
            start_clearance_limited_within_2nm=sum(abs(r['start_d']-r['candidate_d'])<2e-9 for r in limiters),
            warning='Scoring plateaus allow ordering-dependent labels; no causal teacher-rule ablation was performed and no training labels changed')
    dump(root/'interpretation.json',report)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    run(p.parse_args().root.resolve())
