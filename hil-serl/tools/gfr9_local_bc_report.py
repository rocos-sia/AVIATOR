"""Post-run failure diagnostics; never add labels to training or update weights."""
import argparse
import json
from pathlib import Path
import time

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
    jobs={r['id']:r for r in json.loads((ROOT/'outputs/gfr_foresight_n0_20261126/states.json').read_text()) if r['split']=='development'}
    metadata=json.loads((root/'dataset_metadata.json').read_text())
    with np.load(root/'dataset.npz') as data:observations=data['observations'].copy()
    output=root/'failure_diagnostics';output.mkdir(exist_ok=False)
    dump(output/'protocol.json',dict(selection='All s008/s009 failures at update3000, all three seeds',
        purpose='Check current-action feasibility at new student failures; no teacher continuation or new training',
        search='unchanged N0-I dense finite search',heldout_queried=False,source_sha256=sha(__file__)))
    rows=[];started=time.monotonic()
    for model in result['models']:
        if model['update']!=3000:continue
        actor=FrozenActor(model['checkpoint'])
        assert sha(model['checkpoint'])==model['checkpoint_sha256']
        for row in model['rows']:
            if row['state'] not in ('s008','s009') or row['completed']:continue
            sid=row['state'];job=jobs[sid]
            trajectory=json.loads((root/'evaluations'/model['model']/f'{sid}.json').read_text())
            last=trajectory['rows'][-1];state=last['state_before']
            env=make_env(job['record'])
            try:
                obs=unpack_state(env,state);action=actor(obs)
                np.testing.assert_array_equal(action,np.asarray(last['action'],np.float32))
                check=assess(env,action);assert not check['feasible']
                scan=search(env,action,dense=True)
                dump(output/f"seed{model['seed']}_{sid}.json",dict(state=state,observation=obs.tolist(),**scan))
                ids=np.asarray([i for i,m in enumerate(metadata) if m['case']==job['record']['case']])
                distances=np.linalg.norm((observations[ids]-obs)/actor.actor.scale.detach().cpu().numpy(),axis=1)
                nearest=int(ids[np.argmin(distances)])
                rows.append(dict(seed=model['seed'],state=sid,accepted_steps=row['accepted_steps'],
                    reasons=check['reasons'],actor_action=action.tolist(),
                    tested_candidates=scan['evaluations'],feasible_witnesses=scan['feasible_count'],
                    teacher_best_action=scan['best']['action'],
                    closest_training_sample=nearest,closest_training_metadata=metadata[nearest],
                    normalized_observation_l2=float(distances.min()),
                    exact_observation_in_training=bool(np.any(np.all(observations[ids]==obs,axis=1)))))
                dump(output/'progress.json',rows)
            finally:env.close()
        assert actor.unchanged()
    dump(root/'interpretation.json',dict(development_gate_passed=result['passed'],
        models=[dict(seed=m['seed'],update=m['update'],completed=m['completed'],
            retained=m['retained_original_successes'],difficult=m['difficult_completed'],fitting=m['fitting'],
            failures=[dict(state=r['state'],steps=r['accepted_steps']) for r in m['rows'] if not r['completed']]) for m in result['models']],
        final_checkpoint_failure_diagnostics=rows,
        all_probed_failures_have_feasible_alternatives=all(r['feasible_witnesses']>0 for r in rows) if rows else None,
        qualification='Development cases used in label generation; no heldout/generalization/speed claim',
        inference='If fitting audits pass but student rollouts reject with feasible alternatives, this supports new-state action-selection errors, not a proved lack of future viability. It does not isolate the cause to state coverage alone.',
        additional_training_updates=0,elapsed_s=time.monotonic()-started))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    run(p.parse_args().root.resolve())
