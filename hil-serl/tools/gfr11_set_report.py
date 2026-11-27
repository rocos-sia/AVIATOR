"""J4 paired report and bounded feasibility probes at new hard-case failures."""
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
    output=root/'failure_diagnostics';output.mkdir(exist_ok=False)
    dump(output/'protocol.json',dict(scope='Every s008/s009 failure of new J4 models; no baseline repeats',
        search='unchanged N0-I dense search; finite witnesses only',training_updates=0,source_sha256=sha(__file__)))
    probes=[]
    for model in result['models']:
        if model['reused']:continue
        actor=FrozenActor(model['checkpoint'])
        assert sha(model['checkpoint'])==model['checkpoint_sha256']
        for r in model['rows']:
            sid=r['state']
            if sid not in ('s008','s009') or r['completed']:continue
            path=json.loads((root/'evaluations'/model['model']/f'{sid}.json').read_text())
            last=path['rows'][-1];env=make_env(jobs[sid]['record'])
            try:
                obs=unpack_state(env,last['state_before']);action=actor(obs)
                np.testing.assert_array_equal(action,np.asarray(last['action'],np.float32))
                trial=assess(env,action);assert not trial['feasible']
                scan=search(env,action,dense=True)
                dump(output/f"{model['model']}_{sid}.json",dict(state=last['state_before'],observation=obs.tolist(),**scan))
                probes.append(dict(seed=model['seed'],group=model['group'],state=sid,
                    accepted_steps=r['accepted_steps'],reasons=trial['reasons'],
                    tested=scan['evaluations'],feasible_witnesses=scan['feasible_count']))
                dump(output/'progress.json',probes)
            finally:env.close()
        assert actor.unchanged()
    paired=[];baseline={m['seed']:m for m in result['models'] if m['group']=='0_MSE'}
    for m in result['models']:
        base=baseline[m['seed']]
        paired.append(dict(seed=m['seed'],group=m['group'],completed=m['completed'],
            old_retained=m['retained_original_successes'],hard_completed=m['difficult_completed'],
            hard_progress={r['state']:r['accepted_steps'] for r in m['rows'] if r['state'] in ('s008','s009')},
            failed_states=[r['state'] for r in m['rows'] if not r['completed']],
            completion_delta=m['completed']-base['completed'],
            old_fitting=m['old_fitting'],new_fitting=m['new_fitting'],
            all_repair_fitting=m.get('all_repair_fitting'),action_variation=m['action_variation']))
    pool_audits=[]
    for seed in (11,23,37):
        d=json.loads((root/f'pool_audit_seed{seed}.json').read_text());rows=d['rows']
        pool_audits.append(dict(seed=seed,changed_labels=d['changed_labels'],
            already_feasible_nonzero_set_loss=d['already_feasible_but_nonzero_finite_distance'],
            mean_old_label_action_change=float(np.mean([r['old_label_change_from_previous'] for r in rows])),
            mean_new_label_action_change=float(np.mean([r['relabeled_change_from_previous'] for r in rows]))))
    report=dict(summary=result['summary'],paired_models=paired,failure_witnesses=probes,
        pool_audits=pool_audits,baseline_bitwise_reproduced=True,
        caution='Fixed finite-candidate supervision ablation on development states. Not a continuous feasible-set field or robustness certificate. No heldout or full-task claims.',
        additional_training=0,residual_started=False)
    dump(root/'interpretation.json',report)
    # Scientific artifact: completion remains separate from progress before rejection.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    groups=list(result['summary'])
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    names=['Single label','Tie + continuity','Best-tie set','All feasible set']
    for ax,sid in zip(axes,('s008','s009')):
        for i,g in enumerate(groups):
            values=[m['hard_progress'][sid] for m in paired if m['group']==g]
            ax.scatter(np.asarray([i-.12,i,i+.12]),values,s=36,color=['#497da5','#d08c31','#358d7c'][0])
            ax.plot([i-.23,i+.23],[np.mean(values)]*2,color='#222222',lw=2)
        ax.axhline(250,color='#a55050',ls='--',lw=1,label='Segment target')
        ax.set(xticks=range(len(groups)),xticklabels=names,ylim=(0,265),ylabel='Accepted control steps',title=sid)
        ax.tick_params(axis='x',labelrotation=20);ax.grid(axis='y',alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle('J4 fixed-data comparison: three seeds per method')
    fig.savefig(root/'paired_progress.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    run(p.parse_args().root.resolve())
