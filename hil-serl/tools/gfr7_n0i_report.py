"""Audit and summarize completed N0-I records without new rollouts/training."""
import argparse
from pathlib import Path
import json

import numpy as np

from tools.gfr1_remount_evaluate import ROOT, dump


def run(root):
    result=json.loads((root/'results.json').read_text())
    original=ROOT/'outputs/gfr_foresight_n0_20261126'
    n0h=json.loads((ROOT/'outputs/gfr_foresight_n0h_20261127/results.json').read_text())
    labels={s['state']:s for s in n0h['states']}
    checks=[]; summary={}
    for sid,state in result['states'].items():
        for entry in state['rejection_audits']:
            ci=entry['candidate']
            path=json.loads((root/f'{sid}_origin_{ci}.json').read_text())
            previous=json.loads((original/'rollouts'/f'{sid}_{ci:02d}.json').read_text())['result']['rows']
            assert path['accepted_steps']==labels[sid]['candidates'][ci]['directly_accepted_steps']
            for i,row in enumerate(path['rows']):
                np.testing.assert_array_equal(np.asarray(row['action'],np.float32),np.asarray(previous[i]['raw'],np.float32))
                if row['passed']:
                    after=np.asarray(row['state_after'])
                    np.testing.assert_allclose(after[1:3],previous[i]['x'],atol=1e-12,rtol=0)
                    np.testing.assert_allclose(after[3:17],previous[i]['q'],atol=1e-12,rtol=0)
                    np.testing.assert_allclose(after[17:45],np.asarray(previous[i]['n']).ravel(),atol=1e-12,rtol=0)
                    assert abs(after[0]-previous[i]['u'])<1e-12
            checks.append(dict(state=sid,candidate=ci,original_independent_prefix_exact=True))
        origin=json.loads((root/f'{sid}_origin_1.json').read_text())
        trial=origin['rows'][-1]['info']
        first=state['rejection_audits'][0]
        summary[sid]=dict(actor_accepted_steps=origin['accepted_steps'],
             actor_rejected_proposal_number=origin['accepted_steps']+1,
             actor_accepted_progress_ms=origin['accepted_progress_s']*1000,
             rejected_clearance_mm=trial['d_actual_m']*1000,
             rejected_joint_margin_rad=trial['joint_limit_margin_rad'],
             rejected_speed_rad_s=trial['qdot_max_rad_s'],
             first_rejection_actions_tested=first['evaluations'],
             first_rejection_feasible_witnesses=first['feasible_count'],
             single_replacement_accepted_steps=[r['accepted_steps'] for r in first['single_replacements']],
             demo_completed=state['demo_witness']['completed'],controllers=state['controllers'])
    accepted=0; rejected=0
    for file in sorted(root.glob('s0*.json')):
        document=json.loads(file.read_text())
        if not document.get('rows') or 'state_before' not in document['rows'][0]:continue
        for row in document['rows']:
            before=np.asarray(row['state_before']);after=np.asarray(row['state_after'])
            if row['passed']:
                accepted+=1
                assert abs(after[0]-before[0]-.5)<1e-10
                np.testing.assert_array_equal(after[47:51],[1,0,0,0])
                np.testing.assert_allclose(after[45:47],np.asarray(row['action'],np.float32).astype(float)*1.5,atol=0,rtol=0)
            else:
                rejected+=1
                np.testing.assert_array_equal(before[:17],after[:17])
    report=dict(states=summary,replayed_prefix_checks=checks,
        accepted_recorded_transitions=accepted,rejected_recorded_attempts=rejected,
        all_accepted_beta1_unmodified_action=True,all_rejections_no_motion_commit=True,
        no_original_rejection_search_was_empty=all(v['first_rejection_feasible_witnesses'] for v in summary.values()),
        interpretation='Finite witness searches and external controllers; no new Actor learning or Actor-only success gain. No proof of general viability or of absence of lookahead value.')
    dump(root/'interpretation.json',report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for ax,sid in zip(axs,('s008','s009')):
        data=json.loads((root/f'{sid}_scan_1.json').read_text())
        for feasible,color,label in [(False,'#ca6147','Rejected'),(True,'#26977e','Directly feasible')]:
            rows=[r['action'] for r in data['rows'] if r['feasible']==feasible]
            if rows:
                a=np.asarray(rows);ax.scatter(a[:,0],a[:,1],s=11,c=color,label=label,alpha=.8)
        action=data['rows'][0]['action']
        ax.scatter(*action,s=140,c='#171717',marker='*',label='Frozen Actor',zorder=5)
        ax.set(xlim=(-1.08,1.08),ylim=(-1.08,1.08),xlabel='Left normalized redundancy action',
               ylabel='Right normalized redundancy action',title=f'{sid}: state before first Actor rejection')
        ax.set_aspect('equal');ax.grid(alpha=.15)
    axs[0].legend(fontsize=8,loc='best')
    fig.suptitle('Finite direct-feasibility witnesses (beta=1; no projection)')
    fig.savefig(root/'first_rejection_action_search.png',dpi=160)
    plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    run(p.parse_args().root.resolve())
