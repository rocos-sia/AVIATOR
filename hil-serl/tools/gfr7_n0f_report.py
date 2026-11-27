"""Summarize completed single-observation interventions without new inference."""
import argparse
import json
from pathlib import Path
import numpy as np


def main(root):
    status=json.loads((root/'status.json').read_text())
    if status['phase']!='completed':raise RuntimeError('Wait for complete N0-F')
    result=json.loads((root/'results.json').read_text());rows=[]
    baselines={(r['state'],r['original_candidate']):r for r in result['replays'] if r['group']=='normal'}
    for entry in result['replays']:
        base=baselines[(entry['state'],entry['original_candidate'])]['summary'];s=entry['summary']
        rows.append(dict(state=entry['state'],origin=entry['original_candidate'],group=entry['group'],
            changed_input_indices=entry['changed_input_indices'],
            future_calls=s['calls'],baseline_future_calls=base['calls'],calls_saved=base['calls']-s['calls'],
            full_calls=s['calls']+base['first_original_action_calls'],
            fallback_cycles=s['fallback_cycles'],boundary_retries=s['boundary_retries'],
            first_direct=s['first_direct'],completed=s['completed'],
            delta_proposal_inf=entry['delta_proposal_inf'],
            delta_accepted_inf=float(np.max(np.abs(np.array(s['first_accepted'])-base['first_accepted']))),
            delta_final_q_inf=float(np.max(np.abs(np.array(s['final_q'])-base['final_q']))),
            alias_of=entry.get('alias_of')))
    f2=json.loads((root/'f2_local_features.json').read_text());a,b=[x['features'] for x in f2]
    joint=np.abs(np.array(a['q'])-b['q'])
    f2summary=dict(candidates=[x['original_candidate'] for x in f2],
        max_q_difference=float(joint.max()),left_q_difference=float(joint[:7].max()),right_q_difference=float(joint[7:].max()),
        history_a=a['explicit_history'],history_b=b['explicit_history'],
        limiting_signed_joint_indices=[a['limiting_signed_joint_index'],b['limiting_signed_joint_index']],
        next_actor_raw_a=a['next_actor_raw'],next_actor_raw_b=b['next_actor_raw'],
        next_actor_output_difference_inf=float(np.max(np.abs(np.array(a['next_actor_raw'])-b['next_actor_raw']))),
        next_nominal_joint_velocity_difference_inf=float(np.max(np.abs(np.array(a['next_nominal_qdot'])-b['next_nominal_qdot']))),
        next_signed_joint_rate_difference_inf=float(np.max(np.abs(np.array(a['signed_joint_margin_rates'])-b['signed_joint_margin_rates']))),
        collision_pair_margin_difference_inf=float(np.max(np.abs(np.array(a['collision_pair_margins'])-b['collision_pair_margins']))),
        collision_rate_difference_inf={eps:float(np.max(np.abs(np.array(a['collision_pair_rate_finite_difference'][eps])-b['collision_pair_rate_finite_difference'][eps]))) for eps in ('0.0001','1e-05')},
        static_cross_history=[r for r in result['static'] if r['state']=='s009'],
        interpretation='Feature differences are diagnostic, not evidence of generalizable prediction or causal sufficiency')
    report=dict(rows=rows,f2=f2summary,scope='One observation intervention, frozen network, independent original checker/audit; only two F1 states',
        deployment_qualified=False,trained=False)
    path=root/'interpretation.json'
    with path.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True)
    main(p.parse_args().root)
