"""N0-E: select among existing candidates only; no simulator or learner calls."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from tools.gfr7_analyze_existing import prefix


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(source, output):
    output.mkdir(parents=True, exist_ok=False)
    files = [source/'results.json',source/'states.json',source/'protocol.json',
             *sorted((source/'rollouts').glob('*.json'))]
    frozen = {str(p):digest(p) for p in files}
    summaries=json.loads((source/'results.json').read_text())
    rows=[]
    totals=Counter()
    for summary in summaries:
        sid=summary['id']
        docs=[json.loads((source/'rollouts'/f'{sid}_{i:02d}.json').read_text()) for i in range(11)]
        rr=[d['result'] for d in docs]
        assert all(r['completed'] for r in rr)
        assert all(not step['violations'] for r in rr for step in r['rows'])
        assert len({r['target_u'] for r in rr})==1
        costs=[r['candidate_calls'] for r in rr]
        scores=[prefix(r,.005) for r in rr]
        ties=[i for i in range(11) if scores[i]==min(scores)]
        assert all(rr[i]['rows'][0]['direct'] for i in ties)
        first=[r['rows'][0] for r in rr]
        features={
            'joint_margin':[r['joint_margin_rad'] for r in first],
            'clearance':[r['clearance_m'] for r in first],
            'balanced_margin':[min(r['joint_margin_rad']/.1,(r['clearance_m']-.005)/.001) for r in first],
        }
        methods={'actor':dict(selected=1),'original_one_step':dict(selected=ties[0]),
                 'foresight_025_within_ties':dict(selected=min(ties,key=lambda i:prefix(rr[i],.25))),
                 'long_oracle':dict(selected=min(range(11),key=lambda i:(costs[i],rr[i]['steps'])))}
        tolerances={'joint_margin':1e-10,'clearance':1e-9,'balanced_margin':1e-6}
        for name,values in features.items():
            best=max(values[i] for i in ties)
            winners=[i for i in ties if best-values[i]<=tolerances[name]]
            methods[name]=dict(selected=winners[0], remaining_ties=winners,
                               tolerance=tolerances[name], scores={str(i):values[i] for i in ties})
        for name,value in methods.items():
            i=value['selected'];value['cost']=costs[i];value['regret']=costs[i]-min(costs)
            value['completed']=rr[i]['completed'];totals[name]+=value['cost']
        random_cost=sum(costs[i] for i in ties)/len(ties)
        methods['uniform_random_ties']=dict(expected_cost=random_cost,
             expected_regret=random_cost-min(costs), optimal_probability=sum(costs[i]==min(costs) for i in ties)/len(ties))
        totals['uniform_random_ties']+=random_cost
        identical=[]
        for position,i in enumerate(ties):
            for j in ties[:position]:
                if first[i]['joint_margin_rad']==first[j]['joint_margin_rad'] and first[i]['clearance_m']==first[j]['clearance_m'] and costs[i]!=costs[j]:
                    identical.append(dict(indices=[j,i], cost=[costs[j],costs[i]],
                         joint_margin_rad=first[i]['joint_margin_rad'],clearance_m=first[i]['clearance_m'],
                         normalized_proposals=[docs[j]['proposal']['action'],docs[i]['proposal']['action']]))
        rows.append(dict(state=sid,case=summary['case'],scene=summary['scene'],
            actor_cost=costs[1], oracle_cost=min(costs), actor_headroom=costs[1]-min(costs),
            actor_direct_first=first[1]['direct'], previous_direct_first=first[0]['direct'],
            one_step_ties=ties,methods=methods,identical_scalar_geometry_different_cost=identical))
    result=dict(scope='Existing 10 states and 110 rollouts only; no new rollout or training',
        rule_definition=dict(joint_margin='maximize first accepted minimum joint margin',
             clearance='maximize first accepted minimum swept/endpoint clearance',
             balanced_margin='maximize min(joint_margin/.1, (clearance-.005)/.001)',
             ties='all rules operate within the common-progress one-step best set; remaining ties retain original ordering',
             near_tie_tolerances=tolerances,
             foresight='minimize calls then steps to .25 nominal progress within one-step tied actions',
             random='exact expectation over existing one-step tied candidate entries, no random replay',
             oracle='minimum to common 1.25 nominal progress among all completed candidates'),
        caveats=['Retrospective development-only comparisons; rules/scales are not validated on heldout data.',
                 'Saved audit geometry equals controller geometry here. No true-model features are approved for residual-error deployment.',
                 'All continuations use the same Actor, which observes past action/flags and .1/.3 s reference preview.',
                 'Policy is feedforward; explicit history channels are not recurrent hidden state.',
                 'Candidate search costs are excluded, and candidate counts are not milliseconds.',
                 'No geometry/history causal replay or robustness test has been performed by this script.'],
        totals=dict(totals),actor_headroom_total=sum(r['actor_headroom'] for r in rows),
        states_with_actor_headroom=[r['state'] for r in rows if r['actor_headroom']>0],
        rows=rows,source_sha256=frozen,analysis_source_sha256=digest(Path(__file__)))
    assert all(digest(Path(p))==h for p,h in frozen.items())
    result['inputs_unchanged']=True
    (output/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:result[k] for k in ('totals','actor_headroom_total','states_with_actor_headroom','inputs_unchanged')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.source.resolve(),args.output.resolve())
