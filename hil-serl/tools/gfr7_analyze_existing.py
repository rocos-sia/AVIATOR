"""Read-only diagnosis of existing N0 rollouts. No simulator or policy calls."""
import argparse
import hashlib
import json
from pathlib import Path
from collections import Counter

import numpy as np
from scipy.stats import spearmanr, kendalltau
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prefix(result, horizon):
    for i, row in enumerate(result['rows']):
        if (row['u']-result['start_u'])*.01 >= horizon-1e-9:
            return sum(x['candidate_calls'] for x in result['rows'][:i+1]), i+1
    raise ValueError('Prefix target not reached')


def components(result):
    rows = result['rows']
    boundary = sum(r['boundary_iterations'] for r in rows)
    recovery = sum(r['recovery'] for r in rows)
    assert not recovery, 'This decomposition requires no alternative-action recovery'
    assert result['candidate_calls'] == result['steps']+result['projection_calls']
    extra = result['projection_calls']-result['fallback_steps']-boundary
    assert extra >= 0
    return dict(candidate_calls=result['candidate_calls'], direct_trials=result['steps'],
                projection_calls=result['projection_calls'], fallback_cycles=result['fallback_steps'],
                boundary_retry_calls=boundary, extra_governor_attempts=extra,
                alternative_recoveries=recovery)


def stats_pairs(x, y):
    concordant = discordant = x_tie_only = y_tie_only = both_tied = 0
    for i in range(len(x)):
        for j in range(i):
            dx, dy = x[i]-x[j], y[i]-y[j]
            if dx == 0 and dy == 0:
                both_tied += 1
            elif dx == 0:
                x_tie_only += 1
            elif dy == 0:
                y_tie_only += 1
            elif dx*dy > 0:
                concordant += 1
            else:
                discordant += 1
    return dict(concordant=concordant, discordant=discordant,
                one_step_ties_but_different_future=x_tie_only,
                future_ties_but_different_one_step=y_tie_only, both_tied=both_tied)


def pair_details(cache, sid, indices):
    rr = [cache[sid][i]['result'] for i in indices]
    records = []
    for i, r in zip(indices, rr):
        rows = r['rows']
        def first_where(key, predicate):
            found = next((j for j, row in enumerate(rows) if predicate(row[key])), None)
            return None if found is None else dict(zero_based_step=found,
                elapsed_s=(found+1)*.005, nominal_progress_s=(rows[found]['u']-r['start_u'])*.01)
        records.append(dict(index=i, proposal=cache[sid][i]['proposal'], components=components(r),
            prefix={str(h): prefix(r, h) for h in (.005,.05,.1,.15,.2,.25,.3,.5,1.,1.25)},
            first_fallback=first_where('direct', lambda x: not x),
            first_boundary_retry=first_where('boundary_iterations', lambda x: x > 0),
            first_slowdown=first_where('beta', lambda x: x < 1),
            min_clearance_m=min(x['clearance_m'] for x in rows),
            min_joint_margin_rad=min(x['joint_margin_rad'] for x in rows),
            first_row=rows[0]))
    comparable = []
    for h in (.005,.05,.1,.15,.2,.25,.3,.5,1.,1.25):
        samples = [r['rows'][prefix(r,h)[1]-1] for r in rr]
        comparable.append(dict(nominal_progress_s=h,
             first_crossing_q_difference_inf_rad=float(np.max(np.abs(np.array(samples[0]['q'])-samples[1]['q']))),
             progress_overshoot_s=[(s['u']-r['start_u'])*.01-h for s,r in zip(samples,rr)],
             joint_margin_rad=[s['joint_margin_rad'] for s in samples],
             clearance_m=[s['clearance_m'] for s in samples]))
    return dict(state=sid, candidates=records, comparisons=comparable)


def run(source, out):
    out.mkdir(parents=True, exist_ok=False)
    files = [source/'results.json', source/'protocol.json', source/'states.json', *sorted((source/'rollouts').glob('*.json'))]
    before = {str(p.resolve()): sha(p) for p in files}
    summaries = json.loads((source/'results.json').read_text())
    cache = {s['id']: [json.loads((source/'rollouts'/f"{s['id']}_{i:02d}.json").read_text()) for i in range(11)] for s in summaries}
    state_stats = []
    totals = {name: Counter() for name in ('previous','one_step','h100','actor')}
    fig, axes = plt.subplots(2, 5, figsize=(17, 7.5))
    for ax, summary in zip(axes.ravel(), summaries):
        sid = summary['id']; docs = cache[sid]; results = [d['result'] for d in docs]
        assert all(r['completed'] for r in results)
        scores = [prefix(r, .005) for r in results]
        x = np.array([v[0] for v in scores]); y = np.array([r['candidate_calls'] for r in results])
        winner = min(range(len(results)), key=lambda i:scores[i])
        assert winner == summary['analysis']['selections']['one_step']['index']
        ties = [i for i,score in enumerate(scores) if score == scores[winner]]
        best = np.flatnonzero(y==y.min()).tolist()
        unique = []
        for i, d in enumerate(docs):
            if not any(d['proposal']['action']==docs[j]['proposal']['action'] for j in unique):
                unique.append(i)
        rho = float(spearmanr(x,y).statistic) if np.ptp(x)>0 and np.ptp(y)>0 else None
        tau = float(kendalltau(x,y).statistic) if np.ptp(x)>0 and np.ptp(y)>0 else None
        # Re-order existing tied candidates; no new rollout is generated.
        grid_order = list(range(2,11))+[0,1]
        grid_winner = min(grid_order, key=lambda i:scores[i])
        worst_tie = max(ties,key=lambda i:y[i])
        record = dict(state=sid, case=summary['case'], scene=summary['scene'], unique_actions=len(unique),
            one_step_scores=scores, long_costs=y.tolist(), one_step_tie_indices=ties,
            long_optimal_indices=best, near_optimal_within_2_calls=np.flatnonzero(y<=y.min()+2).tolist(),
            cost_gap_to_next_distinct=int(min(y[y>y.min()])-y.min()) if np.any(y>y.min()) else None,
            spearman=rho,kendall_tau_b=tau,pair_ranking=stats_pairs(x,y),
            optimal_probability_uniform_one_step_ties=len(set(ties)&set(best))/len(ties),
            expected_cost_uniform_one_step_ties=float(np.mean(y[ties])),
            grid_first_tiebreak_index=grid_winner,grid_first_tiebreak_cost=int(y[grid_winner]),
            worst_one_step_tie_cost=int(y[worst_tie]),
            chosen_one_step=winner,
            components=[components(r) for r in results])
        state_stats.append(record)
        for name,i in [('previous',0),('one_step',winner),('h100',summary['analysis']['selections']['h100']['index']),('actor',1)]:
            totals[name].update(components(results[i]))
        # Size encodes overplotted candidates, not an invented jittered score.
        grouped=Counter(zip(x.tolist(),y.tolist()))
        for (xx,yy),count in grouped.items():
            ax.scatter(xx,yy,s=35+30*count,color='#3a79b8',alpha=.65)
            if count >= 4:
                ax.annotate(str(count),(xx,yy),xytext=(6,5),textcoords='offset points',fontsize=8)
        ax.scatter(x[winner],y[winner],s=150,marker='x',color='#cc4422',linewidths=2)
        ax.axhline(y.min(),color='#24854b',linestyle=':',linewidth=1)
        ax.set_title(f'{sid} | rho={rho:.2f}' if rho is not None else f'{sid} | all future costs tied')
        ax.set_xlabel('One-step candidate calls');ax.set_ylabel('Calls to common 1.25 s progress')
        ax.set_xticks(sorted(set(x)));ax.grid(alpha=.15);ax.margins(x=.15,y=.15)
    fig.suptitle('Existing 110 rollouts: one-step score vs subsequent execution cost\nPoint size = coincident candidates (counts labeled when >=4); red x = chosen action; green line = finite-set minimum')
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(out/'one_step_vs_future.png',dpi=180);plt.close(fig)

    pairs = [pair_details(cache,'s008',[2,4]),pair_details(cache,'s009',[1,6]),
             pair_details(cache,'s004',[0,1]),pair_details(cache,'s007',[0,1])]
    fig, axes = plt.subplots(5,2,figsize=(13,15),sharex='col')
    for col,(sid,indices) in enumerate([('s008',[2,4]),('s009',[1,6])]):
        for i,color in zip(indices,('#cd5b35','#2476ad')):
            d=cache[sid][i];r=d['result'];rows=r['rows']
            t=np.array([(row['u']-r['start_u'])*.01 for row in rows]);label=f"candidate {i}: {r['candidate_calls']} calls"
            series=[np.cumsum([row['candidate_calls'] for row in rows]),
                    np.cumsum([row['boundary_iterations'] for row in rows]),
                    np.array([row['joint_margin_rad'] for row in rows]),
                    (np.array([row['clearance_m'] for row in rows])-.005)*1000,
                    [row['beta'] for row in rows]]
            for ax,y in zip(axes[:,col],series):ax.plot(t,y,color=color,label=label,linewidth=1.4);ax.grid(alpha=.2)
        axes[0,col].set_title(f'{sid}: '+('long-horizon benefit' if col==0 else '0.1 s selection regression'))
        axes[0,col].legend(fontsize=9)
        for ax in axes[:,col]:ax.axvline(.1,color='gray',ls=':',lw=1);ax.axvline(.25,color='gray',ls=':',lw=1)
        axes[-1,col].set_xlabel('Accepted reference progress from fork [nominal s]')
    for row,label in enumerate(['Cumulative candidate calls','Cumulative joint-boundary retries',
                               'Minimum joint margin [rad]','Clearance above 5 mm [mm]','Governor beta']):
        axes[row,0].set_ylabel(label)
    fig.suptitle('Existing paired rollouts; common progress endpoints, frozen Actor continuation\nVertical guides: 0.1 / 0.25 nominal seconds. Margins reflect the existing numeric acceptance tolerances.')
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/'anomaly_traces.png',dpi=160);plt.close(fig)

    fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True)
    for col,(sid,indices) in enumerate([('s008',[2,4]),('s009',[1,6])]):
        for i,color in zip(indices,('#cd5b35','#2476ad')):
            r=cache[sid][i]['result'];rows=r['rows'];t=np.arange(1,len(rows)+1)*.005
            for dim in (0,1):
                axes[dim,col].plot(t,[row['executed'][dim] for row in rows],
                                  color=color,label=f'candidate {i}',alpha=.8,lw=1.1)
        axes[0,col].set_title(sid);axes[0,col].legend();axes[-1,col].set_xlabel('Elapsed integration time [s]')
        for dim in (0,1):
            axes[dim,col].set_ylabel(f'Executed r[{dim}] [rad/s]')
            axes[dim,col].grid(alpha=.2);axes[dim,col].set_xlim(0,.6)
    fig.suptitle('Executed redundancy actions: first 0.6 seconds of existing paired rollouts')
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/'executed_actions.png',dpi=160);plt.close(fig)

    report = dict(scope='Analysis of existing 10 development states / 110 candidate rollouts only; no training or new rollout',
        score_definition='Reach common progress, then cumulative _candidate call count, then step count; fixed proposal order breaks ties',
        ties_order='previous, frozen Actor, row-major 3x3 grid',
        call_definition='_candidate invocation; one contains lift/integration/checks and may contain one projection. Not individual collision checks.',
        component_identity='calls = direct_trials + fallback_cycles + boundary_retry_calls + extra_governor_attempts; valid here because no alternative recoveries',
        source_sha256=before, analysis_source_sha256=sha(Path(__file__)), totals={k:dict(v) for k,v in totals.items()},
        states=state_stats, anomalous_and_high_gain_pairs=pairs,
        random_tie_expected_optimal_states=sum(s['optimal_probability_uniform_one_step_ties'] for s in state_stats),
        random_tie_expected_total_calls=sum(s['expected_cost_uniform_one_step_ties'] for s in state_stats),
        grid_first_tiebreak_total_calls=sum(s['grid_first_tiebreak_cost'] for s in state_stats),
        all_future_costs_tied_states=[s['state'] for s in state_stats if len(set(s['long_costs']))==1],
        caveats=['Counterfactual outcomes share one frozen continuation Actor; cannot attribute changes to geometry vs history flags separately.',
                 'No alternate continuation, denser action grid, tolerance sensitivity, or calibration residual experiment was performed.',
                 'Only two installations, one reference trajectory, heuristic scene selection; no independent generalization claim.',
                 'The search over candidate proposals is excluded from selected-path costs; no online search latency claim.'])
    assert all(sha(Path(p))==h for p,h in before.items())
    report['inputs_unchanged']=True
    (out/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:report[k] for k in ('totals','random_tie_expected_optimal_states','random_tie_expected_total_calls','grid_first_tiebreak_total_calls','inputs_unchanged')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.source.resolve(),a.output.resolve())
