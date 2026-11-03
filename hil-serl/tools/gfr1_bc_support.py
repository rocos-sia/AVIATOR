"""Pure helpers for the frozen GFR imitation gate."""
from collections import Counter
import hashlib
import math
from pathlib import Path


def policy_state(state, mask_previous_action=False):
    """Mask only the previous executed r inputs, without mutating env/replay."""
    import numpy as np
    value = np.asarray(state)
    if value.shape[-1] != 40:
        raise ValueError('expected 40-D GFR observation')
    if not mask_previous_action:
        return value
    value = value.copy()
    value[..., 33:35] = 0
    return value


def checkpoint_hash(path):
    path = Path(path).resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    files = sorted(p for p in path.rglob('*') if p.is_file()) if path.is_dir() else [path]
    if not files:
        raise ValueError('empty checkpoint')
    h = hashlib.sha256()
    for file in files:
        h.update(str(file.relative_to(path) if path.is_dir() else file.name).encode())
        with file.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(block)
    return h.hexdigest()


def select_covered_records(records):
    """Five task IDs per model, sorted by path; never select by rollout result."""
    models = sorted({r['model'] for r in records})
    if len(models) != 2:
        raise ValueError('expected two teacher models')
    selected = []
    for model in models:
        group = sorted((r for r in records if r['model'] == model), key=lambda r: r['trajectory_path'])
        if len(group) != 50:
            raise ValueError('expected 50 complete teachers per model')
        selected.extend(group[i] for i in (0, 12, 24, 36, 49))
    return selected


def qualify_bc(reports):
    """Both frozen checkpoints must pass; errors never become zero completion."""
    if set(reports) != {10000, 20000}:
        return 'pending'
    unsafe = False
    for report in reports.values():
        if report.get('returncode') != 0 or report.get('instrumentation_unknown', 1) != 0:
            return 'instrumentation_error'
        rows = report.get('episodes', [])
        if len(rows) != 10 or report.get('agent_kind') != 'bc':
            return 'instrumentation_error'
        if sorted(Counter(r.get('model') for r in rows).values()) != [5, 5]:
            return 'instrumentation_error'
        for row in rows:
            if not all(math.isfinite(row.get(k, float('nan'))) for k in (
                'progress', 'minimum_executed_clearance_m', 'max_closure_residual_inf')):
                return 'instrumentation_error'
            if row.get('accepted_validation_violations') != 0:
                unsafe = True
    if unsafe:
        return 'stop_validation_violation'
    for report in reports.values():
        rows = report['episodes']
        success = Counter(r['model'] for r in rows if r.get('completed') and r['progress'] >= 1 - 1e-7)
        if sum(success.values()) < 9 or any(success[m] < 4 for m in {r['model'] for r in rows}):
            return 'stop_bc_not_stable'
    return 'pass'


def validate_bc_warmstart_report(qualification, initialization_only=False):
    """Allow an explicit initialization experiment without relabelling BC success."""
    decision = qualify_bc({int(k): v for k, v in qualification['evaluations'].items()})
    if decision != qualification['decision']:
        raise ValueError('BC report decision disagrees with evaluations')
    allowed = {'pass', 'stop_bc_not_stable'} if initialization_only else {'pass'}
    if decision not in allowed:
        raise ValueError(f'BC report cannot initialize SAC in this mode: {decision}')
    if qualification['training'].get('mask_previous_action', False):
        raise ValueError('masked BC needs a matching SAC observation pipeline')
    if not qualification['sources_unchanged']:
        raise ValueError('BC source audit failed')
    return decision
