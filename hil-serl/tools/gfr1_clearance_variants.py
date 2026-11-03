"""Opt-in candidate generation ablations; frozen final checkers are unchanged.

V0 matches the historical projector. M constrains every mesh-cloud/wall pair.
F backtracks toward a convex-combination anchor inside the current polygon.
Overrides are scoped and serial; never use them across concurrent threads.
"""
from contextlib import contextmanager
import numpy as np
import mujoco
from tools import validate_branch_connector as base

VARIANTS = {'original': (False, False), 'multi': (True, False),
            'affine': (False, True), 'combined': (True, True)}


def pair_distances(checker, x, q):
    """Same point clouds / box metric, separated by mesh-cloud and wall."""
    data = checker.data
    data.qpos[checker.joint_adr] = q
    data.qpos[checker.wheel_adr] = x
    mujoco.mj_forward(checker.model, data)
    values = []
    for _, body, points in checker.clouds:
        xyz = points @ data.xmat[body].reshape(3, 3).T + data.xpos[body]
        for (center, half, rotation), sign in zip(checker.walls, checker.wall_sign):
            local = (xyz-center) @ rotation
            excess = np.abs(local)-half
            outside = np.linalg.norm(np.maximum(excess, 0), axis=1)
            distance = np.where(outside > 0, outside, sign*local[:, 2]-half[2])
            values.append(float(np.min(distance)))
    return np.asarray(values)


def distance_planes(checker, x, q, h, n, dt, d_safe, multi):
    if not multi:
        d0, _ = checker.check(x, q)
        gradient = base.clearance_gradient(checker, x, q)[None, :]
        distances = np.array([d0])
    else:
        distances = pair_distances(checker, x, q)
        gradient = np.zeros((len(distances), len(q)))
        for j in range(len(q)):
            qp, qm = q.copy(), q.copy()
            qp[j] += 1e-5
            qm[j] -= 1e-5
            gradient[:, j] = (pair_distances(checker, x, qp)
                              - pair_distances(checker, x, qm))/(2e-5)
    return [(-dt*(g @ n), float(d-d_safe+dt*(g @ h)))
            for d, g in zip(distances, gradient)]


def inward_step(r, polygon):
    """Convex combination remains in a convex polygon, including degenerate ones."""
    anchor = np.mean(np.asarray(polygon), axis=0)
    return anchor + 0.5*(np.asarray(r)-anchor)


def make_projector(variant, observer=None):
    multi, affine = VARIANTS[variant]

    def project(checker, x, x_next, q, h, n, r_nom, lower, upper, dt,
                qdot_max=1.5, d_safe=0.005, r_max=3.0):
        planes = []
        for j in range(14):
            nj = n[j]
            base.add_plane(planes, nj, qdot_max-h[j])
            base.add_plane(planes, -nj, qdot_max+h[j])
            base.add_plane(planes, dt*nj, upper[j]-q[j]-dt*h[j])
            base.add_plane(planes, -dt*nj, q[j]-lower[j]+dt*h[j])
        distances = distance_planes(checker, x, q, h, n, dt, d_safe, multi)
        planes.extend(distances)
        poly = [np.array([-r_max, -r_max]), np.array([r_max, -r_max]),
                np.array([r_max, r_max]), np.array([-r_max, r_max])]
        diag = dict(variant=variant, distance_constraints=len(distances), attempts=[],
                    linear_empty=False, affine_violation=False)
        for aa, bb in planes:
            poly = base._clip_polygon(poly, aa, bb)
            if not poly:
                diag['linear_empty'] = True
                if observer is not None: observer(diag)
                return np.zeros(2), dict(feasible=False, intervened=True, backtracked=0,
                    r=[0., 0.], endpoint_check_evaluated=False, swept_check_evaluated=False,
                    projection_diagnostic=diag)
        r = base._project_onto_polygon(r_nom, poly)
        intervened = bool(np.linalg.norm(r-r_nom) > 1e-10)
        # Preserve the original polygon's numerical tolerance. Convex
        # interpolation may not exceed the maximum violation of its endpoints.
        anchor = np.mean(np.asarray(poly), axis=0)
        convex_bound = max(0., max(float(aa @ v-bb)
                                  for aa, bb in planes for v in (r, anchor)))
        diag['initial_convex_violation_bound'] = convex_bound
        for attempt in range(6):
            plane_violation = max(float(aa @ r-bb) for aa, bb in planes)
            if affine and plane_violation > convex_bound + 1e-12:
                raise AssertionError(f'affine backtracking left polygon: {plane_violation}')
            diag['affine_violation'] |= plane_violation > 1e-9
            qn = q+dt*(h+n @ r)
            dn, contact = checker.check(x_next, qn)
            ds, swept_contact = base.swept_clearance(checker, x, q, x_next, qn)
            qdot = (qn-q)/dt
            valid = bool(np.max(np.abs(qdot)) <= qdot_max+2e-5
                and np.all(qn >= lower-1e-9) and np.all(qn <= upper+1e-9)
                and dn >= d_safe-1e-9 and ds >= d_safe-1e-9
                and not contact and not swept_contact)
            diag['attempts'].append(dict(attempt=attempt, plane_violation=plane_violation,
                endpoint_m=float(dn), swept_m=float(ds), accepted=valid,
                endpoint_valid=bool(dn >= d_safe-1e-9 and not contact),
                swept_valid=bool(ds >= d_safe-1e-9 and not swept_contact)))
            if valid:
                if observer is not None: observer(diag)
                return r, dict(feasible=True, intervened=intervened, backtracked=attempt,
                    d_next=float(dn), swept_min=float(ds), r=r.tolist(),
                    projection_diagnostic=diag)
            r = inward_step(r, poly) if affine else r*0.5
            intervened = True
        if observer is not None: observer(diag)
        return np.zeros(2), dict(feasible=False, intervened=True, backtracked=6,
            swept_min=None, r=[0., 0.], endpoint_check_evaluated=True,
            swept_check_evaluated=True, endpoint_validation=diag['attempts'][-1]['endpoint_valid'],
            swept_validation=diag['attempts'][-1]['swept_valid'], projection_diagnostic=diag)
    return project


@contextmanager
def use_projector(variant, observer=None):
    from experiments.aviator_manifold import gfr_env
    original = gfr_env.project_r
    gfr_env.project_r = make_projector(variant, observer)
    try:
        yield
    finally:
        gfr_env.project_r = original
