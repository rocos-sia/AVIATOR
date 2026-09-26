"""Tests for the 2-D phase-velocity safety filter (Task 1.3, BLOCKER #4/#5).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

Cases (per the plan Step 1):
  (a) feasible      -> unchanged
  (b) joint-vel violation -> projected
  (c) safe-arc violation (x_next) -> clipped
  (d) empty intersection -> feasible=False
"""

import json
from pathlib import Path

import numpy as np
import pytest

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from examples.experiments.aviator_manifold.safety_filter import project_phi_dot

from examples.experiments.aviator_manifold.test_manifold_lookup import (
    make_manifold,
    N_THETA,
    N_S,
    N_PHI,
    TH_RANGE,
    S_RANGE,
    PHI_RANGE,
)

# The synthetic manifold has CONSTANT derivatives:
#   Q_x  = (C_TH/DTH, C_S/DS) = (0.2, 2.0) per joint (both arms)
#   Q_phi = C_P/DPHI = 0.6 per joint (block-diagonal)
# so the linearized joint velocity is, per left joint,
#   qdot_L = 0.2*theta_dot + 2.0*s_dot + 0.6*phi_dot_L
# and per right joint,
#   qdot_R = 0.2*theta_dot + 2.0*s_dot + 0.6*phi_dot_R
# For a task velocity ẋ = (theta_dot, s_dot), the base is base = 0.2*theta_dot + 2.0*s_dot.


# --------------------------------------------------------------------------
# (a) feasible -> unchanged
# --------------------------------------------------------------------------
def test_feasible_unchanged(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    x = np.array([0.0, -0.1])
    x_next = np.array([0.0, -0.1])
    phi = np.array([0.0, 0.0])
    nom = np.array([0.5, -0.3])
    r, info = project_phi_dot(nom, x, x_next, phi, lk)
    np.testing.assert_allclose(r, nom, atol=1e-9)
    assert info["feasible"]
    assert not info["intervened"]


# --------------------------------------------------------------------------
# (b) joint-vel violation -> projected
# --------------------------------------------------------------------------
def test_joint_vel_projected(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    # theta_dot = 1.0 -> base = 0.2, so |0.2 + 0.6*phi_dot_L| <= 1.5
    # -> phi_dot_L <= (1.5 - 0.2)/0.6 = 2.1667
    x = np.array([0.0, 0.0])
    x_next = np.array([0.01, 0.0])
    phi = np.array([0.0, 0.0])
    nom = np.array([3.0, 0.0])
    r, info = project_phi_dot(nom, x, x_next, phi, lk)
    assert info["feasible"]
    assert info["intervened"]
    np.testing.assert_allclose(r[0], 2.1666667, atol=1e-4)
    np.testing.assert_allclose(r[1], 0.0, atol=1e-9)


# --------------------------------------------------------------------------
# (c) safe-arc (box) violation at x_next -> clipped
# --------------------------------------------------------------------------
def test_safe_arc_clipped_x_next(tmp_path):
    # shrink the safe interval to [-0.05, 0.05]; with phi = 0.03 the box forces
    # phi_dot_L <= (0.05 - 0.03)/0.01 = 2.0, tighter than the joint-vel cap 2.5
    d, _ = make_manifold(tmp_path, safe_pad_L=0.95, safe_pad_R=0.95)
    lk = ManifoldLookup(d)
    x = np.array([0.0, 0.0])
    x_next = np.array([0.0, 0.0])   # no task motion -> base = 0
    phi = np.array([0.03, 0.0])
    nom = np.array([10.0, 0.0])
    r, info = project_phi_dot(nom, x, x_next, phi, lk)
    assert info["feasible"]
    assert info["intervened"]
    np.testing.assert_allclose(r[0], 2.0, atol=1e-6)


# --------------------------------------------------------------------------
# (d) empty intersection -> feasible=False
# --------------------------------------------------------------------------
def _make_shifted_safe_manifold(tmp_path):
    """A synthetic manifold whose safe box at the top-right (theta, s) cell is
    shifted high to [0.06, 0.08] (left arm).  Moving from x=(0, -0.2) to
    x_next=(0.5, 0) drives the task velocity xdot=(50, 20) at dt=0.01, so the
    joint-velocity half-planes (base = 0.2*50 + 2.0*20 = 50) demand r_L <= -80,
    disjoint from the safe box r_L in [6, 8] -> empty feasible polygon."""
    d = Path(tmp_path) / "manifold_shift"
    d.mkdir(parents=True, exist_ok=True)

    from examples.experiments.aviator_manifold.test_manifold_lookup import _arrays
    a = _arrays()

    # default safe is the full phi range [-1, 1]; shift the top-right cell only
    n_s, n_theta = N_S, N_THETA
    lo_L = np.full((n_s, n_theta), PHI_RANGE[0])
    hi_L = np.full((n_s, n_theta), PHI_RANGE[1])
    lo_R = np.full((n_s, n_theta), PHI_RANGE[0])
    hi_R = np.full((n_s, n_theta), PHI_RANGE[1])
    lo_L[-1, -1] = 0.06
    hi_L[-1, -1] = 0.08
    safe = np.stack([lo_L, hi_L, lo_R, hi_R], axis=-1)

    for name in ("qL", "qR", "QxL", "QxR", "QphiL", "QphiR", "dL", "dR"):
        (d / f"{name}.bin").write_bytes(
            np.ascontiguousarray(a[name], dtype=np.float32).tobytes())
    (d / "safe.bin").write_bytes(np.ascontiguousarray(safe, dtype=np.float32).tobytes())
    (d / "phi.bin").write_bytes(a["phi"].astype(np.float32).tobytes())
    (d / "branch.bin").write_bytes(a["branch"].astype(np.uint8).tobytes())
    manifest = {
        "n_theta": N_THETA, "n_s": N_S, "n_phi": N_PHI,
        "L_phi_L": 1.9, "L_phi_R": 1.8,
        "anchor_theta": 0.0, "anchor_s": 0.0,
        "th_min": TH_RANGE[0], "th_max": TH_RANGE[1],
        "s_min": S_RANGE[0], "s_max": S_RANGE[1],
    }
    (d / "manifest.json").write_text(json.dumps(manifest))
    return str(d)


def test_empty_intersection(tmp_path):
    d = _make_shifted_safe_manifold(tmp_path)
    lk = ManifoldLookup(d)
    # x_next = top-right cell (theta=0.5, s=0) -> safe box [0.06, 0.08]
    x = np.array([0.0, -0.2])
    x_next = np.array([TH_RANGE[1], S_RANGE[1]])
    phi = np.array([0.0, 0.0])
    nom = np.array([0.0, 0.0])
    r, info = project_phi_dot(nom, x, x_next, phi, lk)
    assert not info["feasible"]
    assert info["intervened"]
    np.testing.assert_allclose(r, np.zeros(2))
