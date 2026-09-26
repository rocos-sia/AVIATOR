"""Tests for the Q(x, phi) manifold lookup (Task 1.2, BLOCKER #3).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md
  - Task 0.2 (binary file formats + canonical grid indexing)
  - Task 1.2 (`ManifoldLookup.query`)

A *synthetic* manifold (3 x 3 x 5 grid) is written to a tmp dir. The grid values
are multilinear in (i_theta, i_s, i_phi, joint), so trilinear interpolation is
EXACT and can be checked analytically:

    qL[i_theta, i_s, i_phi, j] = B_L + C_TH*i_theta + C_S*i_s + C_P*i_phi + 0.01*j

=>  d qL/d theta = C_TH / dtheta,  d qL/d s = C_S / ds,  d qL/d phi = C_P / dphi
"""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup

# --- synthetic grid --------------------------------------------------------
N_THETA, N_S, N_PHI = 3, 3, 5
TH_RANGE = (-0.5, 0.5)
S_RANGE = (-0.2, 0.0)
PHI_RANGE = (-1.0, 1.0)

C_TH, C_S, C_P = 0.1, 0.2, 0.3
Q_BASE_L, Q_BASE_R = 1.0, -1.0
D_BASE_L, D_BASE_R = 0.010, 0.020
L_PHI_L, L_PHI_R = 1.9, 1.8

# Aviator v0.1 joint limits (rad), mirroring clearance_trajectory.cpp lower/upper.
JL = np.array([
    [-3.1067, np.deg2rad(84.5), -3.1067, -1.0472, -3.1067, -1.0472, -1.0472],
    [-3.1067, np.deg2rad(84.5), -3.1067, -1.0472, -3.1067, -1.0472, -1.0472],
])
JU = np.array([
    [3.1067, np.deg2rad(94.5), 3.1067, 2.5307, 3.1067, 1.0472, 1.0472],
    [3.1067, np.deg2rad(94.5), 3.1067, 2.5307, 3.1067, 1.0472, 1.0472],
])

THETA_AXIS = np.linspace(TH_RANGE[0], TH_RANGE[1], N_THETA)
S_AXIS = np.linspace(S_RANGE[0], S_RANGE[1], N_S)
PHI_AXIS = np.linspace(PHI_RANGE[0], PHI_RANGE[1], N_PHI)
DTH = (TH_RANGE[1] - TH_RANGE[0]) / (N_THETA - 1)
DS = (S_RANGE[1] - S_RANGE[0]) / (N_S - 1)
DPHI = (PHI_RANGE[1] - PHI_RANGE[0]) / (N_PHI - 1)

HIL_SERL_ROOT = Path(__file__).resolve().parents[3]
VALIDATE_TOOL = HIL_SERL_ROOT / "tools" / "validate_manifold.py"


# --------------------------------------------------------------------------
# synthetic manifold writer
# --------------------------------------------------------------------------
def _arrays(safe_pad_L=0.0, safe_pad_R=0.0, excluded=()):
    n_theta, n_s, n_phi = N_THETA, N_S, N_PHI
    i_s = np.arange(n_s)[:, None, None]
    i_th = np.arange(n_theta)[None, :, None]
    i_p = np.arange(n_phi)[None, None, :]
    j = np.arange(7)[None, None, None, :]

    def qbase(base):
        grid = C_TH * i_th + C_S * i_s + C_P * i_p  # (n_s, n_theta, n_phi)
        return base + grid[..., None] + 0.01 * j

    qL = qbase(Q_BASE_L)  # (n_s, n_theta, n_phi, 7)
    qR = qbase(Q_BASE_R)

    QxL = np.empty((n_s, n_theta, n_phi, 7, 2))
    QxL[..., 0] = C_TH / DTH
    QxL[..., 1] = C_S / DS
    QxR = QxL.copy()
    QphiL = np.empty((n_s, n_theta, n_phi, 7, 1))
    QphiL[..., 0] = C_P / DPHI
    QphiR = QphiL.copy()

    dL = D_BASE_L + 0.001 * (i_s + i_th + i_p)  # (n_s, n_theta, n_phi)
    dR = D_BASE_R + 0.0005 * (i_s + i_th + i_p)

    p0, p1 = PHI_RANGE
    lo_L = np.full((n_s, n_theta), p0 + safe_pad_L)
    hi_L = np.full((n_s, n_theta), p1 - safe_pad_L)
    lo_R = np.full((n_s, n_theta), p0 + safe_pad_R)
    hi_R = np.full((n_s, n_theta), p1 - safe_pad_R)
    safe = np.stack([lo_L, hi_L, lo_R, hi_R], axis=-1)  # (n_s, n_theta, 4)

    branch = np.zeros((n_s, n_theta), dtype=np.uint8)
    for a, b in excluded:
        branch[a, b] = 1

    return dict(qL=qL, qR=qR, dL=dL, dR=dR, QxL=QxL, QxR=QxR,
                QphiL=QphiL, QphiR=QphiR, safe=safe, branch=branch,
                phi=PHI_AXIS)


def make_manifold(tmp_path, *, safe_pad_L=0.0, safe_pad_R=0.0, excluded=(),
                  joint_limits=None):
    """Write a synthetic manifold dir. Returns (dir_path, arrays_dict)."""
    d = Path(tmp_path) / "manifold_phi"
    d.mkdir(parents=True, exist_ok=True)
    a = _arrays(safe_pad_L=safe_pad_L, safe_pad_R=safe_pad_R, excluded=excluded)

    (d / "phi.bin").write_bytes(a["phi"].astype(np.float32).tobytes())
    for name in ("qL", "qR", "QxL", "QxR", "QphiL", "QphiR", "dL", "dR"):
        (d / f"{name}.bin").write_bytes(np.ascontiguousarray(a[name], dtype=np.float32).tobytes())
    (d / "safe.bin").write_bytes(np.ascontiguousarray(a["safe"], dtype=np.float32).tobytes())
    (d / "branch.bin").write_bytes(np.ascontiguousarray(a["branch"], dtype=np.uint8).tobytes())

    manifest = {
        "n_theta": N_THETA, "n_s": N_S, "n_phi": N_PHI,
        "L_phi_L": L_PHI_L, "L_phi_R": L_PHI_R,
        "anchor_theta": 0.0, "anchor_s": 0.0,
        "th_min": TH_RANGE[0], "th_max": TH_RANGE[1],
        "s_min": S_RANGE[0], "s_max": S_RANGE[1],
    }
    if joint_limits is not None:
        manifest["joint_lower"] = np.asarray(joint_limits[0]).tolist()
        manifest["joint_upper"] = np.asarray(joint_limits[1]).tolist()
    (d / "manifest.json").write_text(json.dumps(manifest))
    return str(d), a


def node_q(base, it, is_, ip):
    return base + C_TH * it + C_S * is_ + C_P * ip + 0.01 * np.arange(7)


# --------------------------------------------------------------------------
# exact interpolation at nodes
# --------------------------------------------------------------------------
def test_exact_at_nodes_batch(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)

    xs, ps, expL, expR, expdL, expdR = [], [], [], [], [], []
    for it in range(N_THETA):
        for is_ in range(N_S):
            for ip in range(N_PHI):
                xs.append([THETA_AXIS[it], S_AXIS[is_]])
                ps.append([PHI_AXIS[ip], PHI_AXIS[ip]])
                expL.append(node_q(Q_BASE_L, it, is_, ip))
                expR.append(node_q(Q_BASE_R, it, is_, ip))
                expdL.append(D_BASE_L + 0.001 * (is_ + it + ip))
                expdR.append(D_BASE_R + 0.0005 * (is_ + it + ip))

    r = lk.query(np.array(xs), np.array(ps))
    B = len(xs)
    np.testing.assert_allclose(r["qL"], np.array(expL), atol=1e-5)
    np.testing.assert_allclose(r["qR"], np.array(expR), atol=1e-5)
    np.testing.assert_allclose(r["dL"], np.array(expdL), atol=1e-7)
    np.testing.assert_allclose(r["dR"], np.array(expdR), atol=1e-7)
    np.testing.assert_allclose(r["d_min"], np.minimum(expdL, expdR), atol=1e-7)
    assert r["qL"].shape == (B, 7)


def test_exact_at_single_node(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[THETA_AXIS[1], S_AXIS[1]]]),
                 np.array([[PHI_AXIS[2], PHI_AXIS[2]]]))
    np.testing.assert_allclose(r["qL"][0], node_q(Q_BASE_L, 1, 1, 2), atol=1e-6)
    np.testing.assert_allclose(r["qR"][0], node_q(Q_BASE_R, 1, 1, 2), atol=1e-6)


# --------------------------------------------------------------------------
# bilinear in (theta, s), linear in phi
# --------------------------------------------------------------------------
def test_bilinear_theta_s(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    it, is_, ip = 0, 1, 2
    th = 0.5 * (THETA_AXIS[it] + THETA_AXIS[it + 1])
    s = 0.5 * (S_AXIS[is_] + S_AXIS[is_ + 1])
    r = lk.query(np.array([[th, s]]), np.array([[PHI_AXIS[ip], PHI_AXIS[ip]]]))
    exp = 0.25 * (node_q(Q_BASE_L, it, is_, ip) + node_q(Q_BASE_L, it + 1, is_, ip)
                  + node_q(Q_BASE_L, it, is_ + 1, ip) + node_q(Q_BASE_L, it + 1, is_ + 1, ip))
    np.testing.assert_allclose(r["qL"][0], exp, atol=1e-5)


def test_linear_in_phi(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    it, is_, ip = 1, 1, 1
    ph = 0.5 * (PHI_AXIS[ip] + PHI_AXIS[ip + 1])
    r = lk.query(np.array([[THETA_AXIS[it], S_AXIS[is_]]]), np.array([[ph, ph]]))
    exp = 0.5 * (node_q(Q_BASE_L, it, is_, ip) + node_q(Q_BASE_L, it, is_, ip + 1))
    np.testing.assert_allclose(r["qL"][0], exp, atol=1e-5)
    expR = 0.5 * (node_q(Q_BASE_R, it, is_, ip) + node_q(Q_BASE_R, it, is_, ip + 1))
    np.testing.assert_allclose(r["qR"][0], expR, atol=1e-5)


def test_trilinear_midpoint(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    it, is_, ip = 0, 0, 1
    th = 0.5 * (THETA_AXIS[it] + THETA_AXIS[it + 1])
    s = 0.5 * (S_AXIS[is_] + S_AXIS[is_ + 1])
    ph = 0.5 * (PHI_AXIS[ip] + PHI_AXIS[ip + 1])
    r = lk.query(np.array([[th, s]]), np.array([[ph, ph]]))
    acc = np.zeros(7)
    for di in (0, 1):
        for dj in (0, 1):
            for dk in (0, 1):
                acc += node_q(Q_BASE_L, it + di, is_ + dj, ip + dk)
    np.testing.assert_allclose(r["qL"][0], acc / 8.0, atol=1e-5)


def test_theta_linear_alone(tmp_path):
    """Exactly on a theta node boundary between rows must reproduce the node."""
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    th = 0.5 * (THETA_AXIS[0] + THETA_AXIS[1])
    r = lk.query(np.array([[th, S_AXIS[2]]]), np.array([[PHI_AXIS[3], PHI_AXIS[3]]]))
    exp = 0.5 * (node_q(Q_BASE_L, 0, 2, 3) + node_q(Q_BASE_L, 1, 2, 3))
    np.testing.assert_allclose(r["qL"][0], exp, atol=1e-5)


# --------------------------------------------------------------------------
# derivatives + assembled Q_x / Q_phi
# --------------------------------------------------------------------------
def test_assembly_shapes_and_block_structure(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    B = 7
    xs = np.column_stack([
        np.linspace(TH_RANGE[0], TH_RANGE[1], B),
        np.linspace(S_RANGE[0], S_RANGE[1], B),
    ])
    ps = np.column_stack([
        np.linspace(PHI_RANGE[0], PHI_RANGE[1], B),
        np.linspace(PHI_RANGE[0], PHI_RANGE[1], B),
    ])
    r = lk.query(xs, ps)
    assert r["Q_x"].shape == (B, 14, 2)
    assert r["Q_phi"].shape == (B, 14, 2)

    # vertical stack for Q_x (theta, s shared)
    np.testing.assert_allclose(r["Q_x"][:, 0:7, 0], C_TH / DTH, atol=1e-5)
    np.testing.assert_allclose(r["Q_x"][:, 0:7, 1], C_S / DS, atol=1e-5)
    np.testing.assert_allclose(r["Q_x"][:, 7:14, 0], C_TH / DTH, atol=1e-5)
    np.testing.assert_allclose(r["Q_x"][:, 7:14, 1], C_S / DS, atol=1e-5)

    # block-diagonal for Q_phi
    np.testing.assert_allclose(r["Q_phi"][:, 0:7, 0], C_P / DPHI, atol=1e-5)
    np.testing.assert_allclose(r["Q_phi"][:, 7:14, 1], C_P / DPHI, atol=1e-5)
    np.testing.assert_array_equal(r["Q_phi"][:, 0:7, 1], np.zeros((B, 7)))
    np.testing.assert_array_equal(r["Q_phi"][:, 7:14, 0], np.zeros((B, 7)))


def test_Qx_equals_z_stored_per_arm_derivatives(tmp_path):
    """Q_x rows must come from the per-arm 7x2 bins (not a 14x2 bin)."""
    d, a = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    assert lk.QxL.shape[-2:] == (7, 2)
    assert lk.QxR.shape[-2:] == (7, 2)
    assert lk.QphiL.shape[-2:] == (7, 1)
    assert lk.QphiR.shape[-2:] == (7, 1)
    # stored derivatives interpolate exactly (constant in the synthetic grid)
    r = lk.query(np.array([[0.1, -0.05]]), np.array([[0.2, -0.2]]))
    np.testing.assert_allclose(r["Q_x"][0, 0:7, :], [[C_TH / DTH, C_S / DS]] * 7, atol=1e-5)
    np.testing.assert_allclose(r["Q_phi"][0, 0:7, 0], [C_P / DPHI] * 7, atol=1e-5)


# --------------------------------------------------------------------------
# margins, safe interval, d_min, branch
# --------------------------------------------------------------------------
def test_safe_interval_columns_and_margins(tmp_path):
    d, _ = make_manifold(tmp_path, safe_pad_L=0.3, safe_pad_R=0.1)
    lk = ManifoldLookup(d)
    ph = 0.0
    r = lk.query(np.array([[0.0, -0.1]]), np.array([[ph, ph]]))
    lo, hi = PHI_RANGE
    np.testing.assert_allclose(r["phi_safe_lo"][0], [lo + 0.3, lo + 0.1], atol=1e-6)
    np.testing.assert_allclose(r["phi_safe_hi"][0], [hi - 0.3, hi - 0.1], atol=1e-6)
    np.testing.assert_allclose(r["m_phi_minus"][0], [ph - (lo + 0.3), ph - (lo + 0.1)], atol=1e-6)
    np.testing.assert_allclose(r["m_phi_plus"][0], [(hi - 0.3) - ph, (hi - 0.1) - ph], atol=1e-6)


def test_margins_are_one_at_center_zero_at_boundary(tmp_path):
    """R_i = 2*min(m_phi^-, m_phi^+) / (hi - lo) -> 1.0 at centre, 0.0 at the edge."""
    d, _ = make_manifold(tmp_path, safe_pad_L=0.0, safe_pad_R=0.0)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[0.0, -0.1], [0.0, -0.1]]),
                 np.array([[0.0, 0.0], [-1.0, -1.0]]))
    width = 2.0
    R0 = 2.0 * np.minimum(r["m_phi_minus"][0], r["m_phi_plus"][0]) / width
    np.testing.assert_allclose(R0, [1.0, 1.0], atol=1e-6)
    R1 = 2.0 * np.minimum(r["m_phi_minus"][1], r["m_phi_plus"][1]) / width
    np.testing.assert_allclose(R1, [0.0, 0.0], atol=1e-6)


def test_d_min_is_min_of_arms(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[0.0, -0.1]]), np.array([[0.0, 0.0]]))
    np.testing.assert_allclose(r["d_min"][0], min(r["dL"][0], r["dR"][0]))
    assert r["dL"][0] < r["dR"][0]  # D_BASE_L < D_BASE_R by construction


def test_m_q_default_joint_limits(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[0.3, -0.07]]), np.array([[0.15, -0.15]]))
    mL = np.minimum.reduce([r["qL"][0] - JL[0], JU[0] - r["qL"][0]])
    mR = np.minimum.reduce([r["qR"][0] - JL[1], JU[1] - r["qR"][0]])
    np.testing.assert_allclose(r["m_q"][0], min(mL.min(), mR.min()), atol=1e-6)


def test_m_q_manifest_joint_limits_override(tmp_path):
    jl = np.array([[-5.0] * 7, [-5.0] * 7])
    ju = np.array([[5.0] * 7, [5.0] * 7])
    d, _ = make_manifold(tmp_path, joint_limits=(jl, ju))
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[0.0, -0.1]]), np.array([[0.0, 0.0]]))
    exp = np.minimum.reduce([r["qL"][0] + 5.0, 5.0 - r["qL"][0],
                             r["qR"][0] + 5.0, 5.0 - r["qR"][0]]).min()
    np.testing.assert_allclose(r["m_q"][0], exp, atol=1e-6)


def test_branch_is_zero_when_all_consistent(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[0.0, -0.1]]), np.array([[0.0, 0.0]]))
    assert r["branch"].shape == (1,)
    assert int(r["branch"][0]) == 0


def test_branch_flags_excluded_neighbour(tmp_path):
    """A query whose stencil touches an excluded grid point is flagged (conservative)."""
    d, _ = make_manifold(tmp_path, excluded=[(2, 1)])
    lk = ManifoldLookup(d)
    # exactly at the excluded node
    r = lk.query(np.array([[THETA_AXIS[1], S_AXIS[2]]]), np.array([[0.0, 0.0]]))
    assert int(r["branch"][0]) != 0
    # a stencil that touches it (mid-point between (1,2) and (1,1))
    r2 = lk.query(np.array([[0.5 * (THETA_AXIS[0] + THETA_AXIS[1]), S_AXIS[2]]]),
                  np.array([[0.0, 0.0]]))
    assert int(r2["branch"][0]) != 0
    # far away -> clean
    r3 = lk.query(np.array([[THETA_AXIS[2], S_AXIS[0]]]), np.array([[0.0, 0.0]]))
    assert int(r3["branch"][0]) == 0


# --------------------------------------------------------------------------
# batch shapes / metadata
# --------------------------------------------------------------------------
def test_batch_shapes(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    B = 16
    rng = np.random.default_rng(0)
    xs = np.column_stack([
        rng.uniform(*TH_RANGE, B), rng.uniform(*S_RANGE, B)])
    ps = np.column_stack([
        rng.uniform(*PHI_RANGE, B), rng.uniform(*PHI_RANGE, B)])
    r = lk.query(xs, ps)
    assert set(r) == {"qL", "qR", "dL", "dR", "d_min", "m_phi_minus", "m_phi_plus",
                      "m_q", "phi_safe_lo", "phi_safe_hi", "Q_x", "Q_phi", "branch"}
    assert r["qL"].shape == (B, 7)
    assert r["qR"].shape == (B, 7)
    for k in ("dL", "dR", "d_min", "m_q", "branch"):
        assert r[k].shape == (B,)
    for k in ("m_phi_minus", "m_phi_plus", "phi_safe_lo", "phi_safe_hi"):
        assert r[k].shape == (B, 2)
    assert r["Q_x"].shape == (B, 14, 2)
    assert r["Q_phi"].shape == (B, 14, 2)


def test_manifest_roundtrip(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    assert lk.n_theta == N_THETA and lk.n_s == N_S and lk.n_phi == N_PHI
    np.testing.assert_allclose(lk.L_phi, [L_PHI_L, L_PHI_R])
    np.testing.assert_allclose(lk.phi_axis, PHI_AXIS, atol=1e-6)


def test_single_query_matches_batch_row(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    xs = np.array([[0.2, -0.03], [-0.1, -0.15]])
    ps = np.array([[0.1, -0.4], [0.6, 0.2]])
    rb = lk.query(xs, ps)
    for i in range(2):
        r1 = lk.query(xs[i:i + 1], ps[i:i + 1])
        np.testing.assert_allclose(r1["qL"][0], rb["qL"][i], atol=1e-9)
        np.testing.assert_allclose(r1["Q_x"][0], rb["Q_x"][i], atol=1e-9)


# --------------------------------------------------------------------------
# errors
# --------------------------------------------------------------------------
def test_out_of_grid_theta_raises(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    with pytest.raises(ValueError):
        lk.query(np.array([[TH_RANGE[1] + 0.01, -0.1]]), np.array([[0.0, 0.0]]))


def test_out_of_grid_s_raises(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    with pytest.raises(ValueError):
        lk.query(np.array([[0.0, S_RANGE[0] - 0.01]]), np.array([[0.0, 0.0]]))


def test_out_of_grid_phi_raises(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    with pytest.raises(ValueError):
        lk.query(np.array([[0.0, -0.1]]), np.array([[0.0, PHI_RANGE[1] + 0.5]]))


def test_out_of_safe_interval_raises(tmp_path):
    d, _ = make_manifold(tmp_path, safe_pad_L=0.3, safe_pad_R=0.1)
    lk = ManifoldLookup(d)
    # left arm below its safe low (right arm would be fine) -> raises
    with pytest.raises(ValueError):
        lk.query(np.array([[0.0, -0.1]]), np.array([[PHI_RANGE[0], 0.0]]))


def test_grid_boundary_is_inclusive(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    r = lk.query(np.array([[TH_RANGE[1], S_RANGE[1]], [TH_RANGE[0], S_RANGE[0]]]),
                 np.array([[PHI_RANGE[0], PHI_RANGE[1]], [PHI_RANGE[1], PHI_RANGE[0]]]))
    assert np.all(np.isfinite(r["qL"]))


def test_bad_shapes_raise(tmp_path):
    d, _ = make_manifold(tmp_path)
    lk = ManifoldLookup(d)
    with pytest.raises(ValueError):
        lk.query(np.array([0.0, -0.1]), np.array([[0.0, 0.0]]))
    with pytest.raises(ValueError):
        lk.query(np.array([[0.0, -0.1]]), np.array([0.0, 0.0]))
    with pytest.raises(ValueError):
        lk.query(np.array([[0.0, -0.1]]), np.array([[0.0, 0.0], [0.0, 0.0]]))


# --------------------------------------------------------------------------
# validate_manifold tool
# --------------------------------------------------------------------------
def _load_validate_module():
    spec = importlib.util.spec_from_file_location("aviator_validate_manifold", VALIDATE_TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_validate_reports_on_clean_manifold(tmp_path):
    vm = _load_validate_module()
    d, _ = make_manifold(tmp_path)
    # adjacent grid points are phase-transported: the natural step is max(C_TH, C_S)
    rep = vm.validate(d, eps=0.25)
    assert rep["n_excluded_points"] == 0
    assert rep["cycle_consistency_max_violation"] == pytest.approx(max(C_TH, C_S), abs=1e-5)
    assert rep["passed"] is True
    assert "qL.bin" in rep["file_sizes"]
    assert rep["file_sizes"]["qL.bin"] == N_THETA * N_S * N_PHI * 7 * 4
    assert rep["file_sizes"]["QxL.bin"] == N_THETA * N_S * N_PHI * 7 * 2 * 4
    assert rep["file_sizes"]["safe.bin"] == N_THETA * N_S * 2 * 2 * 4
    assert rep["file_sizes"]["branch.bin"] == N_THETA * N_S * 1


def test_validate_counts_excluded_and_flags_discontinuity(tmp_path):
    vm = _load_validate_module()
    d, _ = make_manifold(tmp_path, excluded=[(0, 0)])
    rep = vm.validate(d, eps=1e-3)
    assert rep["n_excluded_points"] == 1


def test_validate_detects_jump(tmp_path):
    """Inject a discontinuous row; the validator must report a large violation."""
    vm = _load_validate_module()
    d, a = make_manifold(tmp_path)
    qL = a["qL"].copy()
    qL[2] += 5.0  # blow up the s = +1 row
    (Path(d) / "qL.bin").write_bytes(np.ascontiguousarray(qL, dtype=np.float32).tobytes())
    rep = vm.validate(d, eps=1e-3)
    assert rep["cycle_consistency_max_violation"] > 1.0
    assert rep["passed"] is False
