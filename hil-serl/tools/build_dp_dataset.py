"""DP CSV -> HIL-SERL demo pkl with calibrated phi_dot_scale (Task 1.4, BLOCKER #11).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.4)

Reads the full-horizon DP teacher's CSV dumps
(``data/aviator/dp_demo/traj_<i>.csv``), computes the per-arm action scale
``phi_dot_scale`` (95% quantile of ``|phi_dot|``, floored at 1.5 rad/s) across
*all* trajectories, then rewrites each trajectory as a HIL-SERL demo ``.pkl``
(a flat list of transition dicts) with the actions normalized to ``[-1, 1]``
and the 40-D observation assembled to match ``AviatorManifoldEnv`` (incl. the
ChunkingWrapper ``obs_horizon=1`` leading dimension).

Transition dict keys (match ``ReplayBuffer`` / ``record_demos.py``)::

    observations       {"state": (1, 40) float32}
    next_observations  {"state": (1, 40) float32}
    actions            (2,) float32   -- normalized a = phi_dot / phi_dot_scale
    rewards            scalar float32
    masks              scalar float32 -- 1.0 - dones
    dones              bool           -- True only on the terminal transition

The CSV stores ``(sin phi, cos phi)`` for the observation and raw ``phi_dot``
(rad/s) for the action label; the unwrapped ``phi`` never reaches the obs.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pickle

import numpy as np

from examples.experiments.aviator_manifold.reward import reward_fn
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup

__all__ = [
    "COLUMNS",
    "read_dp_csv",
    "compute_phi_dot_scale",
    "build_obs40",
    "build_transitions",
]

# frozen v0.1 constants
_DT = 0.01
_LAGS = (2, 5, 10, 20)
_OBS_DIM = 40

# CSV column names (order defined by record_dp, Task 0.3).
COLUMNS = [
    "t", "theta", "s", "theta_dot", "s_dot",
    "sin_phi_L", "cos_phi_L", "sin_phi_R", "cos_phi_R",
    "phi_dot_L", "phi_dot_R",
    "qL1", "qL2", "qL3", "qL4", "qL5", "qL6", "qL7",
    "qR1", "qR2", "qR3", "qR4", "qR5", "qR6", "qR7",
    "qdotL1", "qdotL2", "qdotL3", "qdotL4", "qdotL5", "qdotL6", "qdotL7",
    "qdotR1", "qdotR2", "qdotR3", "qdotR4", "qdotR5", "qdotR6", "qdotR7",
    "dL", "dR", "d_min",
    "m_phi_minus_L", "m_phi_minus_R", "m_phi_plus_L", "m_phi_plus_R",
    "m_q",
]


def read_dp_csv(path: str) -> dict:
    """Parse a record_dp CSV into a dict of float64 arrays (each length N).

    The first line is a ``# L_phi_L=..,L_phi_R=..`` metadata comment and the
    second is a header row; both are skipped.
    """
    with open(path) as f:
        lines = f.readlines()

    lphi = {"L_phi_L": None, "L_phi_R": None}
    for tok in lines[0].lstrip("# ").split(","):
        if "=" in tok:
            k, v = tok.split("=")
            if k.strip() in lphi:
                lphi[k.strip()] = float(v)

    data = np.array(
        [line.split(",") for line in lines[2:] if line.strip()],
        dtype=np.float64,
    )
    assert data.shape[1] == len(COLUMNS), (
        f"{path}: expected {len(COLUMNS)} columns, got {data.shape[1]}"
    )
    out = {name: data[:, i] for i, name in enumerate(COLUMNS)}
    out["L_phi_L"] = lphi["L_phi_L"]
    out["L_phi_R"] = lphi["L_phi_R"]
    return out


def compute_phi_dot_scale(csv_paths, quantile: float = 0.95, floor: float = 1.5,
                          ceil: float = 2.0) -> dict:
    """Compute the calibrated action scale across all trajectories.

    Uses the ``quantile`` of ``|phi_dot|`` (over both arms) instead of the max:
    the max is driven by a single 9.5 rad/s glitch frame and previously produced
    ``phi_dot_scale = 9.5 * 1.1 = 10.45``, which let a full-range actor output
    (|a|=1 -> 10.45 rad/s) blow far past the ~2.97 rad/s joint-speed ceiling in
    one step.  The quantile (0.5 for the hold-dominated demos) is then floored
    at ``floor`` so the actor's full range still maps to a physically meaningful
    phase velocity (1.5 rad/s), and ceiled at ``ceil``.

    Returns ``{"phi_dot_scale": float, "p95_abs_phi_dot": float,
    "max_abs_phi_dot": [maxL, maxR], "n_trajs": int,
    "flag_exceeds_nominal": bool}``.
    """
    maxL = 0.0
    maxR = 0.0
    all_abs = []
    n = 0
    for p in csv_paths:
        d = read_dp_csv(p)
        maxL = max(maxL, float(np.max(np.abs(d["phi_dot_L"]))))
        maxR = max(maxR, float(np.max(np.abs(d["phi_dot_R"]))))
        all_abs.append(np.abs(d["phi_dot_L"]))
        all_abs.append(np.abs(d["phi_dot_R"]))
        n += 1
    p95 = float(np.percentile(np.concatenate(all_abs), 100 * quantile))
    scale = float(np.clip(p95, floor, ceil))
    return {
        "phi_dot_scale": scale,
        "p95_abs_phi_dot": p95,
        "max_abs_phi_dot": [float(maxL), float(maxR)],
        "n_trajs": n,
        "flag_exceeds_nominal": bool(max(maxL, maxR) > ceil),
    }


def _lag_clamped(x: np.ndarray, t: int, k: int) -> np.ndarray:
    """x[max(0, t-k)] -- history lag k clamped to the t=0 value."""
    return x[max(0, t - k)]


def build_obs40(data: dict, a: np.ndarray) -> np.ndarray:
    """Assemble the (N, 40) observation matrix for a single trajectory.

    ``a`` is the (N, 2) normalized action (used for ``a_prev`` and ``hist_a``).
    The layout matches ``AviatorManifoldEnv._get_obs`` exactly.
    """
    n = len(data["t"])
    theta = data["theta"]
    s = data["s"]
    theta_dot = data["theta_dot"]
    s_dot = data["s_dot"]

    x = np.stack([theta, s], axis=1)              # (N, 2)
    xdot = np.stack([theta_dot, s_dot], axis=1)   # (N, 2)

    obs = np.zeros((n, _OBS_DIM), dtype=np.float32)
    for t in range(n):
        o = np.zeros(_OBS_DIM, dtype=np.float32)
        o[0:2] = x[t]
        o[2:4] = xdot[t]
        o[4:6] = [data["sin_phi_L"][t], data["cos_phi_L"][t]]
        o[6:8] = [data["sin_phi_R"][t], data["cos_phi_R"][t]]
        o[8:10] = [data["m_phi_minus_L"][t], data["m_phi_minus_R"][t]]
        o[10:12] = [data["m_phi_plus_L"][t], data["m_phi_plus_R"][t]]
        o[12] = data["d_min"][t]
        o[13] = data["m_q"][t]
        o[14:16] = a[max(0, t - 1)] if t > 0 else np.zeros(2)   # a_prev
        idx = 16
        for k in _LAGS:
            o[idx:idx + 2] = _lag_clamped(x, t, k)
            idx += 2
        for k in _LAGS:
            o[idx:idx + 2] = _lag_clamped(xdot, t, k)
            idx += 2
        for k in _LAGS:
            # action history holds the action taken k steps *before* a_prev
            # (a_prev = a[t-1]); the online env stores action_{t-1-k} in
            # hist_a lag k, so the source row is t-1-k, zero-padded before the
            # episode start.  The old `_lag_clamped(a, t, k)` returned a[t-k]
            # (one step too fresh) and clamped the pre-start history to a[0]
            # instead of zeros (audit 2026-09-23 §1 "初始历史").
            src = t - 1 - k
            o[idx:idx + 2] = a[src] if src >= 0 else np.zeros(2)
            idx += 2
        obs[t] = o
    return obs


def build_transitions(data: dict, phi_dot_scale: float, dt: float = _DT,
                      lookup: ManifoldLookup | None = None) -> list:
    """Build the list of transition dicts for one trajectory.

    Row ``t`` -> transition ``t``: ``observations = obs[t]``,
    ``actions = phi_dot[t]/phi_dot_scale``, ``next_observations = obs[t+1]``.
    A trajectory of N rows yields N-1 transitions; the last (``t == N-2``),
    which lands on the final state ``x[N-1]``, is the terminal transition
    (``dones=True, mask=0``) carrying its real action + reward, matching the
    online env's ``truncated="end"`` step (audit 2026-09-23 §5).
    """
    n = len(data["t"])
    if lookup is not None:
        # The online environment observes and rewards interpolated LUT
        # clearance. DP CSV clearance is a fresh physical evaluation of the
        # same q and can differ by sub-millimetres. Keep the CSV as the
        # geometry audit source, but encode prior transitions with the exact
        # observation/reward value the critic will see online.
        data = dict(data)
        x = np.stack([data["theta"], data["s"]], axis=1)
        phi = np.stack([
            np.arctan2(data[f"sin_phi_{side}"], data[f"cos_phi_{side}"])
            for side in ("L", "R")
        ], axis=1)
        d_lookup = np.empty(n, dtype=np.float64)
        for start in range(0, n, 2048):
            stop = min(n, start + 2048)
            d_lookup[start:stop] = lookup.query(
                x[start:stop], phi[start:stop], check_safe=False)["d_min"]
        data["d_min"] = d_lookup
    phi_dot = np.stack([data["phi_dot_L"], data["phi_dot_R"]], axis=1)  # (N, 2)
    # A clipped label would no longer cause the recorded next phase. Reject the
    # whole trajectory if any executed transition exceeds the policy's range.
    # Keeping only the in-range transitions would retain inconsistent histories.
    if np.any(np.abs(phi_dot[:-1]) > phi_dot_scale + 1e-9):
        raise ValueError("teacher action exceeds phi_dot_scale; trajectory is not replayable")
    a = phi_dot / phi_dot_scale                                        # (N, 2)
    obs = build_obs40(data, a)

    x = np.stack([data["theta"], data["s"]], axis=1)                   # (N, 2)
    m_minus = np.stack([data["m_phi_minus_L"], data["m_phi_minus_R"]], axis=1)
    m_plus = np.stack([data["m_phi_plus_L"], data["m_phi_plus_R"]], axis=1)

    transitions = []
    for t in range(n - 1):
        a_prev = phi_dot[t - 1] if t >= 1 else None
        # a_prev2 (jerk) is deliberately disabled to match the online env,
        # which always passes a_prev2=None.  With dt=0.01 the jerk term
        # C_j = 0.05 * ||phi_dot_t - 2*phi_dot_{t-1} + phi_dot_{t-2}||^2 / dt^4
        # blows up to ~1e6 per step and would train the critic on a reward
        # the actor never observes (audit 2026-09-23 §1).
        rew, _ = reward_fn(
            x[t + 1], phi_dot[t], phi_dot[t], data["d_min"][t + 1],
            m_minus[t + 1], m_plus[t + 1], data["m_q"][t + 1], None,
            a_prev=a_prev, a_prev2=None, dt=dt,
        )
        # finite-horizon termination: the transition landing on the final state
        # ends the episode (mask=0), so the critic does not bootstrap off x[N-1]
        # and the demo's transition count/termination matches the online env's
        # truncated="end" step.  The old code kept this mask=1 and appended a
        # synthetic zero-action terminal row (reward 0, next_obs = obs), which
        # taught Q(s_N, 0)=0 and let the reaching transition bootstrap off a
        # terminal state (audit 2026-09-23 §5).
        is_terminal = (t == n - 2)
        trans = {
            "observations": {"state": obs[t][None].astype(np.float32)},
            "actions": a[t].astype(np.float32),
            "next_observations": {"state": obs[t + 1][None].astype(np.float32)},
            "rewards": np.float32(rew),
            "masks": np.float32(0.0 if is_terminal else 1.0),
            "dones": bool(is_terminal),
        }
        transitions.append(trans)
    return transitions


def build_dataset(csv_dir: str, out_dir: str,
                  manifold_dir: str = "data/aviator/manifold_phi") -> dict:
    """Read all CSVs, calibrate phi_dot_scale, write pkls + phi_dot_scale.json.

    Returns the scale manifest dict.
    """
    csv_paths = sorted(glob.glob(os.path.join(csv_dir, "traj_*.csv")))
    assert csv_paths, f"no traj_*.csv found in {csv_dir}"

    scale_manifest = compute_phi_dot_scale(csv_paths)
    phi_dot_scale = scale_manifest["phi_dot_scale"]
    lookup = ManifoldLookup(manifold_dir)

    os.makedirs(out_dir, exist_ok=True)
    rejected = []
    for p in csv_paths:
        data = read_dp_csv(p)
        stem = os.path.splitext(os.path.basename(p))[0]
        try:
            transitions = build_transitions(data, phi_dot_scale, lookup=lookup)
        except ValueError as exc:
            rejected.append({"trajectory": stem, "reason": str(exc)})
            stale = os.path.join(out_dir, stem + ".pkl")
            if os.path.isfile(stale):
                os.remove(stale)
            continue
        with open(os.path.join(out_dir, stem + ".pkl"), "wb") as f:
            pickle.dump(transitions, f)

    scale_manifest["n_valid_trajs"] = len(csv_paths) - len(rejected)
    scale_manifest["rejected"] = rejected
    scale_manifest["clearance_source"] = "manifold_lookup"

    with open(os.path.join(out_dir, "phi_dot_scale.json"), "w") as f:
        json.dump(scale_manifest, f, indent=2)

    return scale_manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv-dir", default="data/aviator/dp_demo/",
                    help="directory of record_dp CSVs")
    ap.add_argument("--out-dir", default="data/aviator/dp_demo/",
                    help="directory for pkls + phi_dot_scale.json")
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi",
                    help="LUT used by the online environment for observations and rewards")
    ap.add_argument("--check-only", action="store_true",
                    help="validate CSV -> transition round-trip without writing")
    args = ap.parse_args()

    if args.check_only:
        csv_paths = sorted(glob.glob(os.path.join(args.csv_dir, "traj_*.csv")))
        assert csv_paths, f"no traj_*.csv found in {args.csv_dir}"
        scale = compute_phi_dot_scale(csv_paths)
        print(f"phi_dot_scale={scale['phi_dot_scale']:.6g} "
              f"max_abs_phi_dot={scale['max_abs_phi_dot']} "
              f"flag={scale['flag_exceeds_nominal']}")
        for p in csv_paths[:3]:
            data = read_dp_csv(p)
            tr = build_transitions(data, scale["phi_dot_scale"],
                                   lookup=ManifoldLookup(args.manifold_dir))
            assert len(tr) == len(data["t"]) - 1
            assert tr[0]["observations"]["state"].shape == (1, 40)
            assert tr[-1]["dones"] is True and not tr[-2]["dones"]
        print(f"check-only OK: {len(csv_paths)} trajs, "
              f"round-trip valid on first 3")
        return

    manifest = build_dataset(args.csv_dir, args.out_dir, args.manifold_dir)
    print(f"wrote {manifest['n_valid_trajs']} pkls + phi_dot_scale.json to "
          f"{os.path.abspath(args.out_dir)}")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
