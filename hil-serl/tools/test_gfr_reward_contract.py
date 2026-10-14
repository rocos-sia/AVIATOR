#!/usr/bin/env python3
"""Contract probe for the GFR-1 reward decomposition."""

from __future__ import annotations

import argparse

from examples.experiments.aviator_manifold.gfr_env import GFRManifoldEnv


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--trajectory-dir", required=True)
    args = p.parse_args()
    env = GFRManifoldEnv(args.manifold, args.model, args.trajectory_dir, mode="structured")
    env.reset(seed=0, options={"trajectory_index": 0})
    _, _, _, _, info = env.step(env.action_space.sample() * 0.0)
    expected = {"progress", "intervention", "recovery", "compute", "terminal"}
    missing = expected.difference(info.get("reward_components", {}))
    if missing:
        raise AssertionError(f"missing reward components: {sorted(missing)}")
    print("reward contract pass")


if __name__ == "__main__":
    main()
