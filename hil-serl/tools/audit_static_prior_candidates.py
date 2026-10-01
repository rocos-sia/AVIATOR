"""MuJoCo 3.x collision gate for proposed phase-prior segments."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from tools.audit_static_collision import CollisionChecker


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candidates", type=Path, required=True)
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    checker = CollisionChecker(args.model)
    accepted = []
    for path in sorted(args.candidates.glob("*.npz")):
        with np.load(path) as file:
            x, q = file["x"], file["q"]
        safe = True
        for xx, qq in zip(x, q):
            clearance, contact = checker.check(xx, qq)
            if contact or clearance < 0.005:
                safe = False
                break
        if safe:
            for xx, qq in zip((x[:-1] + x[1:]) / 2, (q[:-1] + q[1:]) / 2):
                clearance, contact = checker.check(xx, qq)
                if contact or clearance < 0.005:
                    safe = False
                    break
        if safe:
            accepted.append(path.stem)
    args.output.write_text(json.dumps(accepted) + "\n")
    print(json.dumps({"candidates": len(list(args.candidates.glob('*.npz'))),
                      "accepted": len(accepted)}))


if __name__ == "__main__":
    main()
