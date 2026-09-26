"""Cross-check t6's C++ lookup against the training Python lookup on many points."""
import io
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "hil-serl"))
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup


def main(binary: str):
    directory = ROOT / "hil-serl/data/aviator/manifold_phi"
    lookup = ManifoldLookup(str(directory))
    rng = np.random.default_rng(17)
    n = 600
    th = rng.uniform(lookup.th_min, lookup.th_max, n)
    s = rng.uniform(lookup.s_min, lookup.s_max, n)
    phi = rng.uniform(lookup.phi_axis[0], lookup.phi_axis[-1], (n, 2))
    x = np.column_stack([th, s])
    theta_nodes = (lookup.theta_axis[0], lookup.theta_axis[len(lookup.theta_axis) // 2],
                   lookup.theta_axis[-1])
    s_nodes = (lookup.s_axis[0], lookup.s_axis[len(lookup.s_axis) // 2],
               lookup.s_axis[-1])
    phi_nodes = (lookup.phi_axis[0], lookup.phi_axis[len(lookup.phi_axis) // 2],
                 lookup.phi_axis[-1])
    boundaries = np.array([(a, b, p, p) for a in theta_nodes for b in s_nodes
                           for p in phi_nodes])
    x = np.vstack([x, boundaries[:, :2]])
    phi = np.vstack([phi, boundaries[:, 2:]])
    expected = lookup.query(x, phi, check_safe=False)
    valid = (expected["branch"] < 2) & np.isfinite(expected["d_min"])
    valid &= np.isfinite(expected["qL"]).all(axis=1) & np.isfinite(expected["qR"]).all(axis=1)
    x, phi = x[valid], phi[valid]
    expected = lookup.query(x, phi, check_safe=False)
    points = np.column_stack([x, phi])
    payload = "\n".join(",".join(f"{v:.17g}" for v in row) for row in points) + "\n"
    result = subprocess.run([binary, str(directory)], input=payload,
                            text=True, capture_output=True, check=True)
    actual = np.loadtxt(io.StringIO(result.stdout), delimiter=",")
    q = np.column_stack([expected["qL"], expected["qR"]])
    reference = np.column_stack([
        q, expected["d_min"], expected["m_phi_minus"], expected["m_phi_plus"],
        expected["m_q"], expected["branch"]])
    np.testing.assert_allclose(actual, reference, atol=2e-6, rtol=0)
    print(f"C++/Python LUT parity: {len(points)} valid random and boundary points, "
          f"max difference {np.max(np.abs(actual - reference)):.3g}")


if __name__ == "__main__":
    main(sys.argv[1])
