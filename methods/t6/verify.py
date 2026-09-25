"""Compare the C++ t6 smoke output to the selected Flax actor and Python LUT."""
import os
import re
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")
import flax.linen as nn
import jax.numpy as jnp
import numpy as np
from flax.training import checkpoints

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "hil-serl"))
from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from examples.experiments.aviator_manifold.wrappers import StateEncoder
from serl_launcher.networks.actor_critic_nets import Policy
from serl_launcher.networks.mlp import MLP

HERE = Path(__file__).resolve().parent
CHECKPOINT = (ROOT / "hil-serl/examples/experiments/aviator_manifold"
              / "route_a_lut_native_rlpd_800x64/checkpoint_60000")
LUT = ROOT / "hil-serl/data/aviator/manifold_phi"


def main(binary: Path):
    lookup = ManifoldLookup(str(LUT))
    x = np.array([[0.0, -0.08]])
    phi = np.clip(np.zeros(2), lookup.safe_interval(x)["phi_safe_lo"][0],
                  lookup.safe_interval(x)["phi_safe_hi"][0])
    sample = lookup.query(x, phi[None], check_safe=False)
    obs = np.zeros((1, 40), dtype=np.float32)
    obs[0, :4] = [x[0, 0], x[0, 1], 0, 0]
    obs[0, 4:8] = [np.sin(phi[0]), np.cos(phi[0]), np.sin(phi[1]), np.cos(phi[1])]
    obs[0, 8:10] = sample["m_phi_minus"][0]
    obs[0, 10:12] = sample["m_phi_plus"][0]
    obs[0, 12] = sample["d_min"][0]
    obs[0, 13] = sample["m_q"][0]
    obs[0, 16:24] = np.tile(x[0], 4)
    state = checkpoints.restore_checkpoint(str(CHECKPOINT), target=None)
    actor = Policy(encoder=StateEncoder(),
                   network=MLP(hidden_dims=[256, 256, 256], activations=nn.tanh,
                               use_layer_norm=True, activate_final=True),
                   action_dim=2, tanh_squash_distribution=True,
                   std_parameterization="exp", std_min=1e-5, std_max=5.0)
    expected_action = np.asarray(actor.apply(
        {"params": state["params"]["modules_actor"]},
        {"state": jnp.asarray(obs).reshape(1, 40)}).mode())
    proc = subprocess.run([str(binary), str(HERE / "actor_60k.onnx"), str(LUT),
                           "0", "-0.08", "0", "-0.08", "0", "0"],
                          check=True, capture_output=True, text=True)
    actual_action = np.fromstring(re.search(r"action=([^\n]+)", proc.stdout)[1], sep=",")
    np.testing.assert_allclose(actual_action, expected_action, atol=2e-5, rtol=0)
    q_cpp = np.fromstring(re.search(r"q=([^\n]+)", proc.stdout)[1], sep=",")
    phi_next = phi + actual_action * 1.5 * 0.01
    q_py = lookup.query(x, phi_next[None], check_safe=False)
    np.testing.assert_allclose(q_cpp, np.r_[q_py["qL"][0], q_py["qR"][0]],
                               atol=2e-6, rtol=0)
    phi_cpp = np.fromstring(re.search(r"step_status=[^\n]*phi=([^\n]+)", proc.stdout)[1], sep=",")
    projected = np.fromstring(re.search(r"projected_phi=([^ ]+)", proc.stdout)[1], sep=",")
    rms = np.fromstring(re.search(r" rms=([^\n]+)", proc.stdout)[1], sep=",")
    np.testing.assert_allclose(projected, phi_cpp, atol=1e-7, rtol=0)
    assert np.max(rms) < 1e-6, rms
    print("Selected Flax/C++ actor and Python/C++ LUT parity passed")
    print(f"action={actual_action}, max_q_error={np.max(np.abs(q_cpp - np.r_[q_py['qL'][0], q_py['qR'][0]])):.3g}")


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())
