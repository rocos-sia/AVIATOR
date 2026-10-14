"""Exercise the real SAC loss on accepted and rejected execution labels."""
import jax
import jax.numpy as jnp
import numpy as np
import pytest

from serl_launcher.agents.continuous.sac import SACAgent


class LossFixture:
    config = {'correction_weight': 1., 'correction_action_scale': 1.5}

    def forward_temperature(self):
        return 0.

    def forward_policy(self, observations, rng, grad_params):
        class Distribution:
            def sample_and_log_prob(self, seed):
                return grad_params, jnp.zeros((len(grad_params),))
        return Distribution()

    def forward_critic(self, observations, actions, rng):
        return jnp.zeros((2, len(actions)))


@pytest.mark.parametrize('beta,recovery,expected_valid', [
    (1., 0., True), (.25, 0., True), (0., 1., False), (1., 1., False),
])
def test_exhausted_governor_is_not_a_supervision_label(beta, recovery, expected_valid):
    # Failed next observations used to retain a previous nonzero executed r.
    state = jnp.zeros((1, 1, 40)).at[0, 0, 33:35].set(jnp.array([.6, -.3]))
    state = state.at[0, 0, 35].set(beta).at[0, 0, 36].set(1.)
    state = state.at[0, 0, 38].set(recovery)
    batch = {'rewards': jnp.zeros(1), 'observations': {},
             'next_observations': {'state': state}}

    def objective(actions):
        return SACAgent.policy_loss_fn(LossFixture(), batch, actions,
                                      jax.random.PRNGKey(0))[0]

    actions = jnp.zeros((1, 2))
    loss = float(objective(actions))
    gradient = np.asarray(jax.grad(objective)(actions))
    if expected_valid:
        assert loss == pytest.approx(.2)
        np.testing.assert_allclose(gradient, [[-.8, .4]], atol=1e-6)
    else:
        assert loss == 0.
        np.testing.assert_array_equal(gradient, 0.)
