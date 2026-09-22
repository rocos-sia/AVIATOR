"""Observation adapter for the proprio-only AviatorManifoldEnv (Task 2.1).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 2.1)

HIL-SERL's default ``EncodingWrapper`` assumes at least one camera
(``image_keys`` non-empty) and crashes on ``jnp.concatenate([])`` when there
are none.  Aviator v0.1 is *proprio-only*: the redundancy policy maps the 40-D
state to ``(phi_dot_L, phi_dot_R)`` with no vision at all.  This module
therefore replaces the image+proprio encoder with a minimal state encoder that

  * flattens the ChunkingWrapper stacking dim -- a single obs
    ``{"state": (1, 40)}`` -> ``(40,)``, a batch ``(B, 1, 40)`` -> ``(B, 40)``;
  * returns the raw 40-D state unchanged, matching the frozen
    MLP(256, 256, 256) spec (no 64-D proprio projection, unlike
    ``EncodingWrapper``'s Dense + LayerNorm + tanh).

There is deliberately NO ``"images"`` entry in the observation dict: the demo
transitions produced by ``tools/build_dp_dataset.py`` carry only
``{"state": ...}``, and an empty images dict would break
``ReplayBuffer.insert`` (and, upstream, ``MemoryEfficientReplayBuffer``
requires at least one pixel key).  The redundancy-only observation therefore
stays ``{"state": Box(1, 40)}`` after ``ChunkingWrapper(obs_horizon=1)``.
"""

from __future__ import annotations

import flax.linen as nn
import jax
import jax.numpy as jnp

__all__ = ["StateEncoder"]


class StateEncoder(nn.Module):
    """Flatten the chunked 40-D state observation into a flat feature vector."""

    @nn.compact
    def __call__(self, observations, train: bool = False, stop_gradient: bool = False, **kwargs):
        s = observations["state"]
        if s.ndim == 2:            # single obs {"state": (1, 40)} -> (40,)
            s = s.reshape(-1)
        elif s.ndim == 3:          # batch (B, 1, 40) -> (B, 40)
            s = s.reshape(s.shape[0], -1)
        else:
            raise ValueError(f"unexpected state shape {s.shape}")
        if stop_gradient:
            s = jax.lax.stop_gradient(s)
        return s
