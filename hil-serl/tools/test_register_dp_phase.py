"""Sequence registration checks: action bounds must hold across the whole path."""

import numpy as np

from tools.register_dp_phase import register_sequence


def test_registration_finds_continuous_path_over_pointwise_jumps():
    axis = np.array([0.0, 0.01, 0.02, 0.03])
    error = np.array([
        [0.0, 0.1, 1.0, 1.0],
        [1.0, 0.1, 1.0, 0.0],
        [0.0, 0.1, 1.0, 1.0],
    ])
    path, status, _ = register_sequence(error, axis, np.array([0.01, 0.01]),
                                        np.ones_like(error, dtype=bool), 1.5)
    assert status == "complete"
    assert np.all(np.abs(np.diff(axis[path])) <= 0.015)
    assert np.max(error[np.arange(3), path]) == 0.1


def test_registration_reports_missing_feasible_state():
    error = np.zeros((3, 2))
    allowed = np.ones_like(error, dtype=bool)
    allowed[1] = False
    path, status, stop = register_sequence(error, np.array([0.0, 0.01]),
                                           np.array([0.01, 0.01]), allowed, 1.5)
    assert path is None and status == "no_allowed_state" and stop == 1
