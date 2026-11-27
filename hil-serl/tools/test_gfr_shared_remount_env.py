import unittest
import numpy as np
from tools.gfr1_shared_remount_env import arm_local_basis


class SharedCoordinateTests(unittest.TestCase):
    def jacobian(self):
        rng = np.random.default_rng(19)
        aq = np.zeros((12, 14))
        aq[:6, :7] = rng.normal(size=(6, 7))
        aq[6:, 7:] = rng.normal(size=(6, 7))
        return aq

    def test_axes_are_unit_null_and_have_fixed_arm_meaning(self):
        aq = self.jacobian()
        n, _ = arm_local_basis(aq)
        np.testing.assert_allclose(aq@n, 0., atol=1e-12)
        np.testing.assert_allclose(n.T@n, np.eye(2), atol=1e-12)
        np.testing.assert_array_equal(n[7:, 0], np.zeros(7))
        np.testing.assert_array_equal(n[:7, 1], np.zeros(7))
        self.assertGreater(n[2, 0], 0.)
        self.assertGreater(n[9, 1], 0.)

    def test_constraint_row_representations_do_not_flip_actions(self):
        aq = self.jacobian()
        n, _ = arm_local_basis(aq)
        rng = np.random.default_rng(21)
        for side in range(2):
            rotation, _ = np.linalg.qr(rng.normal(size=(6, 6)))
            aq[side*6:(side+1)*6] = rotation@aq[side*6:(side+1)*6]
        transformed, _ = arm_local_basis(aq)
        np.testing.assert_allclose(n, transformed, atol=1e-12)

    def test_rank_deficiency_cannot_silently_change_action_dimension(self):
        aq = self.jacobian()
        aq[0] = aq[1]
        with self.assertRaisesRegex(ValueError, "Rank deficient"):
            arm_local_basis(aq)


if __name__ == "__main__":
    unittest.main()
