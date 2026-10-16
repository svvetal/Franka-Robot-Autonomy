"""Non-GUI numeric sanity checks for the supplied Franka planning code."""
import math
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_planning"))
from Franka import FrankArm
from RobotUtil import BlockDesc2Points, CheckBoxBoxCollision, rpyxyz2H


class CollisionGeometryTest(unittest.TestCase):
    @staticmethod
    def box(center, dims=(1.0, 1.0, 1.0), rpy=(0, 0, 0)):
        corners, axes = BlockDesc2Points(rpyxyz2H(rpy, center), dims)
        return np.asarray(corners), np.asarray(axes)

    def check_box_pair(self, box_a, box_b):
        return CheckBoxBoxCollision(*box_a, *box_b)

    def test_separated_cuboids(self):
        self.assertFalse(self.check_box_pair(self.box((0, 0, 0)), self.box((3, 0, 0))))

    def test_overlapping_cuboids(self):
        self.assertTrue(self.check_box_pair(self.box((0, 0, 0)), self.box((0.4, 0.4, 0))))

    def test_touching_faces_are_contact(self):
        self.assertTrue(self.check_box_pair(self.box((0, 0, 0)), self.box((1, 0, 0))))

    def test_rotated_overlap_and_separation(self):
        self.assertTrue(self.check_box_pair(self.box((0, 0, 0)), self.box((0.8, 0, 0), rpy=(0, 0, math.pi/4))))
        self.assertFalse(self.check_box_pair(self.box((0, 0, 0)), self.box((4, 0, 0), rpy=(0, 0, math.pi/4))))

    def test_reference_eight_collision_cases(self):
        reference = self.box((0, 0, 0), (3, 1, 2))
        origins = [
            (0, 1, 0), (1.5, -1.5, 0), (0, 0, -1), (3, 0, 0),
            (-1, 0, -2), (1.8, 0.5, 1.5), (0, -1.2, 0.4), (-0.8, 0, -0.5),
        ]
        orientations = [
            (0, 0, 0), (1, 0, 1.5), (0, 0, 0), (0, 0, 0),
            (0.5, 0, 0.4), (-0.2, 0.5, 0), (0, 0.785, 0.785), (0, 0, 0.2),
        ]
        dimensions = [
            (0.8, 0.8, 0.8), (1, 3, 3), (2, 3, 1), (3, 1, 1),
            (2, 0.7, 2), (1, 3, 1), (1, 1, 1), (1, 0.5, 0.5),
        ]
        expected = [False, False, True, True, True, False, True, True]
        observed = [self.check_box_pair(reference, self.box(origin, dimensions[i], orientations[i]))
                    for i, origin in enumerate(origins)]
        self.assertEqual(observed, expected)


class KinematicsTest(unittest.TestCase):
    def test_fk_home_transform(self):
        arm = FrankArm()
        transforms, jacobian = arm.ForwardKin(np.zeros(7))
        self.assertEqual(jacobian.shape, (6, 7))
        self.assertEqual(len(transforms), 8)
        np.testing.assert_allclose(transforms[-1][:3, 3], [0.088, 0, 0.926], atol=1e-6)
        np.testing.assert_allclose(transforms[-1][:3, :3],
                                   np.diag([1., -1., -1.]), atol=1e-6)

    def test_jacobian_values_finite(self):
        arm = FrankArm()
        _, jacobian = arm.ForwardKin([0.1, -0.2, 0.3, -0.6, 0.2, 1.2, 0.1])
        self.assertTrue(np.isfinite(jacobian).all())


if __name__ == '__main__':
    unittest.main()
