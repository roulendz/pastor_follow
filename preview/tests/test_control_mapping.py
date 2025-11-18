import unittest

from raw_tracking.slider_motion import map_deg_to_norm, map_norm_to_deg, plan_center_animation_steps


class TestControlMapping(unittest.TestCase):
    def test_deg_to_norm_center(self):
        self.assertAlmostEqual(map_deg_to_norm(90, 90, 180), 0.0)
        self.assertAlmostEqual(map_deg_to_norm(0, 90, 180), -1.0)
        self.assertAlmostEqual(map_deg_to_norm(180, 90, 180), 1.0)

    def test_norm_to_deg_center(self):
        self.assertAlmostEqual(map_norm_to_deg(0.0, 90, 180), 90.0)
        self.assertAlmostEqual(map_norm_to_deg(-1.0, 90, 180), 0.0)
        self.assertAlmostEqual(map_norm_to_deg(1.0, 90, 180), 180.0)

    def test_animation_steps_monotonic(self):
        steps = plan_center_animation_steps(30.0, 90.0, duration_ms=240)
        # Steps should move toward center and finish close to center
        self.assertGreater(len(steps), 0)
        self.assertTrue(steps[0] > 30.0)  # moving upward
        self.assertAlmostEqual(steps[-1], 90.0, delta=2.0)


if __name__ == '__main__':
    unittest.main()