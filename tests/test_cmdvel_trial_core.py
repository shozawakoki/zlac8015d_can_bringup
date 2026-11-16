"""Offline tests: python3 -m unittest discover -s tests -p test_cmdvel_trial_core.py"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

CORE = Path(__file__).resolve().parents[1] / "scripts" / "cmdvel_trial_core.py"
spec = importlib.util.spec_from_file_location("cmdvel_trial_core", CORE)
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


class TrialCoreTests(unittest.TestCase):
    def test_forward_and_turn_kinematics(self):
        left, right = core.wheel_speeds(0.2, 0.0)
        self.assertAlmostEqual(left, 0.2 / 0.064)
        self.assertAlmostEqual(right, 0.2 / 0.064)
        v, w = core.body_speeds(left, right)
        self.assertAlmostEqual(v, 0.2)
        self.assertAlmostEqual(w, 0.0)

        left, right = core.wheel_speeds(0.15, -0.3)
        v, w = core.body_speeds(left, right)
        self.assertAlmostEqual(v, 0.15)
        self.assertAlmostEqual(w, -0.3)

    def test_default_trial_and_max_duration(self):
        self.assertEqual(
            len(core.validate_trial(0.1, 0.2, 3.0, 20.0, 3.0, 0.2, 0.4)), 2)
        with self.assertRaises(ValueError):
            core.validate_trial(0.1, 0.0, 120.0, 20, 3, 0.2, 0.4)

    def test_reject_nonfinite_or_zero_motion(self):
        for v, w in ((math.nan, 0), (0, math.inf), (0, 0)):
            with self.assertRaises(ValueError):
                core.validate_trial(v, w, 3, 20, 3, 0.2, 0.4)

    def test_speed_caps_and_hardware_margin(self):
        # The combined maximum command is allowed. Its faster wheel has
        # peripheral speed 1 + (0.354/2)*1 = 1.177 m/s < 1.5 m/s.
        left, right = core.validate_trial(
            1.0, 1.0, 3, 20, 3, 1.0, 1.0, 1.5)
        self.assertAlmostEqual(left * 0.064, 0.823)
        self.assertAlmostEqual(right * 0.064, 1.177)
        self.assertEqual(
            len(core.validate_trial(-1.0, -1.0, 3, 20, 3, 1.0, 1.0)), 2)

        with self.assertRaises(ValueError):
            core.validate_trial(1.01, 0, 3, 20, 3, 1.0, 1.0)
        with self.assertRaises(ValueError):
            core.validate_trial(0.0, 1.01, 3, 20, 3, 1.0, 1.0)
        with self.assertRaises(ValueError):
            core.validate_trial(0.0, 0.5, 3, 20, 3, 1.0, 1.1)
        # Application overspeed abort remains configurable only below 2 m/s.
        with self.assertRaises(ValueError):
            core.validate_trial(1.0, 1.0, 3, 20, 3, 1.0, 1.0, 1.1)
        with self.assertRaises(ValueError):
            core.validate_trial(0.1, 0.0, 3, 20, 3, 1.0, 1.0, 2.0)

    def test_publisher_conflict(self):
        own = ("mimo_cmdvel_trial_recorder", "/")
        self.assertEqual(core.other_publishers([own], *own), [])
        publishers = [own, ("path_follower_node", "/")]
        self.assertEqual(core.other_publishers(publishers, *own),
                         [("path_follower_node", "/")])


if __name__ == "__main__":
    unittest.main()
