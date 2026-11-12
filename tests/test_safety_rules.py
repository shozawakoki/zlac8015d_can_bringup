#!/usr/bin/env python3
"""Offline regression tests for the exact feedback checks used by the runner."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from safety_rules import check_feedback_limits


BASE = dict(
    feedback_age_s=0.02,
    left_velocity_rad_s=0.70,
    right_velocity_rad_s=0.72,
    left_current_a=0.4,
    right_current_a=0.3,
    observed_travel_m=0.04,
    observed_rotation_rad=0.0,
    max_wheel_rad_s=1.2,
    max_current_a=4.0,
)


class SafetyRulesTests(unittest.TestCase):
    def check(self, **changes):
        params = BASE.copy()
        params.update(changes)
        return check_feedback_limits(**params)

    def test_nominal(self):
        self.assertIsNone(self.check())

    def test_overspeed_matches_previous_abort(self):
        with self.assertRaisesRegex(RuntimeError, "Wheel overspeed"):
            self.check(left_velocity_rad_s=1.86, right_velocity_rad_s=1.45)

    def test_speed_limit_is_inclusive(self):
        self.assertIsNone(self.check(left_velocity_rad_s=1.44))
        with self.assertRaisesRegex(RuntimeError, "Wheel overspeed"):
            self.check(left_velocity_rad_s=1.441)

    def test_feedback_timeout(self):
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.check(feedback_age_s=0.36)

    def test_current_limit(self):
        with self.assertRaisesRegex(RuntimeError, "Current exceeded"):
            self.check(left_current_a=4.81)

    def test_travel_limit(self):
        with self.assertRaisesRegex(RuntimeError, "travel exceeded"):
            self.check(observed_travel_m=0.651)

    def test_rotation_limit(self):
        with self.assertRaisesRegex(RuntimeError, "rotation exceeded"):
            self.check(observed_rotation_rad=1.01)

    def test_nan_and_inf_fail_closed(self):
        for changed in (
            {"left_velocity_rad_s": float("nan")},
            {"right_current_a": float("inf")},
            {"feedback_age_s": float("nan")},
        ):
            with self.subTest(changed=changed):
                with self.assertRaisesRegex(RuntimeError, "Non-finite"):
                    self.check(**changed)

    def test_negative_age_fails(self):
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.check(feedback_age_s=-0.1)

    def test_negative_velocity_limit_fails(self):
        with self.assertRaisesRegex(RuntimeError, "Invalid"):
            self.check(max_wheel_rad_s=-1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
