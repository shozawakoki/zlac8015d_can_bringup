"""Offline validation for ground-contact PID candidate and preflight plans."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from sweep_core import load_plan


class GroundPidPlanTests(unittest.TestCase):
    def test_ground_preflight_and_sweep(self):
        for name, expected in (
            ("pid_calibration_ground_preflight.yaml", [0.0]),
            ("pid_calibration_ground.yaml", [0.0, 0.01, 0.02]),
        ):
            with self.subTest(name=name):
                plan = load_plan(ROOT / "config" / name)
                self.assertEqual(plan["surface"], "ground_level")
                self.assertEqual(len(plan["trials"]), len(expected))
                self.assertEqual([r["gains"]["pid_kd"] for r in plan["trials"]],
                                 expected)
                self.assertLessEqual(plan["safety"]["max_linear_m_s"], 0.06)
                self.assertLessEqual(plan["safety"]["max_wheel_rad_s"], 1.2)
                self.assertLessEqual(plan["safety"]["max_current_a"], 2.0)
                for trial in plan["trials"]:
                    gains = trial["gains"]
                    self.assertEqual(trial["mode"], "pid_torque")
                    self.assertEqual((gains["pid_kp"], gains["pid_ki"]),
                                     (0.4, 0.2))
                    self.assertEqual(gains["pid_bumpless_handoff"], 0)
                    self.assertEqual(gains["pid_three_stage_transition"], 0)
                    self.assertEqual(trial["stages"][0]["linear_x"], 0.0)
                    self.assertEqual(trial["stages"][-1]["linear_x"], 0.0)
                    commanded_distance = sum(
                        abs(s["linear_x"]) * s["duration_s"]
                        for s in trial["stages"])
                    self.assertLessEqual(commanded_distance, 0.10 + 1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
