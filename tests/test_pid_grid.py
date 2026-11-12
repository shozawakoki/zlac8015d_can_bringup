"""No-motion checks of expanded PID candidate generator."""
import sys
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from generate_pid_grid import generate
from sweep_core import load_plan


class PidGridTests(unittest.TestCase):
    def test_staged_grid_is_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            created = generate(Path(folder))
            self.assertEqual([count for _, count in created], [12, 13, 18])
            expected = {"A_PI_grid": 12, "B_PID_grid": 13,
                        "C_validation": 18}
            for path, count in created:
                plan = load_plan(path)
                self.assertEqual(count, expected[path.stem])
                self.assertEqual(plan["surface"], "ground_level")
                self.assertEqual(len(plan["trials"]), count)
                self.assertLessEqual(plan["safety"]["max_current_a"], 2)
                self.assertLessEqual(plan["safety"]["max_wheel_rad_s"], 1.2)
                for trial in plan["trials"]:
                    self.assertEqual(trial["mode"], "pid_torque")
                    self.assertEqual(trial["gains"]["pid_bumpless_handoff"], 0)
                    self.assertEqual(
                        trial["gains"]["pid_three_stage_transition"], 0)
                    self.assertLessEqual(
                        sum(abs(s["linear_x"]) * s["duration_s"]
                            for s in trial["stages"]), 0.12 + 1e-9)

    def test_first_trial_is_initial_pi_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            created = generate(Path(folder))
            plan = load_plan(created[0][0])
            first = plan["trials"][0]
            self.assertEqual(first["name"], "P040_I020_D000")
            self.assertEqual((first["gains"]["pid_kp"],
                              first["gains"]["pid_ki"],
                              first["gains"]["pid_kd"]), (0.4, 0.2, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
