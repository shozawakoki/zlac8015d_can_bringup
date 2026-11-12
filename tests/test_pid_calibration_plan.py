"""Offline checks for the small PID tuning plan. Does not run ROS or CAN."""
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from sweep_core import load_plan

spec = importlib.util.spec_from_file_location(
    "pid_rank_test", ROOT / "scripts" / "rank_pid_calibration.py")
ranking = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ranking)


class PidPlanTests(unittest.TestCase):
    def test_three_candidates_and_strict_safety(self):
        plan = load_plan(ROOT / "config" / "pid_calibration_elevated.yaml")
        trials = plan["trials"]
        self.assertEqual(len(trials), 3)
        self.assertEqual([x["name"] for x in trials],
                         ["PI_reference", "PID_D001", "PID_D002"])
        self.assertEqual([x["gains"]["pid_kd"] for x in trials],
                         [0.0, 0.01, 0.02])
        for trial in trials:
            gains = trial["gains"]
            self.assertEqual(gains["pid_kp"], 0.4)
            self.assertEqual(gains["pid_ki"], 0.2)
            self.assertEqual(gains["pid_bumpless_handoff"], 0)
            self.assertEqual(gains["pid_three_stage_transition"], 0)
            self.assertEqual(trial["mode"], "pid_torque")
            self.assertEqual(trial["stages"][0]["linear_x"], 0)
            self.assertEqual(trial["stages"][-1]["linear_x"], 0)
        self.assertEqual(plan["surface"], "wheels_elevated")
        self.assertLessEqual(plan["safety"]["max_current_a"], 2.0)
        self.assertLessEqual(plan["safety"]["max_wheel_rad_s"], 1.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
