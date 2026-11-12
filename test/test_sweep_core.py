#!/usr/bin/env python3
"""Offline checks: no ROS node creation, SocketCAN access or motor motion."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sweep_core import compute_metrics, load_plan, HARD_MAX_TRAVEL_M


PLAN = """schema_version: 1
output_root: ~/ros2_experiments
repeats: 2
publish_hz: 20
ready_timeout_s: 10
between_trials_s: 3
wheel_radius_m: 0.064
wheel_separation_m: 0.354
safety:
  max_linear_m_s: 0.2
  max_angular_rad_s: 0.4
  max_wheel_rad_s: 10
  max_current_a: 4
defaults:
  current_limit_a: 4
profiles:
  - name: test
    stages:
      - {duration_s: 1.0, linear_x: 0.0, angular_z: 0.0}
      - {duration_s: 2.0, linear_x: 0.1, angular_z: 0.0}
      - {duration_s: 2.0, linear_x: 0.0, angular_z: 0.0}
candidates:
  - name: p
    mode: pid_torque
    gains: {pid_kp: 0.2, pid_ki: 0.0, pid_kd: 0.0}
  - name: a
    mode: ladrc_torque
    gains: {ladrc_wc: 3, ladrc_wo: 12}
"""


class SweepPlanTests(unittest.TestCase):
    def make_plan(self, text):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        path = Path(self.tempdir.name) / "plan.yaml"
        path.write_text(text, encoding="utf-8")
        return path

    def test_expands_repetitions(self):
        plan = load_plan(self.make_plan(PLAN))
        self.assertEqual(len(plan["trials"]), 4)
        self.assertEqual(plan["trials"][0]["mode"], "pid_torque")
        self.assertEqual(plan["trials"][0]["gains"]["pid_kp"], 0.2)
        self.assertEqual(plan["trials"][2]["mode"], "ladrc_torque")

    def test_rejects_excess_speed(self):
        self.assertRaises(ValueError, load_plan,
                          self.make_plan(PLAN.replace("linear_x: 0.1", "linear_x: 0.7")))

    def test_rejects_nonzero_terminal_velocity(self):
        self.assertRaises(ValueError, load_plan,
                          self.make_plan(PLAN.replace(
                              "duration_s: 2.0, linear_x: 0.0",
                              "duration_s: 2.0, linear_x: 0.1")))

    def test_rejects_long_displacement(self):
        self.assertLessEqual(HARD_MAX_TRAVEL_M, 0.50)
        changed = PLAN.replace(
            "duration_s: 2.0, linear_x: 0.1",
            "duration_s: 10.0, linear_x: 0.1")
        self.assertRaises(ValueError, load_plan, self.make_plan(changed))

    def test_rejects_current_limit_above_safety(self):
        self.assertRaises(ValueError, load_plan,
                          self.make_plan(PLAN.replace("current_limit_a: 4\nprofiles",
                                                       "current_limit_a: 8\nprofiles")))

    def test_rejects_negative_gain(self):
        self.assertRaises(ValueError, load_plan,
                          self.make_plan(PLAN.replace("pid_kp: 0.2", "pid_kp: -0.2")))


class MetricTests(unittest.TestCase):
    def test_exact_tracking(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            with (base / "cmd_vel.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["time_s", "linear_x_m_s", "angular_z_rad_s"])
                writer.writerows([[0, 0, 0], [1, 0.1, 0], [3, 0, 0]])
            with (base / "joint_states.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["time_s", "left_velocity_rad_s",
                                 "right_velocity_rad_s", "left_effort_A",
                                 "right_effort_A"])
                writer.writerows([
                    [1.5, 0.1 / 0.064, 0.1 / 0.064, 1, -1],
                    [2.5, 0.1 / 0.064, 0.1 / 0.064, 1, -1],
                    [3.5, 0, 0, 0, 0],
                ])
            values = compute_metrics(base, 0.064, 0.354)
            self.assertEqual(values["active_samples"], 2)
            self.assertAlmostEqual(values["combined_rmse_rad_s"], 0)
            self.assertAlmostEqual(values["left_current_rms_a"], 1)


if __name__ == "__main__":
    unittest.main()
