#!/usr/bin/env python3
"""Offline test: split recorded velocity steps by actual command transitions."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from stage_metrics import compute_stage_metrics, write_stage_metrics


class StageMetricsTests(unittest.TestCase):
    def test_two_levels_and_tail_statistics(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            with (base / "cmd_vel.csv").open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["time_s", "linear_x_m_s", "angular_z_rad_s"])
                w.writerows([
                    [0, 0, 0], [1, 0.05, 0], [1.5, 0.05, 0],
                    [2, 0.1, 0], [3, 0, 0], [4, 0, 0]
                ])
            with (base / "joint_states.csv").open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["time_s", "left_velocity_rad_s", "right_velocity_rad_s"])
                w.writerows([
                    [1.1, 0.05 / 0.064, 0.05 / 0.064],
                    [1.9, 0.05 / 0.064, 0.05 / 0.064],
                    [2.1, 0.1 / 0.064, 0.1 / 0.064],
                    [2.9, 0.1 / 0.064, 0.1 / 0.064],
                    [3.1, 0, 0]
                ])
            rows = compute_stage_metrics(base, 0.064, 0.354)
            self.assertEqual(len(rows), 2)
            self.assertEqual([row["samples"] for row in rows], [2, 2])
            self.assertAlmostEqual(rows[0]["ref_v_m_s"], 0.05)
            self.assertAlmostEqual(rows[1]["ref_v_m_s"], 0.1)
            self.assertAlmostEqual(rows[0]["combined_rmse_rad_s"], 0)
            self.assertAlmostEqual(rows[1]["left_tail_mean_error_rad_s"], 0)
            output = base / "stage_metrics.csv"
            write_stage_metrics(output, rows)
            self.assertTrue(output.exists())

    def test_zero_only_returns_empty(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            with (base / "cmd_vel.csv").open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["time_s", "linear_x_m_s", "angular_z_rad_s"])
                w.writerows([[0, 0, 0], [1, 0, 0]])
            with (base / "joint_states.csv").open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["time_s", "left_velocity_rad_s", "right_velocity_rad_s"])
                w.writerow([0.5, 0, 0])
            self.assertEqual(compute_stage_metrics(base, 0.064, 0.354), [])


if __name__ == "__main__":
    unittest.main()
