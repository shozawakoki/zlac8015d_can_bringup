#!/usr/bin/env python3
"""Offline tests for MIMO session recorder. No ROS nodes or CAN opened."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_mimo_accel_session.py"
spec = importlib.util.spec_from_file_location("mimo_session_capture", SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class CaptureTests(unittest.TestCase):
    def test_label_cleanup(self):
        self.assertEqual(mod.safe_label("turn / fast"), "turn_fast")
        self.assertEqual(mod.safe_label("  "), "test")

    def test_process_conflicts(self):
        sample = (
            "123 ros2 launch zlac_can_bringup zlac_can_bringup.launch.py\n"
            "124 /opt/ros/humble/lib/controller_manager/ros2_control_node\n"
            "125 python3 path_follow.py\n"
            "126 python3 can_launch_watchdog.py\n"
            "127 grep ros2_control_node\n"
        )
        found = mod.controller_conflicts(sample)
        self.assertEqual(len(found), 3)
        self.assertNotIn("grep", "\n".join(found))

    def test_session_dir_creation_does_not_collide(self):
        with tempfile.TemporaryDirectory() as td:
            a = mod.make_run_dir(td, "sudden_acceleration", "01")
            b = mod.make_run_dir(td, "sudden_acceleration", "01")
            self.assertTrue(a.is_dir())
            self.assertTrue(b.is_dir())
            self.assertNotEqual(a, b)

    def test_200hz_csv_copied_and_validated(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            source = td / "source" / "mimo_ladrc_track_200hz.csv"
            source.parent.mkdir()
            source.write_text(
                "time_s,ref_left_rad_s,vel_left_rad_s,"
                "cmd_current_left_A,actual_current_left_A,"
                "hat_v_m_s,hat_omega_rad_s,hat_d_v_m_s2,"
                "hat_d_omega_rad_s2\n"
                "0.000,1.0,0.0,0.2,0.1,0,0,0,0\n",
                encoding="utf-8"
            )
            with patch.object(mod, "HARDWARE_LOG", source):
                result = mod.save_hardware_csv(
                    td, started_wall_ns=source.stat().st_mtime_ns)
            self.assertEqual(result["status"], "copied")
            self.assertTrue((td / "mimo_ladrc_track_200hz.csv").exists())

    def test_reject_stale_hardware_log(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            source = td / "old.csv"
            source.write_text("time_s\n0\n", encoding="utf-8")
            with patch.object(mod, "HARDWARE_LOG", source):
                result = mod.save_hardware_csv(
                    td, started_wall_ns=source.stat().st_mtime_ns +
                    3_000_000_000)
            self.assertEqual(result["status"], "stale")
            self.assertFalse((td / "mimo_ladrc_track_200hz.csv").exists())

    def test_missing_hardware_log_reported(self):
        with tempfile.TemporaryDirectory() as td:
            with patch.object(mod, "HARDWARE_LOG",
                              Path(td) / "missing.csv"):
                result = mod.save_hardware_csv(Path(td),
                                                started_wall_ns=0)
            self.assertEqual(result["status"], "missing")

    def test_runtime_snapshot_is_read_only_and_saved(self):
        import subprocess
        with tempfile.TemporaryDirectory() as td:
            result = subprocess.CompletedProcess(
                args=["ros2"], returncode=0, stdout="ok\\n", stderr="")
            with patch.object(mod.subprocess, "run", return_value=result) as call:
                mod.capture_runtime_snapshot(Path(td), {"ROS_DOMAIN_ID": "0"})
                self.assertEqual(call.call_count, 5)
            self.assertEqual((Path(td) / "ros_nodes.txt").read_text(), "ok\\n")
            self.assertTrue(
                (Path(td) / "controller_manager_parameters.yaml").exists())

    def test_check_only_does_not_start_ros(self):
        with patch.object(mod, "prepare", return_value=Path("/dev/null")):
            with patch.object(mod, "spawn") as start:
                self.assertEqual(mod.main(["--check-only"]), 0)
                start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
