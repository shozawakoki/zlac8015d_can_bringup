#!/usr/bin/env python3
"""No ROS/CAN: exercise production run_trial exception/finally shutdown path."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "zlac_runner_under_test", SCRIPTS / "run_gain_sweep.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeSession:
    def __init__(self, events, fail_close=False):
        self.events = events
        self.fail_close = fail_close

    def close(self):
        self.events.append("zero_and_close")
        if self.fail_close:
            raise RuntimeError("mock zero publisher failed")


class AbortPathTests(unittest.TestCase):
    def trial(self):
        return {
            "name": "mock_trial",
            "mode": "pid_torque",
            "profile": "mock_profile",
            "repeat": 1,
            "gains": {"pid_three_stage_transition": 1},
            "stages": [{"duration_s": 1.0, "linear_x": 0.05, "angular_z": 0.0}],
            "wheel_radius_m": 0.064,
            "wheel_separation_m": 0.354,
        }

    def run_mock(self, *, fail_close=False, fault=None):
        events = []
        session = FakeSession(events, fail_close=fail_close)
        process = object()
        fault = fault or RuntimeError(
            "Wheel overspeed: left=1.86 right=1.45 rad/s")

        def fake_start(*args):
            events.append("start")
            return process, object()

        def fake_ready(*args):
            events.append("ready")

        def fake_profile(*args):
            events.append("profile")
            raise fault

        def fake_stop(proc, log):
            self.assertIs(proc, process)
            events.append("stop_launch")

        with tempfile.TemporaryDirectory() as temp:
            with contextlib.ExitStack() as stack:
                stack.enter_context(patch.object(runner, "no_existing_robot_controller"))
                stack.enter_context(patch.object(runner, "start_launch", side_effect=fake_start))
                stack.enter_context(patch.object(runner, "MotionSession", return_value=session))
                stack.enter_context(patch.object(runner, "wait_ready", side_effect=fake_ready))
                stack.enter_context(patch.object(runner, "follow_profile", side_effect=fake_profile))
                stack.enter_context(patch.object(runner, "stop_launch", side_effect=fake_stop))
                stack.enter_context(patch.object(runner, "safe_log_copy", return_value=None))
                stack.enter_context(patch.object(runner, "find_recorded_run", return_value=None))
                with contextlib.redirect_stdout(io.StringIO()):
                    record = runner.run_trial(
                        self.trial(), {}, Path(temp), 1, 1)
            self.assertTrue((Path(temp) / "001_mock_trial_mock_profile_rep01" /
                             "trial.json").exists())
        return events, record

    def test_overspeed_calls_zero_close_before_launch_stop(self):
        events, record = self.run_mock()
        self.assertEqual(
            events, ["start", "ready", "profile", "zero_and_close", "stop_launch"])
        self.assertEqual(record["status"], "aborted")
        self.assertIn("Wheel overspeed", record["reason"])

    def test_feedback_timeout_also_calls_shutdown(self):
        events, record = self.run_mock(
            fault=RuntimeError("Wheel-speed feedback stale: 0.360 s"))
        self.assertEqual(events[-2:], ["zero_and_close", "stop_launch"])
        self.assertEqual(record["status"], "aborted")
        self.assertIn("stale", record["reason"])

    def test_zero_publish_failure_still_stops_launch(self):
        events, record = self.run_mock(fail_close=True)
        self.assertEqual(events[-2:], ["zero_and_close", "stop_launch"])
        self.assertEqual(record["status"], "aborted")
        self.assertIn("Wheel overspeed", record["reason"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
