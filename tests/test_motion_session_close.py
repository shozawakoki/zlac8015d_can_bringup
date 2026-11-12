#!/usr/bin/env python3
"""No ROS/CAN: test production MotionSession.zero_for and close with mocks."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "zlac_runner_motion_under_test", SCRIPTS / "run_gain_sweep.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeTwist:
    def __init__(self):
        self.linear = type("Linear", (), {"x": None})()
        self.angular = type("Angular", (), {"z": None})()


class MotionSessionCloseTests(unittest.TestCase):
    def session(self):
        session = object.__new__(runner.MotionSession)
        session.Twist = FakeTwist
        session.pub = MagicMock()
        session.spin = MagicMock()
        session.node = MagicMock()
        session.rclpy = MagicMock()
        session.rclpy.ok.return_value = True
        return session

    def test_zero_for_publishes_zero_commands(self):
        session = self.session()
        # Deterministic monotonic timeline, enough to execute two loop bodies.
        with patch.object(runner.time, "monotonic",
                          side_effect=[0.0, 0.1, 0.2, 0.3]):
            with patch.object(runner.time, "sleep"):
                session.zero_for(0.25)
        self.assertGreaterEqual(session.pub.publish.call_count, 1)
        for args, _ in session.pub.publish.call_args_list:
            self.assertEqual(args[0].linear.x, 0.0)
            self.assertEqual(args[0].angular.z, 0.0)

    def test_close_attempts_zero_before_ros_teardown(self):
        session = self.session()
        calls = []
        session.zero_for = lambda d: calls.append(("zero_for", d))
        session.node.destroy_node.side_effect = lambda: calls.append(("destroy",))
        session.rclpy.shutdown.side_effect = lambda: calls.append(("shutdown",))
        session.close()
        self.assertEqual(calls, [
            ("zero_for", 1.0), ("destroy",), ("shutdown",)
        ])

    def test_close_teardown_even_if_zero_publish_fails(self):
        session = self.session()
        session.zero_for = MagicMock(side_effect=RuntimeError("publisher failed"))
        with self.assertRaisesRegex(RuntimeError, "publisher failed"):
            session.close()
        session.node.destroy_node.assert_called_once_with()
        session.rclpy.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main(verbosity=2)
