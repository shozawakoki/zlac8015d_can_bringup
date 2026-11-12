#!/usr/bin/env python3
"""Offline check for plain and ANSI-colored ROS 2 Humble controller status."""
import importlib.util
from pathlib import Path
import sys
import unittest

scripts = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location("run_gain_sweep", scripts / "run_gain_sweep.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

PLAIN = (
    "diff_drive_controller   diff_drive_controller/DiffDriveController      active\n"
    "joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active\n"
)


class ActiveControllerParserTests(unittest.TestCase):
    def test_plain_status(self):
        self.assertEqual(module.parse_active_controllers(PLAIN), {
            "diff_drive_controller", "joint_state_broadcaster"
        })

    def test_ansi_colored_status(self):
        colored = PLAIN.replace("active", "\x1b[32mactive\x1b[0m")
        self.assertEqual(module.parse_active_controllers(colored), {
            "diff_drive_controller", "joint_state_broadcaster"
        })

    def test_inactive_not_accepted(self):
        self.assertNotIn(
            "diff_drive_controller",
            module.parse_active_controllers(PLAIN.replace(
                "DiffDriveController      active",
                "DiffDriveController      inactive",
            )),
        )

    def test_ros_cli_preamble(self):
        cli = (
            "[INFO] [1794457155] [_ros2cli_10316]: waiting for service\n"
            + PLAIN
        )
        self.assertEqual(len(module.parse_active_controllers(cli)), 2)


if __name__ == "__main__":
    unittest.main()
