"""No-CAN regression test for ANSI-colored ros2 control controller status."""
import importlib.util
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "zlac_mock_controller_parser", SCRIPTS / "run_mock_integration.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ControllerParserTests(unittest.TestCase):
    def test_colored_active_controllers(self):
        output = (
            "\x1b[92mjoint_state_broadcaster\x1b[0m "
            "joint_state_broadcaster/JointStateBroadcaster  "
            "\x1b[92mactive\x1b[0m\n"
            "\x1b[92mdiff_drive_controller  \x1b[0m "
            "diff_drive_controller/DiffDriveController      "
            "\x1b[92mactive\x1b[0m\n"
        )
        self.assertEqual(
            module.parse_active_controllers(output),
            {"joint_state_broadcaster", "diff_drive_controller"},
        )

    def test_inactive_not_accepted(self):
        output = (
            "joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active\n"
            "diff_drive_controller diff_drive_controller/DiffDriveController inactive\n"
        )
        self.assertEqual(
            module.parse_active_controllers(output),
            {"joint_state_broadcaster"},
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
