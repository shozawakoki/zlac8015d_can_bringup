"""Validate CAN-free URDF transform without starting ROS nodes or motor drivers."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "launch" / "zlac_mock_integration.launch.py"


class MockUrdfTests(unittest.TestCase):
    def test_private_mock_urdf_has_no_real_driver_or_can_parameters(self):
        spec = importlib.util.spec_from_file_location("zlac_mock_launch", MODULE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        original = ROOT / "urdf" / "orange_go_2025.urdf"
        original_content = original.read_text(encoding="utf-8")
        mock_description = module.build_mock_description(str(original))
        root = ET.fromstring(mock_description)
        plugin = root.find(".//ros2_control/hardware/plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.text, "mock_components/GenericSystem")
        self.assertNotIn("zlac8015d_hardware/Zlac8015dSystemHardware",
                         mock_description)
        self.assertNotIn("can_interface", mock_description)
        self.assertNotIn("node_id", mock_description)
        joints = [j.attrib["name"] for j in root.findall(".//ros2_control/joint")]
        self.assertEqual(joints, ["left_wheel_joint", "right_wheel_joint"])
        # In-memory rewrite must never mutate the real URDF.
        self.assertEqual(original.read_text(encoding="utf-8"),
                         original_content)


if __name__ == "__main__":
    unittest.main(verbosity=2)
