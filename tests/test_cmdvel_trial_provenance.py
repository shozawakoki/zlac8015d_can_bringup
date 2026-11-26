"""Offline tests of /proc provenance, URDF snapshots, and mode-to-CSV mapping.

Run: python3 -m unittest discover -s tests -p 'test_cmdvel_trial*.py' -v
No ROS, CAN, or wheel movement is needed.
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


CORE = Path(__file__).resolve().parents[1] / "scripts" / "cmdvel_trial_core.py"
spec = importlib.util.spec_from_file_location("cmdvel_trial_core", CORE)
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


class RuntimeProvenanceTests(unittest.TestCase):
    def make_process(self, root, pid, mode=None, stat_start=123456):
        p = root / str(pid)
        p.mkdir(parents=True)
        (p / "cmdline").write_bytes(
            b"/opt/ros/humble/lib/controller_manager/ros2_control_node\x00"
            b"--ros-args\x00")
        parts = [b"SECRET=must_not_copy", b"HOME=/home/ubuntu"]
        if mode:
            parts.append(("ZLAC_CONTROL_MODE=" + mode).encode("utf-8"))
        (p / "environ").write_bytes(b"\x00".join(parts) + b"\x00")
        # Field 22 (starttime) is item 19 after the closing ')' in stat.
        tail = ["S"] + ["0"] * 18 + [str(stat_start)]
        (p / "stat").write_text(
            "%d (ros2_control_node) %s\n" % (pid, " ".join(tail)),
            encoding="utf-8")
        return p

    def test_controller_env_and_pid_identity(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            self.make_process(root, 501, "mimo_pid_no_breakaway")
            (root / "502").mkdir()
            (root / "502" / "cmdline").write_bytes(b"/bin/bash\x00")
            found = core.controller_processes(root)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["env_control_mode"], "mimo_pid_no_breakaway")
        self.assertEqual(found[0]["start_ticks"], 123456)
        self.assertNotIn("SECRET", repr(found))

    def test_urdf_config_is_not_runtime_proof(self):
        xml = """<robot><ros2_control name="drive" type="system">
          <hardware><param name="control_mode">mimo_ladrc_torque</param>
          <param name="mimo_pid_kp_v">0.7</param>
          <param name="mimo_pid_ki_v">0.5</param></hardware>
          </ros2_control></robot>"""
        with tempfile.TemporaryDirectory() as d:
            file = Path(d) / "robot.urdf"
            file.write_text(xml, encoding="utf-8")
            data = core.read_urdf_hardware_config(file)
        self.assertEqual(data["default_control_mode"], "mimo_ladrc_torque")
        self.assertEqual(
            data["hardware_parameters_as_declared_in_urdf"]["mimo_pid_ki_v"], "0.5")
        self.assertFalse(data["parameters_are_runtime_verified"])

    def test_running_mode_beats_urdf_default(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            self.make_process(root, 333, "mimo_pid_no_breakaway")
            found = core.controller_processes(root)
        urdf = {"default_control_mode": "mimo_ladrc_torque"}
        result = core.resolve_controller_runtime(found, urdf)
        self.assertEqual(result["mode"], "mimo_pid_no_breakaway")
        self.assertEqual(result["mode_source"], "running_ros2_control_node_environment")
        self.assertFalse(result["plugin_compiled_mode_verified"])
        self.assertEqual(
            core.hardware_log_for_mode(result["mode"]),
            "/tmp/mimo_pid_no_breakaway_200hz.csv")

    def test_ladrc_selects_actual_current_csv(self):
        self.assertEqual(
            core.hardware_log_for_mode("mimo_ladrc_no_breakaway_actual_current"),
            "/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv")
        self.assertIsNone(core.hardware_log_for_mode("velocity"))
        self.assertIsNone(core.hardware_log_for_mode("made_up_mode"))

    def test_wrong_declared_mode_refuses_motion(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            self.make_process(root, 51, "mimo_pid_no_breakaway")
            found = core.controller_processes(root)
        with self.assertRaisesRegex(ValueError, "control-mode mismatch"):
            core.resolve_controller_runtime(
                found, {"default_control_mode": "mimo_ladrc_torque"},
                "mimo_ladrc_no_breakaway_actual_current")

    def test_duplicate_controllers_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            self.make_process(root, 51, "mimo_pid_no_breakaway")
            self.make_process(root, 52, "mimo_pid_no_breakaway")
            found = core.controller_processes(root)
        with self.assertRaisesRegex(ValueError, "multiple ros2_control_node"):
            core.resolve_controller_runtime(found, None)

    def test_no_process_is_unverified_and_not_guessed_from_recorder_env(self):
        urdf = {"default_control_mode": "mimo_ladrc_torque"}
        result = core.resolve_controller_runtime([], urdf)
        self.assertEqual(result["mode_source"], "installed_urdf_default_unverified")
        self.assertFalse(result["plugin_compiled_mode_verified"])
        self.assertEqual(result["mode"], "mimo_ladrc_torque")

    def test_pid_restarts_detected(self):
        a = core.resolve_controller_runtime([
            {"pid": 51, "start_ticks": 123, "env_control_mode": "mimo_pid_no_breakaway",
             "environment_readable": True}], None)
        b = core.resolve_controller_runtime([
            {"pid": 51, "start_ticks": 124, "env_control_mode": "mimo_pid_no_breakaway",
             "environment_readable": True}], None)
        self.assertFalse(core.same_controller_process(a, b))
        self.assertTrue(core.same_controller_process(a, a))
        self.assertIsNone(core.same_controller_process(a, None))


if __name__ == "__main__":
    unittest.main()
