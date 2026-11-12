#!/usr/bin/env python3
"""CAN-free end-to-end mock diff_drive_controller command/stop smoke test.

Uses ONLY zlac_mock_integration.launch.py (mock_components/GenericSystem).
Refuses to run when an existing controller_manager is discoverable.
No test command is sent to a real robot.
"""
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time


def assert_no_existing_controller():
    result = subprocess.run(
        ["ros2", "node", "list"], capture_output=True, text=True, timeout=8,
        check=True)
    names = [name.strip() for name in result.stdout.splitlines()]
    if any(name.rsplit("/", 1)[-1] == "controller_manager" for name in names):
        raise RuntimeError(
            "Existing /controller_manager detected; refusing mock test")
    if any(name.rsplit("/", 1)[-1] == "zlac_gain_sweep_commander"
           for name in names):
        raise RuntimeError("Existing experiment commander detected")


def parse_active_controllers(output):
    """Parse colored or plain ROS 2 controller-manager CLI output."""
    clean = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", output).replace("\r", "\n")
    active = set()
    for line in clean.splitlines():
        words = line.split()
        if len(words) >= 3 and words[2] == "active":
            active.add(words[0])
    return active


def stop_process(proc):
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=7)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=2)


def run_check():
    assert_no_existing_controller()
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState

    proc = None
    node = None
    ros_started = False
    log_handle = None
    log_path = Path('/tmp/zlac_mock_integration_launch.log')
    last_controller_listing = 'not queried'
    try:
        # Isolated ROS_DOMAIN_ID avoids accidental interaction with live ROS
        # systems. Must be set before importing/initializing rclpy.
        env = os.environ.copy()
        command = [
            "ros2", "launch", "zlac_can_bringup",
            "zlac_mock_integration.launch.py",
        ]
        print("Launching CAN-free mock controller...", flush=True)
        log_handle = log_path.open('w', encoding='utf-8')
        print('Mock launch log:', log_path, flush=True)
        proc = subprocess.Popen(
            command, env=env, start_new_session=True,
            stdout=log_handle, stderr=subprocess.STDOUT)
        rclpy.init(args=None)
        ros_started = True
        node = rclpy.create_node("zlac_mock_integration_checker")
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        states = []

        def on_joint(msg):
            try:
                l = msg.name.index("left_wheel_joint")
                r = msg.name.index("right_wheel_joint")
                states.append((time.monotonic(),
                               float(msg.velocity[l]),
                               float(msg.velocity[r])))
            except (ValueError, IndexError):
                pass

        node.create_subscription(JointState, "/joint_states", on_joint, 10)

        def send(linear, duration):
            stop = time.monotonic() + duration
            while time.monotonic() < stop:
                if proc.poll() is not None:
                    raise RuntimeError("Mock launch exited unexpectedly")
                cmd = Twist()
                cmd.linear.x = linear
                pub.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.04)
                time.sleep(0.01)

        # Let both spawners finish while publishing only zero.
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            send(0.0, 0.20)
            if len(states) >= 5:
                break
        if len(states) < 5:
            raise RuntimeError("No mock joint_states; check mock plugin availability")
        # Do not drive the mock until BOTH controllers are active.
        ready = False
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            cmd_result = subprocess.run(
                ["ros2", "control", "list_controllers",
                 "--controller-manager", "/controller_manager"],
                capture_output=True, text=True, timeout=5)
            last_controller_listing = ('exit=%d stdout=%r stderr=%r' %
                (cmd_result.returncode, cmd_result.stdout, cmd_result.stderr))
            active = parse_active_controllers(cmd_result.stdout)
            ready = cmd_result.returncode == 0 and {
                "diff_drive_controller", "joint_state_broadcaster"
            }.issubset(active)
            if ready:
                break
            send(0.0, 0.20)
        if not ready:
            raise RuntimeError('Mock diff_drive_controller not active; ' +
                               last_controller_listing)
        states.clear()

        # Command only in isolated mock domain. Wheels should mirror roughly
        # 0.05m/s / 0.064m = 0.78125rad/s under GenericSystem.
        send(0.05, 1.2)
        motion = list(states)
        if not any(a > 0.15 and b > 0.15 for _, a, b in motion):
            raise RuntimeError("Mock wheel velocity did not respond to /cmd_vel")

        states.clear()
        send(0.0, 1.2)
        stopped = list(states)
        if not any(abs(a) < 0.05 and abs(b) < 0.05 for _, a, b in stopped[-10:]):
            raise RuntimeError("Mock wheels did not return to zero after /cmd_vel=0")

        print("PASS: mock diff_drive_controller motion -> zero", flush=True)
        print("NOTE: This does NOT test real PI, CAN or physical stopping.")
    finally:
        if node is not None:
            node.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()
        stop_process(proc)
        if log_handle is not None:
            log_handle.close()
        if log_path.is_file():
            print('Mock launch log (last 70 lines):', flush=True)
            for line in log_path.read_text(errors='replace').splitlines()[-70:]:
                print(line, flush=True)


if __name__ == "__main__":
    # A domain different from any in-service robot must be chosen by operator.
    if "ROS_DOMAIN_ID" not in os.environ:
        sys.exit("Set an isolated ROS_DOMAIN_ID before running (e.g. 197).")
    try:
        run_check()
    except Exception as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(1)
