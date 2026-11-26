#!/usr/bin/env python3
"""Bounded /cmd_vel test with raw ROS feedback and hardware CSV collection.

ROS 2 Humble. Does NOT launch motors or change controller mode/gains.
Requires a running zlac_can_bringup.launch.py and exclusive /cmd_vel control.
"""
import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
import shutil
import signal
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cmdvel_trial_core import (
    WHEEL_RADIUS_M, WHEEL_SEPARATION_M, body_speeds, other_publishers,
    validate_trial, wheel_speeds,
    MODE_HARDWARE_LOGS, controller_processes, read_urdf_hardware_config,
    resolve_controller_runtime, hardware_log_for_mode, same_controller_process,
)


def arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish bounded /cmd_vel for N seconds and log wheel/robot feedback")
    parser.add_argument("--linear", type=float, required=True,
                        help="Requested v [m/s], signed")
    parser.add_argument("--angular", type=float, required=True,
                        help="Requested yaw rate [rad/s], signed")
    parser.add_argument("--duration", type=float, required=True,
                        help="Time to publish nonzero /cmd_vel [s]")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Publishing frequency [Hz], default 20")
    parser.add_argument("--post-stop", type=float, default=3.0,
                        help="Continue zero-command publishing and logging [s]")
    parser.add_argument("--max-linear", type=float, default=1.0,
                        help="Allowed magnitude of v [m/s], default 1.0")
    parser.add_argument("--max-angular", type=float, default=1.0,
                        help="Allowed magnitude of yaw rate [rad/s], default 1.0")
    parser.add_argument("--abort-wheel-speed", type=float, default=1.5,
                        help="Application abort speed [m/s], below hardware 2.0 m/s")
    parser.add_argument("--feedback-timeout", type=float, default=0.5,
                        help="Abort after missing /joint_states for this many seconds")
    parser.add_argument("--output-root",
                        default="~/ros2_ws/mimo_cmdvel_trials",
                        help="Parent folder for a new trial result")
    parser.add_argument("--hardware-log", default="auto",
                        help="C++ hardware CSV path; default 'auto' selects "
                             "the file matching the running control mode")
    parser.add_argument("--control-mode", choices=sorted(MODE_HARDWARE_LOGS),
                        default=None,
                        help="Optional operator-declared mode. If a controller "
                             "process declares a different mode, abort preflight.")
    parser.add_argument("--arm", action="store_true",
                        help="Required: acknowledge the robot will move")
    args = parser.parse_args(argv)
    validate_trial(args.linear, args.angular, args.duration, args.rate,
                   args.post_stop, args.max_linear, args.max_angular,
                   args.abort_wheel_speed)
    if not math.isfinite(args.feedback_timeout) or not 0.1 <= args.feedback_timeout <= 3.0:
        parser.error("--feedback-timeout must be within [0.1, 3.0] seconds")
    if not args.arm:
        parser.error("motion is disabled: add --arm after checking E-stop and surroundings")
    return args


def make_directory(root, linear, angular):
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    v = f"{linear:+.3f}".replace("+", "p").replace("-", "m").replace(".", "_")
    w = f"{angular:+.3f}".replace("+", "p").replace("-", "m").replace(".", "_")
    base = Path(root).expanduser().resolve()
    base.mkdir(parents=True, exist_ok=True)
    for number in range(100):
        candidate = base / f"{stamp}_v{v}_w{w}_{number:02d}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("could not create a unique trial directory")


class CmdVelTrial(Node):
    def __init__(self, args, directory):
        super().__init__("mimo_cmdvel_trial_recorder")
        self.args = args
        self.directory = directory
        self.origin_monotonic = time.monotonic()
        self.origin_epoch = time.time()
        self.started = False
        self.has_sent_motion = False
        self.moving_command_started_s = None
        self.last_joint_time = None
        self.last_odom_time = None
        self.last_left_rad_s = float("nan")
        self.last_right_rad_s = float("nan")
        self.error_reason = None
        self.published_count = 0
        self.joint_count = 0
        self.odom_count = 0
        self.invalid_joint_count = 0
        self.metric_n = 0
        self.sse_v = 0.0
        self.sse_w = 0.0
        self.max_abs_yaw_error = 0.0
        self.max_abs_wheel_speed = 0.0
        self.files = []
        self.controller_runtime = None
        self.controller_runtime_after = None
        self.controller_process_unchanged = None
        self.hardware_source = None
        self.hardware_source_selection = "not_selected"
        self.hardware_baseline = None
        self.trial_start_epoch = None

        self.commands_file = self._csv(
            "commands.csv", ["t_s", "epoch_s", "linear_x_m_s", "angular_z_rad_s"])
        self.joints_file = self._csv(
            "joint_states.csv",
            ["t_s", "epoch_s", "header_sec", "header_nanosec",
             "left_pos_rad", "right_pos_rad",
             "left_rad_s", "right_rad_s", "left_actual_current_a",
             "right_actual_current_a", "wheel_v_m_s", "wheel_yaw_rad_s"])
        self.odom_file = self._csv(
            "odom.csv",
            ["t_s", "epoch_s", "header_sec", "header_nanosec",
             "x_m", "y_m", "qx", "qy", "qz", "qw",
             "odom_v_m_s", "odom_yaw_rad_s"])
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint, 50)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odom, 50)

    def _csv(self, filename, columns):
        fh = (self.directory / filename).open(
            "w", encoding="utf-8", newline="", buffering=1)
        writer = csv.writer(fh)
        writer.writerow(columns)
        self.files.append(fh)
        return writer

    def clock(self):
        return time.monotonic() - self.origin_monotonic, time.time()

    @staticmethod
    def value(values, index):
        if 0 <= index < len(values):
            return float(values[index])
        return float("nan")

    def on_joint(self, msg):
        arrival = time.monotonic()
        self.last_joint_time = arrival
        try:
            left = msg.name.index("left_wheel_joint")
            right = msg.name.index("right_wheel_joint")
        except ValueError:
            self.invalid_joint_count += 1
            return
        left_speed = self.value(msg.velocity, left)
        right_speed = self.value(msg.velocity, right)
        if not (math.isfinite(left_speed) and math.isfinite(right_speed)):
            self.invalid_joint_count += 1
            return
        self.last_left_rad_s = left_speed
        self.last_right_rad_s = right_speed
        v, w = body_speeds(left_speed, right_speed)
        if self.started:
            self.max_abs_wheel_speed = max(
                self.max_abs_wheel_speed, abs(left_speed), abs(right_speed))
            self.metric_n += 1
            self.sse_v += (v - self.args.linear) ** 2
            self.sse_w += (w - self.args.angular) ** 2
            self.max_abs_yaw_error = max(
                self.max_abs_yaw_error, abs(w - self.args.angular))
        t, epoch = self.clock()
        self.joints_file.writerow([
            t, epoch, msg.header.stamp.sec, msg.header.stamp.nanosec,
            self.value(msg.position, left), self.value(msg.position, right),
            left_speed, right_speed,
            self.value(msg.effort, left), self.value(msg.effort, right), v, w])
        self.joint_count += 1

    def on_odom(self, msg):
        t, epoch = self.clock()
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.odom_file.writerow([
            t, epoch, msg.header.stamp.sec, msg.header.stamp.nanosec,
            pose.position.x, pose.position.y,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.angular.z])
        self.last_odom_time = time.monotonic()
        self.odom_count += 1

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.command_pub.publish(msg)
        if abs(linear) > 1.0e-9 or abs(angular) > 1.0e-9:
            self.has_sent_motion = True
        t, epoch = self.clock()
        self.commands_file.writerow([t, epoch, linear, angular])
        self.published_count += 1

    def foreign_cmd_publishers(self):
        info = self.get_publishers_info_by_topic("/cmd_vel")
        return other_publishers(
            [(p.node_name, p.node_namespace) for p in info],
            self.get_name(), self.get_namespace())

    def preflight(self):
        """Do not send a motion command until topics are live and exclusive."""
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            foreign = self.foreign_cmd_publishers()
            if foreign:
                raise RuntimeError(
                    "another /cmd_vel publisher is running: "
                    + repr(foreign) + "; stop it before this trial")
            now = time.monotonic()
            if (self.command_pub.get_subscription_count() > 0 and
                self.joint_count >= 3 and self.odom_count >= 3 and
                self.last_joint_time is not None and
                now - self.last_joint_time < self.args.feedback_timeout and
                self.last_odom_time is not None and
                now - self.last_odom_time < 2.0):
                self.capture_control_provenance()
                return
        raise RuntimeError(
            "preflight failed: need /cmd_vel subscriber, live /joint_states "
            "(left_wheel_joint,right_wheel_joint) and /odom")

    def capture_control_provenance(self):
        """Snapshot actual controller process configuration before motion.

        This is configuration evidence, not proof of the compiled hardware
        plugin mode. The distinction is recorded in metadata.json.
        """
        installed_urdf = None
        try:
            share = Path(get_package_share_directory("zlac_can_bringup"))
            installed_urdf = read_urdf_hardware_config(
                share / "urdf" / "orange_go_2025.urdf")
        except (OSError, ValueError, RuntimeError, Exception) as exc:
            self.get_logger().warn(f"Could not read installed URDF: {exc}")

        self.controller_runtime = resolve_controller_runtime(
            controller_processes(), installed_urdf, self.args.control_mode)
        mode = self.controller_runtime["mode"]
        if self.args.hardware_log != "auto":
            self.hardware_source = self.args.hardware_log
            self.hardware_source_selection = "operator_specified_hardware_path"
        else:
            self.hardware_source = hardware_log_for_mode(mode)
            self.hardware_source_selection = (
                "automatic_from_mode" if self.hardware_source
                else "unavailable_or_unknown_mode")

        self.hardware_baseline = hardware_file_snapshot(self.hardware_source)
        self.get_logger().info(
            "Controller mode=%s (source=%s), hardware CSV=%s" %
            (mode or "UNVERIFIED",
             self.controller_runtime["mode_source"],
             self.hardware_source or "not selected"))
        if not self.controller_runtime["plugin_compiled_mode_verified"]:
            self.get_logger().warn(
                "Control mode is inferred from controller process/URDF, "
                "not a confirmed compiled-plugin runtime report; check launch marker.")
        if self.hardware_source is None:
            self.get_logger().warn(
                "No matching hardware CSV was selected. Supply --hardware-log "
                "only if you have verified the running controller.")

    def _guard_motion(self):
        if self.foreign_cmd_publishers():
            raise RuntimeError("another /cmd_vel publisher appeared; aborting trial")
        if self.last_joint_time is None or (
                time.monotonic() - self.last_joint_time > self.args.feedback_timeout):
            raise RuntimeError("/joint_states feedback timeout; aborting trial")
        if (max(abs(self.last_left_rad_s), abs(self.last_right_rad_s))
                * WHEEL_RADIUS_M >= self.args.abort_wheel_speed):
            raise RuntimeError("wheel speed exceeded application abort limit")

    def _phase(self, duration_s, linear, angular, armed):
        end = time.monotonic() + duration_s
        next_publish = time.monotonic()
        last_check = 0.0
        while time.monotonic() < end and rclpy.ok():
            now = time.monotonic()
            if now >= next_publish:
                self.publish(linear, angular)
                next_publish = now + 1.0 / self.args.rate
            rclpy.spin_once(self, timeout_sec=0.005)
            now = time.monotonic()
            if armed and now - last_check >= 0.10:
                self._guard_motion()
                last_check = now
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown during timed trial")

    def run_trial(self):
        self.preflight()
        self.get_logger().info(
            "Preflight OK; /cmd_vel subscriber, /joint_states and /odom live; "
            "no competing /cmd_vel publishers")
        self.trial_start_epoch = time.time()
        self.moving_command_started_s = self.clock()[0]
        self.started = True
        try:
            self._phase(self.args.duration, self.args.linear, self.args.angular, True)
        finally:
            self.started = False
            if self.has_sent_motion:
                self.publish(0.0, 0.0)
        self._phase(self.args.post_stop, 0.0, 0.0, False)

    def send_zero_for(self, seconds):
        # Software stop best-effort only; use the independent physical
        # emergency stop if the motor or ROS process becomes unresponsive.
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            try:
                self.publish(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
            except (Exception, KeyboardInterrupt) as exc:
                self.get_logger().error(f"Could not continue zero commands: {exc}")
                break
            time.sleep(0.03)

    def close_files(self):
        for fh in self.files:
            fh.flush()
            fh.close()

    def summary(self, status):
        return {
            "status": status,
            "error_reason": self.error_reason,
            "time_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "node": self.get_name(),
            "mode_expected": (
                self.controller_runtime.get("mode")
                if self.controller_runtime else None),
            "control_runtime_start": self.controller_runtime,
            "control_runtime_end": self.controller_runtime_after,
            "controller_process_unchanged": self.controller_process_unchanged,
            "controller_not_started_by_script": True,
            "command": {
                "linear_m_s": self.args.linear,
                "angular_rad_s": self.args.angular,
                "duration_s": self.args.duration,
                "publish_rate_hz": self.args.rate,
                "post_stop_s": self.args.post_stop,
                "moving_command_start_t_s": self.moving_command_started_s,
                "wheel_target_left_right_rad_s":
                    wheel_speeds(self.args.linear, self.args.angular),
                "max_linear_m_s": self.args.max_linear,
                "max_angular_rad_s": self.args.max_angular,
                "abort_wheel_peripheral_m_s": self.args.abort_wheel_speed,
            },
            "geometry": {
                "wheel_radius_m": WHEEL_RADIUS_M,
                "wheel_separation_m": WHEEL_SEPARATION_M,
            },
            "source_topics": ["/cmd_vel", "/joint_states", "/odom"],
            "encoder_speed_is_wheel_feedback_not_ground_truth": True,
            "actual_current_source": "/joint_states effort",
            "hardware_source_selection": self.hardware_source_selection,
            "current_command_and_eso_source": self.hardware_source,
            "hardware_trial_start_epoch_s": self.trial_start_epoch,
            "counts": {
                "cmd_vel_commands": self.published_count,
                "joint_state_samples": self.joint_count,
                "odom_samples": self.odom_count,
                "invalid_joint_state_samples": self.invalid_joint_count,
                "tracking_samples": self.metric_n,
            },
            "tracking_metrics_command_phase": {
                "v_rmse_m_s": math.sqrt(self.sse_v / self.metric_n)
                    if self.metric_n else None,
                "yaw_rmse_rad_s": math.sqrt(self.sse_w / self.metric_n)
                    if self.metric_n else None,
                "max_abs_yaw_error_rad_s": self.max_abs_yaw_error
                    if self.metric_n else None,
                "max_abs_wheel_speed_rad_s": self.max_abs_wheel_speed,
            },
            "note": (
                "All ROS CSV timestamps are seconds relative to recorder start. "
                "C++ hardware CSV time_s has a separate origin (controller startup); "
                "do not directly join its time_s to ROS t_s."
            )
        }


def copy_hardware_csv(source, destination, trial_start_epoch, metadata):
    source = Path(source).expanduser()
    if not source.is_file():
        metadata["hardware_csv"] = {
            "copied": False, "reason": f"not found: {source}"}
        return
    try:
        info = source.stat()
        # The active controller continuously writes the CSV. Refuse to use
        # an old CSV from a previous ROS control process as new evidence.
        if info.st_mtime < trial_start_epoch - 2.0:
            metadata["hardware_csv"] = {
                "copied": False, "reason": "hardware log appears stale",
                "source": str(source)}
            return
        target = destination / "hardware_mimo_ladrc_200hz.csv"
        shutil.copy2(source, target)
        metadata["hardware_csv"] = {
            "copied": True, "source": str(source),
            "file": target.name, "bytes": target.stat().st_size}
    except OSError as exc:
        metadata["hardware_csv"] = {
            "copied": False, "reason": str(exc)}


def main(argv=None):
    try:
        args = arguments(argv)
    except ValueError as exc:
        print(f"Invalid trial: {exc}", file=sys.stderr)
        return 2

    directory = make_directory(args.output_root, args.linear, args.angular)
    recorder = None
    result = "failed"
    trial_start_epoch = time.time()
    print(f"[TRIAL] Output: {directory}", flush=True)

    try:
        rclpy.init()

        # Keep the rclpy context alive during Ctrl+C/SIGTERM so the
        # finally block can publish zero commands before shutdown.
        def request_stop(signum, _frame):
            raise KeyboardInterrupt(f"signal {signum}")

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        recorder = CmdVelTrial(args, directory)
        recorder.run_trial()
        result = "completed"
    except (KeyboardInterrupt, SystemExit):
        result = "interrupted"
        if recorder:
            recorder.error_reason = "operator interruption"
        print("[TRIAL] Interrupted: requesting zero velocity", flush=True)
    except Exception as exc:
        result = "aborted"
        if recorder:
            recorder.error_reason = str(exc)
        print(f"[TRIAL] ABORT: {exc}", file=sys.stderr, flush=True)
    finally:
        if recorder is not None:
            # Send zero only if THIS process actually commanded motion.
            # Never fight an existing /cmd_vel owner after preflight refusal.
            if rclpy.ok() and recorder.has_sent_motion:
                recorder.started = False
                # Completed trials already had their requested post-stop
                # phase. Aborted trials get a longer emergency zero hold.
                recorder.send_zero_for(
                    0.3 if result == "completed" else max(1.0, args.post_stop))
            recorder.close_files()
            if recorder.controller_runtime is not None:
                try:
                    recorder.controller_runtime_after = resolve_controller_runtime(
                        controller_processes(),
                        recorder.controller_runtime.get("installed_urdf"))
                    recorder.controller_process_unchanged = same_controller_process(
                        recorder.controller_runtime,
                        recorder.controller_runtime_after)
                except ValueError as exc:
                    recorder.controller_runtime_after = {
                        "error": str(exc), "mode": None}
                    recorder.controller_process_unchanged = False
            metadata = recorder.summary(result)
            recorder.destroy_node()
        else:
            metadata = {
                "status": result, "error": "ROS node was not constructed",
                "command": vars(args)}
        if rclpy.ok():
            rclpy.shutdown()

        if recorder is not None:
            selected_hardware_source = recorder.hardware_source
            pretrial_snapshot = recorder.hardware_baseline
            selected_mode = (recorder.controller_runtime or {}).get("mode")
            process_unchanged = recorder.controller_process_unchanged
            start_epoch = recorder.trial_start_epoch or trial_start_epoch
        else:
            selected_hardware_source = None
            pretrial_snapshot = None
            selected_mode = None
            process_unchanged = None
            start_epoch = trial_start_epoch

        copy_hardware_csv(
            selected_hardware_source, directory, start_epoch, metadata,
            pretrial_snapshot, selected_mode, process_unchanged)
        with (directory / "metadata.json").open("w", encoding="utf-8") as fh:
            json.dump(metadata, fh, indent=2, ensure_ascii=False, allow_nan=False)
        print(f"[TRIAL] {result}: {directory}", flush=True)
    return 0 if result == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
