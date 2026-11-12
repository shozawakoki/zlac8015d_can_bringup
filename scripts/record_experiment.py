#!/usr/bin/env python3
"""Passive ROS 2 CSV recorder for repeatable differential-drive control tests.

Records actual ROS topic messages without sending /cmd_vel or changing gains.
Each invocation uses a unique output directory and stores experiment.json.
"""
import csv
import datetime as dt
import json
import os
from pathlib import Path
import re
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


PARAMETER_DEFAULTS = {
    "mode": "ladrc_torque",
    "test_label": "speed_tracking",
    "planned_linear_m_s": 0.2,
    "planned_angular_rad_s": 0.0,
    "planned_duration_s": 10.0,
    "trial": "01",
    "surface": "unspecified",
    "disturbance": "none",
    "notes": "",
    "output_root": "~/ros2_experiments",
    "pid_kp": 0.7,
    "pid_ki": 0.5,
    "pid_kd": 0.03,
    "pid_derivative_filter_tau_s": 0.05,
    "ladrc_b0_left": 18.0,
    "ladrc_b0_right": 18.0,
    "ladrc_wc": 5.0,
    "ladrc_wo": 20.0,
    "current_limit_a": 8.0,
    "current_rate_limit_a_per_s": 20.0,
    "feedback_timeout_ms": 30.0,
    "wheel_radius_m": 0.064,
    "wheel_separation_m": 0.354,
    "cmd_timeout_s": 0.5,
}


def safe_name(value):
    """Restrict user-controlled labels to a simple directory-name component."""
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())[:48]
    return name.strip("._-") or "unnamed"


def stamp(msg):
    """ROS header timestamp in seconds, or blank for messages without a header."""
    header = getattr(msg, "header", None)
    if header is None:
        return ""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def joint_value(msg, wheel, source):
    try:
        ix = msg.name.index(wheel)
        values = getattr(msg, source)
        return float(values[ix]) if len(values) > ix else ""
    except (ValueError, AttributeError, IndexError, TypeError):
        return ""


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__("zlac_experiment_recorder")
        for key, value in PARAMETER_DEFAULTS.items():
            self.declare_parameter(key, value)
        self.config = {
            key: self.get_parameter(key).value
            for key in PARAMETER_DEFAULTS
        }
        now = dt.datetime.now(dt.timezone.utc)
        self.started_utc = now.isoformat()
        self.start_monotonic = time.monotonic()
        self.counts = {"cmd_vel": 0, "joint_states": 0, "odom": 0}
        self._last_flush = self.start_monotonic
        self._closed = False

        output_root = Path(os.path.expandvars(os.path.expanduser(
            str(self.config["output_root"])
        )))
        run_prefix = "{}_{}_{}".format(
            now.strftime("%Y%m%dT%H%M%SZ"),
            safe_name(self.config["mode"]),
            safe_name(self.config["test_label"]),
        )
        run_prefix += "_trial" + safe_name(self.config["trial"])
        # Unique even if multiple runs start in the same second.
        output_root.mkdir(parents=True, exist_ok=True)
        for _ in range(10):
            candidate = output_root / (run_prefix + "_" + uuid.uuid4().hex[:8])
            try:
                candidate.mkdir(exist_ok=False)
                self.run_dir = candidate
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError("Unable to create a unique experiment directory.")

        self.files = {}
        self.writers = {}
        schemas = {
            "cmd_vel": [
                "time_s", "received_utc", "linear_x_m_s", "angular_z_rad_s",
            ],
            "joint_states": [
                "time_s", "received_utc", "header_stamp_s",
                "left_position_rad", "right_position_rad",
                "left_velocity_rad_s", "right_velocity_rad_s",
                "left_effort_A", "right_effort_A",
            ],
            "odom": [
                "time_s", "received_utc", "header_stamp_s",
                "x_m", "y_m", "yaw_sin", "yaw_cos",
                "linear_x_m_s", "angular_z_rad_s",
            ],
        }
        for topic, cols in schemas.items():
            fh = (self.run_dir / (topic + ".csv")).open(
                "w", newline="", encoding="utf-8"
            )
            self.files[topic] = fh
            self.writers[topic] = csv.writer(fh)
            self.writers[topic].writerow(cols)

        self._write_metadata(status="recording")
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 100)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 100)
        self.create_subscription(Odometry, "/odom", self._on_odom, 100)
        self.create_timer(1.0, self._flush)
        self.get_logger().info("Passive experiment recorder: %s" % self.run_dir)
        self.get_logger().info(
            "No motion commands will be published. "
            "Awaiting /cmd_vel, /joint_states and /odom."
        )

    def _time_fields(self):
        t = max(0.0, time.monotonic() - self.start_monotonic)
        utc = dt.datetime.now(dt.timezone.utc).isoformat()
        return [t, utc]

    def _record(self, topic, values):
        if self._closed:
            return
        self.writers[topic].writerow(self._time_fields() + values)
        self.counts[topic] += 1

    def _on_cmd_vel(self, msg):
        self._record("cmd_vel", [msg.linear.x, msg.angular.z])

    def _on_joints(self, msg):
        left, right = "left_wheel_joint", "right_wheel_joint"
        self._record("joint_states", [
            stamp(msg),
            joint_value(msg, left, "position"),
            joint_value(msg, right, "position"),
            joint_value(msg, left, "velocity"),
            joint_value(msg, right, "velocity"),
            joint_value(msg, left, "effort"),
            joint_value(msg, right, "effort"),
        ])

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Keep quaternion as yaw sin/cos components without a dependency on tf.
        yaw_sin = 2.0 * (q.w * q.z + q.x * q.y)
        yaw_cos = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._record("odom", [
            stamp(msg), p.x, p.y, yaw_sin, yaw_cos,
            msg.twist.twist.linear.x, msg.twist.twist.angular.z,
        ])

    def _flush(self):
        if self._closed:
            return
        for fh in self.files.values():
            fh.flush()
        self._last_flush = time.monotonic()

    def _write_metadata(self, status):
        data = {
            "schema_version": 1,
            "started_at_utc": self.started_utc,
            "ended_at_utc": (
                dt.datetime.now(dt.timezone.utc).isoformat()
                if status != "recording" else None
            ),
            "duration_monotonic_s": (
                time.monotonic() - self.start_monotonic
                if status != "recording" else None
            ),
            "status": status,
            "parameters": self.config,
            "topics": {
                "/cmd_vel": "geometry_msgs/msg/Twist",
                "/joint_states": "sensor_msgs/msg/JointState",
                "/odom": "nav_msgs/msg/Odometry",
            },
            "message_counts": self.counts,
            "notes": [
                "The recorder never commands motion; topics may be empty if not published.",
                "CSV time_s is monotonic seconds since recorder start.",
                "joint_states effort is motor current [A] in this hardware implementation.",
                "/odom may be wheel odometry and must not be mistaken for GPS/IMU fusion.",
                "The driver-internal velocity mode has no hardware current-command CSV.",
                "PID/LADRC hardware current CSVs are written separately to /tmp.",
            ],
        }
        target = self.run_dir / "experiment.json"
        temporary = self.run_dir / ".experiment.json.tmp"
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        os.replace(temporary, target)

    def close(self):
        if self._closed:
            return
        self._flush()
        self._write_metadata(status="completed")
        self._closed = True
        for fh in self.files.values():
            fh.close()


def main(args=None):
    rclpy.init(args=args)
    recorder = None
    try:
        recorder = ExperimentRecorder()
        rclpy.spin(recorder)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if recorder is not None:
            recorder.close()
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
