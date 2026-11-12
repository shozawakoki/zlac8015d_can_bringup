#!/usr/bin/env python3
"""Sequential, supervised ROS 2 gain sweeps for ZLAC8015D differential drive.

Dry run is the default. Real motion requires --execute, an exact consent
phrase, and an operator-present emergency-stop setup. The runner launches
zlac_experiment.launch.py for EACH candidate (no hot-switching of motor mode),
checks controllers and wheel-speed feedback, publishes a bounded Twist profile
at 20 Hz, checks feedback freshness/speed/current, sends zero on exit, stops
the launch, and saves per-trial logs and summary.csv.

NOT a replacement for a physical E-stop or a motor-driver communication loss
safety function. Testing should begin with wheels elevated.
"""
import argparse
import csv
import datetime as dt
import json
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback
import uuid

from sweep_core import compute_metrics, load_plan
from safety_rules import check_feedback_limits
from stage_metrics import compute_stage_metrics, write_stage_metrics

CONSENT_TEXT = "I HAVE AN EMERGENCY STOP"
RESULT_FIELDS = [
    "candidate", "mode", "profile", "repeat", "status", "reason",
    "left_rmse_rad_s", "right_rmse_rad_s", "combined_rmse_rad_s",
    "left_current_rms_a", "right_current_rms_a",
    "left_peak_current_a", "right_peak_current_a",
    "active_samples", "cmd_messages", "joint_messages",
    "trial_directory",
]


def now_iso():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def save_json(path, obj):
    dest = Path(path)
    temp = dest.with_name(dest.name + ".tmp")
    with temp.open("w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
        f.write("\n")
    os.replace(temp, dest)


def ros_environment_check():
    if not shutil.which("ros2"):
        raise RuntimeError("ROS 2 executable not found; source the ROS 2 workspace.")
    if not os.environ.get("AMENT_PREFIX_PATH"):
        raise RuntimeError("ROS 2 environment is not sourced.")
    if not os.environ.get("ROS_DISTRO") == "humble":
        raise RuntimeError("This experiment runner is intended for ROS 2 Humble.")


def process_text(args, timeout_s=4.0):
    try:
        result = subprocess.run(
            args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout_s, check=False,
        )
        return result.returncode, result.stdout
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def no_existing_robot_controller():
    # Avoid two competing controller_manager instances and CANopen masters.
    code, out = process_text(["ros2", "node", "list"], timeout_s=4.0)
    if code:
        raise RuntimeError("Cannot inspect ROS 2 nodes: " + out[:250])
    existing = {
        x.strip().split("/")[-1] for x in out.splitlines()
        if x.strip()
    }
    if "controller_manager" in existing or "zlac_experiment_recorder" in existing:
        raise RuntimeError(
            "An existing controller_manager or recorder is running. "
            "Stop other bringup instances before a sweep."
        )


def append_summary(path, row):
    exists = Path(path).exists()
    with Path(path).open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in RESULT_FIELDS})
        stream.flush()
        os.fsync(stream.fileno())


def stop_launch(process, log_handle, timeout_s=8.0):
    """Ask ros2 launch to stop gracefully; kill the process group on timeout."""
    if process is None:
        return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)
    if log_handle:
        log_handle.flush()
        log_handle.close()


def start_launch(trial, plan, trial_dir):
    root = trial_dir / "raw"
    root.mkdir()
    gains = trial["gains"]
    args = [
        "ros2", "launch", "zlac_can_bringup", "zlac_experiment.launch.py",
        "mode:=" + trial["mode"],
        "test_label:=" + trial["profile"],
        "trial:=" + str(trial["repeat"]),
        "surface:=" + plan["surface"],
        "disturbance:=" + plan["disturbance"],
        "notes:=" + trial["name"],
        "output_root:=" + str(root),
        "planned_linear_m_s:=" + str(max(abs(x["linear_x"]) for x in trial["stages"])),
        "planned_angular_rad_s:=" + str(max(abs(x["angular_z"]) for x in trial["stages"])),
        "planned_duration_s:=" + str(sum(x["duration_s"] for x in trial["stages"])),
    ]
    args.extend(k + ":=" + str(v) for k, v in gains.items())
    log_fh = (trial_dir / "launch.log").open("w", encoding="utf-8")
    env = os.environ.copy()
    # Conflicting external overrides cannot silently select another mode.
    env["ZLAC_CONTROL_MODE"] = trial["mode"]
    proc = subprocess.Popen(
        args, env=env, stdin=subprocess.DEVNULL,
        stdout=log_fh, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return proc, log_fh


def safe_log_copy(trial, trial_dir):
    """Copy plugin-internal trace after stopping, before the next trial."""
    path = (
        "/tmp/pid_track_200hz.csv" if trial["mode"] == "pid_torque" else
        "/tmp/ladrc_track_200hz.csv" if trial["mode"] == "ladrc_torque" else None
    )
    if path and Path(path).is_file():
        target = trial_dir / "hardware_current_trace.csv"
        shutil.copy2(path, target)
        return str(target)
    return None


def find_recorded_run(trial_dir):
    root = trial_dir / "raw"
    if not root.exists():
        return None
    dirs = [p for p in root.iterdir() if p.is_dir() and (p / "experiment.json").exists()]
    return dirs[0] if len(dirs) == 1 else None


class MotionSession:
    """A runner-local ROS node for Twist commands and fresh wheel feedback."""

    def __init__(self, plan):
        import rclpy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        self.Twist = Twist
        self.rclpy.init(args=None)
        self.node = rclpy.create_node("zlac_gain_sweep_commander")
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        # Keep latest feedback instead of queueing a 200-Hz backlog.
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 1)
        self.last_feedback = None
        self.latest = None
        self.samples = 0
        self.plan = plan
        self.last_cmd = (0.0, 0.0)
        self.stopping = False
        self.observed_travel_m = 0.0
        self.observed_rotation_rad = 0.0
        self.last_integration_time = None

    def _on_joints(self, msg):
        try:
            li, ri = msg.name.index("left_wheel_joint"), msg.name.index("right_wheel_joint")
            speed_l = float(msg.velocity[li])
            speed_r = float(msg.velocity[ri])
            if not all(__import__("math").isfinite(x) for x in (speed_l, speed_r)):
                return
            # Effort carries measured current [A] on this hardware.
            # Missing effort is invalid, not a synthetic safe zero.
            if len(msg.effort) <= max(li, ri):
                return
            current_l = float(msg.effort[li])
            current_r = float(msg.effort[ri])
            if not all(__import__("math").isfinite(x) for x in (current_l, current_r)):
                return
            # Reject stale messages when the ROS header clock is valid.
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            clock_s = self.node.get_clock().now().nanoseconds * 1e-9
            if stamp_s > 0 and clock_s - stamp_s > 0.5:
                return
            at = time.monotonic()
            if self.last_integration_time is not None:
                delta_t = at - self.last_integration_time
                if 0.0 < delta_t < 0.35:
                    r = self.plan["wheel_radius_m"]
                    b = self.plan["wheel_separation_m"]
                    self.observed_travel_m += (
                        abs(0.5 * r * (speed_l + speed_r)) * delta_t
                    )
                    self.observed_rotation_rad += (
                        abs((speed_r - speed_l) * r / b) * delta_t
                    )
            self.last_integration_time = at
            self.latest = (speed_l, speed_r, current_l, current_r)
            self.last_feedback = at
            self.samples += 1
        except (ValueError, IndexError, TypeError):
            return

    def spin(self, timeout=0.025):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def publish(self, linear=0.0, angular=0.0):
        msg = self.Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)
        self.last_cmd = (float(linear), float(angular))

    def check_feedback(self):
        if self.last_feedback is None:
            raise RuntimeError("No valid /joint_states wheel-velocity feedback")
        vl, vr, il, ir = self.latest
        check_feedback_limits(
            feedback_age_s=time.monotonic() - self.last_feedback,
            left_velocity_rad_s=vl,
            right_velocity_rad_s=vr,
            left_current_a=il,
            right_current_a=ir,
            observed_travel_m=self.observed_travel_m,
            observed_rotation_rad=self.observed_rotation_rad,
            max_wheel_rad_s=self.plan["safety"]["max_wheel_rad_s"],
            max_current_a=self.plan["safety"]["max_current_a"],
        )

    def assert_single_commander(self):
        # Exclude this publisher; no other /cmd_vel publisher is acceptable.
        publishers = self.node.get_publishers_info_by_topic("/cmd_vel")
        if len(publishers) != 1:
            raise RuntimeError(
                "Expected exactly one /cmd_vel publisher (this runner); found %s" %
                len(publishers)
            )

    def zero_for(self, duration_s):
        until = time.monotonic() + duration_s
        while time.monotonic() < until:
            self.publish()
            self.spin(0.03)
            time.sleep(0.02)

    def close(self):
        try:
            self.zero_for(1.0)
        finally:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()


def parse_active_controllers(output):
    """Parse plain or ANSI-colored ros2 control list_controllers output."""
    ansi_escape = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
    clean_output = ansi_escape.sub("", output).replace("\r", "\n")
    active = set()
    for line in clean_output.splitlines():
        words = line.split()
        if len(words) >= 3 and words[0] in (
            "diff_drive_controller", "joint_state_broadcaster"
        ) and "active" in words[2:]:
            active.add(words[0])
    return active


def wait_ready(proc, session, timeout_s):
    """Wait for active controllers AND a fresh wheel sample after each CLI call.

    ros2 control list_controllers can block for seconds on startup. Previously
    checking feedback age immediately after that blocking call incorrectly
    rejected a healthy controller (and could consume the entire ready timeout).
    """
    started = time.monotonic()
    next_query = started
    last_detail = "waiting for controller manager"
    while time.monotonic() - started < timeout_s:
        if proc.poll() is not None:
            raise RuntimeError(
                "Experiment launch exited before controllers became ready; "
                "inspect the per-trial launch.log")
        session.publish()  # only ZERO commands during startup
        session.spin(0.05)

        if time.monotonic() < next_query:
            continue
        code, output = process_text(
            ["ros2", "control", "list_controllers",
             "--controller-manager", "/controller_manager"],
            timeout_s=4.0)
        # Accept both plain and ANSI colorized CLI output.
        active = parse_active_controllers(output)
        controllers_ready = (
            code == 0 and
            "diff_drive_controller" in active and
            "joint_state_broadcaster" in active
        )

        # The CLI is blocking. Process ROS feedback *after* it returns, then
        # require a newly received, valid velocity/current feedback sample.
        fresh_from = session.samples
        feedback_deadline = time.monotonic() + 0.75
        while time.monotonic() < feedback_deadline:
            if proc.poll() is not None:
                raise RuntimeError(
                    "Experiment launch exited during feedback readiness check")
            session.publish()
            session.spin(0.04)
            if (session.samples > fresh_from and
                session.last_feedback is not None and
                time.monotonic() - session.last_feedback < 0.35):
                break

        if controllers_ready and session.latest is not None and (
            session.last_feedback is not None and
            time.monotonic() - session.last_feedback < 0.35
        ):
            session.assert_single_commander()
            session.check_feedback()
            if max(abs(session.latest[0]), abs(session.latest[1])) > 0.2:
                raise RuntimeError("Robot must be stopped before trial")
            print(
                "READY: diff_drive_controller and joint_state_broadcaster active; "
                "fresh wheel velocity/current feedback received",
                flush=True)
            return

        last_detail = (
            "controller CLI exit=%s, active=%s, joint_samples=%d, "
            "last_feedback_age_s=%s, CLI=%s" % (
                code,
                active,
                session.samples,
                ("%.3f" % (time.monotonic() - session.last_feedback)
                 if session.last_feedback is not None else "none"),
                output.strip().replace("\\n", " ")[:300],
            )
        )
        print("WAITING: " + last_detail, flush=True)
        next_query = time.monotonic() + 1.0

    raise RuntimeError(
        "Controllers/feedback not ready in %.1f s: %s. "
        "Inspect launch.log for spawner and hardware errors." %
        (timeout_s, last_detail)
    )


def follow_profile(trial, plan, proc, session):
    sample_period = 1.0 / plan["publish_hz"]
    result = {"stages": []}
    for stage in trial["stages"]:
        linear, angular = stage["linear_x"], stage["angular_z"]
        duration = stage["duration_s"]
        beginning = time.monotonic()
        ticks = 0
        while time.monotonic() - beginning < duration:
            if proc.poll() is not None:
                raise RuntimeError("Controller launch exited during test")
            session.assert_single_commander()
            session.publish(linear, angular)
            session.spin(min(0.02, sample_period / 3))
            session.check_feedback()
            ticks += 1
            next_tick = beginning + ticks * sample_period
            time.sleep(max(0.0, min(next_tick - time.monotonic(), 0.06)))
        result["stages"].append({
            "linear_x": linear, "angular_z": angular,
            "duration_s": duration, "transmissions": ticks,
        })
    session.zero_for(1.5)
    session.check_feedback()
    # Allow settling after the final zero stage and verify physically stopped.
    if max(abs(session.latest[0]), abs(session.latest[1])) > 0.3:
        raise RuntimeError("Wheels did not settle below 0.3 rad/s after stop")
    return result


def run_trial(trial, plan, root, index, total):
    trial_id = "%03d_%s_%s_rep%02d" % (
        index, trial["name"], trial["profile"], trial["repeat"])
    trial_dir = root / trial_id
    trial_dir.mkdir()
    record = {
        "candidate": trial["name"], "mode": trial["mode"],
        "profile": trial["profile"], "repeat": trial["repeat"],
        "status": "starting", "reason": "",
        "started_utc": now_iso(), "parameters": trial["gains"],
        "stages": trial["stages"],
        "wheel_radius_m": trial["wheel_radius_m"],
        "wheel_separation_m": trial["wheel_separation_m"],
        "trial_directory": str(trial_dir),
    }
    save_json(trial_dir / "trial.json", record)
    print("[%d/%d] %s: %s %s" % (
        index, total, trial_id, trial["mode"], trial["gains"]), flush=True)

    # Prevent accidental reuse of a stale CSV from a previous trial.
    trace = (
        "/tmp/pid_track_200hz.csv" if trial["mode"] == "pid_torque" else
        "/tmp/ladrc_track_200hz.csv" if trial["mode"] == "ladrc_torque" else None)
    if trace:
        Path(trace).unlink(missing_ok=True)

    process = log_fh = session = None
    try:
        no_existing_robot_controller()
        process, log_fh = start_launch(trial, plan, trial_dir)
        session = MotionSession(plan)
        wait_ready(process, session, plan["ready_timeout_s"])
        record["started_moving_utc"] = now_iso()
        record["execution"] = follow_profile(trial, plan, process, session)
        record["status"] = "completed"
    except BaseException as exc:
        record["status"] = "aborted"
        record["reason"] = str(exc)
    finally:
        if session is not None:
            try:
                session.close()
            except Exception as exc:
                if record["status"] == "completed":
                    record["status"] = "aborted"
                    record["reason"] = "Failed to publish stop command: " + str(exc)
        stop_launch(process, log_fh)
        record["ended_utc"] = now_iso()
        record["internal_csv"] = safe_log_copy(trial, trial_dir)
        recorded = find_recorded_run(trial_dir)
        if recorded is not None:
            record["record_directory"] = str(recorded)
            try:
                record["metrics"] = compute_metrics(
                    recorded, trial["wheel_radius_m"], trial["wheel_separation_m"])
                # Segment by the ACTUAL recorded cmd_vel transitions, rather
                # than assumed stage durations or ROS launch startup time.
                stage_rows = compute_stage_metrics(
                    recorded, trial["wheel_radius_m"], trial["wheel_separation_m"])
                stage_file = trial_dir / "stage_metrics.csv"
                write_stage_metrics(stage_file, stage_rows)
                record["stage_metrics_csv"] = str(stage_file)
                record["recorded_active_stages"] = len(stage_rows)
                expected_stages = sum(
                    1 for stage in trial["stages"]
                    if abs(stage["linear_x"]) + abs(stage["angular_z"]) > 1e-6
                )
                if (record["status"] == "completed" and
                    (len(stage_rows) != expected_stages or
                     any(stage["samples"] == 0 for stage in stage_rows))):
                    record["status"] = "aborted"
                    record["reason"] = (
                        "Recorded speed stages do not match the planned profile"
                    )
                if (record["metrics"]["active_samples"] == 0 or
                    record["metrics"]["cmd_messages"] == 0):
                    if record["status"] == "completed":
                        record["status"] = "aborted"
                        record["reason"] = (
                            "No usable active speed-response measurements"
                        )
                    else:
                        record["diagnostic"] = (
                            "No active speed-response samples; "
                            "original execution error is preserved"
                        )
            except Exception as exc:
                if record["status"] == "completed":
                    record["status"] = "aborted"
                    record["reason"] = "Metrics failed: " + str(exc)
                else:
                    record["metric_error"] = str(exc)
        else:
            if record["status"] == "completed":
                record["status"] = "aborted"
                record["reason"] = (
                    "Experiment recorder did not produce a unique dataset"
                )
            else:
                record["recording_error"] = (
                    "Experiment recorder did not produce a unique dataset"
                )
        save_json(trial_dir / "trial.json", record)
    return record


def main():
    parser = argparse.ArgumentParser(
        description="Review a safe sweep YAML, or run supervised sequential tests.")
    parser.add_argument("--config", required=True, help="YAML sweep plan")
    parser.add_argument("--execute", action="store_true",
                        help="Enable actual motor motion (requires typed consent)")
    parser.add_argument("--yes", action="store_true",
                        help="Noninteractive consent requires --i-have-e-stop")
    parser.add_argument("--i-have-e-stop", action="store_true",
                        help="Operator confirms verified physical E-stop")
    opts = parser.parse_args()
    try:
        plan = load_plan(opts.config)
    except Exception as exc:
        parser.error(str(exc))
    print("VALIDATED PLAN: %d trials" % len(plan["trials"]))
    for index, trial in enumerate(plan["trials"], 1):
        print("%3d  %-18s %-13s %-12s rep=%02d  duration=%.1f  gains=%s" % (
            index, trial["name"], trial["mode"], trial["profile"],
            trial["repeat"], sum(x["duration_s"] for x in trial["stages"]),
            {key: value for key, value in trial["gains"].items()
             if key.startswith("pid_") or key in ("ladrc_wc", "ladrc_wo")},
        ))
    print("Limits:", plan["safety"])
    if not opts.execute:
        print("DRY RUN ONLY. No CAN access, motors, launch or ROS topics changed.")
        print("Use --execute for supervised testing after safety verification.")
        return 0
    if not opts.i_have_e_stop:
        parser.error("--execute requires --i-have-e-stop")
    if not (opts.yes and opts.i_have_e_stop):
        if not sys.stdin.isatty():
            parser.error("Interactive TTY required; or pass --yes --i-have-e-stop")
        print("WARNING: This will command real robot movement automatically.")
        answer = input("Type exactly '%s' to proceed: " % CONSENT_TEXT)
        if answer != CONSENT_TEXT:
            print("Cancelled; no motor commands sent.")
            return 2

    ros_environment_check()
    no_existing_robot_controller()
    root = Path(os.path.expanduser(os.path.expandvars(plan["output_root"])))
    root.mkdir(parents=True, exist_ok=True)
    campaign = root / ("sweep_" + dt.datetime.now(dt.timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8])
    campaign.mkdir(exist_ok=False)
    save_json(campaign / "plan.json", {
        "created_utc": now_iso(), "plan": plan,
        "source_config": str(Path(opts.config).resolve()),
        "warning": "This software does not replace the hardware E-stop.",
    })
    summary = campaign / "summary.csv"
    print("Recording campaign in", campaign, flush=True)
    total = len(plan["trials"])
    any_failed = False
    try:
        for index, trial in enumerate(plan["trials"], 1):
            result = run_trial(trial, plan, campaign, index, total)
            row = {
                "candidate": trial["name"], "mode": trial["mode"],
                "profile": trial["profile"], "repeat": trial["repeat"],
                "status": result["status"], "reason": result["reason"],
                "trial_directory": result["trial_directory"],
            }
            row.update(result.get("metrics", {}))
            append_summary(summary, row)
            print("RESULT:", result["status"], result.get("metrics"),
                  result["reason"], flush=True)
            if result["status"] != "completed":
                any_failed = True
                print("SAFETY ABORT: remaining tests skipped.", flush=True)
                break
            if index < total:
                # Pauses are always zero-command/no-active-controller: every
                # trial stopped its launch and starts a fresh configuration.
                time.sleep(plan["between_trials_s"])
    except KeyboardInterrupt:
        any_failed = True
        print("Keyboard interrupt: aborting remaining sweep.", flush=True)
    print("Summary:", summary, flush=True)
    return 1 if any_failed else 0


if __name__ == "__main__":
    sys.exit(main())
