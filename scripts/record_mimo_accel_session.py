#!/usr/bin/env python3
"""Record a MIMO-LADRC acceleration experiment, without generating /cmd_vel.

Starts the exact existing ROS 2 launch with ZLAC_CONTROL_MODE=mimo_ladrc_torque,
plus the existing PASSIVE ROS CSV recorder and optional rosbag2. A separate
teleop/controller supplies /cmd_vel. On Ctrl+C, stops launch, then copies
the hardware plugin's /tmp/mimo_ladrc_track_200hz.csv into the run directory.

ROS topics and the hardware CSV have distinct clocks; do not interpret their
time_s columns as automatically synchronized. Hardware /tmp CSV is generated
by the C++ plugin; this program does not send current/velocity CAN commands.
"""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid


HARDWARE_LOG = Path("/tmp/mimo_ladrc_track_200hz.csv")
ROS_TOPICS = ("/cmd_vel", "/joint_states", "/odom", "/tf", "/tf_static")


def safe_label(value):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip()).strip("_")[:48] or "test"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-root", default="~/ros2_ws/mimo_accel_results")
    p.add_argument("--label", default="sudden_acceleration")
    p.add_argument("--trial", default="01")
    p.add_argument("--no-rosbag", action="store_true",
                   help="skip rosbag2, keep ROS CSV and 200 Hz hardware CSV")
    p.add_argument("--check-only", action="store_true",
                   help="verify dependencies and process status, do not launch")
    return p.parse_args(argv)


def controller_conflicts(ps_text):
    """Find controllers already running before starting this session."""
    found = []
    for line in ps_text.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) < 2 or not fields[0].isdigit():
            continue
        cmd = fields[1]
        if ("ros2_control_node" in cmd or "can_launch_watchdog.py" in cmd or
                ("ros2 launch" in cmd and "zlac_can_bringup" in cmd)):
            # Do not reject a user's text editor or grep command.
            if "grep " not in cmd and "record_mimo_accel_session.py" not in cmd:
                found.append(line.strip())
    return found


def prepare(args):
    if not shutil.which("ros2"):
        raise RuntimeError("ros2 CLI not found; source /opt/ros/humble/setup.bash")
    recorder = Path(__file__).resolve().with_name("record_experiment.py")
    if not recorder.is_file():
        raise RuntimeError(f"Missing existing recorder: {recorder}")
    ps = subprocess.run(["ps", "-eo", "pid=,args="],
                        text=True, capture_output=True, check=True)
    conflicts = controller_conflicts(ps.stdout)
    if conflicts:
        raise RuntimeError(
            "Another controller/launch/watchdog may own CAN. "
            "Stop it first:\n" + "\n".join(conflicts))
    if not args.no_rosbag:
        check = subprocess.run(["ros2", "bag", "--help"], capture_output=True,
                               text=True, timeout=12, check=False)
        if check.returncode != 0:
            raise RuntimeError(
                "ros2 bag is unavailable; install rosbag2 or use --no-rosbag")
    return recorder


def make_run_dir(root, label, trial):
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    for _ in range(10):
        candidate = root / (
            f"{timestamp}_{safe_label(label)}_trial{safe_label(trial)}_"
            f"{uuid.uuid4().hex[:8]}")
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("Unable to create unique run folder")


def spawn(command, logfile, *, env=None, pipe_output=False):
    if pipe_output:
        p = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1,
                             start_new_session=True)
        def relay():
            with open(logfile, "w", encoding="utf-8", buffering=1) as stream:
                for line in p.stdout:
                    stream.write(line)
                    try:
                        sys.stdout.write(line)
                        sys.stdout.flush()
                    except (OSError, BrokenPipeError):
                        pass
        worker = threading.Thread(target=relay, daemon=True)
        worker.start()
        return p, worker
    with open(logfile, "w", encoding="utf-8") as stream:
        p = subprocess.Popen(command, env=env, stdout=stream,
                             stderr=subprocess.STDOUT, start_new_session=True)
    return p, None


def stop_process(name, proc, grace_s=8.0):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, grace_s),
                      (signal.SIGTERM, 3.0),
                      (signal.SIGKILL, 1.0)):
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            print(f"[CAPTURE] {name} did not exit after {sig.name}", flush=True)


def save_hardware_csv(run_dir, *, started_wall_ns):
    """Copy only a CSV modified in this run; never delete or truncate /tmp."""
    if not HARDWARE_LOG.is_file():
        return {"status": "missing",
                "reason": f"{HARDWARE_LOG} was not created"}
    st = HARDWARE_LOG.stat()
    # Allow 1s for filesystem wall-clock rounding, but reject older log files.
    if st.st_mtime_ns + 1_000_000_000 < started_wall_ns:
        return {"status": "stale", "mtime_ns": st.st_mtime_ns,
                "reason": "existing CSV was not updated during this session"}
    dst = run_dir / "mimo_ladrc_track_200hz.csv"
    shutil.copy2(HARDWARE_LOG, dst)
    with dst.open(encoding="utf-8", errors="replace") as stream:
        header = stream.readline().strip().split(",")
    required = (
        "time_s", "ref_left_rad_s", "vel_left_rad_s",
        "cmd_current_left_A", "actual_current_left_A",
        "hat_v_m_s", "hat_omega_rad_s", "hat_d_v_m_s2",
        "hat_d_omega_rad_s2")
    missing = [name for name in required if name not in header]
    return {"status": "copied" if not missing else "header_missing_columns",
            "file": str(dst), "bytes": dst.stat().st_size,
            "missing_columns": missing}


def main(argv=None):
    args = parse_args(argv)
    recorder = prepare(args)
    if args.check_only:
        print("CHECK ONLY: ros2 CLI, passive recorder, and controller "
              "exclusivity checks passed. No processes started.")
        return 0

    run_dir = make_run_dir(args.output_root, args.label, args.trial)
    print(f"[CAPTURE] RESULT_DIR={run_dir}", flush=True)
    print("[CAPTURE] This script starts the MIMO-LADRC launch but DOES NOT "
          "publish /cmd_vel; use your controller in another terminal.", flush=True)

    env = dict(os.environ)
    env["ZLAC_CONTROL_MODE"] = "mimo_ladrc_torque"
    env["ROS_LOG_DIR"] = str(run_dir / "ros_logs")
    (run_dir / "ros_logs").mkdir()

    launch_cmd = ["ros2", "launch", "zlac_can_bringup",
                  "zlac_can_bringup.launch.py"]
    recorder_cmd = [
        sys.executable, str(recorder), "--ros-args",
        "-p", "mode:=mimo_ladrc_torque",
        "-p", "test_label:=" + safe_label(args.label),
        "-p", "trial:=" + safe_label(args.trial),
        "-p", "output_root:=" + str(run_dir / "ros_csv"),
    ]
    bag_cmd = (["ros2", "bag", "record", "-o", str(run_dir / "rosbag2")]
               + list(ROS_TOPICS))
    started = dt.datetime.now(dt.timezone.utc)
    started_wall_ns = time.time_ns()
    metadata = {
        "schema": 1, "start_utc": started.isoformat(),
        "mode": env["ZLAC_CONTROL_MODE"],
        "launch_command": "ZLAC_CONTROL_MODE=mimo_ladrc_torque " +
                          " ".join(launch_cmd),
        "motion_commands_published_by_recorder": False,
        "topics": {"/cmd_vel": "geometry_msgs/msg/Twist",
                   "/joint_states": "sensor_msgs/msg/JointState",
                   "/odom": "nav_msgs/msg/Odometry",
                   "/tf": "tf2_msgs/msg/TFMessage",
                   "/tf_static": "tf2_msgs/msg/TFMessage"},
        "hardware_csv_source": str(HARDWARE_LOG),
        "note": ("ROS CSV time_s is relative to passive recorder start; "
                 "hardware CSV time_s is relative to hardware logger start. "
                 "Do NOT directly subtract without aligning clocks."),
        "status": "recording",
    }
    def write_metadata():
        temporary = run_dir / ".capture_metadata.json.tmp"
        temporary.write_text(json.dumps(metadata, indent=2,
                                        ensure_ascii=False) + "\n",
                             encoding="utf-8")
        os.replace(temporary, run_dir / "capture_metadata.json")

    write_metadata()
    jobs = {}
    launch_worker = None
    outcome = "completed"
    try:
        jobs["recorder"], _ = spawn(
            recorder_cmd, run_dir / "passive_recorder.log", env=env)
        if not args.no_rosbag:
            jobs["rosbag2"], _ = spawn(bag_cmd, run_dir / "rosbag.log", env=env)
        jobs["launch"], launch_worker = spawn(
            launch_cmd, run_dir / "ros_launch.log", env=env, pipe_output=True)
        print("[CAPTURE] Use Ctrl+C HERE after the robot has stopped. "
              "This will shut down launch, finalize rosbag2 and save "
              "the 200 Hz hardware CSV.", flush=True)
        while True:
            time.sleep(0.2)
            if jobs["launch"].poll() is not None:
                outcome = "launch_exited"
                print("[CAPTURE] ROS launch exited unexpectedly or was stopped. "
                      "Finalizing logs.", flush=True)
                break
            if jobs["recorder"].poll() is not None:
                outcome = "recorder_exited"
                print("[CAPTURE] Passive ROS recorder exited; stopping session.",
                      flush=True)
                break
            if "rosbag2" in jobs and jobs["rosbag2"].poll() is not None:
                outcome = "rosbag_exited"
                print("[CAPTURE] rosbag2 exited; stopping session.", flush=True)
                break
    except KeyboardInterrupt:
        print("\n[CAPTURE] Ctrl+C received: stopping ROS launch and saving data.",
              flush=True)
    except Exception:
        outcome = "error"
        raise
    finally:
        # Stop the controller/launch FIRST so the C++ CSV logger flushes before
        # we copy it. Never send velocity/current commands from this script.
        stop_process("launch", jobs.get("launch"))
        if launch_worker is not None:
            launch_worker.join(timeout=2)
        stop_process("rosbag2", jobs.get("rosbag2"))
        stop_process("passive recorder", jobs.get("recorder"))
        try:
            metadata["hardware_csv"] = save_hardware_csv(
                run_dir, started_wall_ns=started_wall_ns)
        except (OSError, UnicodeError) as exc:
            metadata["hardware_csv"] = {
                "status": "copy_error", "reason": str(exc)}
        metadata["stop_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        metadata["status"] = outcome
        metadata["process_exit_codes"] = {
            name: proc.poll() for name, proc in jobs.items()
        }
        write_metadata()
        print(f"[CAPTURE] SAVED={run_dir}", flush=True)
        print(f"[CAPTURE] Hardware CSV: {metadata['hardware_csv']['status']}",
              flush=True)
        print("[CAPTURE] Files: ros_launch.log, passive_recorder.log, "
              "ros_csv/, rosbag2/ (if enabled), capture_metadata.json.",
              flush=True)
    return 0 if outcome in ("completed", "launch_exited") else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"CAPTURE ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
