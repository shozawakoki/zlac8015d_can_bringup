"""Pure, ROS-independent helpers for bounded differential-drive cmd_vel trials."""
import math

WHEEL_RADIUS_M = 0.064
WHEEL_SEPARATION_M = 0.354
MAX_PERIPHERAL_SPEED_M_S = 1.5   # Application abort margin below 2 m/s hardware latch


def wheel_speeds(linear_m_s, angular_rad_s, radius=WHEEL_RADIUS_M,
                 separation=WHEEL_SEPARATION_M):
    """Return signed [left, right] wheel velocity in rad/s."""
    return ((linear_m_s - 0.5 * separation * angular_rad_s) / radius,
            (linear_m_s + 0.5 * separation * angular_rad_s) / radius)


def body_speeds(left_rad_s, right_rad_s, radius=WHEEL_RADIUS_M,
                separation=WHEEL_SEPARATION_M):
    """Return [linear m/s, yaw rad/s] from encoder wheel velocities."""
    return (0.5 * radius * (left_rad_s + right_rad_s),
            radius / separation * (right_rad_s - left_rad_s))


def validate_trial(linear_m_s, angular_rad_s, duration_s, rate_hz,
                   post_stop_s, max_linear_m_s, max_angular_rad_s,
                   abort_wheel_m_s=MAX_PERIPHERAL_SPEED_M_S):
    values = (linear_m_s, angular_rad_s, duration_s, rate_hz, post_stop_s,
              max_linear_m_s, max_angular_rad_s, abort_wheel_m_s)
    if not all(math.isfinite(x) for x in values):
        raise ValueError("all trial numbers must be finite")
    if abs(linear_m_s) < 1e-9 and abs(angular_rad_s) < 1e-9:
        raise ValueError("at least one commanded velocity must be nonzero")
    if not 0.1 <= duration_s <= 60.0:
        raise ValueError("--duration must be between 0.1 and 60 seconds")
    if not 10 <= rate_hz <= 100:
        raise ValueError("--rate must be between 10 and 100 Hz")
    if not 1.0 <= post_stop_s <= 15.0:
        raise ValueError("--post-stop must be between 1 and 15 seconds")
    if not 0 < max_linear_m_s <= 1.0:
        raise ValueError("--max-linear must be within (0, 1.0] m/s")
    if not 0 < max_angular_rad_s <= 1.0:
        raise ValueError("--max-angular must be within (0, 1.0] rad/s")
    if abs(linear_m_s) > max_linear_m_s + 1e-12:
        raise ValueError("linear target exceeds the configured trial cap")
    if abs(angular_rad_s) > max_angular_rad_s + 1e-12:
        raise ValueError("angular target exceeds the configured trial cap")
    if not 0.1 <= abort_wheel_m_s < 2.0:
        raise ValueError("abort wheel-speed guard must remain below 2.0 m/s")
    omega_l, omega_r = wheel_speeds(linear_m_s, angular_rad_s)
    if max(abs(omega_l), abs(omega_r)) * WHEEL_RADIUS_M >= abort_wheel_m_s:
        raise ValueError("requested wheel peripheral speed exceeds abort guard")
    return omega_l, omega_r


def other_publishers(publisher_names, own_name, own_namespace="/"):
    """Return publishers on cmd_vel that are not this recorder node."""
    return [(name, namespace) for name, namespace in publisher_names
            if (name, namespace) != (own_name, own_namespace)]


# Controller provenance is deliberately ROS-independent so it can be
# unit-tested without motors or ROS and never silently guesses a PID/ADRC mode.
import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET


MODE_HARDWARE_LOGS = {
    "mimo_pid_no_breakaway": "/tmp/mimo_pid_no_breakaway_200hz.csv",
    "mimo_ladrc_no_breakaway_actual_current":
        "/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv",
    "mimo_ladrc_no_breakaway": "/tmp/mimo_ladrc_no_breakaway_200hz.csv",
    "mimo_pid_torque": "/tmp/mimo_pid_track_200hz.csv",
    "mimo_ladrc_torque": "/tmp/mimo_ladrc_track_200hz.csv",
    "pid_torque": "/tmp/pid_track_200hz.csv",
    "ladrc_torque": "/tmp/ladrc_track_200hz.csv",
    # Built-in velocity control has no external-current/ESO CSV.
    "velocity": None,
}


def read_urdf_hardware_config(path):
    """Read the installed bringup defaults; NOT proof of active plugin state."""
    p = Path(path)
    data = p.read_bytes()
    root = ET.fromstring(data)
    hardware = root.find(".//ros2_control/hardware")
    if hardware is None:
        raise ValueError("URDF has no ros2_control/hardware section")
    params = {
        elem.attrib["name"]: (elem.text or "").strip()
        for elem in hardware.findall("param")
        if "name" in elem.attrib
    }
    if "control_mode" not in params:
        raise ValueError("URDF does not define control_mode")
    return {
        "path": str(p),
        "sha256": hashlib.sha256(data).hexdigest(),
        "default_control_mode": params["control_mode"],
        "hardware_parameters_as_declared_in_urdf": params,
        "parameters_are_runtime_verified": False,
    }


def controller_processes(proc_root="/proc"):
    """Find local ros2_control_node processes, read ONLY the mode env key.

    The recorder's own environment is not used: ROS launch runs the motor
    controller in a different process. Other environment values are private.
    """
    processes = []
    try:
        entries = list(Path(proc_root).iterdir())
    except OSError:
        return processes
    for entry in entries:
        if not entry.name.isdecimal():
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().split(b"\x00")
            if not cmd or Path(os.fsdecode(cmd[0])).name != "ros2_control_node":
                continue
        except (OSError, ValueError):
            continue
        info = {
            "pid": int(entry.name),
            "executable": "ros2_control_node",
            "environment_readable": False,
            "env_control_mode": None,
            "start_ticks": None,
        }
        try:
            env = (entry / "environ").read_bytes().split(b"\x00")
            info["environment_readable"] = True
            for part in env:
                if part.startswith(b"ZLAC_CONTROL_MODE="):
                    value = os.fsdecode(part.partition(b"=")[2]).strip()
                    info["env_control_mode"] = value or None
                    break
        except (OSError, ValueError):
            pass
        try:
            raw_stat = (entry / "stat").read_text(encoding="utf-8")
            # /proc/<pid>/stat field 2 can contain spaces and parentheses.
            # After the final ')', position 19 is starttime (field 22).
            info["start_ticks"] = int(raw_stat.rsplit(")", 1)[1].split()[19])
        except (OSError, ValueError, IndexError):
            pass
        processes.append(info)
    return sorted(processes, key=lambda item: item["pid"])


def resolve_controller_runtime(processes, urdf_info, declared_mode=None):
    """Return a mode plus explicit evidence quality. Never claim plugin proof."""
    if len(processes) > 1:
        raise ValueError(
            "multiple ros2_control_node processes are running: "
            + repr([p["pid"] for p in processes])
            + ". Stop duplicate controllers before a motion trial."
        )
    process = processes[0] if processes else None
    urdf_default = (urdf_info or {}).get("default_control_mode")
    env_mode = process.get("env_control_mode") if process else None
    env_readable = process.get("environment_readable", False) if process else False

    if env_mode:
        selected = env_mode
        source = "running_ros2_control_node_environment"
    elif process and env_readable and urdf_default:
        selected = urdf_default
        source = "installed_urdf_default_no_process_env_override"
    elif declared_mode:
        selected = declared_mode
        source = "operator_declaration_unverified"
    elif urdf_default:
        selected = urdf_default
        source = "installed_urdf_default_unverified"
    else:
        selected = None
        source = "unknown"

    if declared_mode and env_mode and env_mode != declared_mode:
        raise ValueError(
            "control-mode mismatch: --control-mode=" + declared_mode
            + " but running ros2_control_node has ZLAC_CONTROL_MODE=" + env_mode
        )

    return {
        "mode": selected,
        "mode_source": source,
        "mode_recognized": selected in MODE_HARDWARE_LOGS,
        "plugin_compiled_mode_verified": False,
        "plugin_mode_verification_note": (
            "Process environment / installed URDF is controller configuration "
            "evidence, not proof that the compiled hardware plugin loaded "
            "the requested mode. Verify the plugin startup marker."
        ),
        "operator_declared_mode": declared_mode,
        "controller_process": process,
        "controller_process_count": len(processes),
        "installed_urdf": urdf_info,
    }


def hardware_log_for_mode(mode):
    """Use the exact C++ output path for that mode; unknown -> no log."""
    return MODE_HARDWARE_LOGS.get(mode)


def same_controller_process(before, after):
    """Check PID plus Linux process start ticks to detect controller restart."""
    a = (before or {}).get("controller_process")
    b = (after or {}).get("controller_process")
    if a is None or b is None:
        return None
    if not a.get("start_ticks") or not b.get("start_ticks"):
        return None
    return (a["pid"], a["start_ticks"]) == (b["pid"], b["start_ticks"])
