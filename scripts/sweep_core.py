#!/usr/bin/env python3
"""Pure helpers for validating gain-sweep plans and scoring recorded wheel speeds.

This module never opens CAN, publishes ROS topics, or moves the robot.
"""
import bisect
import csv
import math
from pathlib import Path

import yaml

MODES = ("velocity", "pid_torque", "ladrc_torque")
GAIN_DEFAULTS = {
    "pid_kp": 0.7,
    "pid_ki": 0.5,
    "pid_kd": 0.03,
    "pid_bumpless_handoff": 0,
    "pid_derivative_filter_tau_s": 0.05,
    "ladrc_b0_left": 18.0,
    "ladrc_b0_right": 18.0,
    "ladrc_wc": 5.0,
    "ladrc_wo": 20.0,
    "current_limit_a": 4.0,
    "current_rate_limit_a_per_s": 20.0,
    "feedback_timeout_ms": 30.0,
    "breakaway_start_a": 0.65,
    "breakaway_step_a": 0.10,
    "breakaway_max_a": 2.0,
    "breakaway_step_samples": 20,
    "breakaway_start_speed_rad_s": 0.30,
    "breakaway_start_samples": 6,
    "stall_speed_rad_s": 0.10,
    "stall_samples": 80,
    "stall_min_drive_current_a": 0.20,
}
# Hard upper bounds for a development test runner; configuration cannot raise
# them. They are not a replacement for hardware current/velocity protections.
HARD_MAX_LINEAR_M_S = 0.20
HARD_MAX_ANGULAR_RAD_S = 0.40
HARD_MAX_CURRENT_A = 8.0
HARD_MAX_WHEEL_RAD_S = 10.0
HARD_MAX_TRIAL_SECONDS = 45.0
HARD_MAX_TRAVEL_M = 0.50
HARD_MAX_YAW_RAD = 0.80
HARD_MAX_TRIALS = 120


def number(x, label):
    if isinstance(x, bool):
        raise ValueError("%s must be numeric" % label)
    try:
        value = float(x)
    except (ValueError, TypeError) as exc:
        raise ValueError("%s must be numeric" % label) from exc
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % label)
    return value


def load_plan(filename):
    with Path(filename).open(encoding="utf-8") as stream:
        doc = yaml.safe_load(stream)
    if not isinstance(doc, dict):
        raise ValueError("Sweep YAML must contain an object")
    if doc.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")

    safety = doc.get("safety")
    if not isinstance(safety, dict):
        raise ValueError("safety mapping is required")
    for key in ("max_linear_m_s", "max_angular_rad_s",
                "max_wheel_rad_s", "max_current_a"):
        if key not in safety:
            raise ValueError("safety.%s required" % key)
        safety[key] = number(safety[key], "safety." + key)
        if safety[key] <= 0:
            raise ValueError("safety.%s must be positive" % key)
    if safety["max_linear_m_s"] > HARD_MAX_LINEAR_M_S or (
        safety["max_angular_rad_s"] > HARD_MAX_ANGULAR_RAD_S
    ) or safety["max_current_a"] > HARD_MAX_CURRENT_A or (
        safety["max_wheel_rad_s"] > HARD_MAX_WHEEL_RAD_S
    ):
        raise ValueError("Safety limits exceed development hard bounds")

    defaults = dict(GAIN_DEFAULTS)
    provided = doc.get("defaults", {})
    if not isinstance(provided, dict):
        raise ValueError("defaults must be a mapping")
    for key, val in provided.items():
        if key not in GAIN_DEFAULTS:
            raise ValueError("Unknown default parameter: " + str(key))
        defaults[key] = number(val, key)
    validate_gains(defaults, safety)

    profiles = doc.get("profiles")
    if not isinstance(profiles, list) or not profiles:
        raise ValueError("profiles must be a nonempty list")
    profile_map = {}
    for p in profiles:
        if not isinstance(p, dict):
            raise ValueError("Each profile must be an object")
        name = identifier(p.get("name"), "profile name")
        if name in profile_map:
            raise ValueError("Repeated profile: " + name)
        stages = p.get("stages")
        if not isinstance(stages, list) or len(stages) < 3:
            raise ValueError("Profile %s needs at least 3 stages" % name)
        output = []
        total = 0.0
        travel = 0.0
        rotation = 0.0
        for stage in stages:
            if not isinstance(stage, dict):
                raise ValueError("Each stage must be an object")
            duration = number(stage.get("duration_s"), "duration_s")
            v = number(stage.get("linear_x"), "linear_x")
            w = number(stage.get("angular_z"), "angular_z")
            if not (0.25 <= duration <= 15):
                raise ValueError("Stage duration must be 0.25..15 s")
            if (abs(v) > safety["max_linear_m_s"] or
                    abs(w) > safety["max_angular_rad_s"]):
                raise ValueError("Velocity reference exceeds safety limits")
            output.append({"duration_s": duration, "linear_x": v,
                           "angular_z": w})
            total += duration
            travel += abs(v) * duration
            rotation += abs(w) * duration
        if travel > HARD_MAX_TRAVEL_M or rotation > HARD_MAX_YAW_RAD:
            raise ValueError(
                "Profile %s exceeds path-length/turn bounds (%.2f m, %.2f rad)" %
                (name, travel, rotation))
        if total > HARD_MAX_TRIAL_SECONDS:
            raise ValueError("Total profile duration exceeds 45 seconds")
        for stage in (output[0], output[-1]):
            if stage["linear_x"] != 0 or stage["angular_z"] != 0:
                raise ValueError("Profiles must start and end at zero speed")
        if not any(x["linear_x"] or x["angular_z"] for x in output):
            raise ValueError("Profile has no nonzero command: " + name)
        profile_map[name] = output

    candidates = doc.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("candidates must be a nonempty list")
    trials = []
    names = set()
    repeats = doc.get("repeats", 1)
    if isinstance(repeats, bool) or not isinstance(repeats, int) or not 1 <= repeats <= 10:
        raise ValueError("repeats must be an integer from 1 through 10")
    # Geometry is a calibration property, not another tuning variable.
    radius = number(doc.get("wheel_radius_m", 0.064), "wheel_radius_m")
    separation = number(doc.get("wheel_separation_m", 0.354),
                        "wheel_separation_m")
    if not 0.01 <= radius <= 0.3 or not 0.1 <= separation <= 1.5:
        raise ValueError("Invalid wheel geometry")

    for cand in candidates:
        if not isinstance(cand, dict):
            raise ValueError("Each candidate must be an object")
        name = identifier(cand.get("name"), "candidate name")
        if name in names:
            raise ValueError("Repeated candidate name: " + name)
        names.add(name)
        mode = cand.get("mode")
        if mode not in MODES:
            raise ValueError("Unknown mode for %s: %s" % (name, mode))
        overrides = cand.get("gains", {})
        if not isinstance(overrides, dict):
            raise ValueError("candidate gains must be a mapping")
        gains = dict(defaults)
        for k, v in overrides.items():
            if k not in gains:
                raise ValueError("Unknown gain: " + str(k))
            gains[k] = number(v, k)
        validate_gains(gains, safety)
        use_profiles = cand.get("profiles", list(profile_map))
        if not isinstance(use_profiles, list) or not use_profiles:
            raise ValueError("candidate profiles must be a nonempty list")
        for pname in use_profiles:
            if pname not in profile_map:
                raise ValueError("Unknown profile: " + str(pname))
            stages = profile_map[pname]
            for stage in stages:
                wr = (stage["linear_x"] + separation * stage["angular_z"] / 2) / radius
                wl = (stage["linear_x"] - separation * stage["angular_z"] / 2) / radius
                if max(abs(wr), abs(wl)) > safety["max_wheel_rad_s"]:
                    raise ValueError("Wheel reference exceeds limit for " + pname)
            for rep in range(1, repeats + 1):
                trials.append({
                    "name": name, "mode": mode, "gains": gains.copy(),
                    "profile": pname, "stages": stages,
                    "repeat": rep, "wheel_radius_m": radius,
                    "wheel_separation_m": separation,
                })
    if len(trials) > HARD_MAX_TRIALS:
        raise ValueError("Too many trials; max %d" % HARD_MAX_TRIALS)

    hz = number(doc.get("publish_hz", 20), "publish_hz")
    if not 10 <= hz <= 50:
        raise ValueError("publish_hz must be between 10 and 50 Hz")
    ready_timeout = number(doc.get("ready_timeout_s", 25), "ready_timeout_s")
    if not 5 <= ready_timeout <= 60:
        raise ValueError("ready_timeout_s must be 5..60")
    reset_pause = number(doc.get("between_trials_s", 4), "between_trials_s")
    if not 2 <= reset_pause <= 30:
        raise ValueError("between_trials_s must be 2..30")
    return {
        "trials": trials, "safety": safety, "publish_hz": hz,
        "ready_timeout_s": ready_timeout, "between_trials_s": reset_pause,
        "wheel_radius_m": radius, "wheel_separation_m": separation,
        "surface": str(doc.get("surface", "unspecified")),
        "disturbance": str(doc.get("disturbance", "none")),
        "output_root": str(doc.get("output_root", "~/ros2_experiments")),
    }


def validate_gains(g, safety):
    if g["pid_bumpless_handoff"] not in (0, 1):
        raise ValueError("pid_bumpless_handoff must be 0 or 1")
    for name, val in g.items():
        if not math.isfinite(val):
            raise ValueError(name + " must be finite")
        if val < 0 or (name not in ("pid_kp", "pid_ki", "pid_kd", "pid_bumpless_handoff") and val <= 0):
            raise ValueError(name + " is outside permitted range")
    if sum(g[k] for k in ("pid_kp", "pid_ki", "pid_kd")) <= 0:
        # Disabled controller gains are not a useful sweep candidate.
        raise ValueError("At least one PID gain must be nonzero")
    if g["current_limit_a"] > safety["max_current_a"]:
        raise ValueError("current_limit_a exceeds sweep safety.max_current_a")
    if g["breakaway_start_a"] > g["breakaway_max_a"] or (
        g["breakaway_max_a"] > g["current_limit_a"]
    ):
        raise ValueError(
            "Require breakaway_start_a <= breakaway_max_a <= current_limit_a"
        )
    for k in ("breakaway_step_samples", "breakaway_start_samples", "stall_samples"):
        if int(g[k]) != g[k]:
            raise ValueError(k + " must be an integer")
    if not 1 <= g["breakaway_step_samples"] <= 2000 or (
        not 1 <= g["breakaway_start_samples"] <= 500 or
        not 1 <= g["stall_samples"] <= 5000
    ):
        raise ValueError("Invalid startup/recovery sample count")
    if not 0 < g["stall_min_drive_current_a"] <= g["current_limit_a"]:
        raise ValueError("Invalid stall_min_drive_current_a")
    if not 0 < g["breakaway_start_speed_rad_s"] <= 10 or (
        not 0 < g["stall_speed_rad_s"] < g["breakaway_start_speed_rad_s"]
    ):
        raise ValueError("Invalid startup/stall wheel-speed thresholds")
    if not 0 < g["breakaway_step_a"] <= g["breakaway_max_a"]:
        raise ValueError("Invalid breakaway_step_a")
    if not 0 < g["feedback_timeout_ms"] <= 200:
        raise ValueError("feedback_timeout_ms out of range")
    if not 0 < g["current_rate_limit_a_per_s"] <= 100:
        raise ValueError("current slew limit out of range")
    if not 0 < g["pid_derivative_filter_tau_s"] <= 1:
        raise ValueError("PID derivative filter out of range")
    if g["ladrc_wo"] > 100 or g["ladrc_wc"] > 40:
        raise ValueError("LADRC bandwidth too high for initial sweep")


def identifier(x, label):
    import re
    if not isinstance(x, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", x):
        raise ValueError("%s must use only letters, digits, _ or -" % label)
    return x


def compute_metrics(log_directory, radius, separation):
    """Compute wheel-speed RMSE for segments with nonzero command.

    All CSV time_s fields are measured by the same passive recorder; use
    latest-known Twist command instead of pairing independent ROS callbacks.
    Invalid/missing feedback yields null metrics (never a fictitious zero).
    """
    base = Path(log_directory)
    with (base / "cmd_vel.csv").open(newline="", encoding="utf-8") as stream:
        commands = list(csv.DictReader(stream))
    with (base / "joint_states.csv").open(newline="", encoding="utf-8") as stream:
        feedback = list(csv.DictReader(stream))
    command_records = []
    for row in commands:
        try:
            t = float(row["time_s"])
            v = float(row["linear_x_m_s"])
            w = float(row["angular_z_rad_s"])
        except (TypeError, ValueError, KeyError):
            continue
        if all(map(math.isfinite, (t, v, w))):
            command_records.append((t, v, w))
    command_records.sort()
    ts = [x[0] for x in command_records]
    sum_sq = [0.0, 0.0]
    sum_current_sq = [0.0, 0.0]
    peak_current = [0.0, 0.0]
    n = 0
    for row in feedback:
        try:
            t = float(row["time_s"])
            left = float(row["left_velocity_rad_s"])
            right = float(row["right_velocity_rad_s"])
            i_l = float(row["left_effort_A"])
            i_r = float(row["right_effort_A"])
        except (KeyError, ValueError, TypeError):
            continue
        if not all(map(math.isfinite, (t, left, right, i_l, i_r))):
            continue
        pos = bisect.bisect_right(ts, t) - 1
        if pos < 0:
            continue
        _, v, w = command_records[pos]
        if abs(v) + abs(w) <= 1e-6:
            continue
        ref_l = (v - separation * w / 2.0) / radius
        ref_r = (v + separation * w / 2.0) / radius
        n += 1
        sum_sq[0] += (left - ref_l) ** 2
        sum_sq[1] += (right - ref_r) ** 2
        sum_current_sq[0] += i_l * i_l
        sum_current_sq[1] += i_r * i_r
        peak_current[0] = max(peak_current[0], abs(i_l))
        peak_current[1] = max(peak_current[1], abs(i_r))
    return {
        "active_samples": n,
        "left_rmse_rad_s": math.sqrt(sum_sq[0] / n) if n else None,
        "right_rmse_rad_s": math.sqrt(sum_sq[1] / n) if n else None,
        "combined_rmse_rad_s": (
            math.sqrt((sum_sq[0] + sum_sq[1]) / (2 * n)) if n else None
        ),
        "left_current_rms_a": math.sqrt(sum_current_sq[0] / n) if n else None,
        "right_current_rms_a": math.sqrt(sum_current_sq[1] / n) if n else None,
        "left_peak_current_a": peak_current[0] if n else None,
        "right_peak_current_a": peak_current[1] if n else None,
        "cmd_messages": len(commands),
        "joint_messages": len(feedback),
    }
