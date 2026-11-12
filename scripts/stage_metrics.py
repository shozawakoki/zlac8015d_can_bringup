#!/usr/bin/env python3
"""Per-level speed-tracking metrics for stepped /cmd_vel profiles.

Uses recorder-local monotonic receive times for both streams. A step boundary
is the first recorded change in commanded Twist, not a guessed wall-clock time.
Segment metrics include startup transients; 'tail' is a descriptive final 30%
of that hold, NOT a guarantee that the robot reached steady state.
"""
import csv
import math
from pathlib import Path


COLUMNS = [
    "step_index", "start_s", "end_s", "duration_s", "ref_v_m_s",
    "ref_w_rad_s", "ref_left_rad_s", "ref_right_rad_s", "samples",
    "left_rmse_rad_s", "right_rmse_rad_s", "combined_rmse_rad_s",
    "left_tail_mean_error_rad_s", "right_tail_mean_error_rad_s",
    "left_tail_std_rad_s", "right_tail_std_rad_s",
]


def _float(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(key + " nonfinite")
    return value


def compute_stage_metrics(log_dir, radius, separation):
    base = Path(log_dir)
    with (base / "cmd_vel.csv").open(newline="", encoding="utf-8") as file:
        cmd = list(csv.DictReader(file))
    with (base / "joint_states.csv").open(newline="", encoding="utf-8") as file:
        joints = list(csv.DictReader(file))

    segments = []
    for row in cmd:
        try:
            t, v, w = (_float(row, "time_s"),
                       _float(row, "linear_x_m_s"),
                       _float(row, "angular_z_rad_s"))
        except (ValueError, KeyError, TypeError):
            continue
        key = (round(v, 6), round(w, 6))
        if not segments or segments[-1]["key"] != key:
            if segments:
                segments[-1]["end_s"] = t
            segments.append({
                "key": key, "start_s": t, "end_s": None, "v": v, "w": w
            })

    if not segments:
        return []
    # Trailing zero-speed is not used in active tracking metrics. An
    # unfinished final nonzero segment is not assumed to be complete.
    valid = [seg for seg in segments if seg["end_s"] is not None
             and abs(seg["v"]) + abs(seg["w"]) > 1e-6]
    observations = []
    for row in joints:
        try:
            t = _float(row, "time_s")
            vl = _float(row, "left_velocity_rad_s")
            vr = _float(row, "right_velocity_rad_s")
        except (ValueError, KeyError, TypeError):
            continue
        observations.append((t, vl, vr))
    observations.sort()

    rows = []
    for index, seg in enumerate(valid, 1):
        t0, t1 = seg["start_s"], seg["end_s"]
        ref_l = (seg["v"] - separation * seg["w"] / 2.0) / radius
        ref_r = (seg["v"] + separation * seg["w"] / 2.0) / radius
        block = [(t, l - ref_l, r - ref_r) for t, l, r in observations
                 if t0 <= t < t1]
        n = len(block)
        tail = [(dl, dr) for t, dl, dr in block
                if t >= t0 + 0.70 * (t1 - t0)]

        def mean(values):
            return sum(values) / len(values) if values else None

        def std(values):
            if not values:
                return None
            avg = mean(values)
            return math.sqrt(sum((x - avg) ** 2 for x in values) / len(values))

        le = [x[1] for x in block]
        re = [x[2] for x in block]
        lle = [x[0] for x in tail]
        rre = [x[1] for x in tail]
        rows.append({
            "step_index": index,
            "start_s": t0,
            "end_s": t1,
            "duration_s": t1 - t0,
            "ref_v_m_s": seg["v"],
            "ref_w_rad_s": seg["w"],
            "ref_left_rad_s": ref_l,
            "ref_right_rad_s": ref_r,
            "samples": n,
            "left_rmse_rad_s": (
                math.sqrt(sum(x * x for x in le) / n) if n else None
            ),
            "right_rmse_rad_s": (
                math.sqrt(sum(x * x for x in re) / n) if n else None
            ),
            "combined_rmse_rad_s": (
                math.sqrt((sum(x*x for x in le) + sum(x*x for x in re))
                          / (2*n)) if n else None
            ),
            "left_tail_mean_error_rad_s": mean(lle),
            "right_tail_mean_error_rad_s": mean(rre),
            "left_tail_std_rad_s": std(lle),
            "right_tail_std_rad_s": std(rre),
        })
    return rows


def write_stage_metrics(filename, rows):
    with Path(filename).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
