#!/usr/bin/env python3
"""Rank completed elevated-wheel PID trials from the existing sweep summary.

Never operates ROS/CAN. Ranking is provisional; it is NOT an automatic
safety approval or PID parameter identification from theoretical data.
"""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def finite_number(row, key):
    try:
        value = float(row[key])
    except (ValueError, TypeError, KeyError) as exc:
        raise ValueError(f"missing or invalid {key} for {row.get('candidate')}") from exc
    if not math.isfinite(value):
        raise ValueError(f"nonfinite {key} for {row.get('candidate')}")
    return value


def analyze(filename):
    with Path(filename).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("summary has no trials")
    bad = [
        (r.get("candidate"), r.get("reason", ""))
        for r in rows if r.get("status") != "completed"
    ]
    if bad:
        print("NO GAIN RECOMMENDATION: one or more trials aborted/incomplete.")
        for name, reason in bad:
            print(f"  {name}: {reason}")
        return False

    groups = defaultdict(list)
    for row in rows:
        n = finite_number(row, "active_samples")
        if n < 50:
            raise ValueError("Too few active samples for " + row["candidate"])
        rmse = finite_number(row, "combined_rmse_rad_s")
        left_peak = finite_number(row, "left_peak_current_a")
        right_peak = finite_number(row, "right_peak_current_a")
        if min(rmse, left_peak, right_peak) < 0:
            raise ValueError("Negative reported metric")
        groups[row["candidate"]].append((rmse, max(left_peak, right_peak)))

    ranked = []
    for name, records in groups.items():
        mean_rmse = sum(r[0] for r in records) / len(records)
        peak_current = max(r[1] for r in records)
        ranked.append((mean_rmse, peak_current, name, len(records)))
    ranked.sort()
    print("candidate,trials,mean_combined_RMSE_rad_s,max_measured_current_A")
    for rmse, peak, name, count in ranked:
        print(f"{name},{count},{rmse:.6f},{peak:.6f}")
    print(f"LOWEST RMSE (provisional): {ranked[0][2]}")
    print("Do NOT adopt until speed overshoot, stopping, motor current and "
          "repeated runs are reviewed.")
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("summary_csv", help="Path to a sweep_*/summary.csv")
    args = parser.parse_args()
    try:
        return 0 if analyze(args.summary_csv) else 2
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
