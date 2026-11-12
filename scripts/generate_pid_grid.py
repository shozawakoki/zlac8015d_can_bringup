#!/usr/bin/env python3
"""Generate staged, bounded ground-contact PID plans. NO ROS / CAN access.

Each YAML is independently dry-run validated by sweep_core.load_plan.
Generation never executes tests or sends motion commands. Use the named
stages in order, reviewing logs between stages.
"""
import argparse
import itertools
from pathlib import Path
import sys

import yaml
from sweep_core import load_plan

BASE = Path(__file__).resolve().parents[1] / "config" / "pid_calibration_ground.yaml"


def profile(name, speed):
    return {
        "name": name,
        "stages": [
            {"duration_s": 1.0, "linear_x": 0.0, "angular_z": 0.0},
            {"duration_s": 2.0, "linear_x": float(speed), "angular_z": 0.0},
            {"duration_s": 3.0, "linear_x": 0.0, "angular_z": 0.0},
        ],
    }


def candidate(name, kp, ki, kd, tau=0.05, profiles=None):
    return {
        "name": name, "mode": "pid_torque",
        "gains": {
            "pid_kp": float(kp),
            "pid_ki": float(ki),
            "pid_kd": float(kd),
            "pid_derivative_filter_tau_s": float(tau),
            "pid_bumpless_handoff": 0,
            "pid_three_stage_transition": 0,
        },
        "profiles": list(profiles or ["v005"]),
    }


def generate(output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    template = yaml.safe_load(BASE.read_text(encoding="utf-8"))
    template["surface"] = "ground_level"
    template["output_root"] = "~/ros2_experiments/pid_calibration_ground"
    template["repeats"] = 1
    template["profiles"] = [profile("v005", 0.05)]

    stages = {}
    # Stage A: 12 PI (Kd=0) candidates. Recheck original baseline first.
    stage_a = [candidate("P040_I020_D000", 0.4, 0.2, 0)]
    for kp, ki in itertools.product((0.2, 0.4, 0.6, 0.8), (0.1, 0.2, 0.4)):
        name = f"P{round(kp*100):03d}_I{round(ki*100):03d}_D000"
        if name != "P040_I020_D000":
            stage_a.append(candidate(name, kp, ki, 0))
    stages["A_PI_grid"] = (stage_a, 1)

    # Stage B: vary derivative with a *fixed* baseline P/I. After stage A
    # results, update Kp/Ki of these candidates to the selected reference.
    stage_b = [candidate("P040_I020_D000", 0.4, 0.2, 0)]
    for kd, tau in itertools.product((0.005, 0.01, 0.02, 0.04),
                                      (0.03, 0.05, 0.10)):
        name = f"D{round(kd*1000):03d}_T{round(tau*1000):03d}"
        stage_b.append(candidate(name, 0.4, 0.2, kd, tau))
    stages["B_PID_grid"] = (stage_b, 1)

    # Stage C: repeatability and speed variation. Starts from the original
    # conservative gains, not a pretend 'winning' gain: edit only *after*
    # reviewing A/B results and preflight. Each candidate has 3 speeds x3.
    stage_c = [candidate("reference_PI", 0.4, 0.2, 0,
                         profiles=["v004", "v005", "v006"]),
               candidate("reference_PID", 0.4, 0.2, 0.01,
                         profiles=["v004", "v005", "v006"])]
    stages["C_validation"] = (stage_c, 3)

    created = []
    for stage_name, (candidates, repeats) in stages.items():
        plan = dict(template)
        plan["candidates"] = candidates
        plan["repeats"] = repeats
        if stage_name == "C_validation":
            plan["profiles"] = [
                profile("v004", 0.04), profile("v005", 0.05),
                profile("v006", 0.06),
            ]
        dest = output_dir / (stage_name + ".yaml")
        dest.write_text(yaml.safe_dump(
            plan, allow_unicode=True, sort_keys=False), encoding="utf-8")
        checked = load_plan(dest)
        created.append((dest, len(checked["trials"])))
    return created


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", default="/tmp/pid_ground_grid",
        help="Output folder for validated DRY-RUN YAMLs.")
    args = parser.parse_args()
    for path, count in generate(Path(args.output_dir).expanduser()):
        print(f"{path}: {count} dry-run trials")
    print("Generated plans only: NO motion, ROS launch or CAN access.")
    print("Stages A/B/C must NOT be executed without per-stage result review.")


if __name__ == "__main__":
    sys.exit(main())
