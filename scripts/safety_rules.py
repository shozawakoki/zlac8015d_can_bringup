"""Offline-testable safety thresholds used by the supervised experiment runner.

This module cannot stop hardware on its own. The caller must abort the trial,
command zero, and tear down the launch on any RuntimeError.
"""
import math


def check_feedback_limits(
    *, feedback_age_s, left_velocity_rad_s, right_velocity_rad_s,
    left_current_a, right_current_a, observed_travel_m,
    observed_rotation_rad, max_wheel_rad_s, max_current_a,
):
    values = (
        feedback_age_s, left_velocity_rad_s, right_velocity_rad_s,
        left_current_a, right_current_a, observed_travel_m,
        observed_rotation_rad, max_wheel_rad_s, max_current_a,
    )
    if not all(math.isfinite(x) for x in values):
        raise RuntimeError("Non-finite wheel feedback or safety limit")
    if max_wheel_rad_s <= 0.0 or max_current_a <= 0.0:
        raise RuntimeError("Invalid feedback safety limits")
    if feedback_age_s < 0.0 or feedback_age_s > 0.35:
        raise RuntimeError("Wheel-speed feedback stale: %.3f s" % feedback_age_s)
    if max(abs(left_velocity_rad_s), abs(right_velocity_rad_s)) > min(
        10.0, max_wheel_rad_s * 1.2
    ):
        raise RuntimeError(
            "Wheel overspeed: left=%.2f right=%.2f rad/s" %
            (left_velocity_rad_s, right_velocity_rad_s)
        )
    if observed_travel_m > 0.65:
        raise RuntimeError("Observed wheel travel exceeded 0.65 m")
    if observed_rotation_rad > 1.00:
        raise RuntimeError("Observed wheel rotation exceeded 1.00 rad")
    if max(abs(left_current_a), abs(right_current_a)) > min(
        8.0, max_current_a * 1.2
    ):
        raise RuntimeError(
            "Current exceeded monitoring threshold: L=%.2f R=%.2f A" %
            (left_current_a, right_current_a)
        )
