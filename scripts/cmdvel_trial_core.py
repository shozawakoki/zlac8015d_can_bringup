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
