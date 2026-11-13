# MIMO-LADRC / MIMO-PID: body-velocity current control

**Development branch:** feat/mimo-ladrc-pid-straight-tracking  
**Companion hardware branch:** [zlac8015d_hardware](https://github.com/shozawakoki/zlac8015d_hardware/tree/feat/mimo-ladrc-pid-straight-tracking)

This experimental implementation **preserves** the legacy wheel-independent \`ladrc_torque\` and \`pid_torque\` modes. No old mode is removed. The default URDF remains \`ladrc_torque\` so switching branches does **not** silently engage an untested controller.

## 1. Objective / important limitation

The goal is improved straight-line speed tracking by using the body velocity \`[v, omega]\`, rather than tuning the two wheel-speed channels separately. The **newly introduced tuning freedom** is separate linear-speed and yaw-rate gains.

**Do not equate a 2×2 representation with guaranteed better straightness.** Both body measurements are reconstructed from *the same two encoders*. With identical wheel plant gains and identical v/yaw gains, the transformed MIMO law is equivalent to independent wheel control. Encoder yaw cannot detect physical heading drift due to wheel slip, unequal effective wheel radii, or sensor calibration errors. Add IMU yaw-rate/heading and an outer heading/cross-track controller if those effects dominate.

## 2. Plant and controller matrices

Wheel order is \`[left, right]\`; body order \`[v (m/s), omega (rad/s)]\`; positive yaw is left/CCW.

\`\`\`text
q = [omega_left, omega_right]^T
y = [v, omega]^T = T q

T = [ r/2,      r/2
     -r/track,  r/track ]

q_dot = diag(b0_left, b0_right) * I + f_wheel
y_dot = B * I + d
B = T * diag(b0_left, b0_right)
I = [I_left, I_right]^T
\`\`\`

\`T\` and \`B\` are formed and explicitly inverted (with singular-matrix detection) in \`mimo_control.hpp\`. Inputs to the plant remain **current commands in amperes**. There is no assumption that actual wheel velocities are independently settable actuator inputs.

### MIMO LADRC

\`\`\`text
error_obs = y_measured - z1
z1_dot = z2 + B * I_previous + diag(2 * wo_v, 2 * wo_omega) * error_obs
z2_dot = diag(wo_v^2, wo_omega^2) * error_obs

I_raw = B^-1 * (a_ff
                  + diag(wc_v, wc_omega) * (y_reference - z1)
                  - z2)
\`\`\`

The 4-state observer estimates \`z1=[v_hat, omega_hat]\` and \`z2=[d_v_hat, d_omega_hat]\`. It uses the **previous commanded currents**, not the measured CAN currents, matching the legacy ESO assumption. The motor-driver delay and unmodeled current dynamics are disturbances for this model; changing to MIMO does not remove delay-related errors.

### MIMO PID

\`\`\`text
e = y_reference - y_measured
I_raw = B^-1 * (mean(b0_left,b0_right) *
                 (diag(Kp_v,Kp_omega)*e
                  +diag(Ki_v,Ki_omega)*integral(e)
                  -diag(Kd_v,Kd_omega)*LPF(dy_measured/dt))
                 + a_ff)
\`\`\`

The PID uses derivative-on-measurement to prevent setpoint derivative kick, a first-order derivative filter (\`tau=0.05 s\` baseline) and conservative actuator-coordinate conditional integration for anti-windup. The nominal-b0 conversion lets the original wheel PID gains be used as the initial v/yaw gains; the resulting wheel currents match the independent baseline in the symmetric linear, unsaturated case.

### Which parts remain unchanged

- \`diff_drive_controller\` still accepts the existing \`/cmd_vel\` and supplies wheel angular-speed commands to \`ros2_control\`.
- Wheel-specific BREAKAWAY, TRACK, TRACK_HOLD and stall/lockout logic remain in hardware. MIMO applies when *both* wheels already reached TRACK with nonzero references; otherwise the existing independent controller runs. Both controllers use the same final current clamp and slew limiter.
- Existing independent-mode straight wheel-sync correction is disabled in MIMO modes to avoid double feedback.
- Existing normal-stop wheel deceleration ramp is converted to a body acceleration feedforward.
- Driver/CAN sign convention remains unchanged; left motor current is negated when constructing the RPDO frame.
- MIMO and independent modes are **not** hot-swappable. Stop the robot and restart the node to change modes.

## 3. Initial parameters (from current repositories)

| Parameter | Baseline | Purpose |
| --- | ---: | --- |
| \`ladrc_b0_left/right\` | 18.0 | Wheel current → angular acceleration gains |
| \`mimo_wheel_radius\` | 0.064 m | Effective wheel radius |
| \`mimo_wheel_separation\` | 0.354 m | Effective track width |
| \`mimo_wc_v / mimo_wc_omega\` | 5.0 / 5.0 rad/s | LADRC control bandwidth |
| \`mimo_wo_v / mimo_wo_omega\` | 20.0 / 20.0 rad/s | ESO bandwidth |
| \`mimo_pid_kp_v / mimo_pid_kp_omega\` | 0.7 / 0.7 | PID P gain (wheel-gain seeded) |
| \`mimo_pid_ki_v / mimo_pid_ki_omega\` | 0.5 / 0.5 | PID I gain |
| \`mimo_pid_kd_v / mimo_pid_kd_omega\` | 0.03 / 0.03 | PID D gain |
| \`pid_derivative_filter_tau_s\` | 0.05 s | PID derivative filter |
| \`current_limit_a\` | 8.0 A | Existing URDF limit (not validated for this new controller) |
| \`current_rate_limit_a_per_s\` | 20.0 A/s | Existing TRACK current slew limit |

**These are comparison starting values, not experimentally validated MIMO gains.** Do not automatically increase \`mimo_wc_omega\` or \`mimo_pid_kp_omega\` until the feedback polarity, saturation, and overshoot have been verified. Hardware-specific current limits should be checked against the driver and motor ratings.

## 4. Build and run

Check out the matching branch in **both** source repositories; keep all other ROS packages unchanged:

\`\`\`bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch feat/mimo-ladrc-pid-straight-tracking || git switch --track origin/feat/mimo-ladrc-pid-straight-tracking

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch feat/mimo-ladrc-pid-straight-tracking || git switch --track origin/feat/mimo-ladrc-pid-straight-tracking

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
\`\`\`

MIMO LADRC (a completely stopped robot; wheels raised for initial test):

\`\`\`bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
\`\`\`

MIMO PID (separate stopped/restarted run):

\`\`\`bash
ZLAC_CONTROL_MODE=mimo_pid_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
\`\`\`

Baseline control modes are \`ladrc_torque\` and \`pid_torque\`. To revert to the normal URDF mode, omit \`ZLAC_CONTROL_MODE\` or unset it. Since the environment variable is read on hardware initialization, it must be present in the launch process that starts \`ros2_control_node\`.

For a validation without the ROS environment (using the hardware repository's standalone header test):

\`\`\`bash
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude tests/test_mimo_control.cpp -o /tmp/test_mimo_control
/tmp/test_mimo_control
\`\`\`

## 5. Recorded data and experiments

- MIMO LADRC: \`/tmp/mimo_ladrc_track_200hz.csv\`.
- MIMO PID: \`/tmp/mimo_pid_track_200hz.csv\`.
- Both append \`ref_v_m_s\`, \`ref_omega_rad_s\`, \`v_m_s\`, \`omega_rad_s\`, \`hat_v_m_s\`, \`hat_omega_rad_s\`, \`hat_d_v_m_s2\`, \`hat_d_omega_rad_s2\`, and MIMO PID integral states. The reference fields are derived from *incoming* joint references (not the internal normal-stop ramp). PID \`hat_*\` columns are diagnostics, not observer estimates.

Compare the four modes with the same wheel-angle reference profile, same ramp, current limit and initial conditions. Focus on \`omega RMSE\`, wheel-speed difference, \`v RMSE\`, overshoot and current RMS; separately record IMU yaw-rate/heading and cross-track deviation. Use a same-seed/same-trajectory simulation before driving on the ground.

## 6. Safety / integration limitations

1. Start with raised wheels, emergency stop available, and low-speed commands. Validate that positive desired yaw causes **left current reduction and right current increase** (subject to wheel-speed feedback).
2. The MIMO algorithms are **experimental**: a successful matrix test does not certify the 200-Hz ROS/CAN integration, stable closed-loop behavior with 23-ms transport delay, physical straightness, or safe motor limits.
3. As in the existing implementation, a CAN timeout attempts a zero-current command; a physically interrupted CAN bus does not guarantee that zero reaches the motor driver.
4. Actual yaw-rate here is encoder-derived. A heading or cross-track feedback loop is **not** included. Slip/real-heading correction requires a separate sensing/control change.
