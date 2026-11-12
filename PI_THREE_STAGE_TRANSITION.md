# PI three-stage transition (experimental)

Hardware branch: `feat/pi-three-stage-transition` in `shozawakoki/zlac8015d_hardware`.

This feature is **disabled by default**. Do not use it on the ground until it has passed offline tests, build checks, and a supervised elevated-wheel safety review. Previous bumpless handoff overshot 0.781 rad/s reference to 1.86 rad/s and caused a safety abort. Do not simply raise the speed limit.

## State machine

1. BREAKAWAY: existing current ramp and sustained rotation gate.
2. TRANSITION: after rotation gate, PI starts with zero integral; an independent assist blends prior BREAKAWAY current into the PI request, diminishing both with time and approach to 80% of desired wheel speed. No assist when requesting braking, normal stopping, or direction reversal.
3. TRACK: normal PI current output with existing rate limiting and floor.

Transition is represented by the new `pi_transition_left` and `pi_transition_right` fields in `hardware_current_trace.csv`. `tracking_left/right` remains 1 in both TRANSITION and TRACK for backward compatibility.

## YAML gains

```yaml
gains:
  pid_kp: 0.4
  pid_ki: 0.2
  pid_kd: 0.0
  pid_bumpless_handoff: 0
  pid_three_stage_transition: 1
  pid_transition_duration_s: 0.10
```

Both handoff options cannot be active together. Duration is limited to 0.02 to 0.30 s. Only enable under a dedicated, short, supervised evaluation plan; no ground testing is authorized by this document.

## Acceptance criteria before any motion

- ROS2 packages compile and the plan dry-run validates.
- Disabled option behaves as before. Enabled and disabled startup logs are distinguishable.
- Offline or simulation tests cover positive/negative direction, speed cutoff, zero-stop, normal-stop, direction reversal, coordinator synchronization, repeated-stall lockout, current amplitude/rate limiting, and missing CAN feedback.
- Speed overshoot remains below the **predefined safety-monitor abort threshold**, and emergency stopping works independently of the software monitor.
- The command and actual current/velocity logs show the intended TRANSITION interval; the PI integral remains independent of BREAKAWAY assist.

The 80% speed cutoff and 0.10 s duration are engineering starting points, **not validated control parameters**.

## Offline verification commands (no CAN / no robot motion)

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_pi_transition.cpp -o /tmp/test_pi_transition
/tmp/test_pi_transition

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select zlac8015d_hardware zlac_can_bringup
source install/setup.bash
```

The standalone C++ test uses the same `compute_pi_transition()` function as the hardware controller. It verifies bounded fade, 80% speed cutoff, timeout, braking, normal-stop suppression, and sign reversal. It is **not** a full integrated test of the ROS2 state machine, CAN feedback, or safety monitor. The package build and launch plan dry-run must additionally pass; separate offline integration tests of stop/lockout/synchronization are still required before any new physical-motion test.

To prepare the existing `pi_handoff_1.yaml` plan for **dry-run only**, set gains `pid_bumpless_handoff: 0`, `pid_three_stage_transition: 1`, `pid_transition_duration_s: 0.10`, and confirm `surface: wheels_elevated`. The dry-run must not be invoked with `--execute`.

## Transition lifecycle regression test

The production hardware controller uses `begin_pi_transition`, `advance_pi_transition`, and `cancel_pi_transition` from `pi_transition_lifecycle.hpp`. An additional ROS-independent C++ test invokes the same helpers with mock wheel state. It checks the state progression, cutoff, timeout, stop/brake cancellation, direction-reversal reset, and coordinated cancellation.

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
  tests/test_pi_transition_lifecycle.cpp -o /tmp/test_pi_transition_lifecycle
/tmp/test_pi_transition_lifecycle
```

This does **not** exercise actual `ros2_control` scheduling or physical stop/lockout hardware. The full control plugin must still compile and integration/safety tests are required before motion.

## Offline test of production experiment feedback monitor

The supervised runner now imports `check_feedback_limits()` from `scripts/safety_rules.py`. The function rejects stale/non-finite feedback, overspeed, overcurrent, and measured-travel/rotation excess. The ROS2 runner still owns zero-command publishing and launch teardown.

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
python3 -m unittest discover -s tests -p 'test_safety_rules.py' -v
python3 -m py_compile scripts/run_gain_sweep.py scripts/safety_rules.py scripts/sweep_core.py
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select zlac8015d_hardware zlac_can_bringup
```

The regression test includes the previous observed 1.86/1.45 rad/s safety-abort velocities. Passing these tests does **not** verify real-time stopping latency, ROS2 controller interactions, CAN link interruption, physical emergency stop, or the complete feedback-to-zero-command chain. No `--execute` is part of this procedure.

## Runner abort-to-stop offline integration tests (no ROS/CAN)

The no-motion tests under `tests/test_runner_abort.py` execute the production
`run_trial()` with mocked launch/session boundaries and inject overspeed or
feedback-timeout exceptions. They verify abort recording and the order
`MotionSession.close()` -> `stop_launch()`, including a failure in close.

`tests/test_motion_session_close.py` separately exercises the *real*
`MotionSession.zero_for()` and `MotionSession.close()` methods with fake
Twist/publisher/node objects. It checks zero Twist values and shutdown ordering.

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m py_compile scripts/run_gain_sweep.py scripts/safety_rules.py scripts/sweep_core.py
```

These are partial integration tests with mocked ROS boundaries, **not** a running
`ros2_control` controller or CAN bus, and do not verify the actual stop-current
trajectory or physical E-stop. Hardware movement is still prohibited until
additional HIL/offline validation and an independent emergency-stop review.
