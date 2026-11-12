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
