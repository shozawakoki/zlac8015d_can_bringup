# PID gain tuning with cruise-speed reference steps

This branch supersedes the earlier startup-focused, low-speed PID sweep plan.
It was branched from `feat/pid-gain-calibration`; existing ROS 2 controller,
recorder, software safety checks and scripts are retained. The following is
a **design specification, not an executable experiment**.

## Goal

Identify a reasonably performing PID baseline for subsequent comparison
against LADRC. Tune the wheel-speed controller in the TRACK state, without
using 0-to-0.05 m/s breakaway behavior as the primary gain-selection metric.

## Reference trajectory

- Operator initiates and checks startup to **0.3 m/s cruise**.
- Begin evaluation time zero **only after** cruise has stabilized and TRACK
  state has been confirmed (do not assume that a fixed elapsed time proves
  either condition).
- Hold each reference for 3 seconds:
  - t=0–3 s: 0.3 m/s
  - t=3–6 s: 0.5 m/s
  - t=6–9 s: 0.7 m/s
  - t=9–12 s: 0.5 m/s
  - t=12–15 s: 0.3 m/s
  - t=15–18 s: 0.0 m/s
- Record the actual wheel-speed reference (after any diff-drive limiters),
  measured wheel speed, controller mode, commanded/measured currents and
  timestamps. Determine metrics on the speed-change intervals separately
  from startup and stopping.

The reference alone represents 6.9 m of commanded travel after evaluation
begins. At 0.7 m/s with 0.064 m radius, wheel reference is ~10.94 rad/s.
**These exceed the current runner's hard bounds** (0.2 m/s linear,
10 rad/s wheel, 0.5 m commanded travel). Do not bypass the existing bounds
without separate controller/hardware/space/stopping review and explicit
reconfiguration of validated limits.

## Step 1: proportional gain

- Keep `Ki=0`, `Kd=0`.
- Candidate `Kp` values: 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
  0.9, 1.0 (ten candidates).
- This is a stepped gain search **informed by**, but *not identical to*,
  ultimate-sensitivity/Ziegler–Nichols tuning: no sustained oscillation or
  ultimate period is intentionally sought.
- Reject candidates with unsafe or unstable behavior. Select a provisional
  `Kp` by aggregate wheel tracking RMSE for the four cruising transitions,
  supplemented by overshoot, settling time, steady-state error and current
  peaks. Use repeated runs when needed to resolve close results.

## Step 2: integral gain

Fix the provisional `Kp`. Sweep `Ki` from zero, with `Kd=0`.
Focus on steady-state error and oscillation/overshoot. Determine the sweep
range from the preceding ground test results.

## Step 3: derivative gain

Fix selected `Kp` and `Ki`. Sweep `Kd` and derivative-filter time
constant, emphasizing overshoot, settling and derivative noise/current ripple.

## Step 4: PID vs LADRC

Freeze the tested PID gains and compare on identical speed profiles with
consistent startup/stopping behavior, current limits, test surface,
measurement protocol and repetition count.

## Implementation required before motor motion

1. Review actual robot/driver velocity, current, emergency-stop, feedback and
   available travel/stopping distance limits before supporting this profile.
2. Implement an explicitly bounded high-speed/long-distance experiment
   runner rather than raising hard limits in the earlier 0.05 m/s grid.
3. Add an automatic cruise-ready gate using actual feedback and confirm
   TRACK state instrumentation (the existing runner currently starts from a
   fixed-time staged plan).
4. Keep the existing abort-on-violation, stop and raw-data logging paths.
5. Test the generated plans in dry-run and non-motor integration tests before
   authorized ground operation.

No PID gains have yet been identified with this new profile.
