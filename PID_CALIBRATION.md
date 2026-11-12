# Minimal PID parameter calibration for ADRC comparison

**Research objective**: use a reasonably stable and reproducible PID
baseline for a separate PID-versus-ADRC performance study. This is **not** a
project to optimize PI startup compensation, and no Ubuntu motor-plant
simulation is requested.

## Repository and controller selection

- This is the `feat/pid-gain-calibration` branch of **bringup**.
- Hardware remains on the already-built `feat/pi-three-stage-transition`
  revision. `pid_three_stage_transition=0` and
  `pid_bumpless_handoff=0` in every calibration plan, so neither
  experimental handoff is activated.
- The hardware driver's PID logic implements `pid_torque` with
  `Kp,Ki,Kd` and a filtered D term; test `Kd=0` reference and
  `Kd=0.01,0.02` exploratory settings, keeping `Kp=0.4,Ki=0.2`.
- This plan does not change source-code defaults or the physical stop
  mechanism. Gain units must be checked against the actual code and
  wheel angular-speed feedback before publication.

## Stage 0: offline review only (NO MOTOR MOVEMENT)

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch --track origin/feat/pid-gain-calibration
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select zlac8015d_hardware zlac_can_bringup
source install/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
python3 -m unittest discover -s tests -p 'test_pid_calibration_plan.py' -v
ros2 run zlac_can_bringup run_gain_sweep.py --config \
  ~/ros2_ws/src/zlac8015d_can_bringup/config/pid_calibration_preflight.yaml
ros2 run zlac_can_bringup run_gain_sweep.py --config \
  ~/ros2_ws/src/zlac8015d_can_bringup/config/pid_calibration_elevated.yaml
```

These are dry-runs, printing 1 and 3 planned trials respectively. They
do not start a robot controller or send CAN commands.

## Stage 1: physical preflight (NOT authorized by this document)

**The previous hardware test recorded a 1.86 rad/s wheel overspeed** versus
the 0.781 rad/s intended command. A launch-level error and a `/cmd_vel=0`
command are *not* sufficient proof of a physical safe stop.

Before any `--execute`, an operator must independently demonstrate physical
E-stop and driver-side communication-loss behavior, check free-hanging wheels
with no one near rotating parts, confirm CAN/encoder polarity and both wheel
directions, verify the current limit and speed-monitor behavior, and use an
isolated ROS domain with only one controller. The watchdog or loss of a ROS
publisher is not a substitute for a physical E-stop. Check that the
experimental speed guard at 1.2 rad/s uses a 1.2 tolerance factor, i.e.
triggers above **1.44 rad/s**; this is a software monitor, not a certified
motor overspeed protection.

Only after these checks, and *only with a present operator and actual
emergency stop*, execute the ONE-candidate preflight YAML. Do not start the
three-candidate sweep until the first trial is inspected and has no abort.

## Stage 2: gain screening

After safely validating initial motion and physical stopping with the single
baseline case, test the `pid_calibration_elevated.yaml` candidates in the
same wheels-elevated conditions:
- `PI_reference`: `Kp=0.4,Ki=0.2,Kd=0`
- `PID_D001`: `Kp=0.4,Ki=0.2,Kd=0.01`
- `PID_D002`: `Kp=0.4,Ki=0.2,Kd=0.02`

No automatic gain optimization is performed. The runner records each case to
a fresh campaign directory, with raw `cmd_vel`, `joint_states`, and
wheel current traces. A safety abort ends the campaign.

Review a completed sweep via

```bash
python3 ~/ros2_ws/src/zlac8015d_can_bringup/scripts/rank_pid_calibration.py \
  ~/ros2_experiments/pid_calibration/sweep_YYYYMMDDTHHMMSSZ_ID/summary.csv
```

The ranking script refuses to recommend a candidate when a trial aborts.
Its output is a **provisional RMSE ranking**, not an automatic safety
certification. Read the time series for speed overshoot, current spikes,
steady oscillations, startup behavior, and verified stop. Do not
automatically choose the lowest RMSE when it overshoots.

If the first PID candidates yield excessive noise, instability or
oscillation, revert to the stable PI reference and then adjust the derivative
gain/filter in one short additional round rather than a broad grid search.
For the final PID-versus-ADRC ground comparison use identical reference
profiles, controller current/rate limits, startup/stop handling, safety
guards, sample rate, and repeated runs. Ground experiments require a
separate safety review and must not use the `wheels_elevated` label.

## Not verified yet

No real-world measurement from this new PID plan has been collected.
The numbers `Kp=0.4,Ki=0.2,Kd=0.01,0.02` are initial candidates, **not
identified parameters**. The 3-stage simulation branch is no longer part
of the tuning workflow.

## Ground-contact calibration (operator-confirmed safety)

The user has selected **ground-contact** gain identification rather than
wheels-elevated identification. Ground configurations are separate from the
previous elevated-wheel templates:

- `config/pid_calibration_ground_preflight.yaml`: one PI-reference trial.
- `config/pid_calibration_ground.yaml`: three trials (Kd=0, 0.01, 0.02).

Each has 1s zero, 2s forward at 0.05 m/s, 3s zero. Nominal travel
is 0.10m (actual measured travel is monitored separately). Wheel reference
is 0.78125rad/s. The configured monitor wheel bound is 1.2rad/s, but
the existing runner actually aborts when observed speed **exceeds
1.44rad/s** because it uses a 1.2 multiplier. The running trace must
be reviewed against this distinction. This is not an independent hardware
speed limiter.

Run safe dry-runs first:

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git pull --ff-only
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select zlac_can_bringup
source install/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
python3 -m unittest discover -s tests -p 'test_pid_calibration_ground.py' -v
ros2 run zlac_can_bringup run_gain_sweep.py --config \
  ~/ros2_ws/src/zlac8015d_can_bringup/config/pid_calibration_ground_preflight.yaml
ros2 run zlac_can_bringup run_gain_sweep.py --config \
  ~/ros2_ws/src/zlac8015d_can_bringup/config/pid_calibration_ground.yaml
```

Only when the operator has verified stopping, clear ground travel path,
and E-stop may they run the preflight **interactively** with
`--execute --i-have-e-stop`. Never use `--yes` as an unintended bypass.
Review logs of the first attempt and verify standstill before the next
trial. Run the three-candidate plan only if the preflight passes and
measured speed/current/stopping are acceptable. Aborted trials are not
valid gain measurements.

`Kp=0.4,Ki=0.2,Kd=0.01/0.02` are candidates, not measured optima.
Do not infer gain suitability from nominal model simulations.

## Expanded staged search (43 individual trials, generated offline)

After the one-run ground preflight has been reviewed, use
`scripts/generate_pid_grid.py` to construct THREE separate plans. The program
only writes YAML and runs its validator; it **never operates motors**.

- **Stage A (12 trials)**: `Kp in [0.2,0.4,0.6,0.8]` by
  `Ki in [0.1,0.2,0.4]`, `Kd=0`. The `0.4/0.2` reference is first.
- **Stage B (13 trials)**: reference `Kd=0` plus
  `Kd in [0.005,0.01,0.02,0.04]` ×
  `derivative_tau in [0.03,0.05,0.10]s`, initially fixed
  `Kp=0.4,Ki=0.2`. **Update Kp/Ki after reviewing Stage A**; the generator
  does not automatically transfer the best result.
- **Stage C (18 trials)**: two initially assigned `PI` / `PID`
  candidate settings, speed commands 0.04/0.05/0.06m/s,
  repeated 3 times. **Update the PID candidate gain based on Stage B**.

Total: 43 trials across three independent campaigns, **not** one unattended
43-trial experiment. Each trial is a short stop-start profile, and the
current/speed/travel software guards remain enabled. Tests are ground-contact
and require a clear travel zone, physical E-stop, confirmed hardware stopping,
one live ROS controller manager and active operator.

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git pull --ff-only
python3 -m unittest discover -s tests -p 'test_pid_grid.py' -v
python3 scripts/generate_pid_grid.py --output-dir /tmp/pid_ground_grid
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run zlac_can_bringup run_gain_sweep.py --config /tmp/pid_ground_grid/A_PI_grid.yaml
ros2 run zlac_can_bringup run_gain_sweep.py --config /tmp/pid_ground_grid/B_PID_grid.yaml
ros2 run zlac_can_bringup run_gain_sweep.py --config /tmp/pid_ground_grid/C_validation.yaml
```

These are three **dry runs**. To run real motors requires the existing
runner's explicit `--execute --i-have-e-stop` opt-in per campaign.
No automated mass execution is part of the setup instructions. It is
important to review the baseline trial logs and initial overshoot before
authorizing the larger A sweep, and then review all A data before B, and B
before C. The driver had a historical 1.86rad/s overspeed and the
runner's configured 1.2rad/s speed limit currently triggers at 1.44rad/s
due to 20% monitoring tolerance; do not treat this as a hardware speed cap.

The three stages are designed to find reasonable parameters efficiently,
not to certify optimal gains. Stop exploring once a stable and
reproducible PID baseline is established for ADRC comparison.
