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
