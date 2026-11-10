# Manan's lecturer-based Simulink UAV study

> **Stopped for handoff at Manan's request, 11 November 2026.** Start with
> [CLAUDE_HANDOFF.md](CLAUDE_HANDOFF.md), designed to be pasted below his original
> project brief. The original reproduction and raw position-step mission ran.
> The approved bounded-reference governor now passes pure logic checks but has
> **not** been run through the vehicle model. Phase 1 is not complete.

Phase 1 builds a fixed-yaw, position-command testbench around Dr Feng LIN's
original quadrotor model. It separates command, simulated truth and ideal
controller feedback. It does **not** run PX4, EKF2, an AprilTag estimator or
physical landing/contact logic. See `PROGRESS.md` and `evidence/` for the
execution status; do not infer successful runs from the presence of code.

## Run

Open a fresh MATLAB session in this directory:

```matlab
check_environment;
verify_sources;
check_mission_reference;
result = run_phase1('demo');
```

To rerun only the mission: `result = run_baseline('demo');`.
`run_baseline('team_head_placeholder')` instead uses the pinned team's
configured sweep rectangle and lane spacing. `study_config('assessment')`
deliberately refuses to run until the missing authoritative geometry is
provided. No drone, serial port, MAVLink connection or firmware tool is used.

The MATLAB desktop entry point above is the validated route. An ordinary
PowerShell batch command is also provided below, but a full batch simulation
has not completed through this Codex environment:

```powershell
& 'C:\Program Files\MATLAB\R2026a\bin\matlab.exe' -batch "run_phase1('demo');"
```

The installed R2026a Update 5 has executed the lecturer model. Codex's restricted
terminal launch failed before MATLAB code ran; reviewed batch execution passed
version/licence/logic checks but stalled during Simulink runtime initialization.
Desktop execution produced saved original-model data and figures. Git metadata
also required reviewed execution. These observations do not establish that
ordinary PowerShell outside Codex has the same problem.

Required: MATLAB, Simulink, Aerospace Toolbox and Aerospace Blockset. The model
also calls rotation-conversion functions; `check_environment` records their
actual providers. Robotics System Toolbox is installed on the tested machine.
Optimization Toolbox is **not required** for this baseline; unfinished
minimum-snap assignment scripts are not part of the run.

`run_lecturer_example` executes the original script unchanged, including its
base-workspace variables and `close all`. Use a fresh session to avoid
overwriting personal workspace variables or closing unrelated figures.

## What changes around the original model

`build_study_model.m` creates a separate `MananStudy_RPT` model from the
preserved source. It scopes initialization parameters to that model's workspace,
replaces the trajectory source with the mission generator, adds signal logging
and a mission stop condition, and disables external FlightGear UDP/pacing.
Vehicle dynamics, RPT gains, attitude/rate control and truth-feedback wiring
remain the lecturer's. The generated inspectable SLX is under `generated/`;
the builder is its versioned source. No operational firmware is changed.

The source RPT reference has eleven channels:
`[x y z vx vy vz ax ay az yaw yawrate]`. The position-command baseline uses
zero velocity/acceleration/yaw-rate feedforward and constant yaw. The PID
variant has a different eight-channel interface and is not silently substituted.

The mission generator runs at 20 Hz. Its retained `waypoints` mode sends the
next corner as a position target and uses a 0.15 m gate with a one-tick handoff.
That raw mode was executed and found overly aggressive for this model. Manan
approved the new default `bounded_position` reference governor: a 0.15 m/s
horizontal/vertical sampled-command bound, same corner sequence and no v/a
feedforward. It additionally waits for the command to reach each corner.
Its latest pure logic checks pass, but plant execution remains pending.
Intended corners omit upload sampling/quantisation. Takeoff, home settle and
a 0.10 m/s reference descent are explicit simulation stages, not PX4 landed
detection/disarm. Completion/failure stops the run; the current 360 s cap is
not a success criterion.

The YAML's nominal 0.15 m/s did not limit speed in raw waypoint mode: the actual
diagnostic reached 8.82 m/s and extreme attitudes. The new governor bounds the
sampled command, not actual vehicle speed. It is an explicit approved study
adaptation; controller gains and vehicle parameters remain unchanged.

## Geometry and coordinate frames

The official drawing and raw flight datasets are still missing. Default demo
placement is an **assumption**, marked on every figure:

| Item | Demo arena coordinates (m) | Status |
|---|---|---|
| Outer rectangle | x 0..5.4, y 0..4.5 | Dimensions supplied by Manan; origin/orientation chosen |
| Start square | x 0.15..1.05, y 0.15..1.05 | 0.9 m size supplied; position assumed |
| Home | (0.6, 0.6) | Assumed start-square centre |
| Search rectangle | x 1.2..4.8, y 0.45..4.05 | 3.6 m width supplied; length/offsets assumed |
| Lane spacing | 0.6 | Demonstration choice, not a calibrated camera swath |

`team_head_placeholder` uses x 0..4.8, y -1.35..1.0, lanes 2.5 m apart. Its
official outer/start placement is unknown, so boundary compliance is
**unavailable**, not a claim that the placeholder is the assessment arena.

The explicit horizontal transform is

```text
p_local_xy = home_local_xy + R(arenaToLocalYaw) * (p_arena - home_arena)
p_local_down = home_local_down - height_above_home
```

The demo's zero rotation aligns arena axes with the initial local axes.
It does not establish geographic North/East, especially without a compass.
`check_mission_reference` checks a displaced origin and nonzero rotation.
For example, `cfg=study_config('demo',struct('arenaToLocalYaw',pi/2));`
builds the route through that configured transform; `run_baseline(cfg)` accepts
the resulting configuration. Zero angle is a modelling choice, not a survey.
Camera/body transforms and marker visibility belong to Phase 2; no camera
measurement exists in this baseline.

## Outputs and interpretation

`results/original_circle/` contains the reproduced original logs and plots.
The preserved raw run is `results/fixed_yaw_demo/`. New runs use
`results/fixed_yaw_<preset>_<guidanceMode>/` and contain `run.mat`, `trajectory.csv`,
`metrics.json`, `checks.json`, `sample_times.txt` and three PNG plots.
`evidence/` contains compact dependency, logic and execution summaries.
Generated/runtime results are ignored by Git; review size and provenance
before versioning any data. No raw flight logs are imported or fabricated.

`result.position`, `velocity` and `euler` are **simulated truth**.
`result.reference` is the actual commanded eleven-channel input.
`feedbackPosition` and `feedbackVelocity` equal truth in this ideal baseline.
`estimatedPosition` is empty and `estimateAvailable=false`: no sensor estimator
has been implemented. Euler outputs are radians, and NED Down is positive.

Analysis uses actual simulation timestamps and time-weighted RMS. It reports
setpoint tracking separately from distance to the active route leg and nearest
whole route. A step target is intentionally ahead of the aircraft, so its
distance is not a direct measure of camera coverage or cross-track accuracy.
Metrics are also broken down by mission stage; return error is measured before
descent. Numerical checks and mission completion are separate from performance
findings; no arbitrary tracking pass threshold is invented.

Boundary checks concern the vehicle centre, not rotor clearance. Ground-plane
penetration remains visible; simulated truth is never clamped to manufacture
landing success. The model lacks contact, disarm, a landing detector and the
real PX4 LAND controller. A final point at home cannot validate those behaviours.

## Model versus the real UAV

The source describes a 0.302 kg NUSWARM model, 0.0755 m arm radius, 20 ms motor
lag and zero aerodynamic damping. These are not measurements of the team's
MicoAir aircraft. The RPT outer loop is not the PX4 controller. The saved
`uwb-testing` PX4 1.15.4 parameter export also has different gains; it is not
linked to the missing log185. Sensor fusion, delayed images/telemetry, optical
flow and actual mass/inertia/propulsion remain uncalibrated here.

The original circle directly differentiates wrapped yaw, producing about
-627.52 rad/s command spikes. This is preserved for reproduction and absent
from the fixed-yaw mission reference. Its yaw saturation also references the
pitch rate-limit parameter. Neither proves a real aircraft yaw limitation.

## Provenance and next milestones

Lecturer source: [CDE1302_UAV](https://github.com/linfengfrank/CDE1302_UAV/tree/9d80ddab01b20b01a19cb849b5af5424ca0ae07a),
Dr Feng LIN; immutable asset details are in `vendor/lecturer/PROVENANCE.md`.
`verify_sources` checks all five copied assets byte-for-byte; never repair a
hash mismatch by replacing the expected hash without investigating it.

Team mission reference:
[`claude-one-shot` at e7a231e](https://github.com/eggsacc/CDE1302-Autonomous-UAV/tree/e7a231eff2f9239ff0d6948a058cab19b4b5757c).
Its mission planner and firmware waypoint gate supply the navigation pattern.
The source/docs do not establish which binary/configuration actually flew.
`feature/loc-tags` and `yaw_test_lab5` were audited as references, without merging
their firmware. Existing landmark correction is post-flight, not EKF feedback.

Manan's contribution is experiment definition, adaptation, reproducible
measurements, validation and interpretation. Codex assisted with audit and
study-code implementation; this README does not claim independent authorship
of lecturer, teammate or Codex-generated code.

Phase 2 (targets, anchors, estimator and GUI), Phase 3 (paired yaw experiments)
and optional UWB have not been implemented. Pause for Manan's review after
Phase 1 evidence. No commit or push is authorised yet.
