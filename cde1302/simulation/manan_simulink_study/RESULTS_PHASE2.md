# Phase 2: AprilTag landmark localisation — design, results and limits

Manan Mishra, CDE1302 individual contribution. Simulation study built on
Dr Feng LIN's lecturer quadrotor model (RPT controller). Written 11 November
2026. **Everything below is simulation with assumed sensor errors. Nothing
has been validated against the team's PX4 flight logs (none were supplied).**

## 1. Question

Does placing known-position AprilTags (landmarks) on the floor improve the
drone's position estimate, target-tag positions and return to home, and does
it matter whether the correction happens after the flight or during it?

## 2. What was built

| Part | File | Role |
|---|---|---|
| Scenario | `scenario_config.m`, `anchor_layout.m` | Team flight values, 3 random targets, landmark layouts, seeded sensor errors |
| Sensors + estimator | `nav_core.m`, `nav_estimator.m` | Optical flow, gyro heading, downward camera, Methods A and C |
| Simulink wiring | `build_study_model.m` (model `MananStudy_Nav`) | Controller and mission fly on the ESTIMATE, not truth |
| Post-flight correction | `nav_postcorrect.m` | Method B: MATLAB port of the team's `localisation.py` |
| Scoring | `nav_metrics.m`, `nav_tag_report.m`, `print_nav_summary.m` | Estimate, tracking, target, landing and landmark-availability numbers |
| GUI | `uav_study_gui.m` | Place landmarks, choose yaw/method/seed, run, live map, replay, results |
| Batch study | `run_nav_experiments.m`, `plot_nav_experiments.m` | Paired multi-seed grid (not yet run) |

### How the estimate reaches the controller

Analogy: the autopilot is a driver who can only see a slightly wrong sat-nav.
A 50 Hz block simulates the sensors from the true motion, runs the estimator
on those simulated measurements only, and outputs the estimate's error. That
error is added to the true position, velocity and heading before they reach
the lecturer controller and the mission logic. The controller therefore steers
by the estimate. If the estimate is 20 cm off, the real drone flies 20 cm off.

### The three methods

| Method | What happens | Changes where the drone flies? |
|---|---|---|
| A | Optical-flow speed integrated with gyro heading (dead reckoning). Landmarks are seen and logged but not used. | No (baseline) |
| B | After landing, the team's drift correction fits the logged landmark sightings and corrects the target report. | No: report only |
| C | Each landmark sighting corrects position and heading during flight (3-state Kalman filter with outlier gate). | Yes |

## 3. Assumptions and their sources

Team values (feature/loc-tags `config.yaml`, `camera_mount.py`): camera
320x240, fx 163.5 / fy 153.2 px, straight down, 0.08 m ahead of centre;
10 fps; 0.13 s capture-to-laptop delay; 0.03 m sighting noise; frames
rejected above 30 deg/s yaw rate; 1.5 m range; target tags 0.112 m, landmark
tags 0.127 m; flight 0.8 m high, 1.0 m lanes, 0.30 m/s; confirmation needs 5
sightings within 0.20 m, outliers beyond 0.30 m ignored.

Assumed, not measured (change them in `scenario_config.m`):

| Error source | Value (1 sigma) | Why this size |
|---|---|---|
| Flow scale error | 3 % per axis | team simulator example uses 5 % |
| Flow bias | 1 mm/s | team simulator example creep 0.5 mm/s |
| Flow noise | 0.03 m/s per sample | typical small flow sensor |
| Gyro yaw bias | 0.01 deg/s | after gyro calibration |
| Gyro scale error | 0.5 % | datasheet-scale tolerance |
| Placement heading error | 2 deg | team once saw about 3.5 deg |
| Detection probability | 90 % per visible frame | decision-margin failures, blur |
| Gross outliers | 2 % of sightings, up to 0.5 m | exercises outlier handling |

The same seed gives the same targets, sensor errors and noise in every run,
so A/B/C and different layouts are compared on identical conditions.

## 4. Verification (logic tests, no Simulink)

32 checks pass: `check_nav_core` (10), `check_postcorrect` (4),
`check_mission_reference` (18). Highlights:
- With perfect sensors the estimate equals truth exactly, even while turning.
- Camera view at 0.8 m is 1.57 m across x 1.25 m along the track.
- A 0.5 m outlier sighting is rejected and does not move the estimate.
- A 3 deg placement heading error is corrected to 0.06 deg over the home tag.
- Method B recovers a known 3 deg / 11 cm frame error exactly.

## 5. Results so far (one seed, Manan's desktop runs)

Seed 1, 5 landmarks (home tag + 4 search-area corners), fixed yaw, every
mission completed. Single seed: **indicative, not statistically established.**

| | Perfect sensors | A | B | C |
|---|---:|---:|---:|---:|
| Position estimate error, RMS (airborne) | 0.002 m | 0.109 m | 0.015 m (corrected track) | 0.062 m |
| Position estimate error, max | 0.004 m | 0.232 m | - | 0.179 m |
| Heading estimate error, RMS | 0.00 deg | 1.72 deg | - | 0.65 deg |
| Target position error, mean (3/3 found) | 0.002 m | 0.042 m | 0.006 m | 0.026 m |
| True distance to planned route, RMS | 0.054 m | 0.119 m | as A | 0.090 m |
| Max tilt | 3.2 deg | 4.2 deg | as A | 11.4 deg |
| Camera frames with a landmark in view | 43 % | 48 % | as A | 44 % |
| True landing error at touchdown | 0.002 m | 0.127 m | as A (report only) | 0.007 m |
| Where the drone believed it landed | 0.002 m | 0.005 m | - | 0.005 m |

Verified by `run_phase2_checks` (all 15 verdicts PASS, 9.2 min wall clock).
A GUI run with 6 custom landmarks (Method C) gave estimate RMS 0.043 m,
target mean error 0.018 m and landing 0.007 m: one more landmark in a
useful place helped, but this is one run.

What it means:
1. **Drift is real even with small sensor errors.** A 2 deg placement error
   and 3 % flow scale error put Method A up to 23 cm from where it believes it
   is. Heading error is the main cause: a small angle grows with distance.
2. **Post-flight correction (B) gave the best target report**, 0.6 cm against
   4.2 cm. With the whole flight available it fits the drift between landmark
   visits (it chose the team's "hybrid" model). It cannot change where the
   drone actually flew or landed.
3. **Online correction (C) made the drone fly closer to the plan** (9.0 cm
   against 11.9 cm from the route) and halved the heading error. Its target
   report (2.6 cm) is better than A but worse than B, because each sighting
   uses only what was known at that moment.
4. **C has a cost: corrections jerk the drone.** Each correction moves the
   estimate in a step; the controller chases it, giving the 11.4 deg tilt
   spike. A real system would blend corrections in gradually.
5. **Landmarks are easy to see at 0.8 m**: the 1.57 m wide view had one in
   sight in roughly 45 % of frames with only 5 landmarks.
6. **Only online correction fixes the landing.** Method A touched down
   12.7 cm from home while believing it was 0.5 cm away: it lands exactly
   where its drifted estimate says home is. Method C, corrected by the home
   tag on the way back, landed 0.7 cm away. Method B cannot help here; it
   only edits the report. This is the clearest practical difference.

**Landing scoring.** The lecturer model has no floor, so after touchdown the
drone floats at -1 cm with no horizontal flow feedback and slides (0.2-0.8 m,
reported as an artefact). Landing is therefore scored at touchdown (true
height <= 2 cm); an earlier version that scored after the slide was wrong
and was fixed.

## 6. Limitations

- The controller is the lecturer RPT model, not PX4; the filter is a study
  EKF, not EKF2. Vehicle parameters describe NUSWARM, not the MicoAir build.
- Sensor errors are assumed, not fitted to flight logs.
- Height, roll and pitch estimates are ideal; only horizontal position and
  heading carry error.
- No floor or contact model; landing is a scored touchdown point.
- Arena geometry is an assumed demo layout; the official drawing is missing.
- Wall tags are not modelled: a straight-down camera cannot see them.
- One seed so far. The paired multi-seed grid (`run_nav_experiments`) is
  written but has not been run.

## 7. Reproduce

```matlab
check_nav_core; check_postcorrect; check_mission_reference;
resA = run_nav_scenario(struct('anchors','home_corners5','method','A'));
resC = run_nav_scenario(struct('anchors','home_corners5','method','C'));
print_nav_summary(resA,resC); plot_nav_comparison({resA,resC},{'A (+B)','C'});
uav_study_gui                     % interactive; runs land in guiRuns
T = run_nav_experiments(); plot_nav_experiments();   % full grid, ~1 hour
```

## 8. Not done in Phase 2

- Multi-seed, multi-layout statistics (script ready, not run).
- Fixed vs tangent yaw results: that is Phase 3. The yaw modes, matched turn
  pauses and the strict-tangent conflict report are implemented and tested.
- Comparison with real PX4 logs.
