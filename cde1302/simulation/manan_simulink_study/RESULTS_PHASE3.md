# Phase 3: fixed yaw versus tangent yaw

Manan Mishra, CDE1302 individual contribution. Simulation study on Dr Feng
LIN's lecturer quadrotor model with the Phase 2 estimator in the loop.
Simulations run by Manan on 11-12 November 2026 (`run_phase3_yaw`, 14 runs,
38.5 min). **Simulation with assumed sensor errors and three seeds; not
flight evidence.**

## 1. Question

During the lawnmower search, should the nose stay pointing one way (fixed
yaw, as the team's trajectory mode flies) or turn to face the direction of
travel (tangent yaw)? Does it change tracking, drift, camera detections,
target accuracy or landing?

## 2. How the comparison is made fair

| Kept identical in both modes | How |
|---|---|
| Path, corners, lane spacing (1.0 m), speed (0.30 m/s), height (0.8 m) | same `study_config` route |
| Command timing | the position command pauses for the same planned turn time at every leg junction in BOTH modes (`turnPauseMode = matched`) |
| Targets, landmarks (home + 4 corners), sensor errors, noise | same seed |
| Controller, vehicle, estimator | unchanged |

Only the commanded heading differs. No yaw-dependent error is added by hand:
differences come from the vehicle dynamics, the team's 30 deg/s camera gate,
camera geometry and the assumed gyro and flow error models.

Yaw handling: the heading command is unwrapped and always turns the shorter
way. At zero horizontal speed the heading is held. Yaw rate is limited to
0.6 rad/s (34.4 deg/s), the default of Manan's `yaw_test_lab5` uploader.
Zero yaw-rate feedforward is sent, as for position.

## 3. The yaw-rate conflict (exact, from the route)

| Junction | Heading change | Time at 0.6 rad/s | Pause used (+0.5 s settle) |
|---|---:|---:|---:|
| Home to first corner, onto lane 1 | -14.0, +14.0 deg | 0.41 s each | 0.91 s each |
| 8 lane-end cross-overs | +-90 deg | 2.62 s each | 3.12 s each |
| Last corner back home | -140.6 deg | 4.09 s | 4.59 s |
| **Total** | 888.7 deg turned, -140.6 deg net | **25.9 s** | **31.4 s** |

A strictly tangent nose needs an instantaneous heading change at all 11
junctions: an unbounded yaw rate at 0.30 m/s. This is the same blocker
Manan's `yaw_test_lab5` met on its ellipse. The study stops and turns instead
of rounding corners or raising limits. The simulated drones turned 885-904
deg in total, confirming the plan.

## 4. Predictions made before running, and how they fared

| Prediction | Outcome |
|---|---|
| Fixed yaw loses no frames to the 30 deg/s gate | **Confirmed**: 0 % in all 6 fixed runs |
| Tangent loses about 15-17 % of frames (all of the 25.9 s of turning) | **Wrong: 6.3-6.7 %.** The lecturer yaw loop lags the ramped command (no yaw-rate feedforward), so the true rate exceeds 30 deg/s only near the middle of each turn. Turning (> 5 deg/s) lasted 41 s, not 25.9 s |
| The matched pause lets the nose finish turning before moving | **Wrong.** The true heading lagged the command by up to 30-34 deg, so the drone often started the next leg still rotating. Even stop-and-turn did not give a strictly tangent nose with this controller |
| A flow-sensor offset error cannot affect fixed yaw | **Confirmed**: < 1 mm change |
| An offset error does affect tangent yaw | **Confirmed**: 2-7 cm changes (single seed) |
| Camera coverage is the same on lanes in both modes | **Consistent**: all 3 targets confirmed in all 12 runs |

## 5. Results

All 12 baseline missions completed. (The runner printed `completed=2` for two
groups: a scoring bug, since fixed. Both runs stopped at about 155 s, which
only happens when the mission's done flag fires; the 100 Hz scoring grid
missed the final DONE tick. Completion is now read from the mission log.)

### 5.1 Means over seeds 1-3 (std in brackets where useful)

| Metric | A fixed | A tangent | C fixed | C tangent |
|---|---:|---:|---:|---:|
| Frames rejected by 30 deg/s gate | 0 % | 6.4 % | 0 % | 6.5 % |
| Camera frames with a landmark used | 43.3 % | 35.7 % | 37.8 % | 33.6 % |
| Position estimate RMS (m) | 0.117 | 0.133 | 0.058 | 0.056 |
| Heading estimate RMS (deg) | 2.16 | 2.22 | 0.54 | 0.68 |
| True distance to route RMS (m) | 0.118 | 0.122 | 0.080 | 0.077 |
| Target mean error, in flight (m) | 0.068 | 0.099 | 0.030 | 0.033 |
| Target mean error, Method B (m) | 0.012 | 0.014 | - | - |
| Targets confirmed | 3/3 every run | 3/3 | 3/3 | 3/3 |
| Landing error at touchdown (m) | 0.134 | 0.137 | 0.014 | 0.021 |
| True yaw vs command RMS (deg) | 2.15 | 11.2 | 0.55 | 10.8 |
| Max true yaw rate (deg/s) | 0.4 | 34.3 | 2.9 | 34.3 |
| Max tilt (deg) | 4.1 | 4.1 | 9.7 | 13.8 (one run 25.6) |

In fixed yaw the "yaw vs command" error equals the heading-estimate error:
the drone holds what it believes is the commanded heading.

### 5.2 Paired difference, tangent minus fixed (same seed)

| Metric | Method A: mean [min, max], worse in | Method C: mean [min, max], worse in |
|---|---|---|
| Target mean error | +3.1 cm [+0.3, +7.3], **3 of 3** | +0.3 cm [-2.3, +2.1], 2 of 3 |
| Position estimate RMS | +1.6 cm [0.0, +4.1], 2 of 3 | -0.2 cm [-2.6, +1.1], 2 of 3 |
| Landing error | +0.3 cm [-8.3, +14.7], 1 of 3 | +0.7 cm [-0.2, +1.3], 2 of 3 |
| True distance to route | +0.4 cm [-0.3, +0.7] | -0.4 cm [-2.4, +0.9] |
| Landmark frames used | -7.6 points, **3 of 3** | -4.2 points, **3 of 3** |
| Method B target error | +0.2 cm [-0.4, +0.8] | - |

### 5.3 Flow-sensor offset sensitivity (5 cm, uncompensated, Method A, seed 1)

| | Fixed: baseline -> offset | Tangent: baseline -> offset |
|---|---|---|
| Position estimate RMS (m) | 0.110 -> 0.109 | 0.151 -> 0.131 |
| Target mean error (m) | 0.042 -> 0.042 | 0.115 -> 0.091 |
| Landing error (m) | 0.127 -> 0.128 | 0.275 -> 0.207 |

The offset happened to reduce the error in this one tangent run, because it
partly cancelled other errors. The point is not the sign but that tangent
results move by several centimetres with an unverified PX4 parameter, while
fixed yaw does not move at all.

### 5.4 What it means

1. **Tangent yaw improved nothing.** No metric got consistently better in
   either method.
2. **Without online correction, tangent yaw made the target report worse
   in every pair** (+3.1 cm on average). It also cost landmark sightings
   in every pair: 6.4 % of frames are discarded by the team's 30 deg/s gate,
   exactly at the corners where the landmarks sit. The exact share of the
   target-error increase due to fewer sightings versus heading excursions
   was not isolated.
3. **With online correction (Method C), the penalty mostly disappears.**
   Differences are within a few centimetres and of mixed sign. Landmark
   corrections absorb the extra heading error.
4. **Tangent yaw made Method C rougher.** One tangent run had a 0.52 m
   estimate jump and a 25.6 deg tilt spike, the largest in the study. Turning
   while correcting stresses the filter. This was seen once and its cause
   was not investigated (seed 3 data were not saved in full).
5. **Tracking along the route was unaffected** (+-0.4 cm).
6. **Post-flight correction (Method B) cancels most of the difference**
   (1.2 vs 1.4 cm), as it does for drift in general.

## 6. Recommendation

**Keep fixed yaw for the lawnmower search** (what the team's trajectory mode
already flies). In this simulation tangent yaw:
- cannot be flown strictly on this path (unbounded corner yaw rates; even
  stop-and-turn left the nose lagging by up to ~33 deg);
- discarded 6.5 % of camera frames and 4-8 % of landmark frames;
- made dead-reckoned target positions worse in every paired seed;
- made results sensitive to the unverified flow-sensor offset;
- offered no measured benefit in tracking, drift, detection or landing.

If tangent yaw is ever required (for example for a forward-facing sensor):
turn below 30 deg/s or add yaw-rate feedforward so the nose keeps up, use
online landmark correction, and verify PX4 `SENS_FLOW_POS_X/Y`.

## 7. What is assumption and what is evidence

| Statement | Basis |
|---|---|
| 11 junctions, 888.7 deg, 25.9 s at the limit, unbounded strict-tangent rate | geometry of the configured route (exact) |
| 30 deg/s frame rejection | team code (`tags.max_yaw_rate_deg_s`) |
| 0.6 rad/s yaw limit | Manan's `yaw_test_lab5` uploader default, not an airframe measurement |
| Yaw lag of 30-34 deg in turns | lecturer RPT yaw loop with zero feedforward; PX4's yaw controller may differ |
| Gyro scale and bias, flow errors, 5 cm offset | assumptions in `scenario_config.m` |
| All effect sizes in section 5 | simulation, 3 seeds, lecturer controller, study EKF |
| Real yaw-induced drift on the team's drone | **not established**: needs paired real flights with PX4 logs |

## 8. Reproduce

```matlab
check_mission_reference          % 18 logic checks incl. yaw-rate limit and pairing
out = run_phase3_yaw();          % 14 Simulink runs, ~38 min on Manan's laptop
```
Raw table: `results/phase3_yaw/summary.csv`; seed-1 runs and figures in the
same folder.
