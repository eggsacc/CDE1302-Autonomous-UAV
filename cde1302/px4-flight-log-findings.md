# PX4 flight log review: log101.ulg (offboard tag-search drone)

**Vehicle:** MicoAir H743 V2, PX4 v1.15.4 (micoair-1.15.4)
**Sensors:** optical flow (no internal gyro) and downward rangefinder only; no GPS, no mag.
**Flight:** about 65 s. Offboard (nav_state 14) with position-only setpoints, then AUTO_LAND (nav_state 18).

**Mission context:** the drone flies legs over an area, detects tags with the ESP32 camera, and reports tag positions. Grading uses **lateral tag displacement only**.

The goal of this doc is to hand the codebase-side fixes to whoever owns the offboard setpoint generator and tag localisation.

---

## TL;DR

| # | Issue | Evidence | Fix location |
|---|-------|----------|--------------|
| 1 | Step position setpoints combined with a 0.4 m/s speed cap mean cross-track error is barely corrected while flying a leg | ~0.35–0.4 m off-axis error during every leg | **Companion code** (setpoint generator) |
| 2 | No heading reference; yaw is gyro-integrated only | `EKF2_MAG_TYPE=5` (None), `cs_yaw_align=0` all flight, EKF yaw σ grows from ~3.7° to ~5.1° in ~60 s | Procedure + params + code assumptions |
| 3 | Autotune fails immediately because of the RC roll centre offset | ch1 raw 1536 µs vs `RC1_TRIM=1500` gives manual roll = 0.052 the whole flight; the autotune abort threshold is 0.05 | Params / RC calibration |
| 4 | Leftover external-vision and GPS fusion enabled | `EKF2_EV_CTRL=1`, `EKF2_GPS_CTRL=7`, but neither source exists | Params |
| 5 | Logging too sparse to verify flow scale or gyro compensation | estimator topics logged at ~2–5 Hz | Params (`SDLOG_PROFILE`) |
| 6 | Possible tag-position error from camera latency and timestamp mismatch | Not in log; inferred from architecture (ESP32 camera over WiFi) | **Companion code** (tag localisation) |

Rate and attitude loops are **not** the bottleneck: attitude std is about 1° in flight. Skip further autotune work until 1–2 are fixed.

---

## 1. Setpoint generation: cross-track error not corrected (highest priority, code change)

### What the log shows
- `offboard_control_mode`: `position=1`; velocity, acceleration and attitude are all 0.
- `trajectory_setpoint.velocity` is NaN about 87% of the time, so no feedforward is sent.
- Setpoints are **instant steps** to the next waypoint:

  | t (s) | from (x, y) | to (x, y) |
  |-------|-------------|-----------|
  | 101.6 | (0.00, 0.04) | (0.00, −1.35) |
  | 106.5 | (0.00, −1.35) | (4.79, −1.34) |
  | 119.5 | (4.79, −1.34) | (4.79, 1.00) |
  | 127.5 | (4.79, 1.00) | (0.00, 1.00) |
  | 141.9 | (0.00, 1.00) | (0.02, 0.00) |

- During each leg, the axis that should be held drifts about 0.35–0.4 m and is not pulled back until the leg ends:
  - Y wanders up to about 1.38 m (setpoint 1.00) during the X leg at 128–142 s.
  - X sags to about 4.45 m (setpoint 4.79) during the Y leg at 120–128 s.

### Why
PX4 `PositionControl` computes `vel_sp = MPC_XY_P * (pos_sp - pos)` (with `MPC_XY_P = 1.3`). It then limits the horizontal velocity **by norm** to `MPC_XY_VEL_MAX` (0.4 m/s) while keeping the direction.

Example: 4.8 m along-track error and 0.35 m cross-track error give a vector that points almost entirely along-track. After scaling to 0.4 m/s, the cross-track component is about 0.4 × 0.35 / 4.8 ≈ **0.03 m/s**. Any sideways disturbance (flow bias, trim, airflow) goes uncorrected for the whole leg.

No gain change fixes this; it's a setpoint-shape problem.

### Required change (companion code)
Replace step setpoints with a **moving reference along the leg, plus velocity feedforward**:

```
for each leg p0 -> p1:
    dir = (p1 - p0) / |p1 - p0|
    L   = |p1 - p0|
    s(t): trapezoidal profile, cruise v_c ≈ 0.25–0.3 m/s, accel ≈ 0.3–0.5 m/s², ends at s = L, v = 0
    publish at 20–50 Hz:
        position = p0 + dir * s(t)
        velocity = dir * s_dot(t)        # feedforward, NOT NaN
        acceleration = NaN (or dir * s_ddot(t))
        yaw = constant (keep current behaviour)
    offboard_control_mode: position = true, velocity = true
    at leg end: hold p1 until |pos err| < 0.10 m and |v| < 0.05 m/s for ~1 s, then start next leg
```

Why this works:
- The position error stays small (the reference moves with the drone), so the P term is a pure correction term.
- In `ControlMath::constrainXY`, PX4 gives **priority to the position-correction component** and scales the feedforward, so cross-track correction is never squashed.

Keep headroom between cruise speed and the cap. Either keep `MPC_XY_VEL_MAX = 0.4` with cruise ≤ 0.3, or raise the cap to 0.5–0.6 and cruise at 0.3.

**Simpler alternative (carrot):** publish a position setpoint a fixed 0.3–0.5 m ahead of the drone's projection onto the leg line, clamped at p1. Cross-track error is then a large share of the error vector. This is less smooth than the feedforward version.

### Acceptance check
In the new log, during straight legs:
- |cross-track error| is under about 0.10 m.
- No overshoot above about 0.1 m at leg ends.

---

## 2. No heading reference (yaw drift rotates everything)

### What the log shows
- `EKF2_MAG_TYPE = 5` (None), and no mag is calibrated (`CAL_MAG0_ID = 0`).
- `estimator_status_flags.cs_yaw_align = 0` for the entire flight, and `vehicle_local_position.heading_good_for_control = 0`.
- The EKF's own yaw variance (`estimator_states.covariances[2]`) goes from 0.0042 to 0.0080 rad², i.e. σ ≈ **3.7° → 5.1°** over about 60 s. A 5° heading error over a 5 m leg is about 0.44 m of lateral error, and this does **not** appear in the estimate-vs-setpoint plots.

### Implications for the codebase
- The local frame's X axis is set at EKF init. Yaw is initialised arbitrarily, typically to the nose direction at init. The arena-to-local-frame mapping is only valid if the drone is **physically placed in the same orientation** at takeoff every run.
- Heading drift accumulates with flight time.

### Actions
- **Procedure:**
  - Mark a takeoff pad and an orientation line in the arena, and always place the drone aligned to it.
  - Reboot, or reset the EKF, just before each run so yaw re-initialises from that known orientation.
  - Keep total flight time short.
- **Code:**
  - Define waypoints in the arena frame and transform to local NED using the known takeoff pose. Don't assume local NED equals the arena frame.
  - If tags or arena features have known positions, consider using a detected reference tag to correct position and heading offset.
- **Optional:** try `EKF2_MAG_TYPE = 0` (auto) with a mag calibration done **in the venue**, only if the mag field there is clean (check field-norm stability). Indoor steel often makes this worse, so test before committing.

---

## 3. Autotune fails immediately (RC roll offset)

### Root cause
`mc_autotune_attitude_control` (v1.15) aborts to FAIL at any state-machine tick where `|manual_control_setpoint.roll| > 0.05` or `|pitch| > 0.05`.

From the log:
- `input_rc` ch1 (`RC_MAP_ROLL = 1`) raw = **1535–1537 µs** for the whole flight.
- `RC1_TRIM = 1500`, `RC1_DZ = 10`, so roll = (1536 − 1510) / (2012 − 1510) ≈ **0.052**.
- `manual_control_setpoint.roll` = 0.050–0.054 throughout (`data_source = RC`).

So autotune dies on its first tick, whatever `MC_AT_SYSID_AMP` is set to.

### Fix (any of these)
- Re-centre the transmitter roll trim or subtrim to 1500, then redo RC calibration in QGC (preferred).
- Or set `RC1_TRIM = 1536`.
- Also check ch4 (yaw): raw 1478 vs trim 1500. Yaw isn't checked by autotune, but recalibrate anyway.

Verify in the MAVLink console with `listener manual_control_setpoint`. Hands-off roll and pitch should be about 0.

Reset `MC_AT_SYSID_AMP` back toward default (the log shows 0.7).

---

## 4. Parameter cleanup

| Param | Current | Suggested | Reason |
|-------|---------|-----------|--------|
| `EKF2_EV_CTRL` | 1 | 0 | No external vision source (UWB dropped); avoid accidental fusion |
| `EKF2_GPS_CTRL` | 7 | 0 | No GPS on vehicle |
| `EKF2_OF_POS_X/Y/Z` | 0 / 0 / 0 | measured | Flow sensor offset from CoG (m). Matters for lever-arm compensation during rotation |
| `EKF2_RNG_POS_X/Y/Z` | 0 / 0 / 0 | measured | Same, for the rangefinder |
| `MPC_XY_VEL_MAX` | 0.4 | 0.5–0.6 if cruise is 0.3 | Headroom for correction above feedforward (see §1) |
| `MPC_XY_P` | 1.3 | keep | Revisit only after §1 is done |
| `SDLOG_PROFILE` | 161 | 179 (+ estimator replay, + high rate) | Current logging is ~2–5 Hz for estimator topics. Too sparse to check flow scale or gyro compensation, and replay lets EKF params be re-tested offline |
| `RC1_TRIM` | 1500 | recalibrate (see §3) | Autotune |

Leave the rate and attitude gains alone for now. The vehicle is heavily overpowered (`MPC_THR_HOVER ≈ 0.155`), which explains the low rate gains, and attitude tracking is fine.

---

## 5. Flow sensor checks (needs high-rate log)

Facts from the log:
- Flow sensor reports **no internal gyro** (`delta_angle_available = 0`), so PX4 compensates with the FC gyro using `EKF2_OF_DELAY = 20 ms`.
- Mean flight height is about **0.47 m** (`dist_bottom`, range 0.11–0.58 m). At this height, flow velocity = flow rate × height, so 1 cm of height error is about 2% velocity error. Rangefinder accuracy matters a lot.
- Flow innovations are about 0.1 rad/s RMS with test ratios ≤ 0.14, so fusion is healthy.
- Flow quality is 71–95.

To do after setting `SDLOG_PROFILE = 179`:
1. **Rotation test:** hold the drone over a textured floor, keep the position fixed, and rock it in roll and pitch by hand. `estimator_optical_flow_vel.flow_rate_compensated` should stay about 0. If it tracks gyro rate, adjust `EKF2_OF_DELAY` or the sensor's scale.
2. **Scale test:** fly or carry it exactly a measured distance, e.g. 4.00 m taped, and compare against the integrated position in the log. Correct the flow scale on the sensor side (MTF-01 config) if the error is over about 3%.
3. **Height:** if the camera's tag detection allows it, flying around 0.8–1.0 m reduces the relative impact of rangefinder error. This is a trade-off against tag pixel size.

---

## 6. Tag localisation (code review checklist; not visible in log)

Grading is on lateral tag position, so the tag position pipeline matters as much as flight accuracy:

- [ ] **Latency:** ESP32 camera → WiFi → host has a capture-to-detection delay. At 0.3 m/s, 200 ms of latency is 6 cm of error. Tag world position must use the **drone pose at image capture time**, either by buffering poses and interpolating by timestamp, or by briefly hovering on detection.
- [ ] **Projection:** tag_world = drone_pos + R(yaw, pitch, roll) · (camera offset + ray × height). At 0.47 m, 1° of tilt is only about 8 mm, but camera mount offset and yaw are not negligible.
- [ ] **Frame:** report tags in the arena frame using the same takeoff-pose transform as §2.
- [ ] **Averaging:** if a tag is seen in several frames, average the estimates. This reduces noise but not drift bias.

---

## Suggested order of work
1. §1 setpoint generator (biggest win for lateral accuracy).
2. §3 RC trim (unblocks autotune, but autotune is low priority).
3. §4 params, plus a high-rate logging profile.
4. §2 takeoff alignment procedure and arena-frame transform.
5. §6 tag latency and timestamp review.
6. §5 flow scale and compensation tests with the new logs.
