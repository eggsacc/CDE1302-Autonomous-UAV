# Flight log analysis: sudden odometry jump (consolidated)

**Logs:** Flight A `eaeb93f7…ulg` (55 s), Flight B `0c0bde59…ulg` (122 s). Time is measured in seconds since arming.
**Platform:** MicoAir743v2-AIO, PX4 v1.15.4 (`micoair-1.15.4`), OFFBOARD control from the companion.
**IMUs:** IMU0 = BMI088 (SPI2; gyro ID 6684690, accel ID 6946834). IMU1 = BMI270 (SPI3; ID 3604506). The mapping comes from the boot log plus the bus field of each device ID.
**Aiding:** downward optical flow + rangefinder only. The AIO board **has no compass** (vendor docs; the boot log shows `qmc5883l: no instance started`). `EKF2_MAG_TYPE=5`, `SYS_HAS_MAG=0`. No GPS or EV data. Baro is not fused (`EKF2_BARO_CTRL=0`).
**Figures:** `figs/fig1…fig6` (PNG). Generated with pyulog + matplotlib directly from the logs.

---

## 1. Summary
The jump is the **EKF selector switching primary instance from EKF0 (BMI088) to EKF1 (BMI270)**. `EKF2_MULTI_IMU=3` runs two instances, one per IMU.

1. **Yaw is unobservable.** There is no heading measurement of any kind: `cs_yaw_align` was never set and `heading_good_for_control=0` throughout. Each instance integrates its own gyro z, and the z-gyro bias cannot be estimated.
2. **The BMI270 has an in-flight yaw-rate offset relative to the BMI088:** about −2.5°/s (A) and −1.8°/s mean (B). EKF1's heading therefore drifted by −105° (A) and about −205° (B) by the time of the switch (Fig. 1, 2, 4).
3. **Position follows heading.** Optical flow measures body-frame velocity, and each instance rotates it into NED using its own yaw. The two xy tracks diverged by 5.4 m (A) and 6.9 m (B) (Fig. 3, 4).
4. **A short innovation transient made the selector switch.** A spike lasting about 60 ms at descent onset (A) or touchdown impact (B) briefly made EKF1 look better. The selector's relative score went from +1 to about −0.5 and it switched (Fig. 5). PX4 then published the switch as a position reset of 5.3 m / 6.9 m and a heading reset of −105.9° / +156.5°.
5. **The selector has no way to detect the bad heading.** A wrong yaw produces no flow innovation, because flow is self-consistent with each instance's own yaw. The IMU-disagreement fault check (7°/s) is never approached (Fig. 2).

## 2. Evidence

| | Flight A | Flight B |
|---|---|---|
| Switch time | 45.44 s, airborne at 0.5 m, start of descent | 117.06 s, on the ground, 0.28 s after kill switch |
| Published xy reset (Δx, Δy) | (+0.44, −5.27) m | (−6.84, +0.76) m |
| Published heading reset | −105.9° | +156.5° (≡ −203.5°) |
| EKF0 yaw before switch | ≈ 0° ± 1° | ≈ 0° ± 1° |
| EKF1 yaw drift | ≈ −2.4°/s, −105° at switch | ≈ −1.8°/s mean, about −205° at switch |
| EKF0–EKF1 xy divergence | 0 → 5.4 m (from about 12 s) | 0 → 6.9 m (from about 20 s) |
| Selector trigger | EKF0 height test ratio 1.04 vs EKF1 0.27 at descent onset | EKF0 combined ratio 2.17 vs EKF1 1.16 at impact; EKF0 already degraded from 116.3 s (range below `SENS_FLOW_MINHGT`) |
| `gyro_fault_detected` / `accel_fault_detected` | 0 / 0 | 0 / 0 |

**Per-IMU yaw rate** (`vehicle_imu`, Fig. 2):
- IMU0 (BMI088) is about 0°/s in flight.
- IMU1 (BMI270) is −2.47°/s mean in A and −1.77°/s mean in B. In B it decreases from about −2.5 to −1.2°/s over the flight.
- At rest before takeoff, both read about 0°/s, but that is only 3 samples, so it is weak evidence.
- The offset is therefore flight-dependent, not a static calibration error. Likely causes are vibration rectification or other in-flight sensitivity of the BMI270.
- IMU1 accel vibration is about 2× IMU0's (0.50 vs 0.23 m/s²). Absolute levels are low and there is no gyro clipping.

**Caveat on which IMU is "wrong":** in offboard, yaw was held on EKF0, so the BMI088 reads ≈ 0 by construction. The claim that the BMI270 is the faulty sensor rests on two facts: the two IMUs share one rigid board, and only one can be right. Confirming it needs external evidence. Video showing the airframe did not rotate about 100° in A would settle it.

## 3. Why the selector switched
- **Selector inputs:** `estimator_selector_status`. `combined_test_ratio[i]` comes from instance *i*'s innovation test ratios. `relative_test_ratio[1]` is EKF1's running score against the primary, clamped to ±1. It moves only when the gap exceeds `EKF2_SEL_ERR_RED` (0.2).
- **Before the switch:** EKF1 sat at +1 (slightly worse) for the whole flight.
- **At the switch:** within about 60 ms of an EKF0-only spike, the score fell below about −0.5 and the selector switched. The −0.5 threshold is inferred from the logged values (Fig. 5b panels).
- **The IMU fault check never fired:**
  - The fault check uses `sensors_status_imu.gyro_inconsistency_rad_s`, which is each IMU's deviation from the mean of the two, i.e. half the difference.
  - That metric stayed at about 1–1.5°/s, against a threshold of `EKF2_SEL_IMU_RAT` = 7°/s.
  - One spike to about 8°/s happened at the Flight A kill/impact (49 s). It was too short to accumulate past `EKF2_SEL_IMU_ANG` (15°).
- **FIFO overflows are not the cause (Fig. 6):**
  - They are BMI088 **accel** FIFO overflows: 1 in A at 16.9 s; 3 in B at 1.9, 23.8 and about 53 s.
  - Each happened at least 28 s before the switch, on the IMU whose heading stayed good.
  - The BMI088 gyro and the BMI270 had none. Perf counters show `ekf2: IMU message missed` ≤ 1.

## 4. Vehicle reaction (Fig. 5)
- **Flight A, airborne:**
  - At the switch, the published pose jumped 5.3 m and −105.9°.
  - The companion kept sending setpoints in the old frame (yaw 0°, position about (3.9, 1.0)). That produced a 105° yaw error and a 5.2 m position error. The controller yawed at up to **94°/s**.
  - AUTO_LAND engaged 0.29 s after the switch, at 45.73 s. Its trigger cannot be identified from the log. It re-anchored the setpoints to the current pose.
  - Total physical rotation was about +50°; both EKFs agree (Fig. 1).
  - Kill engaged at 49.1 s.
- **Flight B, already killed:** no control consequence. The odometry frame is shifted 6.9 m and rotated about 156° for the rest of the log.
- **Mission impact:** tag positions logged in odometry before and after a switch are in different frames. Even with no switch, EKF0's heading is unreferenced and drifts slowly.

## 5. Recommendations
1. Set `EKF2_MULTI_IMU=0` (single instance) and `SENS_IMU_MODE=1`. This removes switch jumps.
2. Run the EKF on the BMI088: lower the BMI270's priority (`CAL_GYRO1_PRIO`, `CAL_ACC1_PRIO`). Check the BMI270's mounting and damping.
3. Add a heading reference. Either:
   - an external I2C compass on a mast away from the AIO's ESC currents: `SYS_HAS_MAG=1`, `EKF2_MAG_TYPE=1` (heading-only), calibrate, and do a throttle-sweep interference test; or
   - EV yaw from the companion (`EKF2_EV_CTRL` bit 3) if the tags give an absolute orientation.
4. Companion: handle `ODOMETRY.reset_counter`, and re-anchor setpoints and tag coordinates on a reset.
5. Logging: set `SDLOG_PROFILE` bit 1 (estimator replay). Estimator topics were logged at only 2–5 Hz.
6. Verification: hover for 60 s with yaw hold, filmed from above, and compare `estimator_attitude` [0]/[1] yaw against video.

*Minor:* `EKF2_GPS_CTRL=7` and `EKF2_EV_CTRL=1` are enabled with no source present. This is harmless; clean it up.

## 6. Data sources (ULog topics)
- `estimator_attitude[0/1]`: heading per instance.
- `estimator_local_position[0/1]`: xy per instance.
- `estimator_sensor_bias[0/1]`: instance↔IMU mapping and gyro bias estimates.
- `vehicle_imu[0/1]`: per-IMU yaw rate.
- `estimator_selector_status`: selector scores and fault flags.
- `sensors_status_imu`: IMU inconsistency.
- `vehicle_local_position`: reset counters and deltas.
- `estimator_status`, `estimator_innovation_test_ratios`: innovation test ratios.
- `vehicle_attitude(_setpoint)`, `trajectory_setpoint`, `vehicle_angular_velocity`, `vehicle_status`: vehicle reaction.
- `vehicle_imu_status`: vibration and accel error count.
- `perf_counter_*`, `boot_console_output`: FIFO counters and IMU identification.

## 7. Figure list
| File | Shows |
|---|---|
| `fig1_heading_per_ekf.png` | Yaw of EKF0, EKF1 and the published estimate; switch and kill markers |
| `fig2_imu_yaw_rate.png` | Per-IMU yaw rate (5 s mean) and the IMU-inconsistency metric vs the 7°/s fault threshold |
| `fig3_xy_tracks.png` | Top-down EKF0/EKF1/published tracks with the jump arrow |
| `fig4_divergence.png` | Heading and xy divergence between instances vs time |
| `fig5a_switch_zoom_A.png`, `fig5b_switch_zoom_B.png` | ±3 s around the switch: selector inputs and score, published position vs setpoint, yaw vs setpoint, body yaw rate |
| `fig6_fifo_overflow_timing.png` | BMI088/BMI270 accel error count vs switch time |
