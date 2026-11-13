## X. Anomaly: Sudden Odometry Jump (Flights A and B)

### X.1 Observation
In two test flights, the published odometry jumped abruptly: by 5.3 m and −106° in heading in Flight A while airborne, and by 6.9 m and +157° in Flight B at touchdown. Log analysis shows that neither jump came from a sensor fault. Both were caused by PX4's estimator selector switching between two parallel state estimates that had drifted apart (Fig. 3).

### X.2 System context
The flight controller (MicoAir743v2-AIO) carries two IMUs, a BMI088 and a BMI270. By default, PX4 runs one EKF instance per IMU (EKF0 on the BMI088, EKF1 on the BMI270) and publishes whichever instance it currently rates as healthiest. The vehicle's only horizontal aiding is a downward optical flow sensor with a rangefinder. The AIO variant has **no magnetometer**, and no GPS or vision heading was used. The estimator therefore had **no absolute heading reference**.

### X.3 Root cause: unobservable yaw and gyro drift
Accelerometers sense gravity, so roll and pitch are corrected continuously. Rotation about the gravity vector (yaw) produces no accelerometer signal. Without a compass or other heading source, each EKF can only integrate its gyro's yaw rate. Any rate error therefore accumulates directly into heading and cannot be estimated or removed.

During flight, the BMI270 reported a yaw rate about 2°/s lower than the BMI088 (−2.5°/s mean in Flight A, −1.8°/s in Flight B; Fig. 2). Both sensors read near zero at rest, which suggests a flight-dependent error such as vibration sensitivity rather than a calibration offset. The BMI270 is a consumer-grade IMU, while the BMI088 is designed for vibration-robust operation in UAVs. Integrated over the flight, this offset caused EKF1's heading to diverge from EKF0's by −105° (Flight A, 45 s) and about −205° (Flight B, 117 s) (Fig. 1, Fig. 4).

Because optical flow measures velocity in the body frame, each EKF rotates the same flow measurement into world coordinates using its own heading. The heading error therefore produced a growing position discrepancy: 5.4 m and 6.9 m at the time of the switch (Fig. 3, Fig. 4). The heading error is also invisible in the flow innovations, since each instance remains self-consistent. To PX4's health monitoring, both estimates appeared equally valid.

### X.4 Trigger: estimator selection switch
PX4 ranks instances only by their innovation test ratios. The IMU-consistency check flags a gyro only when the IMUs disagree by more than 7°/s. The observed inconsistency metric stayed at about 1–1.5°/s, so no fault was raised (Fig. 2).

The switch was triggered by a brief (≈60 ms) innovation transient on EKF0: the onset of descent in Flight A, and the touchdown impact in Flight B. This transient pushed the selector's relative score past its switching threshold (Fig. 5a, 5b). PX4 then adopted EKF1's state and published the 5–7 m and 105–157° difference as a position and heading reset.

Occasional BMI088 accelerometer FIFO overflows were also found in both logs. They occurred at least 28 s before either switch, on the IMU whose heading remained consistent, so they are ruled out as the trigger (Fig. 6).

### X.5 Consequences
In Flight A, the companion computer continued to send setpoints in the pre-switch frame. This produced a sudden 105° yaw error and a 5 m position error. The vehicle yawed at up to 94°/s before an automatic landing re-anchored the setpoints, and the flight was terminated with the kill switch (Fig. 5a). In Flight B, the switch occurred after the kill switch was engaged and had no control impact. In both cases, any tag positions recorded in the odometry frame before and after the switch are not directly comparable.

### X.6 Corrective actions
1. **Run a single EKF instance on the BMI088** (`EKF2_MULTI_IMU=0`, BMI270 deprioritised). This eliminates instance-switch jumps.
2. **Add an absolute heading reference.** Options are an external magnetometer mounted away from the AIO's power stage (heading-only fusion, `EKF2_MAG_TYPE=1`), or vision-derived yaw from the tag-detection pipeline.
3. **Handle estimator resets on the companion.** Monitor the odometry reset counter and re-anchor setpoints and recorded tag coordinates when it changes.
4. **Increase estimator logging rate** (`SDLOG_PROFILE` estimator-replay bit) and **verify the residual heading drift of the BMI088** against overhead video in a 60 s hover test.

### X.7 Limitations
The conclusion that the BMI270 (not the BMI088) carries the yaw-rate error rests on the two IMUs sharing a rigid mount, together with the vehicle holding heading on the BMI088 estimate. It has not yet been independently verified against an external heading reference. Estimator topics were logged at only 2–5 Hz, which limits timing resolution of short transients.

**Figure captions**
- **Fig. 1.** Yaw estimate of each EKF instance and of the published attitude. EKF1 (BMI270) drifts steadily with no heading reference; the published yaw jumps when the selector switches.
- **Fig. 2.** Top: yaw rate measured by each IMU (5 s moving mean). Bottom: PX4's IMU-inconsistency metric, which stays well below the 7°/s fault threshold.
- **Fig. 3.** Top-down position tracks of both instances and the published odometry. The arrow marks the jump at the switch.
- **Fig. 4.** Heading and horizontal-position divergence between the two instances, growing until the switch.
- **Fig. 5a/5b.** ±3 s around the switch: (a) selector innovation test ratios, (b) EKF1 relative score vs switch threshold, (c) published position and setpoint, (d) published yaw and yaw setpoint, (e) measured body yaw rate.
- **Fig. 6.** Accelerometer error (FIFO overflow) count per IMU vs time. Overflows on the BMI088 precede the switch by ≥ 28 s.
