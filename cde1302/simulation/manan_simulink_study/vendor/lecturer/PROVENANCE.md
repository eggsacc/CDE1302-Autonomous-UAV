# Lecturer source provenance

The five original assets are preserved copies supplied by Dr Feng LIN:
https://github.com/linfengfrank/CDE1302_UAV at
`9d80ddab01b20b01a19cb849b5af5424ca0ae07a` (8 October 2026).
The source README describes NUSWARM calibration. This is not the team's PX4 firmware.

`verify_sources.m` records and checks exact SHA-256 values and byte counts.
The MATLAB files retain CRLF endings from the audited Windows checkout;
`.gitattributes` disables text conversion for this immutable directory.
The RPT model requires its two initialization scripts and `icon/pixhawk.png`.
FlightGear simulation blocks are active in the original even though its
FlightGear icon command is commented out. Optional PID and animation assets
are not needed for this reproduction.

The original circle ran in MATLAB R2026a: 15 s, 15001 finite position samples.
Recorded sample-mean 3D RMSE: 0.383743 m overall, 0.064710 m after 5 s;
final error: 0.011931 m. Its wrapped-yaw derivative reaches -627.518531 rad/s.
The study runner also reports time-weighted RMS, with a slightly different
initial-sample weighting. These are circle results, not mission results.

The circle starts with position, velocity and yaw mismatches and no takeoff
or landing sequence. Initial and final Down references are zero. No touchdown,
disarm, sensor fusion, or real-flight accuracy is validated by this example.

The lecturer supplied the vehicle, controllers and original example. The team
supplied the mission/planning/telemetry/AprilTag code used as a reference.
Manan's contribution is the experiment design, mission adaptation, measurements,
verification and interpretation. Codex assisted the audit and implementation;
the study does not claim he independently authored the lecturer's, teammates',
or Codex-generated source. See the top-level study README for adaptation details.
