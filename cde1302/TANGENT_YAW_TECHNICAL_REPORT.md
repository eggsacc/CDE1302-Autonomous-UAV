# Tangent-yaw implementation and verification report

Date: 26 October 2026 (Singapore). Target: Drone 2. Branch: `yaw_test_lab5`.
Base team commit: `222d9160c71642ae0f52e94fac27c971ca715b17`.

## Result

The software now carries an explicit unwrapped yaw reference from a separate CSV
through Python, versioned UDP, ESP32 PSRAM/interpolation and the existing MAVLink
position/velocity/yaw function. PX4 remains the flight controller. Fixed heading
remains the default and retains the original wire format and initial-heading hold.

This is a safe **offline-tested candidate with a known flight blocker**, not a
claim that the requested strict tangent mission can be flown at the current
limits. The original source CSV contains tiny direction reversals. Strict tangent
yaw requires 54.377437 rad/s at 2.13-2.14 s and 31.86-31.87 s. The 20 Hz float32
upload still requires 39.295540 rad/s at 2.10-2.15 s. Audited limit and firmware
ceiling: 1.0 rad/s; offline default: 0.6 rad/s. Rejection is intentional.

The user chose to preserve strict tangent and document this issue. No trajectory,
duration, gain, safety-limit or low-speed-exception change was made to avoid it.
The first defined heading is 80.041 degrees clockwise from North, so the +North
lab placement is also misaligned. Python/firmware require initial alignment;
neither commands an automatic turn.

## Evidence

| Check actually executed | Result |
|---|---|
| Full laptop regression suite | 102 passed, 47 skipped |
| Native compilation/execution of `main/traj_yaw.c` via Zig C compiler | Passed |
| Python uploader -> real C upload/ACK core with fake mission/network transport | Passed |
| Legacy packet layout and CSV resampling tests | Passed |
| Every original time/XYZ field vs new strict CSV | 3,401 rows match verbatim |
| Default uploaded fixed/tangent XYZ reference | Same 681 points and 34 s before yaw-limit rejection |
| Wrap crossing, stationary policy, source and wire rate/speed validation | Passed |
| Invalid length/version/ID/bounds, incomplete/corrupt upload, CRC, duplicates, cancellation, replay tests | Passed |
| Incompatible firmware cannot trigger ARM/TAKEOFF/LAND during preflight | Passed with fake transport |
| Drone-2 isolated config and generated header checks | Passed with file fixtures |
| Local ULog adapter/error maths/plot exports | Passed with explicitly synthetic fixtures |
| Original ellipse fixed-yaw CLI dry-run | Passed; no sockets created |
| Strict tangent ellipse CLI at 1 rad/s | Correctly refused before networking |
| MATLAB export from original CSV | Executed; same initial heading and 54.377437 rad/s peak |
| Full fixed/tangent Simulink comparison | Incomplete; stopped at user's request |
| Pinned ESP-IDF build | Not run; installation/toolchain unavailable |
| USB flashing, PX4 communication, actual flight | Not performed |

The 47 skips are existing AprilTag host tests requiring their supported GCC setup.
The separate yaw C tests did run. A pre-existing optional telemetry parser/test
mismatch was resolved by supporting an additive status trailer; legacy telemetry
still decodes and missing state stays unknown. No PX4 firmware changes were made.

## Integration details and remaining work

Yaw v1 packets cannot be confused with legacy XYZ packets. A capability ACK must
precede any arming; data is uploaded after takeoff because the existing mission
startup cancels stale navigation/upload state. Playback requires complete verified
data, a fresh armed Offboard state and heading within 5 degrees. Repeated START
does not reset time or replay a completed plan. CRC32 detects accidental corruption,
not malicious senders. Default chunk size is 971 bytes against a 1,024-byte limit.

Added storage is 38,400 PSRAM bytes plus a 300-byte bitmap and small state.
Original position interpolation, feedforward, hover-relative origin, MAVLink masks,
hold/landing, Wi-Fi loss handling and stale-setpoint watchdog remain. Tangent
playback adds fresh attitude/heartbeat and mode checks. Existing collision/ToF
limitations were observed, not modified; TA review is still necessary.

All source changes and the detailed file list are under `cde1302/`; unrelated team
files, original ellipse CSV, sdkconfig, Simulink model and controller sources are
unchanged. Local ULogs, telemetry and analysis outputs are ignored by Git.

Next software/hardware gate: the teammate obtains the exact handoff commit,
installs the pinned IDF commit, generates the separate Drone-2 configuration,
builds and verifies its generated header, and obtains TA review of board/network
settings and the specific build. Only then may the authorised teammate perform
deployment and props-off checks. Flight with the supplied strict tangent ellipse
remains blocked; any future resolution changes the experiment and requires user
approval plus TA review. No calibration or live execution on the development
laptop is needed.

See `README_TANGENT_YAW_EXPERIMENT.md` for Windows/Bash commands, logging,
compatibility checks, supervised validation gates and rollback.
