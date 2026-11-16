# AGENTS.md

Guidance for coding agents working in `code/`. The repo-wide context (mission,
hardware, documentation rules) is in `../CLAUDE.md`. For a map of the folders,
see the tree in `README.md`.

## What this is

Software for a single indoor quadcopter (CDE1302 mini project). It flies a
lawnmower sweep over a rectangle, finds tag16h5 AprilTags on the floor,
returns home and lands. Then it reports each tag's position (drift-corrected
by localisation tags) and the route it flew.

- **Laptop (`mission/`, Python):** plans the sweep, runs the state machine and
  safety checks, detects tags in the camera stream and writes the run log.
- **ESP32-S3 (`main/`, ESP-IDF C):** flies the uploaded trajectory by sending
  position setpoints to PX4 over MAVLink UART, and streams JPEGs that carry the
  pose at capture time.
- **PX4:** EKF2 with IMU + optical flow + rangefinder. No GPS, no compass.

## Ownership: what you may edit

| Path | Status |
|---|---|
| `mission/` | Ours. Main place for changes |
| `main/` | Course template, heavily modified by us. Edit carefully; needs a reflash |
| `laptop/` | Course code. **Treat as read-only.** `mission/laptop_path.py` puts it on `sys.path` |
| `components/`, `managed_components/` | Vendored third-party. Don't edit (one deliberate exception below) |
| `trajectory/`, `waypoints/` | Course Lab 5 material, unused by the mission |

Present but not used in the final mission: UWB (`main/uwb_*`,
`main/nlink_parser.*`, `mission/uwb_*.py`, `mission/nlink.py`;
`uwb.send_to_px4` stays `false`; test procedures in `mission/UWB_*.md`); the ToF ring and VFH avoidance
(`TOF_ENABLED 0`); the course swarm scripts in `laptop/` (`run_*.py`,
`exploration.py`, `relay.py`, …). Don't remove them without asking.

## Data flow

1. `mission/mission.py` ticks at 10 Hz: `CONNECTING → READY → TAKING_OFF →
   HOVERING → EXPLORING → RETURNING → LANDING → DONE`, plus `ABORT_LANDING`
   from any airborne state on geofence, stale link or time limit.
2. `planner.py` makes lawnmower waypoints. `trajectory.py` turns them into a
   constant-speed path, uploaded with `CMD_TRAJ_DATA` + `CMD_TRAJ_START`.
3. On the ESP32, `nav_task.c` steps `traj_gate.c`, which places the setpoint
   along the path. The mode is chosen in menuconfig: **waypoints**
   (`CONFIG_TRAJ_WAYPOINTS`, current `sdkconfig`, 15 cm corners: target = next
   lane end), **follow** (`CONFIG_TRAJ_FEEDFORWARD`: velocity feedforward) or
   **carrot** (`CONFIG_TRAJ_LOOKAHEAD_CM` ahead). `mavlink_task.c` sends
   `SET_POSITION_TARGET_LOCAL_NED` at 20 Hz. The heading never changes.
4. `camera_stream.c` embeds the PX4 pose at capture time (`pose_hist.c`) in a
   JPEG COM segment, parsed by `mission/frame_pose.py`.
5. `tag_pipeline.py` detects tags with `laptop/apriltag_host.py` (the
   firmware's esp-apriltag compiled for the laptop) and maps them camera →
   body (`camera_mount.py`) → NED, giving a `Sighting`.
6. `tag_map.py` clusters sightings per ID (densest seed, median, outlier
   reject). Localisation tags (`localisation.tags` in `config.yaml`; 29 =
   home) go to `localisation.py` instead. It sets the landing spot and
   precision landing, aligns the sweep to the home tag, and at the report
   fits one PX4 → arena transform from every loc-tag visit that corrects the
   reported tags. It is never sent to PX4.
7. `mission_log.py` writes `mission/logs/<YYYYmmdd_HHMMSS>/`: `events.csv`,
   `trajectory.csv`, `detections.csv`, `tags_final.csv` (PX4 frame),
   `tags_corrected.csv`, `localisation.csv`, `trajectory_corrected.csv`,
   `correction.png`, `config_used.yaml`, and `map.png` (with `--map`).

## Conventions

- **Frames:** PX4 local NED in metres, origin where EKF2 started (the takeoff
  spot). Yaw in radians, 0 = North, clockwise positive. Body: x forward,
  y right, z down.
- **Every mission number lives in `mission/config.yaml`.** `settings.py`
  validates it and rejects unknown keys, so a new key needs a matching field in
  `settings.py`. Firmware tunables live in `main/Kconfig.projbuild`
  (`idf.py menuconfig` → Drone Configuration) and need a reflash.
- **Wire formats must match on both sides:** `main/wifi_task.*` and
  `main/camera_stream.c` ↔ `laptop/protocol.py`; the JPEG pose COM ↔
  `mission/frame_pose.py`; the detector gate in `main/at_detect.c` ↔
  `laptop/apriltag_host.py`.
- Pure-C99 firmware modules (`traj_gate`, `pose_hist`, `nlink_parser`,
  `uwb_px4`) are compiled and tested on the laptop by
  `mission/tests/test_*_c.py`, with mains in `mission/tests/host_c/`. Keep
  them free of ESP-IDF headers.
- Addresses: drone `192.168.1.(200+ID)`, laptop `192.168.1.(100+ID)`. Our
  drone is **ID 2** (`.202` / `.102`).
- UDP ports: 5005 telemetry, 5006 commands, 5007 ToF debug, 5008 AprilTag
  debug, 5009 camera, 5010 UWB debug. QGC MAVLink bridge: 14550 out, 8888 in.

## Commands

Run from `code/` on Windows in the conda env `cde1302-laptop` (Python 3.12,
MinGW-w64 gcc). The system Python is too old for the tests.

```bash
python -m pytest mission/tests -q              # all mission + C-module tests, no hardware
python mission/mission.py --sim --auto --map   # full mission in the simulator
python mission/mission.py --dry-run --map      # real drone: telemetry + tags, never flies
python mission/planner.py --plot               # preview the sweep
python mission/tools/relocalise.py mission/logs/<run> --plot   # redo the loc-tag correction
```

Firmware (PowerShell, in a **separate window**, because it swaps that
window's Python): `. .\tools\idf_pinned.ps1` then
`idf.py -p COM18 build flash monitor`. `flash_drone.sh` is macOS-only.

Never run a real flight (`mission.py` without `--sim`/`--dry-run`, or the
`laptop/simple_*` scripts) yourself. A person must be holding the RC kill
switch.

## Gotchas

- `components/esp-apriltag/apriltag/tag16h5.c` was deliberately edited: the
  course shipped it truncated to IDs 0-21, and IDs 22-29 were restored for the
  localisation tags (`EXPECTED_NCODES = 30` in `laptop/apriltag_host.py`). The
  laptop detector rebuilds itself into `laptop/.apriltag-host/`. It needs the
  conda env's gcc on PATH, because an old `C:\MinGW` gcc fails
  (`CONDITION_VARIABLE`).
- `mission/logs/` is gitignored except `full-exploration/` and
  `truncated-exploration-2/`, the two demo flights. `relocalise.py --plot` and
  `--write` overwrite files in the run folder they're given.
- Only one camera viewer can stream at a time. Don't run `tag_stream.py` or
  `camera_stream.py` alongside `mission.py`.
- `calibrate` and `laptop/check_px4_esp32_link.py` use UDP 14550, so
  QGroundControl must be closed.
- `CONFIG_HOST_IPV4_ADDR` in `QGC_MAVLINK_BRIDGE.md` (192.168.50.x) is stale;
  the real value is in `sdkconfig` (192.168.1.102).
- `laptop/` docs (`TEST_GUIDE.md`) use example drone 22 and `python3`; on
  this machine use drone 2 and `python`.
