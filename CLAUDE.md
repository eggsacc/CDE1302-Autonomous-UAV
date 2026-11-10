# CLAUDE.md: project context for agents

## What this project is

Group documentation + code repo for the **CDE1302 (Intro to UAVs) mini project,
AY2026/27**. Mission: a small indoor quadcopter flies a lawnmower sweep over an
allocated rectangle and finds AprilTags (tag16h5, 120 mm) on the floor. It then
returns home and lands, reports each tag's position, and reconstructs the
flight route. The drone is pseudo-autonomous: the laptop plans the sweep and
detects tags over Wi-Fi, the drone flies the uploaded path itself, and an
operator starts it and holds the RC kill switch.

Beyond the course baseline, the group also **redesigned the drone frame**. The
course frame was meant for carbon fibre, but we print it in PLA, which made it
flexible. The redesign is in `mechanical/`; its source repo is
`github.com/eggsacc/CDE1302-drone-frame`.

## Repo layout

| Path | What | Origin |
|---|---|---|
| `README.md` | Short intro: mission, system + control block diagrams, links to subsystems | ours |
| `cde1302/main/` | ESP32-S3 firmware (ESP-IDF, FreeRTOS) | course template (`esp-everything-qgc-bridge`), modified by us |
| `cde1302/laptop/` | Course laptop tools: `comms.py`, `protocol.py`, `camera_stream.py`, `apriltag_host.py`, lab scripts | course. **Treat as read-only.** The mission imports it via `mission/laptop_path.py` |
| `cde1302/mission/` | **Our mission code**: `mission.py` (state machine), `planner.py`, `trajectory.py`, `tag_pipeline.py`, `tag_map.py`, `live_map.py`, `sim.py`, `config.yaml`, tests, `tools/` | ours |
| `cde1302/mission/logs/` | Per-run flight logs (gitignored; copy runs worth keeping elsewhere) | generated |
| `cde1302/trajectory/`, `cde1302/waypoints/` | Lab 5 trajectory / waypoint material | course |
| `cde1302/components/`, `managed_components/` | Vendored MAVLink, esp-apriltag, VL53L5CX, esp32-camera | third-party, don't edit |
| `mechanical/` | Frame redesign: `README.md` (design notes + print guide), `STEP/`, `STLs/`, `assets/` | ours |
| `electrical/` | Hardware architecture, wiring, network | ours |
| `UWB_documentation/` | LinkTrack UWB datasheets | reference only |

## Documentation rules (from the user)

- The root `README.md` stays a **short intro**: mission, block diagrams,
  subsystem links. Details go in each subsystem's own README.
- Image placeholders are written as `> **[Image placeholder]** <description>`.
  The user adds the images by hand, so don't invent image files.
- Confirm structural changes (new folders, moves, renames) with the user
  before writing.

## Hardware (see `electrical/README.md`)

PX4 flight controller with EKF2 fusing IMU + **MTF-02P optical flow +
rangefinder**. No compass, no GPS. Companion computer: **Seeed XIAO ESP32-S3
Sense** with OV3660 camera, linked to PX4 by MAVLink UART at 921600 (GPIO4 RX,
GPIO5 TX). Laptop and drone share a dedicated router.

**Not used:** LinkTrack **UWB** (the flight area is too small for it to help;
code exists in `uwb_*` files, `uwb.send_to_px4` should stay `false`) and the
**VL53L5CX ToF ring** (compiled out, `TOF_ENABLED 0`).

## Data flow and control

1. `mission.py` (laptop, 10 Hz tick) steps through `CONNECTING → READY →
   TAKING_OFF → HOVERING → EXPLORING → RETURNING → LANDING → DONE`, plus
   `ABORT_LANDING` from any airborne state on geofence, stale link or time limit.
2. `planner.py` builds lawnmower waypoints. `trajectory.py` turns them into a
   constant-speed trajectory, uploaded with `CMD_TRAJ_DATA` + `CMD_TRAJ_START`.
3. ESP32 `traj_gate.c` keeps the setpoint `CONFIG_TRAJ_LOOKAHEAD_CM` (25 cm)
   ahead of the drone along the path. It stops at each corner until within
   `CONFIG_TRAJ_CORNER_CM`. `mavlink_task.c` sends
   `SET_POSITION_TARGET_LOCAL_NED` to PX4 at 20 Hz. The heading never changes.
4. The camera stream (`camera_stream.c`) embeds the PX4 pose at capture time
   in a JPEG COM segment (`pose_hist.c`, parsed by `mission/frame_pose.py`).
5. `tag_pipeline.py` detects tags with `laptop/apriltag_host.py` (the
   firmware's own esp-apriltag, compiled for the laptop). It maps each one
   camera → body (`camera_mount.py`) → NED, giving a `Sighting`.
6. `tag_map.py` clusters sightings per ID (densest seed, median, outlier
   reject). A tag is confirmed after `min_sightings` sightings.
7. `mission_log.py` writes `mission/logs/<YYYYmmdd_HHMMSS>/`: `events.csv`,
   `trajectory.csv` (reconstructed route), `detections.csv`, `tags_final.csv`,
   `config_used.yaml`, and `map.png` (from `--map`).

## Conventions

- **Frames:** PX4 local NED, metres, origin = where EKF2 started (the takeoff
  spot). Yaw in radians, 0 = North, clockwise positive. Body frame: x forward,
  y right, z down.
- **Every mission number lives in `mission/config.yaml`.** `settings.py`
  validates it and rejects unknown keys. Firmware tunables are in
  `idf.py menuconfig` → *Drone Configuration* and need a reflash.
- **Wire formats must match on both sides:** `main/camera_stream.c` ↔
  `laptop/protocol.py`, the JPEG pose COM ↔ `mission/frame_pose.py`, detector
  gate in `main/at_detect.c` ↔ `laptop/apriltag_host.py`.
- Pure-C99 firmware modules (`traj_gate`, `pose_hist`, `nlink_parser`,
  `uwb_px4`) are compiled and tested on the laptop from `mission/tests/`
  (`test_*_c.py`).
- UDP ports: 5005 telemetry, 5006 commands, 5007 ToF debug, 5008 AprilTag
  debug, 5009 camera, 5010 UWB debug. QGC MAVLink bridge: 14550 out, 8888 in.
- Addresses: drone `192.168.1.(200+ID)`, laptop `192.168.1.(100+ID)`. Our
  drone is **ID 2**, so the drone is .202 and the laptop .102.

## Commands (run from `cde1302/`, Windows, conda env `cde1302-laptop`)

```bash
python -m pytest mission/tests -q              # all mission + C-module tests, no hardware
python mission/mission.py --sim --auto --map   # full mission in the simulator
python mission/mission.py --dry-run --map      # real drone: telemetry + tags, never flies
python mission/mission.py --map                # real flight: type `takeoff`, then `start`
python mission/planner.py --plot               # preview the sweep
python mission/tools/compare_flights.py --last 6 --plot drift.png
```

Flashing (PowerShell, **separate window**, because it swaps that window's Python):
`. .\tools\idf_pinned.ps1` then `idf.py -p COM18 build flash monitor`.
`flash_drone.sh` is macOS-only.

## Branches

| Branch | State |
|---|---|
| `main` | Still only the original course template; not yet merged |
| `claude-one-shot` | Main development line for the mission (trajectory sweep, frame-pose tagging) |
| `camera-fps` | Split off `claude-one-shot` at `240bccd`: faster OV3660 stream, **onboard detector removed**, full-resolution laptop detection. Not merged |
| `docs/restructure` | This documentation restructure, based on `claude-one-shot` |
| `uwb-testing`, `nikidudu`, `yaw_test_lab5` | Older experiments |

## Gotchas

- `cde1302/CLAUDE.md` and parts of `cde1302/README.md` come from the upstream
  course template. They describe a SAFMC swarm, onboard detection and ToF
  obstacle avoidance, which this project doesn't use. Trust `mission/` docs
  and this file over them.
- `CONFIG_HOST_IPV4_ADDR` in `cde1302/QGC_MAVLINK_BRIDGE.md` (192.168.50.x) is
  stale; the real value is in `sdkconfig`.
- Only one camera viewer can stream at a time. Don't run `tag_stream.py`
  alongside `mission.py`.
- Open TODOs: image placeholders across the docs; TODOs in
  `electrical/README.md` (FC model, motors, battery, power budget); duplicate
  `* - Copy.stl` files in `mechanical/STLs/`.
