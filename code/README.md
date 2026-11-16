# Software

All the software for the CDE1302 AprilTag search drone: the ESP32-S3
companion-computer firmware, the course's laptop tools, and our search
mission. The project overview is in the [root README](../README.md).

To fly the mission, read [`mission/README.md`](mission/README.md).

Started from the course template `esp-everything-qgc-bridge`. `main/` was
extended by us, `laptop/` is course code kept as-is, and `mission/` is ours.

## Where to find things

```
code/
├── main/                  ESP32-S3 firmware (ESP-IDF, FreeRTOS)
│   ├── main.c             Startup, task creation, takeoff/landing state machine (mission_task)
│   ├── mavlink_task.c     UART link to PX4: telemetry, setpoints at 20 Hz, QGC MAVLink bridge
│   ├── wifi_task.c        UDP telemetry to the laptop (5005) and commands from it (5006)
│   ├── nav_task.c         Runs an uploaded trajectory through traj_gate (also course VFH code, unused)
│   ├── traj_gate.c        Where the setpoint sits along the path: waypoint / follow / carrot modes
│   ├── camera_stream.c    JPEG stream to the laptop (5009), each frame tagged with the PX4 pose
│   ├── pose_hist.c        Pose history, so a frame gets the pose at its capture time
│   ├── at_detect.c        Onboard AprilTag detector (debug only; starts the camera stream)
│   ├── uwb_*.c, nlink_parser.c   LinkTrack UWB support (built, not used in the final mission)
│   ├── tof_task.c, vfh.c, odom.c Course template: ToF ring + obstacle avoidance (compiled out)
│   └── Kconfig.projbuild  `idf.py menuconfig` → Drone Configuration (drone ID, trajectory mode, UWB, camera)
│
├── mission/               Our AprilTag search mission (laptop, Python)
│   ├── mission.py         Entry point: state machine CONNECTING → … → EXPLORING → RETURNING → LANDING
│   ├── config.yaml        Every mission number (area, sweep, speeds, safety, camera, localisation tags)
│   ├── settings.py        Loads and validates config.yaml
│   ├── planner.py         Lawnmower (boustrophedon) waypoints
│   ├── trajectory.py      Waypoints → constant-speed trajectory for the firmware
│   ├── drone_link.py      Telemetry, commands and pose history for the real drone
│   ├── tag_pipeline.py    Camera frames → AprilTag detections → Sightings in PX4 NED
│   ├── camera_mount.py, frame_pose.py   Camera → body transform; pose embedded in each JPEG
│   ├── tag_map.py         Clusters sightings into one position per tag
│   ├── localisation.py    Localisation-tag drift correction (fit PX4 → arena transform) + precision landing
│   ├── correction_plot.py Before/after picture of that correction (correction.png)
│   ├── gyro_cal.py        Gyro calibration from the laptop (`calibrate` command)
│   ├── mission_log.py     Writes the per-run log folder
│   ├── live_map.py        Live 2-D map window (`--map`)
│   ├── sim.py, synth_camera.py   Simulator: fake drone + rendered camera frames
│   ├── uwb_*.py, nlink.py UWB alignment and decoding (not used in the final mission)
│   ├── tools/             relocalise.py (redo the correction from a log), loc_layout.py (place loc tags),
│   │                      cam_bench.py (camera calibration), compare_flights.py, UWB bench tools
│   ├── tests/             pytest suite; test_*_c.py compile the C firmware modules and test them on the laptop
│   ├── logs/              Run logs. Tracked: full-exploration/ and truncated-exploration-2/ (the demo flights)
│   └── px4_logs/          PX4 flight logs (.ulg) of the two demo flights
│
├── laptop/                Course-provided laptop tools (kept as-is; mission/ imports from it)
│   ├── comms.py, protocol.py      UDP link and wire format shared with the firmware
│   ├── apriltag_host.py           The firmware's own esp-apriltag detector, compiled for the laptop
│   ├── camera_stream.py, tag_stream.py   Live camera viewers (connection checks)
│   ├── simple_*.py, send_trajectory.py   Lab 5 flight scripts
│   └── run_*.py, exploration.py, relay.py, …   Course swarm template, not used by us
│
├── components/            Vendored third-party: MAVLink, esp-apriltag (tag16h5 restored to IDs 0-29), VL53L5CX
├── managed_components/    ESP-IDF component manager deps (esp32-camera, esp_jpeg)
├── tools/                 Pinned ESP-IDF version and Windows install/activate scripts
├── trajectory/            Lab 5: MATLAB trajectory generators and CSVs
├── waypoints/             Lab 5: waypoint files and test guide
├── sdkconfig              Firmware configuration as flashed on our drone
└── flash_drone.sh         macOS build + flash script (course)
```

Quick pointers:

| Looking for | Go to |
|---|---|
| Mission logic and safety checks | `mission/mission.py` |
| Localisation-tag correction algorithm | `mission/localisation.py` (`solve_fixes`, `fit_transform`, `drift_correction`) |
| How a tag's position is computed | `mission/tag_pipeline.py`, `mission/camera_mount.py` |
| How the drone follows the path | `main/traj_gate.c`, `main/nav_task.c` |
| Demo flight results | `mission/logs/full-exploration/`, `mission/logs/truncated-exploration-2/` |
| Tunable numbers | `mission/config.yaml`; firmware: `idf.py menuconfig` |

## Addresses

Our drone is **ID 2**. The ESP32 is `192.168.1.202` and sends everything to
the laptop at `192.168.1.102`, so the laptop must use that IP on the drone
router. In general: drone `192.168.1.(200 + ID)`, laptop
`192.168.1.(100 + ID)`.

UDP ports: 5005 telemetry, 5006 commands, 5008 AprilTag debug, 5009 camera,
5010 UWB debug. QGroundControl bridge: 14550 (to the laptop), 8888 (to the
drone).

## Build and flash the firmware

Only needed after changing `main/` or menuconfig settings. Plug the ESP32 in
over USB and close QGroundControl and any serial monitor first.

**Windows** (PowerShell, from `code/`). Use a separate window, since the script
changes that window's Python:

```powershell
powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1   # once, if ESP-IDF isn't installed
. .\tools\idf_pinned.ps1
idf.py menuconfig                    # optional: Drone Configuration, save with S
idf.py -p COM18 build flash monitor  # COM port from Device Manager; Ctrl+] leaves the monitor
```

**macOS:** `./flash_drone.sh 2 /dev/tty.usbmodem2101`. It sets the drone ID
and laptop IP, builds, flashes, then restores `sdkconfig`.

The ESP-IDF version is pinned in `tools/idf-pin.env`; the scripts refuse a
different one.

## Connection checks

Run from `code/` with `conda activate cde1302-laptop`. None of these fly.

```bash
ping -n 30 192.168.1.202                                          # Wi-Fi link; any timeout = don't fly
python laptop/simple_arm_takeoff_land.py --monitor-only --drone-id 2   # telemetry heartbeat for 15 s
python laptop/check_px4_esp32_link.py --drone-id 2                # PX4 ↔ ESP32 MAVLink health (close QGC first)
python laptop/camera_stream.py --esp-ip 192.168.1.202             # camera; s saves a frame, q quits
python laptop/tag_stream.py --esp-ip 192.168.1.202                # camera with AprilTags drawn on
python mission/mission.py --dry-run --map                         # whole mission pipeline, never flies
```

`--monitor-only` should print `Connected: drone=2 ...` and then `MONITOR`
lines with `age` well under 1 s. Only one camera viewer can run at a time, so
close these before starting `mission.py`.

**QGroundControl over Wi-Fi:** the ESP32 forwards PX4's MAVLink. Add a UDP
link in QGC: listening port 14550, target host `192.168.1.202`, target port
8888. Close QGC before `calibrate` or `check_px4_esp32_link.py`, which use the
same port.

## Run the mission

Full procedure, PX4 parameters and troubleshooting:
[`mission/README.md`](mission/README.md).

```bash
python mission/mission.py --sim --auto --map   # simulator, no drone
python mission/mission.py --map                # real flight: `calibrate`, `takeoff`, then `start`
python mission/tools/relocalise.py mission/logs/<run> --plot   # redo the drift correction from a log
python mission/planner.py --plot               # preview the sweep
```

## Tests

No hardware needed:

```bash
python -m pytest mission/tests -q
```

The `test_*_c.py` tests compile the firmware's C modules (`traj_gate`,
`pose_hist`, `nlink_parser`, `uwb_px4`) with the conda env's gcc.

## Course lab scripts (Lab 5)

Kept from the course template. Each asks for confirmation (`ARM-2`,
`MISSION-2`, `TRAJ-2`) before flying.

```bash
python laptop/simple_arm_takeoff_land.py --drone-id 2      # take off, hover, land
python laptop/simple_waypoint_mission.py --waypoints-file waypoints/waypoints_example.txt --confirm --drone-id 2
python laptop/send_trajectory.py --takeoff --takeoff-wait 12 --trajectory trajectory/ellipse_min_snap_traj.csv --confirm --drone-id 2
```

Details: [`laptop/TEST_GUIDE.md`](laptop/TEST_GUIDE.md),
[`waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md`](waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md),
[`trajectory/README.md`](trajectory/README.md).
