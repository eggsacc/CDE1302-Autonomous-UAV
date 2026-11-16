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
├── mission/          Our AprilTag search mission (laptop, Python)
│   ├── mission.py        Entry point and state machine
│   ├── config.yaml       Every mission setting
│   ├── localisation.py   Localisation-tag drift correction and precision landing
│   ├── tag_pipeline.py   Camera frames → tag positions
│   ├── tools/            Offline tools (redo the correction from a log, place loc tags, camera bench)
│   ├── tests/            pytest suite, including the firmware's C modules
│   ├── logs/             The two demo flights' run logs
│   └── px4_logs/         The two demo flights' PX4 logs (.ulg)
├── main/             ESP32-S3 firmware (ESP-IDF, FreeRTOS)
│   ├── traj_gate.c       Path following: where the setpoint goes next
│   ├── mavlink_task.c    Link to PX4 and the QGroundControl bridge
│   └── camera_stream.c   Camera stream, each frame tagged with the drone's pose
├── laptop/           Course-provided laptop tools (comms, camera viewers, detector, Lab 5 scripts)
├── components/       Vendored libraries: MAVLink, esp-apriltag
├── tools/            Pinned ESP-IDF version and install scripts
└── trajectory/, waypoints/   Course Lab 5 material
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
8888. Close QGC before `check_px4_esp32_link.py`, which uses the same port.

## Run the mission

Full procedure, PX4 parameters and troubleshooting:
[`mission/README.md`](mission/README.md).

```bash
python mission/mission.py --sim --auto --map   # simulator, no drone
python mission/mission.py --map                # real flight: type `takeoff`, then `start`
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
