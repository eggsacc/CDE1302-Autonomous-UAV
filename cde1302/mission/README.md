# Autonomous AprilTag search mission

The drone takes off, sweeps a lawnmower path, records where each AprilTag is in
PX4's local frame, returns to its takeoff spot and lands. Everything runs from a
Windows laptop over Wi-Fi.

## Status of this version

**The path is reliable and accurate, but the motion is slow and jittery.**

- **Not constant velocity.** The drone speeds up and slows down all along the
  lanes (typically 0.06–0.20 m/s), and it feels like "stop, go, stop, go".
  Expect a real speed of about 0.1 m/s.
- **Why:** the firmware's target stays very close to the drone, so small position
  wobbles change the speed. The target also slows down when the drone falls
  behind. This is expected here, not a fault.
- **What works well:** the drone flies every lane, stops accurately at corners
  and keeps a fixed heading. Sweep time depends on how fast the drone can go.

Other known limits:

| Limit | Effect |
|---|---|
| No compass | A yaw gyro error makes the drone turn while the map still looks straight. See [Troubleshooting](#troubleshooting). |
| Height from the rangefinder | Blocks on the floor make the drone jump. See [Troubleshooting](#troubleshooting). |
| Camera looks straight down | Wall tags can't be seen, so `tags.expected_count` may never be reached. The sweep then just runs to its end. |
| Tag positions | They rely on PX4's position, so PX4 drift moves them too. Sightings near the image edge are less accurate. |
| UWB | Built but **off**, and not tested in flight. See [UWB_PX4_FLIGHT_TEST.md](UWB_PX4_FLIGHT_TEST.md). |

## Setup

### Addresses

| Device | Value |
|---|---|
| Wi-Fi router | `TP-Link_E95A`, 192.168.1.1 |
| Laptop | **192.168.1.102** (manual IP on that Wi-Fi) |
| Drone 2 ESP32 | 192.168.1.202 |
| USB port | `COM18` here; check Device Manager |

These must match. `sdkconfig` sets the drone's side, and `drone_id` in
`mission/config.yaml` must be 2.

### Laptop (once)

```bash
conda env create -f laptop/environment.yml
conda activate cde1302-laptop
conda install -c conda-forge gcc
```

### ESP-IDF (once)

The firmware needs the ESP-IDF version pinned in `tools/idf-pin.env`. The
released 6.0 does not build the camera library. From `cde1302/` in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1
```

It downloads about 1 GB and uses about 3 GB of disk.

## Flash the ESP32

1. Plug the ESP32 in over USB.
2. Close QGroundControl, NAssistant and any serial monitor. They can hold the
   port and cause `Access is denied`.
3. In PowerShell, from `cde1302/`:

```powershell
. .\tools\idf_pinned.ps1
idf.py -p COM18 build flash monitor
```

Press `Ctrl+]` to leave the monitor. The first build takes several minutes.

To change drone ID, IPs, Wi-Fi or trajectory settings, run `idf.py menuconfig`
before building, and save with `S`.

**Do not use `flash_drone.sh` on Windows.** It's a Mac script, and it sets the
drone ID from its argument.

`idf_pinned.ps1` changes which Python that window uses, so **use a separate
window for the mission**. Otherwise you get `No module named 'numpy'`.

## Fly

Before flying:

1. Put the drone on the floor, connect the battery and **don't touch it for 30 s**.
2. Connect the laptop to the drone router. It has no internet.
3. Check the link. Any `Request timed out` means don't fly:

```bash
ping -n 30 192.168.1.202
```

Then, in a window that hasn't run `idf_pinned.ps1`, from `cde1302/`:

```bash
conda activate cde1302-laptop
python mission/mission.py --map
```

Type `takeoff`, then `start` once it says it's hovering. Other commands: `land`,
`status`, `quit`. Ctrl+C lands and exits.

| Command | Use |
|---|---|
| `python mission/mission.py --sim --auto` | Simulator, no drone. Add `--map` to watch it. |
| `python mission/mission.py --dry-run --map` | Watch telemetry and camera. Sends no flight commands. |
| `python mission/mission.py --map --auto` | The search starts by itself after takeoff. |
| `python -m pytest mission/tests -q` | Laptop tests, about 1 minute. |

Each flight is saved in `mission/logs/<date>_<time>/`: events, trajectory, tag
detections, final tags, the config used and the map.

## How the flight works

The laptop uploads the whole path once. After that the ESP32 feeds PX4 one target
at a time over its wire, so Wi-Fi gaps don't stop the flight. PX4 has no idea
there is a path: in offboard mode it only flies toward the newest target.

The firmware moves the target along the path, and it checks PX4's position:

- **On lanes:** the target slows from 24 cm behind and stops at 40 cm.
- **At corners and the last point:** the target stops until the drone is within
  20 cm.

Heading stays as it was at sweep start.

## Settings

In `mission/config.yaml` (no reflash, restart `mission.py`):

| Setting | Now | Meaning |
|---|---|---|
| `flight.cruise_speed_mps` | 0.12 | Commanded speed. Above what the drone can reach, it surges more. |
| `flight.traj_dt_s` | 0.20 | Time between path points. Fewer points mean a safer upload. |
| `flight.traj_upload_gap_s` | 0.06 | Gap between upload packets. |
| `flight.traj_timeout_factor` | 4.0 | Allowed time as a multiple of the planned time. |
| `safety.stale_timeout_s` | 10 | Abort if the laptop gets no telemetry for this long. |
| `area.*`, `sweep.lane_spacing` | | Search area size and lane spacing. |
| `sweep.start_at_home` | true | The sweep starts from the takeoff spot. |
| `sweep.start_in_middle` | true | Takeoff spot is the middle of the area's start edge. The drone first flies sideways to lane 1. |
| `sweep.camera_offset_m` | 0 | Shift for a tilted camera. 0 for straight down. |
| `tags.camera_pitch_deg` | 90 | Camera tilt: 90 = straight down, 45 = old mount. |

In `idf.py menuconfig` → Drone Configuration (needs a reflash):

| Setting | Now | Meaning |
|---|---|---|
| Trajectory: setpoint stops... | 40 cm | Lane gap. 0 = run on the clock, as before. |
| Trajectory: how close to get to each corner | 20 cm | Corner accuracy. Lower is more accurate but slower. |

PX4 parameters in QGroundControl that matter most: `MPC_XY_VEL_MAX`,
`MPC_XY_CRUISE`, `MPC_XY_P`, `MPC_TKO_RAMP_T`, `MPC_Z_P`. Write down the old value
before changing one, and change one at a time.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Access is denied` on COM18 | Another program has the port | Close QGroundControl, NAssistant and monitors |
| `No module named 'numpy'` | `idf_pinned.ps1` ran in this window | Use a new window |
| `idf.py` not recognised | ESP-IDF not active | `. .\tools\idf_pinned.ps1` |
| Can't connect, QGroundControl slow | Laptop on the wrong Wi-Fi, or router overloaded | Check Wi-Fi and IP, then ping. Keep the router close |
| `telemetry stale` abort, `dropped` high | Wi-Fi congestion | Fewer devices on the router, or a lower `camera.fps` and `camera.quality` |
| `sweep: not started after 5 attempts` | Upload packets lost | Same as above |
| `TRAJ_END sweep: stopped` | Firmware left flying: RC, disarm or lost router | Read the serial log. **Losing the router for over 3 s cuts the motors** |
| Nose turns in flight, map stays straight | Yaw gyro error | Recalibrate the gyro warm in QGroundControl, keep still at power-up, check the FC mount |
| Jumps up over blocks | Height comes from the rangefinder | Try `EKF2_HGT_REF` = baro, `EKF2_RNG_CTRL` = 0. Hover test first |
| Very slow | PX4 speed limit | Check `MPC_XY_VEL_MAX` and `MPC_XY_CRUISE` |

## More documents

- [BENCH_TEST_SETUP.md](BENCH_TEST_SETUP.md): dry-run bench test
- [UWB_ESP32_BENCH_TEST.md](UWB_ESP32_BENCH_TEST.md): UWB module to ESP32 test
- [UWB_PX4_FLIGHT_TEST.md](UWB_PX4_FLIGHT_TEST.md): UWB to PX4, not yet flown
