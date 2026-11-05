# Autonomous AprilTag search mission

The drone takes off, flies a lawnmower sweep, records where each AprilTag is,
flies back to its takeoff spot and lands. It runs from a Windows laptop over
Wi-Fi. UWB can be added to stop the position drifting.

## Status

| Part | Status |
|---|---|
| Sweep with the lookahead target | **Flown and working.** Steady speed, fixed heading, accurate corners and return home |
| UWB sent to PX4 | **Flown and working**, but in some flights the drone pulled sideways at the far end of the area |
| Fix for that sideways pull (continuous UWB) | **Built, not flown yet** |

The fallback without the lookahead target is commit `cd06102`: reliable, but
slow and stop-go.

## 1. Setup (once per laptop)

From `cde1302/`.

**Laptop software:**

```bash
conda env create -f laptop/environment.yml
conda activate cde1302-laptop
conda install -c conda-forge gcc
```

**ESP-IDF** (needed to flash; about 1 GB download), in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1
```

**Network.** These must match:

| Device | Address |
|---|---|
| Wi-Fi router | `TP-Link_E95A`, 192.168.1.1 |
| Laptop | **192.168.1.102**, set as a manual IP on that Wi-Fi |
| Drone 2 (ESP32) | 192.168.1.202 |

The router has no internet. If the laptop keeps switching back to another
network, set that network to connect manually.

## 2. Flash the ESP32

1. Plug the ESP32 in over USB, and close QGroundControl, NAssistant and any
   serial monitor.
2. In PowerShell, from `cde1302/`:

```powershell
. .\tools\idf_pinned.ps1
idf.py -p COM18 build flash monitor
```

`COM18` may differ on your laptop; check Device Manager. Press `Ctrl+]` to leave
the monitor.

To change settings first, run `idf.py menuconfig` (save with `S`). Use a
**separate window** for the mission: `idf_pinned.ps1` changes that window's
Python.

Don't use `flash_drone.sh` on Windows. It's a Mac script.

## 3. PX4 parameters (QGroundControl)

Go to **Vehicle Setup → Parameters**, and search for each name. Write down the
old value before changing it. **Reboot PX4** after changing any of these.

### Always

| Parameter | Value | Why |
|---|---|---|
| `EKF2_MAG_TYPE` | None | No compass on this drone |
| `EKF2_OF_CTRL` | Enabled | Optical flow for position |
| `EKF2_HGT_REF` | Range sensor | Height from the rangefinder |
| `MPC_XY_VEL_MAX` | at least 0.3 | Otherwise PX4 limits the sweep speed |
| `MPC_XY_CRUISE` | at least 0.3 | Same |

### For UWB

| Parameter | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | **1** | Use UWB for x/y position only |
| `EKF2_EV_NOISE_MD` | **0** (EV reported variance) | PX4 trusts each reading by its precision |
| `EKF2_EVP_NOISE` | **0.15** | PX4 never trusts a reading more than 15 cm |
| `EKF2_EVP_GATE` | **3** | Rejects readings that disagree too much |
| `EKF2_EV_DELAY` | **50** | Delay of a UWB reading (ms) |
| `EKF2_EV_POS_X` | **-0.04** | Antenna 4 cm behind the flight controller |
| `EKF2_EV_POS_Y` | **0** | |
| `EKF2_EV_POS_Z` | **0.03** | Antenna 3 cm below it |

These can stay set when flying without UWB: with no UWB data arriving, they do
nothing.

### For logging UWB in the flight log (optional)

| Parameter | Value | Why |
|---|---|---|
| `SDLOG_PROFILE` | **129** (Default set + Computer Vision) | Records the UWB positions PX4 received |

## 4. Fly

**Before every flight:**

1. Put the drone on the floor, connect the battery and **don't touch it for
   30 s**.
2. Connect the laptop to the drone router, then check the link. Any
   `Request timed out` means don't fly:
   ```bash
   ping -n 30 192.168.1.202
   ```
3. Hand on the RC kill switch.

**Run the mission** in a window that hasn't run `idf_pinned.ps1`:

```bash
conda activate cde1302-laptop
python mission/mission.py --map
```

Type `takeoff`, then `start` once it says it's hovering. `land` lands,
Ctrl+C lands and exits.

Other ways to run it:

| Command | Use |
|---|---|
| `python mission/mission.py --sim --auto --map` | Simulator, no drone |
| `python mission/mission.py --dry-run --map` | Watch telemetry and camera, no flight commands |
| `python mission/mission.py --map --auto` | The search starts by itself after takeoff |

Each flight is saved in `mission/logs/<date>_<time>/`.

## 5. Flying with UWB

**Once:** set up the anchors in NAssistant (A0 is the origin), and set the
drone's module to role TAG, protocol NODE_FRAME2, 50 Hz, 921600 baud. The module
plugs into **D10** on the XIAO (GPIO 9).

**Switches:**

| What | UWB off | UWB on |
|---|---|---|
| Firmware (`menuconfig` → UWB (LinkTrack)) | Can stay on | UWB on, RX pin 9, **Send UWB position to PX4** on |
| `mission/config.yaml` → `uwb.send_to_px4` | `false` | `true` |
| PX4 parameters | | The UWB table above |

Keep `uwb.mirror: "yes"`, with the quotes.

**What happens in flight:** the drone starts the sweep on optical flow alone.
After about 0.8 m of flying, the laptop works out how UWB lines up with PX4
(`UWB_ALIGNED`) and sends that to the ESP32 (`UWB_ALIGN_CONFIRMED`). From then on
the ESP32 sends every UWB reading to PX4 by itself, and PX4 combines it with
flow. Expect one small nudge at that moment. If no alignment is found, the sweep
just carries on without UWB.

**Check it worked**, in the flight log in PlotJuggler or Flight Review:
`estimator_status_flags/cs_ev_pos` should stay at 1 once it turns on, and
`vehicle_local_position/xy_reset_counter` should barely rise. If they flicker or
climb, PX4 is restarting UWB, which causes sideways jumps.

**Bench test, props off** (optional):

```bash
python mission/tools/uwb_px4_bench.py           # fit only
python mission/tools/uwb_px4_bench.py --send    # also feed PX4
python mission/tools/uwb_px4_bench.py --clear   # stop feeding PX4
```

Carry the drone slowly in an L shape. **Reboot PX4 afterwards**
(QGroundControl MAVLink Console: `reboot`), because `--send` moves PX4's
position.

## 6. Comparing flights with and without UWB

With `uwb.record: true` (the default), every flight saves `uwb.csv`, even when
UWB isn't sent to PX4. Then:

```bash
python mission/tools/compare_flights.py --last 6 --plot drift.png
```

It prints, per flight: distance from the planned lanes, speed and how steady it
was, corner accuracy, return-home error, tags found, and how far PX4 drifted
from UWB. Add `--truth tags.csv` (columns `tag_id,forward,right`, tape-measured
from the takeoff spot) for real tag errors.

For a fair comparison, change only `uwb.send_to_px4` between flights.

## 7. Settings

`mission/config.yaml` (restart `mission.py`, no reflash):

| Setting | Now | Meaning |
|---|---|---|
| `area.x_max`, `area.y_max` | 3.0, 1.0 | Search area size (m) |
| `sweep.lane_spacing` | 0.5 | Distance between lanes (m) |
| `sweep.start_in_middle` | true | Takeoff spot is the middle of the area's start edge |
| `flight.cruise_speed_mps` | 0.15 | Planned speed, used for timeouts |
| `safety.stale_timeout_s` | 10 | Land if no telemetry for this long (s) |
| `uwb.send_to_px4` | false | Send UWB to PX4 |
| `uwb.record` | true | Save UWB readings in the flight log |

`idf.py menuconfig` → Drone Configuration (reflash needed):

| Setting | Now | Meaning |
|---|---|---|
| Trajectory: target distance ahead of the drone | 25 cm | Sets the speed: about 0.24 m/s |
| Trajectory: how close to get to each corner | 15 cm | Keep below the target distance |

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| `Access is denied` on COM18 | Close QGroundControl, NAssistant and serial monitors |
| `No module named 'numpy'` | Use a window that hasn't run `idf_pinned.ps1` |
| Can't connect, QGroundControl slow | Laptop on the wrong Wi-Fi, or the router is busy. Check IP and ping |
| `telemetry stale` or sweep won't start | Wi-Fi congestion. Move the router closer, fewer devices on it |
| Motors cut in flight | The ESP32 lost the router for over 3 s |
| Nose turns, map looks straight | Yaw gyro error: recalibrate the gyro warm, keep still at power-up |
| Jumps up over blocks | Height from the rangefinder. Try `EKF2_HGT_REF` = Baro, `EKF2_RNG_CTRL` = 0, hover test first |
| Very slow | Raise `MPC_XY_VEL_MAX` and `MPC_XY_CRUISE` |
| Pulls sideways with UWB | PX4 restarting UWB, or poor UWB coverage there. Check `cs_ev_pos` in the log, and the anchors' view of that area |
| `uwb.mirror must be text` | Write `"yes"` with quotes |

## More documents

- [BENCH_TEST_SETUP.md](BENCH_TEST_SETUP.md): bench test without flying
- [UWB_ESP32_BENCH_TEST.md](UWB_ESP32_BENCH_TEST.md): testing the UWB module and wiring
- [UWB_PX4_FLIGHT_TEST.md](UWB_PX4_FLIGHT_TEST.md): UWB to PX4 in detail
