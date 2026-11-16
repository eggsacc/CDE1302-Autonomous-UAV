# Autonomous AprilTag search mission

The drone takes off, flies a lawnmower sweep, records where each AprilTag is,
flies back to its takeoff spot and lands. It runs from a Windows laptop over
Wi-Fi. UWB can be added to stop the position drifting.

## Status

| Part | Status |
|---|---|
| Sweep with the lookahead target | **Flown and working.** Steady speed, fixed heading, accurate corners and return home. A bit jerky: it slows into every corner and surges after |
| Sweep in follow mode (velocity feedforward) | **Built and tested on the laptop, not flown yet.** Now the firmware default |
| UWB sent to PX4 | **Flown and working**, but in some flights the drone pulled sideways at the far end of the area |
| Fix for that sideways pull (continuous UWB) | **Built, not flown yet** |
| Localisation tags: drift correction of the report + precision landing | **Built and tested in the simulator, not flown yet** |

The fallback without the lookahead target is commit `cd06102`: reliable, but
slow and stop-go.

## 1. Setup (once per laptop)

From `cde1302/`.

**Laptop software:**

```bash
conda env create -f laptop/environment.yml
conda activate cde1302-laptop
conda install -c conda-forge gcc
pip install pymavlink            # for `calibrate` (gyro calibration from the laptop)
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

With the drone on the floor and **perfectly still**, type `calibrate`: PX4
calibrates its gyros through the ESP32's MAVLink bridge (no QGroundControl
needed; QGC must be **closed**, it uses the same port 14550). Wait for
`GYRO_CAL_DONE` (it reports the old and new gyro offsets), about 5-8 s. Best after the drone has been powered for 1-2
minutes, so the gyros are warm. `safety.require_gyro_cal: true` makes
`takeoff` refuse until this has succeeded.

Then type `takeoff`, and `start` once it says it's hovering. `land` lands,
Ctrl+C lands and exits.

Other ways to run it:

| Command | Use |
|---|---|
| `python mission/mission.py --sim --auto --map` | Simulator, no drone |
| `python mission/mission.py --dry-run --map` | Watch telemetry and camera, no flight commands |
| `python mission/mission.py --map --auto` | The search starts by itself after takeoff |

Each flight is saved in `mission/logs/<date>_<time>/`.

## 5. Localisation tags

Three extra tag16h5 tags at measured spots on the floor give the drone's
drifting PX4 frame something fixed to compare against. They use the last IDs
of the family, so they never clash with the course's tags from the front:

| ID | Where | Used for |
|---|---|---|
| **29** | Centre of the start box. Put the drone on it, facing along the search length | Landing spot, drift fix at start and end |
| **27** | Near the far end of the search length, `y_min` side, inside the area | Drift fix |
| **28** | Near the far end of the search length, `y_max` side, inside the area | Drift fix |

Print them at the same 120 mm black-square size as the course tags, from the
full tag16h5 family (the course sheet only goes up to 21).

**Before the flight**, put the tags on grid nodes and click where they are:

```bash
python mission/tools/loc_layout.py
```

The window shows the floor grid (nodes 0.46 m apart, seen from behind the
drone: start at the bottom, facing up). Pick an ID (22-29), click its node;
right-click removes it. The tag on START (0, 0) is the home tag. **Save to
config** writes `localisation.tags` and `home_tag_id` into `config.yaml`
(the old file is kept as `config.yaml.bak`). Node (a, b) is x = b × 0.46 m
forward, y = a × 0.46 m to the right. More tags spread over the area give the
correction more to work with; keep them under a lane so the camera sees them.

**What happens:**

- **Takeoff:** the home tag seen while climbing and hovering becomes the
  landing spot (`HOME_TAG` event), instead of wherever the drone was put down.
- **Landing:** back over home, if the home tag is now more than
  `flight.land_radius_m` from the landing spot (drift during the flight), the
  drone is moved over it before landing (`LAND_TARGET`, at most
  `land_max_corrections` times). If the tag isn't in view it lands as before.
- **Report:** each time a localisation tag was in view is a *visit*. Every
  visit gives one "PX4 says here, really there" pair. One transform
  (turn + scale + shift) is fitted to all of them and applied to the found
  tags. The terminal shows a second table, `CORRECTED TAG REPORT`, and the run
  folder gets `tags_corrected.csv` and `localisation.csv` (each visit and its
  residual). `tags_final.csv` stays in the raw PX4 frame.

None of this goes to PX4 as position: the drone only flies differently by the
alignment of the plan before the sweep and the small move over the home tag
before landing. Turn off with `localisation.enabled: false`.

- **Alignment, before the sweep** (`align_to_home_tag`): PX4 has no compass,
  so its x axis is wherever the nose pointed at power-up, and 3° off square is
  about 25 cm off at the far end. While hovering, the drone measures how tag 29
  is turned in the camera image and turns (and shifts) the whole sweep to match
  (`ALIGNED` event). Tag 29 must be taped square to the tiles; any side can face
  forward. Turns over `align_max_deg` (15°) are ignored as wrong readings.

**Before/after picture**: saved automatically after every flight (after
landing) as `correction.png` in the run folder, with `trajectory_corrected.csv`.
It shows the recorded vs corrected route and tags, every tag labelled with its
corrected position and when it was seen (loc tags: each visit), and, for test
flights with `localisation.truth_tags` filled in, the error before/after. To
redo it from the log (e.g. after re-measuring the loc tags):

```bash
python mission/tools/relocalise.py mission/logs/<run> --truth "2:1.3,-1.35 4:2.76,0.92" --plot
```

It saves `correction.png` in the run folder. `--write` also saves
`trajectory_corrected.csv` (the route in the arena frame), and rewrites
`tags_corrected.csv` and `localisation.csv`. If the tag positions were measured
wrong, fix `config.yaml` and add `--config mission/config.yaml`.

Try it in the simulator by setting `sim.drift_scale`, `drift_rot_deg`,
`drift_x_mps` and `drift_y_mps` (for example 0.05, 3, 0.0004, -0.0004).

### Camera bench calibration

Checks how accurately the camera places a tag, and works out the fix. No
flying (props off): it only streams the camera. Close `mission.py` first.

```bash
python mission/tools/cam_bench.py
```

Put the drone on a stand, camera down, **level**, its centre over a grid node.
Measure the lens height above the floor and type `h 0.80` (your number). Put a
tag on a node and type where it is: `n 1 0` = 1 node right, `n 0 -1` = 1 node
back (or `c <forward> <right>` in metres). Do 5-6 spots spread over the view
(centre, forward, back, left, right, a corner), then `fit`. It reports the
tag size the camera implies (compare with your ruler), the lens focal lengths,
where the lens really sits, and whether the camera is twisted. `apply` writes
`tags.tag_size_m`, `tags.camera_forward_m` / `_right_m` and `camera.fx` / `fy`
into `config.yaml` (old file: `config.yaml.bak`). Enter alone shows the live
reading.

## 6. Flying with UWB

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

## 7. Comparing flights with and without UWB

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

## 8. Settings

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
| `localisation.tags` | 29 at (0, 0), 27 and 28 placeholders | Measured positions of the localisation tags (m) |
| `localisation.model` | similarity | `similarity` (turn + scale + shift), `rigid` or `translation` |
| `localisation.precision_landing` | true | Move over the home tag before landing |

`idf.py menuconfig` → Drone Configuration (reflash needed):

| Setting | Now | Meaning |
|---|---|---|
| Trajectory: fly straight from corner to corner | **on** (branch `feature/loc-tags-waypoints`, for comparison) | Waypoint mode: the target is the next lane end; PX4 flies at up to `MPC_XY_VEL_MAX`. Holds the lane line less tightly. `feature/loc-tags` has it off (follow mode) |
| Trajectory: steady speed with velocity feedforward | (hidden: waypoint mode is on) | Follow mode. Speed = `flight.cruise_speed_mps` in `config.yaml` |
| Trajectory: speed-up / slow-down rate | (hidden) | Follow mode: how gently it starts and stops at corners |
| Trajectory: target distance ahead of the drone | 25 cm | Carrot mode only (follow off): sets the speed, about 0.33 m/s |
| Trajectory: how close to get to each corner | 15 cm | Waypoint radius, as on `claude-one-shot`. Follow mode used 5 cm |

## 9. Troubleshooting

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
| `GYRO_CAL_FAILED ... close QGroundControl` | QGC is holding UDP 14550. Close it and type `calibrate` again |
| `GYRO_CAL_FAILED ... motion` | The drone moved during calibration. Keep it still and retry |
| `GYRO_CAL_FAILED ... offsets did not change` | PX4 accepted but did not finish: usually it moved. Keep it still and retry |
| `GYRO_CAL_FAILED ... no answer from PX4` | No MAVLink bridge reply: check the drone is on and pingable |
| ESP-IDF: `supports Python 3.10 or newer but you are using 3.7` | pyenv's global Python is old. In that window: `$env:PYENV_VERSION = "3.10.0"` |
| `idf_pinned.ps1 cannot be loaded because running scripts is disabled` | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` first |
| ESP-IDF activation: `call ...activate_xxx.bat is not recognized` | pyenv's `python.bat` makes ESP-IDF think it runs in cmd.exe. `$env:ESP_IDF_SHELL = "powershell.exe"` before `idf_pinned.ps1` |
| `LOC_FIT_FAILED` / report not corrected | Check `localisation.csv`: a tag never seen (it must pass under a lane), or a position measured wrong (one large residual) |
| `apriltag_host` build error `CONDITION_VARIABLE` | An old `C:\MinGW` gcc is first on PATH. `conda activate cde1302-laptop` (it has MinGW-w64 gcc), then run again |

## More documents

- [BENCH_TEST_SETUP.md](BENCH_TEST_SETUP.md): bench test without flying
- [UWB_ESP32_BENCH_TEST.md](UWB_ESP32_BENCH_TEST.md): testing the UWB module and wiring
- [UWB_PX4_FLIGHT_TEST.md](UWB_PX4_FLIGHT_TEST.md): UWB to PX4 in detail
