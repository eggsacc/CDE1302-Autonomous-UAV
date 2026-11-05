# UWB with PX4: how it works and how to calibrate

## The method

**Loosely coupled UWB fusion with frame alignment.**

- **Loosely coupled:** the UWB module works out its own x/y position. PX4 is
  given that position, not the raw anchor distances, and combines it with the IMU
  and optical flow in its estimator (EKF2).
- **Frame alignment:** UWB and PX4 measure position in different coordinate
  systems. A one-time 2-D rigid fit (a rotation, an optional mirror and an
  offset) converts UWB positions into PX4's coordinates.

It is **not** tightly coupled UWB/IMU fusion, where the estimator also works out
heading from UWB all the time. Here heading comes from PX4's gyro after the
alignment, so a slow gyro drift still turns the nose a little, while UWB keeps
the **position** correct.

## Why an alignment is needed

| | UWB | PX4 |
|---|---|---|
| Origin | Anchor A0 | Where PX4 powered up |
| x axis | Set by the anchors | The way the drone faced at power-up (no compass) |
| Up | z up | z down |

So the same spot has two different coordinates. The alignment is

```
PX4 position = R(theta) x M x UWB position + t
```

| Value | Meaning |
|---|---|
| `theta` | Angle between UWB's x axis and the drone's nose at power-up |
| `mirror` | `M` flips UWB's y axis. Always **yes** here, because UWB is z-up and PX4 is z-down |
| `t` | Where A0 is in PX4's coordinates |

Theta and t depend on **where and which way the drone sat when PX4 started**.
The same start spot and direction gives the same values.

## How it's done

```
LinkTrack tag ──UART──▶ ESP32 ──Wi-Fi debug stream──▶ laptop: fits theta, mirror, t
                          ▲                              │
                          └──── CMD_UWB_ALIGN ───────────┘   (once)
                          │
                          └──── UWB in PX4 coordinates, 20 Hz ──▶ PX4 EKF2
```

1. **Pairs:** while the drone moves, the ESP32 sends the laptop every UWB reading,
   together with PX4's position at that instant.
2. **Fit:** the laptop finds the theta, mirror and t that make the UWB path
   match the PX4 path best, in the least-squares sense. Outliers are dropped
   once and the fit is repeated.
3. **Send:** the result goes to the ESP32 once.
4. **Fuse:** from then on the ESP32 converts every UWB reading into PX4
   coordinates and sends it to PX4, with the reading's precision. PX4 trusts each
   reading according to that precision and rejects ones that disagree too much.

The path must not be a straight line: a straight line can't tell a mirrored axis
from a normal one, and it pins the angle only weakly. A **square** is ideal: two
directions, long sides, and it ends where it started.

## The calibration flight: `simple_uwb_calibration.py`

```bash
python laptop/simple_uwb_calibration.py --drone-id 2 --confirm
```

1. Takes off and waits for the hover.
2. Flies a square (1 m sides by default) with the heading fixed, and records
   UWB/PX4 pairs.
3. Ends back at the start and lands.
4. Prints the fit and saves it to `laptop/uwb_calibration.json`.

```
=== UWB calibration ===
theta   -84.3 deg
mirror  yes
t       (+4.159, +3.817) m   (UWB origin A0 in PX4's frame)
rms     0.034 m from 412 of 420 pairs
VERDICT: good
```

| Option | Meaning |
|---|---|
| `--size 1.2` | Square side in metres. Bigger pins the angle better |
| `--side left` | Square to the drone's left instead of its right |
| `--speed 0.15` | Planned speed (m/s) |
| `--send` | Also send the result to the ESP32 after two sides, so PX4 uses UWB for the rest of the flight |
| `--mirror auto` | Test both mirror choices instead of assuming yes |

**Good result:** rms under about 0.10 m. If it's higher, the set-goal script still flies (and prints POOR FIT), but people or objects are
probably blocking anchors, or the anchor layout in NAssistant is off.

**When the saved values are valid:** until PX4 restarts. To reuse them across
power-ups, always power up PX4 with the drone on the same marked spot, facing the
same marked direction. A 1° placement error moves positions about 1.7 cm per
metre from the start.

## Flying to a goal: `simple_uwb_set_goal.py`

It takes off, gets an alignment, turns UWB on in PX4, flies a straight line to
the goal with the heading fixed, hovers for 5 s, prints the error and lands.
The goal is in **UWB coordinates** (the x/y NAssistant shows, metres from A0).

```bash
# no alignment given: flies the calibration square first
python simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --confirm

# alignment from a calibration file (same power-up spot and heading)
python simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --calibration uwb_calibration.json

# alignment typed in
python simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --theta 0 --mirror yes --tx -1.2 --ty 3.4
```

A given alignment (typed in or from a file) is checked against the drone's
position first. If it is more than 0.30 m off (`--max-jump`), the drone lands
instead, because the values were made with a different start pose.
Goals more than 4 m away are refused (`--max-distance`).

**Working out theta and t by hand:**

- **theta = 0** when the drone's nose points along the UWB **x axis** at PX4
  power-up. In the usual LinkTrack layout that is the direction from **A0 to
  A3**. Check this in NAssistant: the x axis runs from A0 to whichever anchor
  sits at y = 0. Pointing the nose the other way along that line gives
  theta = 180; a quarter turn gives ±90.
- **mirror = yes**, always (UWB is z-up, PX4 is z-down, so y flips).
- **t** is where UWB's origin (A0) lands in PX4 coordinates. With theta = 0 and
  mirror = yes: **t = (−x0, +y0)**, where (x0, y0) is the takeoff spot in UWB
  coordinates (read it in NAssistant before power-up). Example: takeoff at UWB
  (1.2, 3.4) gives `--tx -1.2 --ty 3.4`.

The hand-worked theta is only as good as the nose alignment (1° ≈ 1.7 cm per
metre), so the calibration square is more accurate.

## Settings

**Firmware** (`idf.py menuconfig` → UWB (LinkTrack)):
- Read a LinkTrack UWB tag on UART2: on, RX pin 9
- Debug stream: on
- Send UWB position to PX4: on (needed for `--send`)

**PX4** (QGroundControl → Vehicle Setup → Parameters), then reboot PX4:

| Parameter | Value |
|---|---|
| `EKF2_EV_CTRL` | 1 |
| `EKF2_EV_NOISE_MD` | 0 |
| `EKF2_EVP_NOISE` | 0.15 |
| `EKF2_EVP_GATE` | 3 |
| `EKF2_EV_DELAY` | 50 |
| `EKF2_EV_POS_X`, `_Y`, `_Z` | -0.04, 0, 0.03 |

**UWB module** (NAssistant): role TAG, protocol NODE_FRAME2, 50 Hz, 921600 baud.

**Laptop:** on the drone router with IP 192.168.1.102. Windows Firewall must allow
UDP 5010 in.

## Troubleshooting

| Problem | Fix |
|---|---|
| `no UWB debug stream on UDP 5010` | UWB or the debug stream is off in the firmware, the laptop IP is wrong, or the firewall blocks it |
| `only N usable UWB/PX4 pairs` | Module has no fix, or poor precision. Check anchors and line of sight |
| rms above 0.10 m | Something blocks anchors, PX4's flow drifted, or the anchor positions are wrong |
| Mirror comes out "no" | The anchors were set up in a different order. Use `--mirror auto` and note the result |
