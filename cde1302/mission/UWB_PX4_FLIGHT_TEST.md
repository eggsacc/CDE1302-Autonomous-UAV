# UWB → PX4: bench and flight test

Goal: PX4 uses UWB to stop x/y drifting. Optical flow still handles short-term
motion and the rangefinder still handles height.

Do this **after** [UWB_ESP32_BENCH_TEST.md](UWB_ESP32_BENCH_TEST.md) passes,
including its Wi-Fi steps (4–6).

**Status:** the code has passed its tests on the laptop only. None of this has
run on the drone yet, so go through the steps in order.

```
LinkTrack → ESP32 ─ debug stream ─→ laptop: pairs UWB with PX4, fits the alignment
              ▲                       │
              └── CMD_UWB_ALIGN ──────┘  (only once, during the sweep)
              │
              └── ODOMETRY, 20 Hz ──→ PX4 EKF2
```

The laptop is only needed to work out the alignment. After that the ESP32
converts each fix and sends it to PX4 by itself.

Everything is **off by default**: both firmware and laptop have a switch.

| Off (normal) | On |
|---|---|
| Firmware `CONFIG_UWB_SEND_TO_PX4` not set | set |
| `mission/config.yaml` → `uwb.send_to_px4: false` | `true` |

---

## 1. PX4 parameters (QGroundControl)

| Parameter | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | **1** | Horizontal position only |
| `EKF2_EV_NOISE_MD` | **EV reported variance** (0) | PX4 weights each reading by the precision the ESP32 sends, never trusting it more than `EKF2_EVP_NOISE` |
| `EKF2_EVP_NOISE` | **0.15 m** | Your still test scattered 5–7 cm; flying adds more |
| `EKF2_EVP_GATE` | **3.0 SD** | Rejects readings that disagree by more than about 3x their expected error |
| `EKF2_EV_DELAY` | **50 ms** | A starting guess, refine it from a flight log |
| `EKF2_EV_POS_X` | **-0.04 m** | Antenna 4 cm behind the flight controller |
| `EKF2_EV_POS_Y` | **0 m** | |
| `EKF2_EV_POS_Z` | **0.03 m** | Antenna 3 cm below it (down is positive) |

Leave `EKF2_MAG_TYPE`, `EKF2_OF_CTRL` and `EKF2_HGT_REF` as they are.

Reboot the flight controller after changing them.

**`EKF2_EV_CTRL` must be 1.** At 15 EKF2 expects height, velocity and yaw from
vision too.

## 2. Flash the firmware

From `cde1302/`:

```powershell
. .\tools\idf_pinned.ps1
idf.py menuconfig
```

- **Drone Configuration**: Drone ID 2, host IP 192.168.1.102, drone IP 192.168.1.202.
- **UWB (LinkTrack)**: UWB on, RX pin 9, debug UDP on,
  **Send UWB position to PX4** on.

```powershell
idf.py -p COM18 build flash monitor
```

Every 2 s the serial log gets one more line:

```
I (...) uwb: PX4: waiting for alignment, fed 0, rejected 0, alignments 0
```

## 3. Bench test: props OFF, no flight

Laptop on the drone router with IP 192.168.1.102, PX4 powered, QGroundControl
connected. If the Wi-Fi steps of the ESP32 guide have passed, the firewall is
already set up.

### 3a. Fit only, nothing sent to PX4

```bat
python mission/tools/uwb_px4_bench.py
```

Carry the drone slowly along an L shape, about 1 m each way, keeping it level
and low enough for the rangefinder and flow to work.

| Output | Meaning |
|---|---|
| `no packets yet` | Wi-Fi: go back to the ESP32 guide, step 4 |
| `fit: 12/50 usable samples` | Keep moving; PX4 position or a UWB fix is missing |
| `fit: path only 0.40 m long` | Keep moving |
| `fit: path too straight ...` | Make the turn |
| `fit: mirror unclear` | Longer path, or set `uwb.mirror` (below) |
| `fit: fit too poor` | UWB and PX4 paths disagree: PX4 flow is poor, or the anchor layout is wrong |
| `FIT OK: theta ...` | Good |

Then keep carrying it. `apart` should stay below about **0.2 m**. If it keeps
growing, stop here and send me the output.

Note the `mirror` value in the `FIT OK` line and set it in `config.yaml`
(`uwb.mirror: yes` or `no`). It only depends on the anchor layout, and fixing
it lets the alignment finish on the first straight lane.

### 3b. Feed PX4

```bat
python mission/tools/uwb_px4_bench.py --send
```

Pass:
- `alignments` goes above 0 and `px4_fed` keeps rising.
- The serial log says `PX4: aligned`.
- In the QGroundControl **MAVLink Console**:

```
listener estimator_status_flags
```

`cs_ev_pos` must be **True**. If it's False, also run `ekf2 status` and send me
both outputs.

`rejected` rising while `px4_fed` also rises is fine (those fixes had poor
precision). Only `rejected` rising means no fix is good enough or PX4 has
stopped sending its position.

To stop it without reflashing:

```bat
python mission/tools/uwb_px4_bench.py --clear
```

A new takeoff (`CMD_START`) also clears it, so a flight can never start on an
old alignment.

## 4. First flight: short and low

Set `uwb.send_to_px4: true` in `mission/config.yaml`. Keep the first flight
short: a smaller `area` and someone ready on the RC kill switch.

```bat
python mission/mission.py --map --auto
```

Watch the console and `mission/logs/<time>/events.csv` for:

| Event | Meaning |
|---|---|
| `UWB_COLLECTING` | Sweep started, pairing positions |
| `UWB_ALIGN_WAITING` | Not enough data yet; the reason follows |
| `UWB_ALIGNED` | Fit accepted; note the rms and jump |
| `UWB_ALIGN_CONFIRMED` | ESP32 has it: UWB now goes to PX4 |
| `UWB_ALIGN_REFUSED` | Would have moved PX4's position too far (`uwb.max_start_jump_m`), so nothing was sent |

**Expect a small sideways nudge at `UWB_ALIGN_CONFIRMED`**: PX4 snaps its
position to UWB once, then the drone corrects by that amount. It should be under
about 0.3 m. If it lurches, land and send me the log.

If no alignment is found, the sweep carries on without UWB, exactly as today.

## 5. Things to know

- **Height stays with the rangefinder.** UWB z is unreliable on your setup. The
  ESP32 sends PX4's own z back only because PX4 insists on a number; with
  `EKF2_EV_CTRL = 1` it's ignored.
- **One alignment per flight.** It isn't refined during the flight.
- **A reading goes to PX4 if** it has a real x/y position with x/y `eop` at
  most 1 m (`CONFIG_UWB_PX4_MAX_EOP_CM`), PX4's position is live, and the
  reading is under 100 ms old. The height and its precision are ignored. Each
  reading's `eop` goes to PX4 as its variance, so PX4 trusts poor readings
  less. Sending steadily matters: if readings stop for about 0.4 s, PX4 stops
  UWB fusion, and when it restarts it jumps to the next reading unchecked.
- **The laptop can drop out** after `UWB_ALIGN_CONFIRMED`. UWB keeps reaching
  PX4, but the 3 s telemetry safety check on the laptop still lands the drone.
- **`uwb_sniff.py` and the mission can't run together**: both listen on UDP 5010.
- **The debug stream format changed** (version 2), so firmware and laptop code
  must be from the same commit. Otherwise the laptop reports a version mismatch.

## 6. Turning it off

- Laptop only: `uwb.send_to_px4: false`. Nothing is sent to PX4, because the
  ESP32 never receives an alignment.
- Firmware: turn off **Send UWB position to PX4** in menuconfig and reflash.
- PX4: `EKF2_EV_CTRL` can stay at 1. With no vision data arriving, it does nothing.

## 7. What to send back

- The bench tool's full output.
- The `listener estimator_status_flags` output.
- After a flight: `events.csv`, `map.png`, and the `.ulg` log from the flight
  controller.
