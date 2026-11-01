# UWB bench test: LinkTrack P-AS2 → ESP32 → Wi-Fi → laptop

Goal: prove that everything built so far works on the real drone, **one link at
a time**, so that if something is wrong you know exactly where.

```
LinkTrack ─UART─▶ ESP32 receive ─▶ ESP32 decode ─▶ + PX4 position ─Wi-Fi─▶ laptop re-decode
  (power,         [1] bytes,        [2] frames,      [6] PX4 age        [3] packet loss  [4] frames OK
   wiring)            framing err,      checksum err                                    [5] ESP32 vs laptop
                      overflows                                                              counts match
```

The ESP32 forwards the **raw** frames it received, so the laptop decodes them a
second time with its own (separately tested) parser and compares counts with
the drone. Nothing is sent to PX4: the drone flies exactly as before. No
anchors are needed: without them the module reports "NO FIX", which is fine.

---

## 0. Before touching hardware (laptop only, 1 min)

From `cde1302/` with the conda env active:

```bat
python -m pytest mission/tests -q
```

All tests must pass. Among them, the drone's C code (`main/nlink_parser.c`,
`main/uwb_debug_pack.c`) is compiled on the laptop and checked against the
Python side on a real 10 s capture from the P-A, so the parsing and the packet
format are already proven before the drone is involved.

## 1. Module settings (NAssistant, via your CH340)

The P-AS2 has no USB port: connect the CH340 to its GH1.25 UART
(module **T** → adapter **RX**, **R** → adapter **TX**, **G** → **GND**, and
power the module from 5 V). In NAssistant set, then write and note down:

| Setting | Value |
|---|---|
| Role | TAG |
| Protocol | **NODE_FRAME2** (same as the P-A we captured) |
| Update rate | 50 Hz |
| Baud | 921600 (if your CH340 fails at that speed, use 460800 everywhere: module, firmware, tools) |

Optional check with NAssistant closed:
`python mission/tools/uwb_sniff.py --port COMx --duration 10` → `VERDICT: frames OK`.

## 2. Wiring to the drone

**Choose a free ESP32 pin first.** Physically check which header pins are
unused on your drone. The firmware refuses pins that the drone already uses
(PX4 UART GPIO4/5, camera, flash/PSRAM, USB, console, boot strapping) and says
so in the log.

| XIAO pin | GPIO | Notes |
|---|---|---|
| D2 | 3 | Strapping pin (JTAG select): works, the log warns; prefer another |
| D5 | 6 | |
| D8 | 7 | Also the Sense board's microSD lines (unused by this firmware) |
| D9 | 8 | Same |
| D10 | 9 | Same |
| D0/D1 | 1/2 | Only if the ToF ring is **not** fitted (ToF I2C) |

**Do not use D3 (GPIO4) or D4 (GPIO5):** those are the PX4 UART. On D4 the
ESP32's PX4 TX output and the module's TX would drive the same wire.

**Use the GPIO number, not the D number, in the settings below.**

| Module (GH1.25) | Connect to |
|---|---|
| **V** | **5 V from the FC / power board** (module needs 3.6–5.5 V, ~1.4 W). **Never** the ESP32's 3V3 pin. |
| **G** | GND, **shared with the ESP32** |
| **T** (module TX) | the chosen ESP32 GPIO |
| **R** (module RX) | nothing |

Screw the antenna on **before** powering the module.

## 3. Firmware settings and flashing

The UWB code is off by default. Turn it on with **one** of these:

### A. Mac, `flash_drone.sh` (the usual team flow)

`flash_drone.sh` resets `sdkconfig` to the committed version, so put bench
settings in an **untracked** file `cde1302/sdkconfig.local` (already in
`.gitignore`):

```
CONFIG_UWB_ENABLED=y
CONFIG_UWB_RX_PIN=6
```

(replace `6` with your GPIO). Then flash as usual:

```bash
./flash_drone.sh 2 /dev/tty.usbmodemXXXX
```

The script prints `Applying sdkconfig.local` and, after building,
`verified: sdkconfig.local settings are in the build`. If a value did not make
it into the firmware (e.g. a pin out of range) it stops **before** flashing.
Delete `sdkconfig.local` to build without UWB again.

### B. Windows, `idf.py`

**Use the ESP-IDF version pinned in `tools/idf-pin.env`** (a v6.1 development
commit). The full firmware does **not** build with the released ESP-IDF 6.0:
the camera library (`esp32-camera` 2.1.5) fails with *"ll_cam.c includes
hal/clk_gate_ll.h ... esp_hal_clock is not in the requirements list"*. That is
unrelated to UWB.

**Once per laptop** (~1 GB download, ~3 GB on disk, leaves any other ESP-IDF
install alone; installs to `C:\esp\idf-pinned` and `C:\Espressif\tools-pinned`
because ESP-IDF does not support paths with spaces):

```powershell
powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1
```

**In every new PowerShell window**, from `cde1302/` (note the leading dot):

```powershell
. .\tools\idf_pinned.ps1
```

It refuses if the installed ESP-IDF is not exactly the pinned commit.

Verified on this laptop (2026-10-29): with this ESP-IDF the full firmware
builds with UWB enabled, no warnings from the UWB code, image 1043 KB (30 %
of the app partition free). `idf.py` may print *"WARNING: Your environment is
not configured to handle Unicode characters"*: cosmetic, ignore it. Then:

```powershell
idf.py menuconfig
```

- **Drone Configuration**: check Drone ID = 2, host IP = 192.168.1.102, drone
  IP = 192.168.1.202 (the committed `sdkconfig` is set up for another drone;
  `flash_drone.sh` normally fixes these).
- **UWB (LinkTrack)**: enable, set the RX pin; leave the rest at defaults.

Save, then:

```powershell
idf.py -p COMx build flash monitor
```

Afterwards `git restore sdkconfig` so the bench settings are not committed.

Optional settings (same menu, or extra lines in `sdkconfig.local`):
`CONFIG_UWB_BAUD` (default 921600), `CONFIG_UWB_LOG_PERIOD_MS` (2000),
`# CONFIG_UWB_DEBUG_UDP is not set` (serial only), `CONFIG_UWB_DEBUG_PORT` (5010).

## 4. The test, link by link

### Step 1: boot log (serial monitor)

Expected line:

```
I (...) uwb: UART2 RX=GPIO6 (module TX), TX=not used, 921600 baud; debug UDP -> 192.168.1.102:5010 every 100 ms
```

| Instead you see | Meaning |
|---|---|
| no `uwb:` lines at all | UWB not enabled in this build (step 3) |
| `UWB_RX_PIN is not set` | set the pin (step 3) |
| `... is used by the camera / PX4 UART / ...` | pick another free pin |
| `UART2 setup failed` | report the exact message |

The UWB task starts **before** Wi-Fi, so steps 1–3 work even with the router
off. Without a router, boot stops at `Connecting to '...'` and only the UWB task
runs (no PX4 link, no camera, nothing can arm), and `debug sent` stays 0: both
are expected. Steps 4–6 need Wi-Fi.

### Step 2: link [1], module → ESP32 UART

Every 2 s:

```
I (...) uwb: 12000 B, 100 frames (50.0/s) | checksum err 0, length err 0, skipped 0 B, uart overflow 0, uart framing err 0 | debug sent ..., send err 0, dropped ...
```

Pass: about 50 frames/s (your update rate), all error counters **0**
(`dropped` may grow while Wi-Fi is not up yet; that is normal).

| Symptom | Likely cause |
|---|---|
| `no bytes on GPIOn` | module not powered, T not on that pin, or no common GND |
| bytes but `no valid frames`, framing errors growing | baud mismatch between module and `CONFIG_UWB_BAUD` |
| `bytes arrive but no valid frames`, no framing errors | protocol set to NMEA, or wrong protocol: set NODE_FRAME2 |
| uart overflow > 0 | ESP32 could not keep up: report it (should not happen at 50 Hz) |
| a few checksum errors | electrical noise: shorten/twist the T and G wires |

### Step 3: link [2], ESP32 decoding

Second line each period:

```
I (...) uwb: NODE_FRAME2 id 0: pos (1.000, 1.000, 1.000) m NO FIX, eop (2.55, 2.55, 2.55) m, 4.9xx V, local_time ..., anchors 0/0
```

Pass: **this is exactly what the P-A sent with no anchors**: placeholder
position (1, 1, 1), precision 2.55 m = "no fix", supply voltage ≈ the
module's real supply, `local_time` increasing by ~2000 per line (2 s).

### Step 4: link [3], Wi-Fi to the laptop

1. Laptop on the drone router with IP **192.168.1.102** (= the firmware's host IP).
2. Firewall: allow UDP 5010 once (Administrator PowerShell):
   ```powershell
   netsh advfirewall firewall add rule name="CDE1302 UWB debug" dir=in action=allow protocol=UDP localport=5010
   ```
3. Run (from `cde1302/`):
   ```bat
   python mission/tools/uwb_sniff.py --udp 5010 --duration 30 --csv uwb_bench.csv
   ```

During the run, every second: the frame check (as in the USB test) plus

```
          drone: 300 pkts (0 lost, 0 bad) | ESP32 decoded 1500, checksum err 0, uart overflow 0, framing err 0, not forwarded 2 | PX4 (+0.01, -0.02) 87 deg, age 40 ms
```

On the serial monitor, `debug sent` keeps increasing and `send err` stays 0.

### Step 5: read the verdict

At the end `uwb_sniff.py` checks each link separately:

```
=== drone chain: module -> ESP32 -> Wi-Fi -> laptop ===
1. module -> ESP32 UART     OK         ... B received, framing errors 0, overflows 0
2. ESP32 decoding           OK         ... frames decoded, checksum errors 0, length errors 0
3. Wi-Fi ESP32 -> laptop    OK         ... packets, 0 lost (0.0%), 0 unreadable
4. laptop re-decode         OK         ... raw frames re-decoded on the laptop, checksum errors 0
5. ESP32 vs laptop count    OK         ESP32 counted N frames; laptop received N - d + d not forwarded
6. PX4 position pairing     OK         last PX4 position 40 ms old at the UWB frame
VERDICT: whole chain OK
```

| Line says PROBLEM | Where to look |
|---|---|
| 1 | wiring / power / baud (step 2) |
| 2 | ESP32 parser: send the serial log and the CSV |
| 3 | Wi-Fi: distance to the router, interference; a little loss is tolerable, but report it |
| 4 | frames damaged on the way: send the CSV |
| 5 | frames lost inside the ESP32: send the serial log |
| 6 | PX4 position stale: PX4 link problem (not UWB) |

`NOT TESTED` on line 6 just means PX4 is not connected or not sending a
position. Line 5 is `NOT TESTED` when packets were lost (the count cannot add
up then); fix line 3 first.

### Step 6: PX4 pairing (if PX4 is connected and powered)

Props off. Carry or rotate the drone by hand during a run; in `uwb_bench.csv`
`px4_x`, `px4_y` and `px4_heading` must change, and `px4_age_ms` stay below
~200. These are the UWB/PX4 pairs the alignment estimator
(`mission/uwb_align.py`) will use once the anchors are up.

### Step 7 (optional): a third opinion on the UART line

While the drone runs, connect **only** the CH340's **RX and GND** to the
module's **T** line (not the adapter's TX or VCC) and in a second terminal run
`python mission/tools/uwb_sniff.py --port COMx --duration 30`. Its frames/s
must equal the ESP32's. If the laptop sees clean frames but the ESP32 doesn't,
the problem is on the ESP32 side of the wire.

### Step 8: nothing else broke

With UWB on, run the normal bench test (`mission/BENCH_TEST_SETUP.md`,
`python mission/mission.py --dry-run`) at the same time as step 4: telemetry
age stays ~0.1 s and camera fps is unchanged.

## 5. What to send back

- Serial monitor: the boot line and three consecutive status pairs.
- The full `uwb_sniff.py --udp` output (including the chain verdict).
- `uwb_bench.csv`.
- Your NAssistant settings and the GPIO you used.

## 6. Turning it off

Delete `sdkconfig.local` (A) or disable it in menuconfig (B) and reflash.
With `UWB_ENABLED` off the firmware is identical to before.
