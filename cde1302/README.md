# CDE1302 UWB testing

ESP32-S3 firmware (`main/`) and laptop scripts (`laptop/`) for flying the
CDE1302 drone with UWB position (Nooploop LinkTrack). PX4 fuses the UWB position
with its optical flow, and the laptop sends goals and sweeps over Wi-Fi.

How the UWB method works, and more detail on each script:
[laptop/UWB_README.md](laptop/UWB_README.md).

Commands are run from this folder (`cde1302/`) unless stated otherwise. The
examples use drone ID **2**.

## 1. Hardware

- CDE1302 drone: PX4 1.15.4 flight controller (no compass), optical flow and
  range sensor, XIAO ESP32-S3 Sense connected to PX4 over UART.
- LinkTrack P-AS2 UWB tag on the drone, plus 4 anchors set up in NAssistant.
- Wiring: UWB tag **TX → ESP32 D10 (GPIO 9)**, GND → GND, tag powered from
  3.6–5.5 V.
- A laptop and a 2.4 GHz Wi-Fi network (router or hotspot).

## 2. Install ESP-IDF (once)

The firmware needs one exact ESP-IDF version (`tools/idf-pin.env`). Install it
once per laptop.

**Windows:** run in PowerShell (about 1 GB download, 10–30 min):

```bash
powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1
```

It installs to `C:\esp\idf-pinned` and `C:\Espressif\tools-pinned`, separate
from any other ESP-IDF.

**macOS / Linux:**

```bash
git clone https://github.com/espressif/esp-idf.git ../esp-idf
cd ../esp-idf
git checkout "$(sed -n 's/^IDF_CHECKOUT_REF=//p' ../cde1302/tools/idf-pin.env)"
git submodule update --init --recursive
./install.sh esp32s3
cd -
```

## 3. Set up the network

The drone joins your Wi-Fi with a fixed IP and sends everything to one laptop
IP. Both are built into the firmware.

1. Start the network. It must be **2.4 GHz** (the ESP32 can't use 5 GHz).
   - **Windows Mobile Hotspot:** Settings → Network & internet → Mobile hotspot.
     Set the band to 2.4 GHz. The laptop is then always `192.168.137.1`.
   - **Router:** connect the laptop to it.
2. Find the laptop's IP and the gateway: `ipconfig` (Windows) or `ifconfig`
   (macOS / Linux).
3. Set these in `sdkconfig` (or `idf.py menuconfig` → Drone Configuration):

   | Setting | Windows hotspot | Router (example) |
   |---|---|---|
   | `CONFIG_WIFI_SSID` | your hotspot name | your router name |
   | `CONFIG_WIFI_PASSWORD` | your hotspot password | your router password |
   | `CONFIG_HOST_IPV4_ADDR` (laptop) | `192.168.137.1` | `192.168.1.102` |
   | `CONFIG_DRONE_STATIC_IPV4_ADDR` | `192.168.137.202` | `192.168.1.202` |
   | `CONFIG_WIFI_GATEWAY_IPV4_ADDR` | `192.168.137.1` | `192.168.1.1` |
   | `CONFIG_WIFI_NETMASK_IPV4_ADDR` | `255.255.255.0` | `255.255.255.0` |
   | `CONFIG_DRONE_ID` | `2` | `2` |

   The drone IP must be on the same network as the laptop and unused.

## 4. Flash the ESP32

The UWB settings are already on in `sdkconfig` (UWB on GPIO 9 at 921600 baud,
debug stream to the laptop, UWB sent to PX4). Plug the ESP32 into the laptop with
USB and close QGroundControl and any serial monitor.

**Windows** (PowerShell, in this folder):

```bash
. .\tools\idf_pinned.ps1
```

```bash
idf.py -p COM18 build flash monitor
```

Replace `COM18` with your port (Device Manager → Ports).

**macOS / Linux:**

```bash
. ../esp-idf/export.sh
```

```bash
idf.py -p /dev/tty.usbmodem2101 build flash monitor
```

Every 2 s the monitor should show the UWB tag, for example
`... id 0: pos (3.230, 4.400, 0.120) m ...`, and `anchors 4/4`. `Ctrl+]` leaves
the monitor.

## 5. Set up the laptop

1. Install conda once. **Windows:** install Miniforge from
   <https://github.com/conda-forge/miniforge/releases/latest> and use "Miniforge
   Prompt". **macOS / Linux:**

   ```bash
   curl -LO "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
   bash "Miniforge3-$(uname)-$(uname -m).sh"
   ```

2. Create the environment (once):

   ```bash
   conda env create -f laptop/environment.yml
   ```

3. Activate it in every new terminal:

   ```bash
   conda activate cde1302-laptop
   ```

4. **Windows only:** allow Python through the firewall. In PowerShell **as
   administrator**, with the path of your environment's `python.exe`:

   ```bash
   New-NetFirewallRule -DisplayName "CDE1302 Python UDP" -Direction Inbound -Program "C:\conda-envs\cde1302-laptop\python.exe" -Protocol UDP -Action Allow -Profile Any
   ```

   Run `where python` with the environment active to find the path.

5. Check that the UWB code works (no drone needed):

   ```bash
   cd laptop
   python -m pytest tests/test_uwb_calibration.py tests/test_uwb_set_goal.py tests/test_uwb_frame_goal.py tests/test_uwb_lawnmower.py
   cd ..
   ```

## 6. Set up PX4 and the UWB module

**PX4** (QGroundControl → Vehicle Setup → Parameters). Reboot PX4 afterwards.

| Parameter | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | 1 | use UWB horizontal position |
| `EKF2_EV_NOISE_MD` | 0 | use the noise the ESP32 sends with each reading |
| `EKF2_EVP_NOISE` | 0.15 | |
| `EKF2_EVP_GATE` | 3 | |
| `EKF2_EV_DELAY` | 50 | |
| `EKF2_EV_POS_X`, `_Y`, `_Z` | -0.04, 0, 0.03 | tag position on the drone (m) |
| `EKF2_MAG_TYPE` | None | no compass |
| `EKF2_OF_CTRL` | 1 | optical flow on |
| `EKF2_HGT_REF` | Range sensor | |
| `MPC_XY_VEL_MAX` | 0.3 to start | top speed for waypoint flights |

**UWB module** (NAssistant): tag role TAG, protocol NODE_FRAME2, 50 Hz,
921600 baud. Set up the 4 anchors, and note which way **UWB +x** points (from A0
to the anchor at y = 0).

## 7. Fly

All flights use the UWB anchors as the coordinate system, so **line up the drone
before powering PX4**: put it on the start spot, nose along a UWB axis, then plug
in the battery. Don't rotate it afterwards. Tell the scripts which way the nose
points:

| Nose points along | `--nose-deg` |
|---|---|
| UWB +x | 0 |
| UWB +y | 90 |
| UWB −x | 180 |
| UWB −y | -90 |

Every script takes `--confirm`: it checks everything on the ground first and asks
you to type a code before takeoff. Type anything else to cancel. Ctrl+C in the
air lands the drone.

**Check the link** (no flight):

```bash
python laptop/simple_arm_takeoff_land.py --monitor-only --drone-id 2
```

**Fly to one goal.** The goal is in metres from the takeoff spot, along the UWB
axes:

```bash
python laptop/simple_uwb_frame_goal.py --drone-id 2 --nose-deg -90 --goal 0.0 -1.0 --confirm
```

**Lawnmower sweep**, stopping at each corner. Bounds are in metres from the takeoff spot:

```bash
python laptop/simple_uwb_lawnmower.py --drone-id 2 --nose-deg -90 --xmin -0.5 --xmax 0.5 --ymin -3 --ymax 0 --confirm
```

Add `--track run.csv` to the sweep to save the flown UWB track.

**Without lining up the drone:** `simple_uwb_calibration.py` flies a small square
to measure the alignment instead, and `simple_uwb_set_goal.py` flies to a goal
with it. See [laptop/UWB_README.md](laptop/UWB_README.md).

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| `no telemetry from drone 2` | Laptop IP in `sdkconfig` is wrong, the drone isn't on the Wi-Fi (2.4 GHz?), or the firewall blocks Python |
| `no usable UWB on UDP 5010` | Tag not powered or not wired to GPIO 9, anchors off, or no line of sight |
| `PX4 did not move onto the UWB position` | `EKF2_EV_CTRL` isn't 1, PX4 wasn't rebooted, or the old firmware is still flashed |
| Drone goes the wrong way sideways | `--nose-deg` doesn't match how the drone was powered up, or the anchor layout is mirrored (try `--mirror no`) |
| Too fast or too slow | `MPC_XY_VEL_MAX` in QGroundControl |
| `COM18` busy when flashing | Close QGroundControl and other serial monitors |
