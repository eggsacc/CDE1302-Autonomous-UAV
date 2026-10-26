# Bench test setup: `mission.py --dry-run` on the real drone

This is for whoever runs the first real-drone test on their own Windows laptop.
`--dry-run` connects to the drone, shows telemetry and runs the camera +
AprilTag pipeline, but **never sends a flight command** (takeoff, goto, hold
and land are all blocked in code). Take the propellers off anyway.

Time needed: about 30 min of setup the first time, then 20-30 min of testing.

---

## 1. Install the tools (once)

1. **Git for Windows**: <https://git-scm.com/download/win>
2. **Miniforge** (conda): <https://github.com/conda-forge/miniforge>. Use the
   Windows installer, then open **"Miniforge Prompt"** from the Start menu for
   every command below.

## 2. Get the code (once)

```bat
cd %USERPROFILE%
git clone https://github.com/eggsacc/CDE1302-Autonomous-UAV.git
cd CDE1302-Autonomous-UAV
git checkout nikidudu
```

Later, to get updates: `git pull` inside that folder.

Everything below is run **from the `cde1302` folder**:

```bat
cd %USERPROFILE%\CDE1302-Autonomous-UAV\cde1302
```

## 3. Python environment (once)

```bat
conda env create -f laptop/environment.yml
conda activate cde1302-laptop
conda install -c conda-forge gcc
```

`gcc` is needed once, to compile the drone's own AprilTag detector for the
laptop. Build and check it:

```bat
python laptop/apriltag_host.py
```

Expected: `built ...libapriltag_host-AMD64.dll` and `tag16h5 ncodes = 22 (expected 22)`.

**Every new prompt window:** `conda activate cde1302-laptop`, then `cd` into `cde1302`.

## 4. Check the code works (no drone needed)

```bat
python -m pytest mission/tests -q
```

Expected: `177 passed` (about 30 s). If anything fails, stop and send the output.

## 5. Network setup

The drone sends telemetry to **one fixed laptop IP that was set when it was
flashed**: `192.168.1.(100 + drone ID)`, i.e. **`192.168.1.102` for drone 2**.
Your laptop must have exactly that address, or no telemetry will arrive.

1. Connect to the drone router's Wi-Fi.
2. Give the laptop a static IP: *Settings → Network & internet → Wi-Fi → (this
   network) → Hardware properties → IP assignment → Edit → Manual → IPv4 on*:
   - IP address `192.168.1.102`
   - Subnet mask `255.255.255.0` (prefix length `24`)
   - Gateway `192.168.1.1`
3. On the same page, set **Network profile type = Private**.
4. Check with `ipconfig`: the Wi-Fi adapter must show `192.168.1.102`.
5. Power the drone, wait about 20 s, then `ping 192.168.1.202` (the drone's ESP32).

If the drone was flashed with a different host IP, use that one instead. The
ESP32 serial monitor prints it at boot (`Telemetry → 192.168.1.x:5005 | Commands ← port 5006`).

**Windows Firewall.** The first time `mission.py` runs, Windows asks whether
Python may use the network: tick **Private networks** and click **Allow**. If
you missed the prompt, run this once in an **Administrator** PowerShell:

```powershell
netsh advfirewall firewall add rule name="CDE1302 drone UDP" dir=in action=allow protocol=UDP localport=5005,5009
```

(5005 = telemetry, 5009 = camera.) Undo it later with
`netsh advfirewall firewall delete rule name="CDE1302 drone UDP"`.

## 6. Check the config

Open `mission/config.yaml` and check that `drone_id: 2` matches the drone's
flashed ID. Nothing else needs changing for a dry run.

## 7. Run the dry run

1. **Close** `tag_stream.py`, `camera_stream.py` and any other course script.
   Only one program can use the telemetry and camera ports.
2. Put the drone where it will later take off, facing the direction you want,
   **then power it on**. PX4's position origin (0, 0) is wherever it boots.
3. Optional: open QGroundControl to watch the EKF and rangefinder.
4. Start:

```bat
python mission/mission.py --dry-run
```

Within a few seconds you should see:

```
Run folder: ...\mission\logs\<date_time>
CONNECTED drone 2 at 192.168.1.202, pose (0.00, 0.00) heading 87 deg
DRY RUN: watching telemetry and tags, no flight commands. Type `quit` or Ctrl+C to stop.
[READY] pose (0.00, 0.00) 87 deg IDLE, telem age 0.1 s, wp 0/6, camera 45 frames/0 sightings, tags: none
```

The status line repeats every 5 s. Type `status` + Enter for one now.

## 8. The tests

Write the results down as you go.

**A. Telemetry and heading**
- Rotate the drone slowly by hand. The heading should change smoothly.
- **Note which real-world direction reads heading ≈ 0°.** That is PX4's x axis
  ("North"). The search area in `config.yaml` uses this frame.
- Carry it about 1 m in a straight line at about 0.5 m height. The pose should
  change by roughly 1 m. Optical flow needs a textured floor, so some error is
  expected.

**B. One tag on the floor**
- Put a tag (ID 0-21) on the floor and measure its position from the power-on
  spot with a tape, along the x (heading 0°) and y (heading 90°) directions.
- Hold the drone **level at about 0.5 m**, 0.5-1 m back from the tag, camera
  facing it, and **keep still** (turning faster than 30°/s is ignored).
- Within a couple of seconds:
  `[EVENT] ...  T+   --.--- s  TAG_CONFIRMED id=3 x=0.812 y=0.305 (5 sightings)`
  (`T+` is blank because there is no takeoff in a dry run.)
- Compare `x`, `y` with your tape measurement.

**C. Range.** Repeat B from 1.0 m and 1.5 m, and with the tag off to one side.
Note the largest distance that still adds sightings in the status line.

**D. Several tags.** Put 2-3 tags in view at once. Each should get its own
`TAG_CONFIRMED`.

**E. Wall tag.** Optional: try one tag on a wall at its real height.

**F. Safety.** Type `takeoff` + Enter. It must reply
`takeoff: --dry-run never sends flight commands`, and the drone must do nothing.

**Stop:** type `quit` + Enter, or press Ctrl+C. The last log line is a camera
summary (frames, detections, accepted, rejected by reason).

## 9. What to send back

1. The **whole console output**: select all in the prompt window and copy-paste
   it into a text file.
2. The run folder `mission/logs/<date_time>/` (zip it). It contains
   `events.csv`, `trajectory.csv`, `detections.csv`, `config_used.yaml`.
3. Your notes: which way heading 0° points, tape-measured tag positions, the
   maximum detection distance, and anything odd.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `config error: ...` | A typo in `config.yaml`; the message names the key. |
| `no telemetry from drone 2 within 30 s` | Laptop IP isn't `192.168.1.102` (`ipconfig`), not on the drone Wi-Fi, firewall blocking UDP 5005, or `drone_id` doesn't match the flashed ID. |
| Connected, but `camera 0 frames` | Firewall blocking UDP 5009, or another viewer is open. |
| `cannot listen for camera frames on UDP 5009 ... close tag_stream.py` | Close the other camera viewer and retry. |
| `no C compiler: 'gcc' is not on PATH` | `conda activate cde1302-laptop`, then `conda install -c conda-forge gcc`. |
| `ModuleNotFoundError` | The environment isn't active: `conda activate cde1302-laptop`. |
| `can't open file ... mission.py` | You're not in the `cde1302` folder. |
| Frames arrive but tags never confirm | Too far, too fast, or not level. Check the camera summary's `rejected:` reasons when you quit. |
| Pose jumps or drifts a lot | EKF or optical flow issue: check QGroundControl (rangefinder valid? flow quality?). |
