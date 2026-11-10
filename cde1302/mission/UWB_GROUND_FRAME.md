# UWB as the coordinate system from the ground (`uwb.ground_frame`)

Ported from `laptop/simple_uwb_frame_goal.py` on the `uwb-testing` branch.

Before takeoff, PX4's position is moved onto the UWB position. From then on
PX4 flies **in the anchors' coordinates**, so the position cannot drift, the
drone returns to the real takeoff spot, and tags land where they really are.

## What happens on `takeoff`

1. The laptop sends the ESP32 a rotation-only alignment: angle from
   `uwb.nose_deg`, y mirrored, **no offset**.
2. The ESP32 starts feeding UWB to PX4 while the drone is still on the ground.
   With no GPS, PX4 resets its position onto the first UWB reading.
3. The laptop waits until PX4 has sat within `snap_tol_m` (0.15 m) of UWB
   for `snap_hold_s` (2 s). Only then does it send `CMD_START`.
   It gives up after `snap_timeout_s` (20 s) and doesn't take off; type
   `takeoff` again once it's fixed.
4. In flight, `UWB_FRAME_OFF` is logged if PX4 and UWB ever differ by more than
   `frame_warn_m` (0.3 m): UWB lost, or PX4 rejecting it.

Events in `events.csv`: `UWB_FRAME_SNAP`, `UWB_FRAME_READY`, `UWB_FRAME_OFF`.

## Coordinates

With `nose_deg: 0` (nose along UWB +x at power-up):

| | PX4 / mission | UWB (NAssistant) |
|---|---|---|
| x | = UWB x | x |
| y | = **−** UWB y | y |

UWB is z-up and PX4 is z-down, so one axis has to flip.

- **`sweep.start_at_home: true`** (as now): the sweep is laid out from the
  takeoff spot along PX4's axes, same as before. Only the frame underneath
  is now the anchors.
- **`sweep.start_at_home: false`**: the `area` box is in PX4 coordinates, i.e.
  UWB x and −UWB y. Example: anchors cover UWB x 0..4, y 0..3 → area x 0..4,
  y −3..0.
- The tag report also prints every tag in UWB coordinates.

## The one thing to get right: the nose at power-up

PX4 has no compass. Its heading 0 is wherever the nose pointed when PX4
powered up, so `nose_deg` must say which UWB direction that was:

| `nose_deg` | Nose at PX4 power-up |
|---|---|
| 0 | along UWB +x (A0 → A3 in the usual layout) |
| 90 | along UWB +y |
| 180 | along UWB −x |
| −90 | along UWB −y |

Mark the spot and the direction on the floor. A wrong angle can't be seen on
the ground (the positions match where the drone stands), but gets worse with
distance: about 1.7 cm per metre per degree. In flight it shows up as
optical flow and UWB disagreeing: wobble, drift, `UWB_FRAME_OFF`.

## Settings

**`mission/config.yaml`**

```yaml
uwb:
  send_to_px4: false     # the in-flight method; must be off
  ground_frame: true
  nose_deg: 0.0
```

**Firmware** (menuconfig → UWB (LinkTrack)): UWB on, RX pin, debug stream on,
**Send UWB position to PX4 on**. This firmware keeps a ground alignment through
`CMD_START` (it's cleared before each READY instead) and re-captures the hold
position after `CMD_START`, so OFFBOARD starts on the new spot. Reflash.

**PX4** (QGroundControl, then reboot): `EKF2_EV_CTRL` 1, `EKF2_EV_NOISE_MD` 0,
`EKF2_EVP_NOISE` 0.15, `EKF2_EVP_GATE` 3, `EKF2_EV_DELAY` 50,
`EKF2_EV_POS_X/Y/Z` −0.04 / 0 / 0.03.

## If it won't take off

| Message | Fix |
|---|---|
| `takeoff refused: no usable UWB reading` | UWB or debug stream off in firmware, anchors not up, or the firewall blocks UDP 5010 |
| `PX4 did not move onto UWB within 20 s` | `EKF2_EV_CTRL` not 1 (or PX4 not rebooted), firmware without `UWB_SEND_TO_PX4`, or UWB precision too poor (`UWB_PX4_MAX_EOP_CM`) |
