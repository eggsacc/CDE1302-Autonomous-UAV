# CDE1302 Autonomous UAV: AprilTag search

CDE1302 *Introduction to UAVs* mini project, AY2026/27.

A small indoor quadcopter takes off, sweeps an allocated rectangle in a
lawnmower pattern and records every AprilTag on the floor. It then returns to
its takeoff spot and lands. After the flight it reports each tag's position and
reconstructs the route it flew.

The drone is **pseudo-autonomous**. A laptop plans the sweep and runs the tag
detection over Wi-Fi, and the drone flies the uploaded path by itself. An
operator starts the mission and keeps the RC kill switch in hand.

> **[Image placeholder]** Drone photo / final run map (`map.png`)

## How it works

> **[Image placeholder]** System overview: laptop, ESP32-S3 and PX4, and what
> flows between them

| Device | Role |
|---|---|
| **Laptop** (Python) | Mission state machine and safety checks, lawnmower planner, AprilTag detection, run logs |
| **ESP32-S3** (companion computer) | Flies the uploaded path by sending position setpoints to PX4; streams camera frames, each tagged with PX4's pose at capture time |
| **PX4** (flight controller) | Position, velocity and attitude control. EKF2 fuses IMU, optical flow and rangefinder (no GPS, no compass) |

**Flight.** The laptop plans the sweep and uploads it to the ESP32 in one go.
The ESP32 then steers the drone corner to corner by itself, so a Wi-Fi drop
doesn't change the flight path. The laptop watches telemetry and lands the
drone if it leaves the geofence, the link goes stale, or time runs out.

**Tags.** The laptop detects tag16h5 tags in each frame and places them in PX4's
frame (camera → body → PX4 local NED) using the pose recorded with that frame.
A tag is confirmed once enough sightings cluster together.

**Drift correction.** PX4's position drifts over a flight. Localisation tags
at measured spots on the floor give it something fixed to compare against.
The home tag sets the landing spot. At the end of the flight, every visit to
a localisation tag is used to fit one transform that corrects the reported
tag positions and route.

## Control architecture

> **[Image placeholder]** Cascaded control loop: laptop (mission, 10 Hz) →
> ESP32 (path following, 20 Hz setpoints) → PX4 (position → velocity →
> attitude → motors), with EKF2 feeding position back

| Layer | Rate | Does |
|---|---|---|
| Laptop | 10 Hz | Mission states (take off → hover → sweep → return → land) and safety checks |
| ESP32 | 20 Hz | Picks the next corner of the path and sends it to PX4 as a position setpoint |
| PX4 | Onboard loops | Position → velocity → attitude → rate control, with EKF2 state estimation |

## Subsystems

| Subsystem | Contents |
|---|---|
| [Software](code/) | ESP32 firmware, course laptop tools and the search mission. [`code/README.md`](code/README.md) maps every folder; [`code/mission/README.md`](code/mission/README.md) explains how to fly |
| [Mechanical](frame/) | Redesigned 3D-printed frame: stiffer arms, no print supports, faster assembly. CAD + STLs |
| [Electrical](electrical/) | Onboard hardware, wiring and network |

## Repository layout

```
├── code/                 Software
│   ├── main/             ESP32-S3 firmware (ESP-IDF, FreeRTOS)
│   ├── mission/          AprilTag search mission, simulator, tests, demo flight logs
│   ├── laptop/           Course-provided laptop tools (comms, camera viewer, detector)
│   ├── trajectory/       MATLAB trajectory generators (course labs)
│   └── waypoints/        Waypoint test files (course labs)
├── frame/                Frame redesign: STEP, STLs, design notes
└── electrical/           Hardware architecture and wiring
```
