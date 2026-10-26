#!/usr/bin/env python3
"""
Fly a CSV trajectory smoothly on one drone.

The whole trajectory is uploaded first; the ESP32 then plays it on its own
20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
jitter cannot disturb the flight.

CSV: fixed t,x,y,z; tangent t,x,y,z,yaw_rad (s, NED m, rad CW from North); '#' comments are allowed.
The trajectory is flown relative to where the drone hovers when it starts.

Example:
    python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
        --trajectory trajectory/circle_traj.csv
"""

import argparse
import logging
from pathlib import Path

import numpy as np



logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("send_trajectory")

UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


def load_trajectory(path: str, dt: float) -> np.ndarray:
    """Read t,x,y,z, resample every dt, return offsets from the first point."""
    from trajectory_reference import read_reference, sample_reference
    try:
        return sample_reference(read_reference(path), dt).xyz
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc



def main() -> None:
    ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--trajectory", help="CSV with columns t,x,y,z (s, NED m)")
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
                    help="Fleet config (optional)")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Upload sample period (s); 0.05 = ESP nav loop rate")
    ap.add_argument("--max-speed", type=float, default=0.30,
                    help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
    ap.add_argument("--takeoff-wait", type=float, default=8.0,
                    help="Seconds from CMD_START to trajectory upload")
    ap.add_argument("--finish", choices=["land", "hold"], default="land",
                    help="Action after the trajectory")
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing TRAJ-<drone id> before sending any flight commands")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    ap.add_argument("--yaw-mode", choices=["fixed", "tangent"], default="fixed")
    ap.add_argument("--dry-run", action="store_true", help="Validate without opening any sockets")
    ap.add_argument("--check-firmware", action="store_true", help="Read-only yaw v1 capability query")
    ap.add_argument("--esp-ip", help="Optional drone IP; otherwise discovered from telemetry")
    ap.add_argument("--max-yaw-rate", type=float, help="TA-reviewed rad/s; offline default 0.6")
    ap.add_argument("--stationary-speed", type=float, default=1e-4,
                    help="Direction-undefined speed in m/s; strict experiment uses 0.0001")
    ap.add_argument("--stale-timeout", type=float, default=3.0)
    ap.add_argument("--log-dir", default="logs/tangent_yaw", help="Local experiment records")
    from trajectory_runner import run
    try:
        run(ap.parse_args())
    except (ValueError, RuntimeError, OSError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
