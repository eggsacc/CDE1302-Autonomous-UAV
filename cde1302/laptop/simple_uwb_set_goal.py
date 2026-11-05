#!/usr/bin/env python3
"""
Fly to a goal given in UWB coordinates, with PX4 using UWB.

    takeoff -> alignment (given, or a calibration square) -> PX4 uses UWB
            -> fly to the goal (heading fixed) -> hover, measure the error -> land

The goal is in UWB (room) coordinates: the x/y NAssistant shows, metres from
anchor A0.  The alignment converts it to PX4 coordinates
(p = R(theta) x M x u + t, see UWB_README.md).  It comes from, in this order:

    --theta --mirror --tx --ty    values you give
    --calibration FILE            a file saved by simple_uwb_calibration.py
    neither                       a calibration square is flown first

A given alignment is checked before use: if it would put the drone more than
--max-jump away from where PX4 thinks it is, the drone lands instead, because
the values were made with a different start pose.

theta = 0 when the drone's nose points along UWB's x axis at PX4 power-up.
With the usual LinkTrack layout that is the direction from A0 to A3 (check in
NAssistant: the anchor at y = 0 besides A0 defines the x axis).  Then, with
mirror = yes, t = (-x0, +y0), where (x0, y0) is the takeoff spot in UWB
coordinates.

    python laptop/simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --confirm
    python laptop/simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --calibration laptop/uwb_calibration.json
    python laptop/simple_uwb_set_goal.py --drone-id 2 --goal 2.0 1.5 --theta 0 --mirror yes --tx -1.2 --ty 3.4
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import random
import time
from pathlib import Path

import numpy as np

from comms import CommsNode
from protocol import CMD_LAND, CMD_START, NAV_TRAJ, CommandPacket, build_traj_packets, build_traj_start
from simple_uwb_calibration import (DT_S, MIN_PAIRS, UPLOAD_GAP_S, UwbPairs, calibrate,
                                    send_alignment, square_path)

log = logging.getLogger("uwb_set_goal")


def to_px4(al: dict, ux: float, uy: float) -> tuple[float, float]:
    """UWB (x, y) -> PX4 local (x, y) with an alignment dict (theta_deg, mirror, tx, ty)."""
    th = math.radians(al["theta_deg"])
    my = -uy if al["mirror"] else uy
    return (math.cos(th) * ux - math.sin(th) * my + al["tx"],
            math.sin(th) * ux + math.cos(th) * my + al["ty"])


def to_uwb(al: dict, px: float, py: float) -> tuple[float, float]:
    """The inverse of to_px4."""
    th = math.radians(al["theta_deg"])
    dx, dy = px - al["tx"], py - al["ty"]
    ux = math.cos(th) * dx + math.sin(th) * dy
    my = -math.sin(th) * dx + math.cos(th) * dy
    return ux, (-my if al["mirror"] else my)


def line_path(start, goal, speed: float) -> np.ndarray:
    """Offsets (x, y, z) from start to goal at constant speed."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    n = max(1, int(math.ceil(np.linalg.norm(goal - start) / (speed * DT_S))))
    xy = np.array([(goal - start) * k / n for k in range(n + 1)])
    return np.column_stack([xy, np.zeros(len(xy))])


def alignment_from_args(args) -> dict | None:
    given = [args.theta, args.mirror, args.tx, args.ty]
    if any(v is not None for v in given):
        if any(v is None for v in given):
            raise SystemExit("give all of --theta --mirror --tx --ty, or none of them")
        return {"theta_deg": args.theta, "mirror": args.mirror == "yes", "tx": args.tx, "ty": args.ty}
    if args.calibration:
        r = json.loads(Path(args.calibration).read_text())
        return {k: r[k] for k in ("theta_deg", "mirror", "tx", "ty")}
    return None


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s",
                        datefmt="%H:%M:%S")
    ap = argparse.ArgumentParser(description="Fly to a goal in UWB coordinates using UWB")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--goal", type=float, nargs=2, required=True, metavar=("X", "Y"),
                    help="goal in UWB coordinates (m, from anchor A0)")
    ap.add_argument("--theta", type=float, help="alignment angle (deg)")
    ap.add_argument("--mirror", choices=["yes", "no"], help="alignment mirror")
    ap.add_argument("--tx", type=float, help="alignment offset x (m)")
    ap.add_argument("--ty", type=float, help="alignment offset y (m)")
    ap.add_argument("--calibration", help="alignment file from simple_uwb_calibration.py")
    ap.add_argument("--size", type=float, default=1.0, help="calibration square side (m)")
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--speed", type=float, default=0.15, help="planned speed (m/s)")
    ap.add_argument("--hover", type=float, default=5.0, help="seconds to hover at the goal")
    ap.add_argument("--max-distance", type=float, default=4.0, help="refuse goals farther than this (m)")
    ap.add_argument("--max-jump", type=float, default=0.30,
                    help="refuse a given alignment this far from PX4's position (m)")
    ap.add_argument("--takeoff-wait", type=float, default=15.0)
    ap.add_argument("--confirm", action="store_true", help="require typing GOAL-<id> first")
    ap.add_argument("--uwb-port", type=int, default=5010)
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()
    alignment = alignment_from_args(args)

    latest = {}

    def on_telemetry(pkt, _ip):
        if pkt.drone_id == args.drone_id:
            latest["pkt"] = pkt

    def wait_for(cond, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if "pkt" in latest and cond(latest["pkt"]):
                return True
            time.sleep(0.05)
        return False

    def fly(pts: np.ndarray, name: str) -> None:
        """Upload a trajectory, start it and wait until the ESP32 reports it done."""
        traj_id = random.randint(1, 255)
        packets = build_traj_packets(traj_id, pts)
        start = build_traj_start(traj_id, round(DT_S * 1000))
        for attempt in range(1, 6):
            for pkt in packets:
                comms.send_raw(args.drone_id, pkt)
                time.sleep(UPLOAD_GAP_S)
            comms.send_raw(args.drone_id, start)
            if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
                break
            log.warning("%s attempt %d: not started (see ESP log)", name, attempt)
        else:
            raise RuntimeError(f"drone refused the {name}")
        end = time.monotonic() + 4 * len(pts) * DT_S + 15.0   # the ESP waits for a slow drone
        while latest["pkt"].nav_state == NAV_TRAJ and time.monotonic() < end:
            p = latest["pkt"]
            log.info("%s: pos (%+.2f, %+.2f)", name, p.ned_x, p.ned_y)
            time.sleep(1.0)
        log.info("%s done, nav state %s", name, latest["pkt"].nav_state_name)

    def land() -> None:
        for _ in range(3):
            comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
            time.sleep(0.1)
        log.info("CMD_LAND sent")

    uwb = UwbPairs(args.uwb_port)
    comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port)
    comms.on_telemetry(on_telemetry)
    comms.start()
    uwb.start()
    flying = False
    try:
        if not wait_for(lambda p: True, 30.0):
            raise RuntimeError(f"no telemetry from drone {args.drone_id}")
        time.sleep(2.0)
        if uwb.latest is None:
            raise RuntimeError(f"no usable UWB on UDP {args.uwb_port}: UWB and the debug "
                               "stream on in the firmware? anchors up? firewall?")
        if args.confirm and input(f"Type GOAL-{args.drone_id} to fly: ").strip() != f"GOAL-{args.drone_id}":
            log.info("cancelled")
            return

        comms.send_command(args.drone_id, CommandPacket(CMD_START))
        flying = True
        log.info("CMD_START sent, waiting %.0f s for the hover", args.takeoff_wait)
        time.sleep(args.takeoff_wait)

        if alignment is None:
            log.info("no alignment given: flying a %.1f m calibration square", args.size)
            uwb.recording = True
            fly(square_path(args.size, args.speed, args.side), "calibration square")
            uwb.recording = False
            if len(uwb.pairs) < MIN_PAIRS:
                raise RuntimeError(f"only {len(uwb.pairs)} usable UWB/PX4 pairs")
            alignment = calibrate(uwb.pairs, "yes")
            log.info("calibrated: theta %+.1f deg, t (%+.3f, %+.3f), rms %.3f m",
                     alignment["theta_deg"], alignment["tx"], alignment["ty"], alignment["rms_m"])
            if alignment["rms_m"] > 0.10:
                log.warning("POOR FIT: rms %.3f m (over 0.10 m) - flying to the goal anyway, "
                            "expect it to land off target", alignment["rms_m"])
        else:
            ux, uy, px, py = uwb.latest
            jump = math.dist(to_px4(alignment, ux, uy), (px, py))
            log.info("given alignment: UWB and PX4 differ by %.2f m here", jump)
            if jump > args.max_jump:
                raise RuntimeError(f"given alignment is {jump:.2f} m off (> --max-jump): made "
                                   "with a different start pose? Calibrate again")

        send_alignment(comms, args.drone_id, {**alignment, "rms_m": alignment.get("rms_m", 0.0)})
        time.sleep(1.0)                                     # let PX4 take UWB in

        here = (latest["pkt"].ned_x, latest["pkt"].ned_y)
        goal = to_px4(alignment, *args.goal)
        dist = math.dist(here, goal)
        log.info("goal UWB (%.2f, %.2f) = PX4 (%+.2f, %+.2f), %.2f m away",
                 *args.goal, *goal, dist)
        if dist > args.max_distance:
            raise RuntimeError(f"goal {dist:.1f} m away (> --max-distance {args.max_distance})")
        fly(line_path(here, goal, args.speed), "goal")

        readings = []
        end = time.monotonic() + args.hover
        while time.monotonic() < end:
            if uwb.latest is not None:
                readings.append(uwb.latest[:2])
            time.sleep(0.1)
        land()
        flying = False
        if readings:
            mean = np.mean(readings, axis=0)
            err = math.dist(mean, args.goal)
            print("\n=== UWB goal ===")
            print(f"goal       ({args.goal[0]:.2f}, {args.goal[1]:.2f}) m, UWB coordinates")
            print(f"reached    ({mean[0]:.2f}, {mean[1]:.2f}) m, UWB average over {len(readings)} readings")
            print(f"error      {err:.3f} m")
            if "rms_m" in alignment:
                poor = "  POOR FIT (over 0.10 m)" if alignment["rms_m"] > 0.10 else ""
                print(f"fit rms    {alignment['rms_m']:.3f} m{poor}")
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted - landing")
            land()
        raise
    finally:
        uwb.stop()
        comms.stop()


if __name__ == "__main__":
    main()
