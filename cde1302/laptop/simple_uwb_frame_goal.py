#!/usr/bin/env python3
"""
Fly to a goal with the UWB anchors as the coordinate system from the ground up.

No calibration flight and no offset: before takeoff, the ESP32 starts sending
the raw UWB position to PX4, PX4 moves its position onto it, and from then on
PX4's position IS the UWB position (y flipped, see below).

    UWB on the ground -> PX4 snaps to UWB -> takeoff -> goal waypoint (heading fixed)
                      -> UWB confirms it -> hover, measure the error -> land

The goal is sent to PX4 as one waypoint: PX4 flies there by itself, so its
speed comes from PX4 (MPC_XY_P x distance, capped by MPC_XY_VEL_MAX: set that
low, e.g. 0.3 m/s, before flying indoors).

What this needs from you: THE DRONE'S NOSE MUST POINT IN A KNOWN DIRECTION
when PX4 powers up, because PX4 has no compass and its heading 0 is the nose
direction at power-up.

    --nose-deg 0     nose along UWB +x (A0 -> A3 in the usual layout)  [default]
    --nose-deg 90    nose along UWB +y
    --nose-deg 180   nose along UWB -x
    --nose-deg -90   nose along UWB -y

Every degree the nose is off by puts the drone about 1.7 cm per metre off, and
does not show up on the ground (see UWB_README.md).

PX4 x = UWB x, PX4 y = -UWB y (with --nose-deg 0): UWB's z points up and PX4's
points down, so one horizontal axis must flip.

The goal is relative to the takeoff spot, in metres along the UWB axes:
--goal 1 0 is 1 m along UWB +x from where the drone stands.  Positions are
printed in absolute UWB coordinates (from anchor A0).

    python simple_uwb_frame_goal.py --drone-id 2 --goal 1.0 0.0 --confirm

Needs the firmware that keeps a ground alignment through CMD_START
(main.c clears it only before READY).
"""

from __future__ import annotations

import argparse
import logging
import math
import time

import numpy as np

from comms import CommsNode
from simple_uwb_calibration import UwbPairs, send_alignment
from simple_uwb_set_goal import Drone, hover_and_report, to_px4, to_uwb

log = logging.getLogger("uwb_frame_goal")


def frame_alignment(nose_deg: float, mirror: bool = True) -> dict:
    """The alignment that makes PX4's frame the UWB frame: rotation only, no offset.

    theta turns the nose direction onto PX4 +x.  The mirror flips UWB y first,
    which also flips the sign of the nose angle, so theta = +nose with the
    mirror and -nose without it."""
    theta = nose_deg if mirror else -nose_deg
    return {"theta_deg": theta, "mirror": mirror, "tx": 0.0, "ty": 0.0, "rms_m": 0.0}


def snap_error(al: dict, latest) -> float:
    """How far PX4's position is from the UWB position (m), from one debug record."""
    ux, uy, px, py = latest
    return math.dist(to_px4(al, ux, uy), (px, py))


def wait_for_snap(comms, drone_id: int, uwb: UwbPairs, al: dict, tol_m: float,
                  hold_s: float = 2.0, timeout_s: float = 20.0) -> bool:
    """Send the alignment until PX4's position sits on UWB for hold_s."""
    deadline = time.monotonic() + timeout_s
    next_send = 0.0
    good_since = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:                 # resend: the ESP may still be getting READY
            send_alignment(comms, drone_id, al)
            next_send = now + 2.0
        err = snap_error(al, uwb.latest) if uwb.latest is not None else math.inf
        if err <= tol_m:
            good_since = good_since or now
            if now - good_since >= hold_s:
                return True
        else:
            good_since = None
        log.info("PX4 vs UWB: %.2f m", err)
        time.sleep(0.5)
    return False


class Reach:
    """Has the drone reached target?  Averages the last n UWB readings."""

    def __init__(self, target, tol_m: float, n: int = 10, need: int = 5):
        self.target, self.tol, self.n, self.need = target, tol_m, n, need
        self.recent: list = []
        self.err = math.inf

    def add(self, xy) -> float:
        self.recent = (self.recent + [tuple(xy)])[-self.n:]
        self.err = math.dist(np.mean(self.recent, axis=0), self.target)
        return self.err

    @property
    def reached(self) -> bool:
        return len(self.recent) >= self.need and self.err <= self.tol


def fly_to(drone: Drone, uwb: UwbPairs, al: dict, target_uwb, yaw: float, tol_m: float,
           timeout_s: float, name: str) -> tuple[bool, float]:
    """Send one waypoint (resent every second) and wait until UWB confirms it.

    Returns (reached, error in m).  Nothing else is sent until it returns."""
    px, py = to_px4(al, *target_uwb)
    reach = Reach(target_uwb, tol_m)
    deadline = time.monotonic() + timeout_s
    next_send = next_log = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:                 # resend: one UDP packet can be lost
            drone.waypoint(px, py, yaw)
            next_send = now + 1.0
        if uwb.latest is not None:
            reach.add(uwb.latest[:2])
            if reach.reached:
                return True, reach.err
            if now >= next_log:
                log.info("%s: UWB (%.2f, %.2f), %.2f m to go", name, *uwb.latest[:2],
                         math.dist(uwb.latest[:2], target_uwb))
                next_log = now + 1.0
        time.sleep(0.1)
    return False, reach.err


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s",
                        datefmt="%H:%M:%S")
    ap = argparse.ArgumentParser(description="Fly to a goal in the UWB frame, no calibration")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--goal", type=float, nargs=2, required=True, metavar=("X", "Y"),
                    help="goal in m from the takeoff spot, along the UWB axes")
    ap.add_argument("--nose-deg", type=float, default=0.0,
                    help="nose direction at PX4 power-up, deg from UWB +x towards +y")
    ap.add_argument("--mirror", choices=["yes", "no"], default="yes")
    ap.add_argument("--reach-tol", type=float, default=0.15, help="the goal counts as reached this close (m)")
    ap.add_argument("--reach-timeout", type=float, default=30.0,
                    help="seconds to wait for the goal before measuring anyway")
    ap.add_argument("--hover", type=float, default=5.0, help="seconds to hover at the goal")
    ap.add_argument("--max-distance", type=float, default=4.0, help="refuse goals farther than this (m)")
    ap.add_argument("--snap-tol", type=float, default=0.15,
                    help="PX4 must sit this close to UWB before takeoff (m)")
    ap.add_argument("--takeoff-wait", type=float, default=15.0)
    ap.add_argument("--confirm", action="store_true", help="require typing FRAME-<id> first")
    ap.add_argument("--uwb-port", type=int, default=5010)
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()
    al = frame_alignment(args.nose_deg, args.mirror == "yes")

    uwb = UwbPairs(args.uwb_port)
    comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port)
    drone = Drone(comms, args.drone_id)
    comms.start()
    uwb.start()
    flying = False
    try:
        if not drone.wait_for(lambda p: True, 30.0):
            raise RuntimeError(f"no telemetry from drone {args.drone_id}")
        time.sleep(2.0)
        if uwb.latest is None:
            raise RuntimeError(f"no usable UWB on UDP {args.uwb_port}: UWB and the debug "
                               "stream on in the firmware? anchors up? firewall?")

        ux, uy = uwb.latest[:2]
        log.info("on the ground at UWB (%.2f, %.2f); sending the UWB frame to PX4", ux, uy)
        if not wait_for_snap(comms, args.drone_id, uwb, al, args.snap_tol):
            raise RuntimeError("PX4 did not move onto the UWB position: is EKF2_EV_CTRL 1 and "
                               "the new firmware flashed? (not taking off)")
        log.info("PX4 is in the UWB frame")

        x0, y0 = uwb.latest[:2]                           # the takeoff spot, in UWB
        goal_uwb = (x0 + args.goal[0], y0 + args.goal[1])
        dist = math.hypot(*args.goal)
        if dist > args.max_distance:
            raise RuntimeError(f"goal {dist:.1f} m away (> --max-distance {args.max_distance})")
        log.info("goal %+.2f, %+.2f m from takeoff = UWB (%.2f, %.2f), %.2f m away",
                 *args.goal, *goal_uwb, dist)

        if args.confirm and input(f"Type FRAME-{args.drone_id} to fly: ").strip() != f"FRAME-{args.drone_id}":
            log.info("cancelled")
            return

        drone.start()
        flying = True
        log.info("CMD_START sent, waiting %.0f s for the hover", args.takeoff_wait)
        time.sleep(args.takeoff_wait)

        err = snap_error(al, uwb.latest)
        if err > 2 * args.snap_tol:
            raise RuntimeError(f"PX4 is {err:.2f} m off UWB after takeoff: UWB lost?")

        yaw = drone.pkt.heading_rad                       # hold this heading
        log.info("at UWB (%.2f, %.2f), heading %.1f deg: sending the goal waypoint",
                 *to_uwb(al, *drone.pos()), math.degrees(yaw))
        ok, err = fly_to(drone, uwb, al, goal_uwb, yaw, args.reach_tol, args.reach_timeout,
                         f"goal UWB ({goal_uwb[0]:.2f}, {goal_uwb[1]:.2f})")
        if ok:
            log.info("goal reached, %.2f m off", err)
        else:
            log.warning("goal NOT reached within %.0f s (%.2f m off)", args.reach_timeout, err)

        hover_and_report(uwb, goal_uwb, args.hover,
                         f"takeoff    ({x0:.2f}, {y0:.2f}) m, UWB coordinates; goal was "
                         f"{args.goal[0]:+.2f}, {args.goal[1]:+.2f} m from it")
        drone.land()
        flying = False
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted - landing")
            drone.land()
        raise
    finally:
        uwb.stop()
        comms.stop()


if __name__ == "__main__":
    main()
