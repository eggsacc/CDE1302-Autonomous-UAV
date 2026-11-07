#!/usr/bin/env python3
"""
Lawnmower sweep in the UWB frame, with the bounds relative to the takeoff spot.

    UWB on the ground -> PX4 snaps to UWB -> takeoff
        -> corner by corner: send the waypoint, UWB confirms it, hover -> home -> land

The bounds are offsets (m) from where the drone stands before takeoff, along
the UWB axes.  Lanes run along y and step across in x by --spacing:

  * the first lane is on xmax (--first-lane xmax, default) or xmin;
  * each lane starts on the y edge nearer the takeoff spot, so the first leg is
    straight across, not diagonal.

    e.g. --xmin -0.5 --xmax 0.5 --ymin -3 --ymax 0, nose along -y (--nose-deg -90):
        (0.5, 0) (0.5, -3) (0, -3) (0, 0) (-0.5, 0) (-0.5, -3), then home (0, 0)
        = left, forward, right, back, right, forward, home   (facing -y, +x is left)

Every corner is sent to PX4 as one waypoint (heading fixed) and PX4 flies there
by itself; its speed comes from PX4 (MPC_XY_P x distance, capped by
MPC_XY_VEL_MAX: set that low, e.g. 0.3 m/s, before flying indoors).  The next
corner is only sent once UWB says the drone is within --reach-tol of this one,
and after a --corner-hover second hover.  If a corner is not reached within
--reach-timeout it says so and carries on.

The heading stays fixed the whole time.  It uses the UWB frame the same way
simple_uwb_frame_goal.py does, so the nose must point along a known UWB
direction when PX4 powers up (--nose-deg, see that script), and it needs the
firmware that keeps a ground alignment through CMD_START.

    python simple_uwb_lawnmower.py --drone-id 2 --nose-deg -90 \\
        --xmin -0.5 --xmax 0.5 --ymin -3 --ymax 0 --confirm
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import time

from comms import CommsNode
from simple_uwb_calibration import UwbPairs
from simple_uwb_frame_goal import fly_to, frame_alignment, snap_error, wait_for_snap
from simple_uwb_set_goal import Drone, to_uwb

log = logging.getLogger("uwb_lawnmower")


def lawnmower(xmin: float, xmax: float, ymin: float, ymax: float, spacing: float,
              first_lane: str = "xmax") -> list:
    """Corner points (x, y) of the sweep, relative to takeoff.

    Lanes along y, stepping in x from the first_lane edge to the other one (a
    last, narrower lane lands exactly on it).  Each sweep starts on the y edge
    nearer 0 (the takeoff spot)."""
    if xmin > xmax or ymin > ymax:
        raise ValueError("need xmin <= xmax and ymin <= ymax")
    if spacing <= 0:
        raise ValueError("spacing must be > 0")
    if first_lane not in ("xmin", "xmax"):
        raise ValueError("first_lane must be xmin or xmax")
    start, end = (xmax, xmin) if first_lane == "xmax" else (xmin, xmax)
    step = spacing if end >= start else -spacing
    n = int(math.floor(abs(end - start) / spacing + 1e-9))
    xs = [start + k * step for k in range(n + 1)]
    if abs(end - xs[-1]) > 1e-6:
        xs.append(end)
    near, far = (ymin, ymax) if abs(ymin) <= abs(ymax) else (ymax, ymin)
    pts = []
    for i, x in enumerate(xs):
        y0, y1 = (near, far) if i % 2 == 0 else (far, near)
        pts += [(round(x, 6), y0), (round(x, 6), y1)]
    return pts


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s",
                        datefmt="%H:%M:%S")
    ap = argparse.ArgumentParser(description="Lawnmower sweep in the UWB frame, relative to takeoff")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--xmin", type=float, required=True, help="m from takeoff, along UWB x")
    ap.add_argument("--xmax", type=float, required=True)
    ap.add_argument("--ymin", type=float, required=True, help="m from takeoff, along UWB y")
    ap.add_argument("--ymax", type=float, required=True)
    ap.add_argument("--spacing", type=float, default=0.5, help="distance between lanes (m)")
    ap.add_argument("--first-lane", choices=["xmax", "xmin"], default="xmax",
                    help="which x edge the first lane is on")
    ap.add_argument("--corner-hover", type=float, default=2.0, help="seconds to hover at each corner")
    ap.add_argument("--reach-tol", type=float, default=0.15, help="a corner counts as reached this close (m)")
    ap.add_argument("--reach-timeout", type=float, default=30.0,
                    help="seconds to wait for a corner before carrying on")
    ap.add_argument("--nose-deg", type=float, default=0.0,
                    help="nose direction at PX4 power-up, deg from UWB +x towards +y")
    ap.add_argument("--mirror", choices=["yes", "no"], default="yes")
    ap.add_argument("--max-distance", type=float, default=5.0,
                    help="refuse a sweep reaching farther than this from takeoff (m)")
    ap.add_argument("--snap-tol", type=float, default=0.15,
                    help="PX4 must sit this close to UWB before takeoff (m)")
    ap.add_argument("--takeoff-wait", type=float, default=15.0)
    ap.add_argument("--track", help="save the flown UWB and PX4 track to this CSV")
    ap.add_argument("--confirm", action="store_true", help="require typing SWEEP-<id> first")
    ap.add_argument("--uwb-port", type=int, default=5010)
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()
    al = frame_alignment(args.nose_deg, args.mirror == "yes")

    corners = lawnmower(args.xmin, args.xmax, args.ymin, args.ymax, args.spacing, args.first_lane)
    far = max(math.hypot(x, y) for x, y in corners)
    if far > args.max_distance:
        raise SystemExit(f"sweep reaches {far:.1f} m from takeoff (> --max-distance {args.max_distance})")
    route = [(0.0, 0.0)] + corners + [(0.0, 0.0)]          # out from takeoff, and back
    if math.dist(route[0], route[1]) < 1e-6:                # sweep starts on the takeoff spot
        route = route[1:]
    legs = route[1:]
    length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    print("\nSweep corners (m from takeoff, UWB axes):")
    for k, (x, y) in enumerate(legs, 1):
        print(f"  {k:2d}  ({x:+.2f}, {y:+.2f})" + ("   home" if k == len(legs) else ""))
    print(f"  {len(corners) // 2} lanes, {length:.1f} m, {args.corner_hover:.0f} s hover per corner\n")

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

        if not wait_for_snap(comms, args.drone_id, uwb, al, args.snap_tol):
            raise RuntimeError("PX4 did not move onto the UWB position: is EKF2_EV_CTRL 1 and "
                               "the new firmware flashed? (not taking off)")
        x0, y0 = uwb.latest[:2]                           # the takeoff spot, in UWB
        log.info("PX4 is in the UWB frame; takeoff spot UWB (%.2f, %.2f)", x0, y0)
        log.info("sweep area: UWB x %.2f..%.2f, y %.2f..%.2f",
                 x0 + args.xmin, x0 + args.xmax, y0 + args.ymin, y0 + args.ymax)
        print("\nCorners in UWB coordinates:")
        for k, (x, y) in enumerate(legs, 1):
            print(f"  {k:2d}  ({x0 + x:.2f}, {y0 + y:.2f})" + ("   home" if k == len(legs) else ""))
        print()

        if args.confirm and input(f"Type SWEEP-{args.drone_id} to fly: ").strip() != f"SWEEP-{args.drone_id}":
            log.info("cancelled")
            return

        drone.start()
        flying = True
        log.info("CMD_START sent, waiting %.0f s for the hover", args.takeoff_wait)
        time.sleep(args.takeoff_wait)
        err = snap_error(al, uwb.latest)
        if err > 2 * args.snap_tol:
            raise RuntimeError(f"PX4 is {err:.2f} m off UWB after takeoff: UWB lost?")

        yaw = drone.pkt.heading_rad                       # hold this heading for every corner
        log.info("holding heading %.1f deg", math.degrees(yaw))
        uwb.recording = True
        results = []
        for k, (x, y) in enumerate(legs, 1):
            target_uwb = (x0 + x, y0 + y)
            name = f"corner {k}/{len(legs)} UWB ({target_uwb[0]:.2f}, {target_uwb[1]:.2f})"
            ok, err = fly_to(drone, uwb, al, target_uwb, yaw, args.reach_tol,
                             args.reach_timeout, name)
            if ok:
                log.info("%s reached, %.2f m off; hovering %.0f s", name, err, args.corner_hover)
            else:
                log.warning("%s NOT reached within %.0f s (%.2f m off) - carrying on",
                            name, args.reach_timeout, err)
            results.append((k, *target_uwb, ok, err))
            time.sleep(args.corner_hover)
        uwb.recording = False

        print("\n=== UWB sweep ===")
        print(f"takeoff    UWB ({x0:.2f}, {y0:.2f})")
        for k, x, y, ok, err in results:
            print(f"  corner {k:2d} UWB ({x:.2f}, {y:.2f})  {'reached' if ok else 'MISSED '}  {err:.3f} m off")
        print(f"recorded   {len(uwb.pairs)} UWB readings")
        drone.land()
        flying = False
        if args.track:
            with open(args.track, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["uwb_x", "uwb_y", "px4_as_uwb_x", "px4_as_uwb_y"])
                for ux, uy, px, py in uwb.pairs:
                    qx, qy = to_uwb(al, px, py)
                    w.writerow([f"{ux:.3f}", f"{uy:.3f}", f"{qx:.3f}", f"{qy:.3f}"])
            log.info("track saved to %s (UWB coordinates)", args.track)
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
