#!/usr/bin/env python3
"""
UWB calibration flight: fly a square, work out how UWB lines up with PX4,
then land back where it started.

    takeoff -> square (side --size, heading fixed) -> back at the start -> land

While flying, the ESP32 streams every UWB reading together with PX4's position
at that same instant (UDP 5010).  Afterwards the script fits the one rotation,
mirror and offset that map UWB onto PX4 (see UWB_README.md) and prints them:

    theta   angle between UWB's x axis and PX4's x axis (the drone's nose at
            PX4 power-up), degrees
    mirror  whether one axis is flipped (UWB is z-up, PX4 is z-down: expect yes)
    t       where UWB's origin (anchor A0) is in PX4's frame, metres
    rms     how well the fit matches, metres; under about 0.10 is good

They are saved to --out.  They stay valid as long as PX4 is not restarted, or
if it is always powered up on the same spot facing the same way.
With --send the result also goes to the ESP32, which then feeds UWB to PX4
for the rest of the flight (the return to the start already uses it).

Needs firmware with UWB on and the debug stream on (menuconfig -> UWB
(LinkTrack)); --send also needs "Send UWB position to PX4".

    python laptop/simple_uwb_calibration.py --drone-id 2 --confirm
    python laptop/simple_uwb_calibration.py --drone-id 2 --size 1.2 --send
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import random
import socket
import struct
import threading
import time
from pathlib import Path

import numpy as np

from comms import CommsNode
from protocol import (
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_TRAJ,
    PKT_CMD,
    CommandPacket,
    build_traj_packets,
    build_traj_start,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("uwb_calibration")

UPLOAD_GAP_S = 0.06       # ESP drains commands at 10 Hz from a 6-packet UDP queue
DT_S = 0.20               # trajectory sample period; the ESP interpolates between samples
MAX_EOP_M = 0.30          # ignore UWB readings with worse x/y precision
MAX_PX4_AGE_MS = 200      # ignore pairs whose PX4 position is older than this
MIN_PAIRS = 50

# ---------------------------------------------------------------------------
# UWB debug stream from the ESP32 (main/uwb_debug_pack.h, version 2)
# ---------------------------------------------------------------------------
DBG_HEADER = struct.Struct("<4sBBHII12I")      # magic, version, drone, count, seq, ms, counters
DBG_RECORD = struct.Struct("<Q4fHH")           # esp time us, px4 x y z heading, age ms, frame len
PX4_AGE_NONE = 0xFFFF

# CMD_UWB_ALIGN (main/uwb_px4.h): pkt, cmd, flags, theta, tx, ty
CMD_UWB_ALIGN = 0x0A
ALIGN_CMD = struct.Struct("<BBBfff")


def parse_debug_packet(data: bytes):
    """Yield (px4_x, px4_y, px4_age_ms or None, nlink_frame) from one UDP payload."""
    if len(data) < DBG_HEADER.size:
        return
    magic, version, _drone, n, *_ = DBG_HEADER.unpack_from(data)
    if magic != b"UWBD" or version != 2:
        return
    pos = DBG_HEADER.size
    for _ in range(n):
        if pos + DBG_RECORD.size > len(data):
            return
        _t, x, y, _z, _h, age, flen = DBG_RECORD.unpack_from(data, pos)
        pos += DBG_RECORD.size
        frame = data[pos:pos + flen]
        pos += flen
        yield x, y, (None if age == PX4_AGE_NONE else age), frame


def int24(b: bytes, i: int) -> int:
    v = b[i] | b[i + 1] << 8 | b[i + 2] << 16
    return v - (1 << 24) if v & 0x800000 else v


def decode_node_frame2(f: bytes):
    """(x, y, eop_xy) in metres from a LinkTrack NODE_FRAME2, or None if unusable."""
    if len(f) < 22 or f[0] != 0x55 or f[1] != 0x04 or sum(f[:-1]) & 0xFF != f[-1]:
        return None
    eop_x, eop_y = f[10] / 100.0, f[11] / 100.0
    x, y = int24(f, 13) / 1000.0, int24(f, 16) / 1000.0
    if max(eop_x, eop_y) >= 2.55 or abs(x) > 1000 or abs(y) > 1000:
        return None                                   # no x/y estimate
    return x, y, max(eop_x, eop_y)


class UwbPairs:
    """Collects (uwb_x, uwb_y, px4_x, px4_y) pairs from the debug stream."""

    def __init__(self, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.pairs: list[tuple[float, float, float, float]] = []
        self.latest = None          # newest usable (uwb_x, uwb_y, px4_x, px4_y), any time
        self.readings = 0
        self.recording = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.sock.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.sock.recv(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            for px, py, age, frame in parse_debug_packet(data):
                self.readings += 1
                u = decode_node_frame2(frame)
                if u is not None and u[2] <= MAX_EOP_M and age is not None and age <= MAX_PX4_AGE_MS:
                    self.latest = (u[0], u[1], px, py)
                    if self.recording:
                        self.pairs.append(self.latest)


# ---------------------------------------------------------------------------
# Fit: p = R(theta) @ M @ u + t  (2-D rigid fit, with an optional mirror)
# ---------------------------------------------------------------------------
def fit(u: np.ndarray, p: np.ndarray, mirror: bool):
    """Least-squares rotation + offset.  Returns (theta, tx, ty, residuals)."""
    um = u * np.array([1.0, -1.0]) if mirror else u
    cu, cp = um.mean(axis=0), p.mean(axis=0)
    a, b = um - cu, p - cp
    theta = math.atan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]),
                       np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = cp - R @ cu
    res = np.linalg.norm(um @ R.T + t - p, axis=1)
    return theta, float(t[0]), float(t[1]), res


def robust_fit(u: np.ndarray, p: np.ndarray, mirror: bool):
    """fit(), drop pairs more than 3x the median residual away, fit again."""
    theta, tx, ty, res = fit(u, p, mirror)
    keep = res <= max(3.0 * float(np.median(res)), 0.01)
    if keep.sum() >= MIN_PAIRS and not keep.all():
        theta, tx, ty, res = fit(u[keep], p[keep], mirror)
    return theta, tx, ty, float(np.sqrt(np.mean(res ** 2))), int(len(res))


def calibrate(pairs, mirror: str) -> dict:
    data = np.array(pairs)
    u, p = data[:, :2], data[:, 2:]
    choices = {"yes": [True], "no": [False], "auto": [False, True]}[mirror]
    fits = sorted(((robust_fit(u, p, m), m) for m in choices), key=lambda f: f[0][3])
    (theta, tx, ty, rms, n), m = fits[0]
    out = {"theta_deg": math.degrees(theta), "mirror": m, "tx": tx, "ty": ty,
           "rms_m": rms, "pairs_used": n, "pairs_total": len(pairs),
           "span_m": float(np.ptp(p, axis=0).max())}
    if len(fits) > 1:
        out["other_mirror_rms_m"] = fits[1][0][3]
    return out


# ---------------------------------------------------------------------------
# Flight
# ---------------------------------------------------------------------------
def square_path(size: float, speed: float, side: str) -> np.ndarray:
    """Offsets (x, y, z) around a square from the start, back to the start."""
    sy = size if side == "right" else -size
    corners = np.array([[0, 0], [size, 0], [size, sy], [0, sy], [0, 0]], float)
    pts = [corners[0]]
    step = speed * DT_S
    for a, b in zip(corners, corners[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        pts += [a + (b - a) * k / n for k in range(1, n + 1)]
    xy = np.array(pts)
    return np.column_stack([xy, np.zeros(len(xy))])


def main() -> None:
    ap = argparse.ArgumentParser(description="Fly a square and calibrate UWB against PX4")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--size", type=float, default=1.0, help="square side (m)")
    ap.add_argument("--side", choices=["right", "left"], default="right",
                    help="which way the square goes from the start (seen from the drone)")
    ap.add_argument("--speed", type=float, default=0.15, help="planned speed (m/s)")
    ap.add_argument("--mirror", choices=["yes", "no", "auto"], default="yes",
                    help="yes for LinkTrack + PX4; auto tests both")
    ap.add_argument("--send", action="store_true",
                    help="send the result to the ESP32 so PX4 uses UWB from then on")
    ap.add_argument("--out", default=str(Path(__file__).with_name("uwb_calibration.json")))
    ap.add_argument("--takeoff-wait", type=float, default=15.0,
                    help="seconds from CMD_START to the trajectory upload")
    ap.add_argument("--confirm", action="store_true", help="require typing CAL-<id> first")
    ap.add_argument("--uwb-port", type=int, default=5010)
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()

    pts = square_path(args.size, args.speed, args.side)
    duration = (len(pts) - 1) * DT_S
    log.info("square %.2f m to the %s: %d points, about %.0f s",
             args.size, args.side, len(pts), duration)

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

    def land() -> None:
        for _ in range(3):                                  # UDP: repeat LAND
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
        if uwb.readings == 0:
            raise RuntimeError(f"no UWB debug stream on UDP {args.uwb_port}: UWB on in the "
                               "firmware? debug stream on? firewall?")
        if args.confirm and input(f"Type CAL-{args.drone_id} to fly: ").strip() != f"CAL-{args.drone_id}":
            log.info("cancelled")
            return

        comms.send_command(args.drone_id, CommandPacket(CMD_START))
        flying = True
        log.info("CMD_START sent, waiting %.0f s for the hover", args.takeoff_wait)
        time.sleep(args.takeoff_wait)

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
            log.warning("attempt %d: square not started (see ESP log)", attempt)
        else:
            raise RuntimeError("drone refused the trajectory")

        uwb.recording = True
        log.info("flying the square and recording UWB/PX4 pairs")
        end = time.monotonic() + 4 * duration + 15.0      # the ESP waits for a slow drone
        result = None
        while latest["pkt"].nav_state == NAV_TRAJ and time.monotonic() < end:
            p = latest["pkt"]
            log.info("pos (%+.2f, %+.2f)  pairs %d", p.ned_x, p.ned_y, len(uwb.pairs))
            if args.send and result is None and len(uwb.pairs) > 3 * MIN_PAIRS:
                done = np.array(uwb.pairs)[:, 2:]
                if np.ptp(done, axis=0).min() > 0.6 * args.size:       # two sides flown
                    result = calibrate(list(uwb.pairs), args.mirror)
                    send_alignment(comms, args.drone_id, result)
            time.sleep(1.0)
        uwb.recording = False
        log.info("square finished, nav state %s", latest["pkt"].nav_state_name)
        land()
        flying = False

        if len(uwb.pairs) < MIN_PAIRS:
            raise RuntimeError(f"only {len(uwb.pairs)} usable UWB/PX4 pairs")
        final = calibrate(uwb.pairs, args.mirror)
        report(final, args)
        Path(args.out).write_text(json.dumps({**final, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                                              "size_m": args.size}, indent=2))
        log.info("saved to %s", args.out)
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted - landing")
            land()
        raise
    finally:
        uwb.stop()
        comms.stop()


def send_alignment(comms, drone_id: int, r: dict) -> None:
    flags = 0x01 | (0x02 if r["mirror"] else 0)
    pkt = ALIGN_CMD.pack(PKT_CMD, CMD_UWB_ALIGN, flags, math.radians(r["theta_deg"]),
                         r["tx"], r["ty"])
    for _ in range(3):
        comms.send_raw(drone_id, pkt)
        time.sleep(0.1)
    log.info("alignment sent to the ESP32 (rms %.3f m): PX4 now uses UWB", r["rms_m"])


def report(r: dict, args) -> None:
    ok = r["rms_m"] <= 0.10 and r["span_m"] >= 0.6 * args.size
    print("\n=== UWB calibration ===")
    print(f"theta   {r['theta_deg']:+.1f} deg")
    print(f"mirror  {'yes' if r['mirror'] else 'no'}")
    print(f"t       ({r['tx']:+.3f}, {r['ty']:+.3f}) m   (UWB origin A0 in PX4's frame)")
    print(f"rms     {r['rms_m']:.3f} m from {r['pairs_used']} of {r['pairs_total']} pairs")
    if "other_mirror_rms_m" in r:
        print(f"        other mirror choice: {r['other_mirror_rms_m']:.3f} m")
    print("VERDICT: " + ("good" if ok else "poor: fly again, check the anchors and keep "
                         "people out of their line of sight"))


if __name__ == "__main__":
    main()
