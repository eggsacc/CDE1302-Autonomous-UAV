"""
Bench test of the UWB -> PX4 link, props off, no flight.  Run from cde1302/:

    python mission/tools/uwb_px4_bench.py            # only watch and report
    python mission/tools/uwb_px4_bench.py --send     # also send the alignment

Carry the drone slowly (at least uwb.align_min_span_m) with PX4 running.  Every
second this prints how far the fit has got; once it is good, it shows how far
apart the two position sources are and the ESP32's counters.  With --send the
alignment goes to the drone, which starts feeding PX4.  `--clear` stops that.

Never run this while the drone is flying: it does not check, and a bad
alignment moves PX4's position.  In flight the mission does the same job.
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from settings import DEFAULT_CONFIG_PATH, load_settings  # noqa: E402
from uwb_feed import UwbFeed  # noqa: E402
from uwb_link import build_align_clear, build_align_command  # noqa: E402

PRINT_PERIOD_S = 1.0


def status(feed: UwbFeed) -> str:
    return (f"esp32: px4_fed {feed.counter('px4_fed')}, "
            f"rejected {feed.counter('px4_rejected')}, "
            f"alignments {feed.align_received} | "
            f"wifi: {feed.link.packets} pkts, {feed.link.lost} lost")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    ap.add_argument("--seconds", type=float, default=120.0, help="stop after this long")
    ap.add_argument("--send", action="store_true", help="send the alignment to the drone")
    ap.add_argument("--clear", action="store_true", help="tell the drone to stop, then exit")
    args = ap.parse_args(argv)

    settings = load_settings(Path(args.config))
    dest = (settings.esp_ip, settings.network.cmd_port)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.clear:
        tx.sendto(build_align_clear(), dest)
        print(f"Clear sent to {dest[0]}. The ESP32 stops feeding PX4.")
        return 0

    feed = UwbFeed(settings)
    try:
        feed.start()
    except OSError as exc:
        print(f"error: cannot listen on UDP {settings.uwb.debug_port}: {exc}", file=sys.stderr)
        return 1
    feed.begin()
    print(f"Listening. Mirror setting: {settings.uwb.mirror}. Carry the drone slowly. "
          "Ctrl+C to stop.")
    alignment = None
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(PRINT_PERIOD_S)
            if feed.link.packets == 0:
                print("no packets yet: Wi-Fi up? laptop IP = firmware host IP? firewall?")
                continue
            r = feed.solve()
            if alignment is None and r.ok:
                alignment = r.alignment
                feed.finish()
                print(f"FIT OK: theta {math.degrees(alignment.theta):.1f} deg, "
                      f"mirror {alignment.mirror}, "
                      f"t ({alignment.tx:.3f}, {alignment.ty:.3f}) m, "
                      f"rms {r.rms_m:.3f} m, {r.n_used} samples")
            if alignment is None:
                print(f"fit: {r.reason} | {status(feed)}")
                continue
            if args.send:
                tx.sendto(build_align_command(alignment.theta, alignment.mirror,
                                              alignment.tx, alignment.ty), dest)
            s = feed.latest()
            if s is None:
                print(f"no usable fix right now | {status(feed)}")
                continue
            ux, uy = alignment.to_px4(s.ux, s.uy)
            print(f"UWB in PX4 frame ({ux:+.2f}, {uy:+.2f}), PX4 ({s.px:+.2f}, {s.py:+.2f}), "
                  f"apart {math.hypot(ux - s.px, uy - s.py):.2f} m | {status(feed)}")
    except KeyboardInterrupt:
        pass
    finally:
        feed.stop()
    return 0 if alignment is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
