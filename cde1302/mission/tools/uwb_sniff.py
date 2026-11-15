#!/usr/bin/env python3
"""
Check a LinkTrack UWB module's NLink output: are complete, correct frames
arriving, and at what rate?  Read-only: never writes to the module.

Run from the cde1302/ folder with the conda env active:

    python mission/tools/uwb_sniff.py --list                    # which COM port?
    python mission/tools/uwb_sniff.py --port COM5               # live check
    python mission/tools/uwb_sniff.py --port COM5 --save p-a.bin --duration 30
    python mission/tools/uwb_sniff.py --replay p-a.bin          # re-check a capture
    python mission/tools/uwb_sniff.py --udp 5010 --csv bench.csv  # drone's Wi-Fi stream

--udp reads the ESP32's UWB debug stream (main/uwb_task.c) instead of a
serial port: the raw frames the drone received, re-decoded here, plus the
drone's own counters and PX4's position.  The summary then checks each link
(module -> ESP32 UART -> ESP32 decoding -> Wi-Fi -> laptop) separately.
--csv writes one row per frame: time, UWB position/precision, PX4 position.

Works the same over the P-A's USB cable and over a USB-TTL adapter on a
UART (e.g. the P-AS2's GH1.25 port: module TX -> adapter RX, GND -> GND).
Close NAssistant first: only one program can open the port.

Every --interval seconds it prints throughput, frames per type, checksum
errors and the newest decoded frame.  At the end it prints a verdict and
exits 0 only if good frames arrived with no checksum errors.

Byte order is not stated in the NLink PDF; the decoder assumes little-endian.
The "sanity" line checks it: supply voltage must be plausible and the
module's local_time must count up.
"""

from __future__ import annotations

import argparse
import csv
import socket
import sys
import time
from pathlib import Path
from typing import BinaryIO, Iterator, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from nlink import (ROLES, NLinkParser, NodeFrame2, NodeFrame3,  # noqa: E402
                   TagFrame0, decode)
from uwb_link import LinkStats, UwbRecord, parse_packet  # noqa: E402

DEFAULT_BAUD = 921600          # LinkTrack factory default (User Manual §3.3)
VOLTAGE_RANGE = (3.0, 6.0)     # plausible interface supply, V (spec 3.6-5.5)


class Monitor:
    """Accumulates parser output and prints periodic and final reports."""

    def __init__(self, interval: float) -> None:
        self.parser = NLinkParser()
        self.interval = interval
        self.t0 = time.monotonic()
        self._next = self.t0 + interval
        self._last_bytes = 0
        self._last_good = 0
        self._last_t = self.t0
        self.latest = None                     # newest decoded frame
        self.latest_kind = ""
        self.prev_local_time: Optional[int] = None
        self.local_time_backwards = 0
        self.local_time_steps: list[int] = []
        self.voltages: list[float] = []
        self.ascii_bytes = 0                   # printable bytes seen (NMEA hint)

    def feed(self, chunk: bytes) -> None:
        self.ascii_bytes += sum(1 for c in chunk if 32 <= c < 127 or c in (10, 13))
        for frame in self.parser.feed(chunk):
            obj = decode(frame)
            self.latest_kind = frame.kind
            if obj is None:
                continue
            self.latest = obj
            if hasattr(obj, "voltage"):
                self.voltages.append(obj.voltage)
            if self.prev_local_time is not None:
                step = obj.local_time - self.prev_local_time
                if step < 0:
                    self.local_time_backwards += 1
                else:
                    self.local_time_steps.append(step)
            self.prev_local_time = obj.local_time

    def maybe_report(self, force: bool = False) -> bool:
        """Print the periodic report if due; True if it printed."""
        now = time.monotonic()
        if not force and now < self._next:
            return False
        self._next = now + self.interval
        st = self.parser.stats
        dt = max(now - self._last_t, 1e-9)
        bps = (st.total_bytes - self._last_bytes) / dt
        fps = (st.good - self._last_good) / dt
        self._last_bytes, self._last_good, self._last_t = st.total_bytes, st.good, now
        kinds = ", ".join(f"{k} {v}" for k, v in sorted(st.frames.items())) or "none"
        print(f"[{now - self.t0:6.1f} s] {bps:8.0f} B/s  {fps:5.1f} frames/s  "
              f"good: {kinds}  bad checksum {st.bad_checksum}  "
              f"skipped {st.skipped_bytes} B")
        if self.latest is not None:
            print("          " + describe(self.latest))
        elif st.total_bytes and not st.good:
            print("          " + self.no_frames_hint())
        return True

    def no_frames_hint(self) -> str:
        st = self.parser.stats
        if st.total_bytes and self.ascii_bytes / st.total_bytes > 0.9:
            return ("bytes arrive but are mostly text: protocol set to NMEA-0183? "
                    "(set Tag_Frame0 or Node_Frame2 in NAssistant)")
        return ("bytes arrive but no valid frame: wrong --baud? "
                f"(module default {DEFAULT_BAUD}), or a cheap CH340 failing at this speed")

    def final_report(self, duration: Optional[float]) -> int:
        """Print the summary; duration None = replay (no wall-clock rate)."""
        st = self.parser.stats
        print("\n=== summary ===")
        rate = "" if duration is None else f" in {duration:.1f} s ({st.good / max(duration, 1e-9):.1f}/s)"
        print(f"{st.total_bytes} bytes, {st.good} good frames{rate}, "
              f"bad checksum {st.bad_checksum}, bad length {st.bad_length}, "
              f"skipped {st.skipped_bytes} B")
        for k, v in sorted(st.frames.items()):
            print(f"  {k}: {v}")
        if not st.total_bytes:
            print("VERDICT: no bytes at all. Wrong port? Module powered? "
                  "NAssistant still holding the port?")
            return 1
        if not st.good:
            print("VERDICT: no valid frames. " + self.no_frames_hint())
            return 1
        print("sanity (checks the little-endian assumption):")
        if self.voltages:
            lo, hi = min(self.voltages), max(self.voltages)
            ok_v = VOLTAGE_RANGE[0] <= lo and hi <= VOLTAGE_RANGE[1]
            print(f"  supply voltage {lo:.3f}-{hi:.3f} V  {'OK' if ok_v else 'IMPLAUSIBLE'}")
        if self.local_time_steps:
            steps = sorted(self.local_time_steps)
            med = steps[len(steps) // 2]
            print(f"  local_time steps: median {med} ms (= {1000 / med:.1f} Hz)"
                  if med else "  local_time not advancing")
        print(f"  local_time went backwards {self.local_time_backwards} time(s)")
        verdict = st.bad_checksum == 0
        print("VERDICT: " + ("frames OK" if verdict else
                             f"{st.bad_checksum} checksum errors: check baud / cable / noise"))
        return 0 if verdict else 1


def _xyz(v) -> str:
    return "(" + ", ".join(f"{c:+.3f}" for c in v) + ")"


def describe(obj) -> str:
    """One-line summary of a decoded frame."""
    if isinstance(obj, (TagFrame0, NodeFrame2)):
        head = (f"id {obj.id} {ROLES.get(obj.role, obj.role)}  pos {_xyz(obj.pos)} m "
                f"{'' if obj.pos_valid else '(NO FIX)'}  eop {_xyz(obj.eop)} m  "
                f"V {obj.voltage:.3f}  local_time {obj.local_time}")
        if isinstance(obj, TagFrame0):
            dis = ", ".join(f"A{i} {d:.3f}" for i, d in enumerate(obj.dis) if d) or "none"
            return head + f"  dist: {dis}"
        nodes = ", ".join(f"A{n.id} {n.dis:.3f} m ({n.rx_rssi:.0f} dB)" for n in obj.nodes)
        return head + f"  anchors: {nodes or 'none'}"
    if isinstance(obj, NodeFrame3):
        nodes = ", ".join(f"A{n.id} {n.dis:.3f} m ({n.rx_rssi:.0f} dB)" for n in obj.nodes)
        return (f"id {obj.id} {ROLES.get(obj.role, obj.role)}  V {obj.voltage:.3f}  "
                f"local_time {obj.local_time}  anchors: {nodes or 'none'}")
    return repr(obj)


def serial_chunks(port: str, baud: int) -> Iterator[bytes]:
    import serial

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.05
    # Leave the modem-control lines alone: many USB-serial boards wire DTR/RTS
    # to reset/boot pins, and a read-only checker must never reset anything.
    ser.dtr = False
    ser.rts = False
    ser.open()
    with ser:
        print(f"Listening on {port} at {baud} baud (read-only). Ctrl+C to stop.")
        while True:
            yield ser.read(4096)


def file_chunks(path: Path, size: int) -> Iterator[bytes]:
    with open(path, "rb") as fh:
        while chunk := fh.read(size):
            yield chunk


def list_ports() -> None:
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Is the module / adapter plugged in (and its driver installed)?")
    for p in ports:
        print(f"{p.device:8s} {p.description}  [{p.hwid}]")


# ---------------------------------------------------------------------------
# --udp: the drone's debug stream
# ---------------------------------------------------------------------------

PX4_FRESH_MS = 500             # a PX4 position younger than this counts as live

CSV_FIELDS = ["esp_time_s", "kind", "has_fix", "uwb_x", "uwb_y", "uwb_z", "eop_max",
              "n_anchors", "voltage", "px4_x", "px4_y", "px4_z", "px4_heading", "px4_age_ms"]


def decode_raw(frame: bytes):
    """Decode one complete raw NLink frame (as carried in a debug record)."""
    frames = NLinkParser().feed(frame)
    return decode(frames[0]) if frames else None


def csv_row(rec: UwbRecord) -> dict:
    obj = decode_raw(rec.frame)
    row = {"esp_time_s": f"{rec.esp_time_us / 1e6:.6f}", "px4_age_ms": rec.px4_age_ms}
    if rec.px4 is not None:
        row.update(zip(("px4_x", "px4_y", "px4_z", "px4_heading"),
                       (f"{v:.4f}" for v in rec.px4)))
    if obj is not None:
        row["kind"] = type(obj).__name__
        row["voltage"] = f"{obj.voltage:.3f}"
        if isinstance(obj, (TagFrame0, NodeFrame2)):
            row.update(has_fix=int(obj.pos_valid), eop_max=f"{max(obj.eop):.2f}",
                       uwb_x=f"{obj.pos[0]:.4f}", uwb_y=f"{obj.pos[1]:.4f}",
                       uwb_z=f"{obj.pos[2]:.4f}")
        nodes = getattr(obj, "nodes", None)
        row["n_anchors"] = len(nodes) if nodes is not None else sum(1 for d in obj.dis if d)
    return row


def udp_status_line(link: LinkStats, last: Optional[UwbRecord]) -> str:
    if link.last is None:
        return "drone stream: no packets yet"
    c = link.last.counters
    px4 = "no PX4 position"
    if last is not None and last.px4 is not None:
        x, y, _, h = last.px4
        px4 = f"PX4 ({x:+.2f}, {y:+.2f}) {h * 57.29578:+.0f} deg, age {last.px4_age_ms} ms"
    return (f"drone: {link.packets} pkts ({link.lost} lost, {link.bad_packets} bad) | ESP32 "
            f"decoded {c['frames_decoded']}, checksum err {c['bad_checksum']}, uart overflow "
            f"{c['uart_overflows']}, framing err {c['uart_frame_errors']}, not forwarded "
            f"{c['records_dropped']} | {px4}")


def udp_final_report(link: LinkStats, mon: Monitor, last: Optional[UwbRecord]) -> int:
    """Check each link of the chain separately; 0 only if all checked links pass."""
    print("\n=== drone chain: module -> ESP32 -> Wi-Fi -> laptop ===")
    if link.packets == 0:
        print("no packets from the drone.  Check: laptop IP = the firmware's "
              "HOST_IPV4_ADDR (192.168.1.(100 + drone id)); Windows firewall allows UDP "
              "on this port; UWB_ENABLED and UWB_DEBUG_UDP are on; the serial monitor "
              "shows 'debug sent' increasing.")
        return 1
    d = link.esp_delta
    ok_all = True

    def verdict(n: int, name: str, ok: Optional[bool], detail: str) -> None:
        nonlocal ok_all
        tag = "OK" if ok else ("NOT TESTED" if ok is None else "PROBLEM")
        ok_all = ok_all and ok is not False
        print(f"{n}. {name:<24} {tag:<10} {detail}")

    uart_ok = d("bytes") > 0 and d("uart_frame_errors") == 0 and d("uart_overflows") == 0
    verdict(1, "module -> ESP32 UART", uart_ok,
            f"{d('bytes')} B received, framing errors {d('uart_frame_errors')}, "
            f"overflows {d('uart_overflows')}")
    dec_ok = d("frames_decoded") > 0 and d("bad_checksum") == 0 and d("bad_length") == 0
    verdict(2, "ESP32 decoding", dec_ok,
            f"{d('frames_decoded')} frames decoded, checksum errors {d('bad_checksum')}, "
            f"length errors {d('bad_length')}")
    lost_pct = 100.0 * link.lost / max(link.lost + link.packets, 1)
    verdict(3, "Wi-Fi ESP32 -> laptop", link.lost == 0 and link.bad_packets == 0,
            f"{link.packets} packets, {link.lost} lost ({lost_pct:.1f}%), "
            f"{link.bad_packets} unreadable")
    st = mon.parser.stats
    verdict(4, "laptop re-decode", st.good > 0 and st.bad_checksum == 0,
            f"{st.good} raw frames re-decoded on the laptop, checksum errors {st.bad_checksum}")
    esp_frames = d("frames_decoded") + d("frames_other")
    got = link.records - len(link.first.records)       # records sent after the first packet
    dropped = d("records_dropped")
    # Lost packets take their records with them; only a clean link must add up exactly.
    verdict(5, "ESP32 vs laptop count", got + dropped == esp_frames if link.lost == 0 else None,
            f"ESP32 counted {esp_frames} frames; laptop received {got} + "
            f"{dropped} not forwarded by the ESP32 (should add up)")
    if last is None or last.px4 is None:
        verdict(6, "PX4 position pairing", None,
                "no PX4 position in the stream (fine if PX4 is not connected)")
    else:
        verdict(6, "PX4 position pairing", last.px4_age_ms <= PX4_FRESH_MS,
                f"last PX4 position {last.px4_age_ms} ms old at the UWB frame")
    print("VERDICT: " + ("whole chain OK" if ok_all else "see PROBLEM lines above"))
    return 0 if ok_all else 1


def run_udp(args: argparse.Namespace) -> int:
    mon = Monitor(args.interval)
    link = LinkStats()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.udp))
    except OSError as exc:
        print(f"error: cannot listen on UDP {args.udp}: {exc}", file=sys.stderr)
        return 2
    sock.settimeout(0.1)
    print(f"Listening for the drone's UWB debug stream on UDP {args.udp}. Ctrl+C to stop.")
    csv_fh = open(args.csv, "w", newline="", encoding="utf-8") if args.csv else None
    writer = csv.DictWriter(csv_fh, CSV_FIELDS) if csv_fh else None
    if writer:
        writer.writeheader()
    last: Optional[UwbRecord] = None
    source: Optional[str] = None
    t_start = time.monotonic()
    try:
        while True:
            try:
                data, (ip, _) = sock.recvfrom(65535)
            except socket.timeout:
                data = None
            if data:
                try:
                    pkt = parse_packet(data)
                except ValueError as exc:
                    link.bad_packets += 1
                    if link.bad_packets == 1:
                        print(f"unreadable packet from {ip}: {exc}")
                    continue
                if source is None:
                    source = ip
                    print(f"stream from {ip}, drone {pkt.drone_id}")
                link.add(pkt)
                for rec in pkt.records:
                    mon.feed(rec.frame)
                    last = rec
                    if writer:
                        writer.writerow(csv_row(rec))
            if mon.maybe_report():
                print("          " + udp_status_line(link, last))
            if args.duration and time.monotonic() - t_start >= args.duration:
                break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        sock.close()
        if csv_fh:
            csv_fh.close()
            print(f"CSV written to {args.csv}")
    rc_frames = mon.final_report(time.monotonic() - t_start)
    rc_chain = udp_final_report(link, mon, last)
    return rc_frames or rc_chain


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Check LinkTrack NLink frames (read-only).")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, e.g. COM5")
    src.add_argument("--replay", type=Path, help="raw capture file from --save")
    src.add_argument("--list", action="store_true", help="list serial ports and exit")
    src.add_argument("--udp", type=int, metavar="PORT",
                     help="read the drone's UWB debug stream on this UDP port (e.g. 5010)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--save", type=Path, help="also write every raw byte to this file")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--interval", type=float, default=1.0, help="report period (s)")
    ap.add_argument("--csv", type=Path, help="--udp: write one row per frame to this CSV")
    ap.add_argument("--bind", default="0.0.0.0", help="--udp: local address to listen on")
    args = ap.parse_args(argv)

    if args.list:
        list_ports()
        return 0
    if args.udp is not None:
        return run_udp(args)

    mon = Monitor(args.interval)
    save: Optional[BinaryIO] = None          # opened on the first data, not before
    chunks = (serial_chunks(args.port, args.baud) if args.port
              else file_chunks(args.replay, 1024))
    t_start = time.monotonic()
    try:
        for chunk in chunks:
            if args.save and chunk:
                if save is None:
                    save = open(args.save, "wb")
                save.write(chunk)
            mon.feed(chunk)
            if args.port:
                mon.maybe_report()
                if args.duration and time.monotonic() - t_start >= args.duration:
                    break
    except KeyboardInterrupt:
        print("\nstopped")
    except Exception as exc:                        # e.g. port busy / not found
        print(f"error: {exc}", file=sys.stderr)
        return 2
    finally:
        if save is not None:
            save.close()
            print(f"raw bytes saved to {args.save}")
    if args.replay:
        if mon.latest is not None:
            print("newest frame: " + describe(mon.latest))
        return mon.final_report(None)
    return mon.final_report(time.monotonic() - t_start)


if __name__ == "__main__":
    sys.exit(main())
