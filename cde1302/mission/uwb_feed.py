"""
UwbFeed: turn the drone's UWB debug stream (uwb_link.py) into an alignment.

A background thread reads the UDP stream and pairs each UWB fix with the PX4
position recorded at the same instant on the drone.  While collecting, those
pairs go to an AlignmentEstimator; Mission solves it once a second and sends
the result to the drone, which then feeds PX4 itself (main/uwb_task.c).

    feed = UwbFeed(settings)
    feed.start()
    feed.begin()              # the drone is about to move
    result = feed.solve()     # uwb_align.AlignResult
    feed.align_received       # how many alignments the ESP32 has accepted
    feed.stop()

Only the laptop needs this while aligning: after that the drone carries on
without it, so losing Wi-Fi cannot stop UWB reaching PX4.

on_reading, if given, is called for every decoded UWB reading (the mission
writes them to uwb.csv), including flights that never send UWB to PX4.
"""

from __future__ import annotations

import logging
import math
import socket
import threading
from typing import Callable, Optional

from nlink import EOP_NO_FIX, INVALID_COORD, NLinkParser, decode
from settings import Settings
from uwb_align import AlignmentEstimator, AlignResult, AlignSample
from uwb_link import LinkStats, UwbDebugPacket, parse_packet

log = logging.getLogger("uwb_feed")

RECV_TIMEOUT_S = 0.2            # how often the thread checks for stop()
RECV_BUFFER = 2048              # larger than UWB_DBG_MAX_PACKET


def xy_fix(pos, eop) -> bool:
    """A usable horizontal position: x and y valid with a real precision.  z is
    ignored, as in the firmware (uwb_reading_usable): the module often reports
    no z fix (eop 2.55) while x and y are good."""
    return (all(math.isfinite(v) and v != INVALID_COORD for v in pos[:2])
            and max(eop[:2]) < EOP_NO_FIX)


class UwbFeed:
    """Thread-safe bridge from the debug stream to the alignment estimator."""

    def __init__(self, settings: Settings,
                 on_reading: Optional[Callable[..., None]] = None) -> None:
        self.settings = settings
        self._on_reading = on_reading
        self.estimator = AlignmentEstimator.from_settings(settings.uwb)
        self.link = LinkStats()
        self._lock = threading.Lock()
        self._collecting = False
        self._latest: Optional[AlignSample] = None
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Open the UDP port and start reading (raises OSError if it is taken)."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self.settings.uwb.debug_port))
        self._sock.settimeout(RECV_TIMEOUT_S)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="uwb-feed", daemon=True)
        self._thread.start()
        log.info("listening for the UWB debug stream on UDP %d", self.settings.uwb.debug_port)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2 * RECV_TIMEOUT_S + 1.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._sock.recv(RECV_BUFFER)
            except socket.timeout:
                continue
            except OSError:
                return
            self.feed_datagram(data)

    # -- data ---------------------------------------------------------------

    def feed_datagram(self, data: bytes) -> None:
        """Take in one UDP payload (the thread's work; also the test entry point)."""
        try:
            pkt = parse_packet(data)
        except ValueError as exc:
            log.warning("bad UWB debug packet: %s", exc)
            with self._lock:
                self.link.bad_packets += 1
            return
        if pkt.drone_id != self.settings.drone_id:
            return
        with self._lock:
            self.link.add(pkt)
            for rec in pkt.records:
                self._add_record(rec)

    def _add_record(self, rec) -> None:
        """Pair one record; called with the lock held."""
        u = self.settings.uwb
        frames = NLinkParser().feed(rec.frame)
        obj = decode(frames[0]) if frames else None
        if obj is None or not hasattr(obj, "pos"):
            return
        if self._on_reading is not None:
            try:
                self._on_reading(
                    esp_time_s=rec.esp_time_us / 1e6, uwb_x=obj.pos[0], uwb_y=obj.pos[1],
                    uwb_z=obj.pos[2], eop_max=max(obj.eop), fix=int(obj.pos_valid),
                    px4_x=rec.px4[0] if rec.px4 else None,
                    px4_y=rec.px4[1] if rec.px4 else None, px4_age_ms=rec.px4_age_ms)
            except Exception:
                log.exception("UWB reading callback failed")
        if rec.px4 is None or rec.px4_age_ms > u.align_max_px4_age_ms:
            return
        sample = AlignSample(rec.esp_time_us / 1e6, obj.pos[0], obj.pos[1],
                             rec.px4[0], rec.px4[1], obj.pos_valid, max(obj.eop))
        if xy_fix(obj.pos, obj.eop):
            self._latest = sample
        if self._collecting:
            self.estimator.add(sample)

    # -- alignment ----------------------------------------------------------

    def begin(self) -> None:
        """Start collecting pairs, forgetting any earlier ones."""
        with self._lock:
            self.estimator.clear()
            self._collecting = True

    def finish(self) -> None:
        """Stop collecting: the alignment is decided."""
        with self._lock:
            self._collecting = False

    def solve(self) -> AlignResult:
        with self._lock:
            return self.estimator.solve()

    def latest(self) -> Optional[AlignSample]:
        """The newest UWB fix paired with PX4's position, or None."""
        with self._lock:
            return self._latest

    def counter(self, name: str) -> int:
        """An ESP32 counter from the newest packet (0 before the first one)."""
        with self._lock:
            pkt: Optional[UwbDebugPacket] = self.link.last
            return pkt.counters[name] if pkt is not None else 0

    @property
    def align_received(self) -> int:
        return self.counter("align_received")
