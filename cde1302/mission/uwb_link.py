"""
Reader for the drone's UWB debug stream (ESP32 -> laptop UDP, default port 5010).

Byte layout: main/uwb_debug_pack.h (the C side).  Each packet carries the
ESP32's counters and a batch of records; each record is one raw NLink frame,
exactly as the ESP32 received it, plus PX4's local position at that instant.

    pkt = parse_packet(udp_payload)
    pkt.counters["frames_decoded"], pkt.seq
    for rec in pkt.records:
        rec.frame          # raw NLink bytes -> mission/nlink.py
        rec.px4            # (x, y, z, heading) or None if PX4 had no position
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

MAGIC = b"UWBD"
VERSION = 2
HEADER = struct.Struct("<4sBBHII12I")       # 64 bytes
RECORD = struct.Struct("<Q4fHH")            # 28 bytes, then the frame
PX4_AGE_NONE = 0xFFFF
COUNTER_NAMES = ("bytes", "frames_decoded", "frames_other", "bad_checksum",
                 "bad_length", "skipped_bytes", "uart_overflows", "uart_frame_errors",
                 "records_dropped", "px4_fed", "px4_rejected", "align_received")
assert HEADER.size == 64 and RECORD.size == 28

# CMD_UWB_ALIGN (main/uwb_px4.h).  Defined here because laptop/protocol.py is
# the course's file.  Change it together with the firmware.
PKT_CMD = 0x02
CMD_UWB_ALIGN = 0x0A
ALIGN_CMD = struct.Struct("<BBBfff")        # 15 bytes
ALIGN_FLAG_VALID = 0x01
ALIGN_FLAG_MIRROR = 0x02
assert ALIGN_CMD.size == 15


def build_align_command(theta: float, mirror: bool, tx: float, ty: float) -> bytes:
    """The datagram telling the ESP32 to use p = R(theta) @ M @ u + t."""
    flags = ALIGN_FLAG_VALID | (ALIGN_FLAG_MIRROR if mirror else 0)
    return ALIGN_CMD.pack(PKT_CMD, CMD_UWB_ALIGN, flags, theta, tx, ty)


def build_align_clear() -> bytes:
    """The datagram telling the ESP32 to stop sending UWB to PX4."""
    return ALIGN_CMD.pack(PKT_CMD, CMD_UWB_ALIGN, 0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class UwbRecord:
    esp_time_us: int
    px4: Optional[tuple[float, float, float, float]]   # x, y, z (m, NED), heading (rad)
    px4_age_ms: Optional[int]                          # None = PX4 position never received
    frame: bytes


@dataclass(frozen=True)
class UwbDebugPacket:
    drone_id: int
    seq: int
    esp_time_ms: int
    counters: dict[str, int]
    records: tuple[UwbRecord, ...]


def parse_packet(data: bytes) -> UwbDebugPacket:
    """Decode one UDP payload.  Raises ValueError if it is not a valid packet."""
    if len(data) < HEADER.size:
        raise ValueError(f"packet too short ({len(data)} B)")
    magic, version, drone_id, n, seq, esp_ms, *counters = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"version {version}, expected {VERSION}: update laptop or firmware")
    pos = HEADER.size
    records = []
    for _ in range(n):
        if pos + RECORD.size > len(data):
            raise ValueError("truncated record header")
        t_us, x, y, z, hdg, age, flen = RECORD.unpack_from(data, pos)
        pos += RECORD.size
        if pos + flen > len(data):
            raise ValueError("truncated record frame")
        has_px4 = age != PX4_AGE_NONE
        records.append(UwbRecord(t_us, (x, y, z, hdg) if has_px4 else None,
                                 age if has_px4 else None, bytes(data[pos:pos + flen])))
        pos += flen
    if pos != len(data):
        raise ValueError(f"{len(data) - pos} unexpected trailing bytes")
    return UwbDebugPacket(drone_id, seq, esp_ms, dict(zip(COUNTER_NAMES, counters)),
                          tuple(records))


def build_packet(drone_id: int, seq: int, esp_time_ms: int, counters: dict[str, int],
                 records: list[UwbRecord]) -> bytes:
    """Python twin of the C packer, for tests and simulators."""
    out = bytearray(HEADER.pack(MAGIC, VERSION, drone_id, len(records), seq, esp_time_ms,
                                *(counters.get(k, 0) for k in COUNTER_NAMES)))
    for r in records:
        x, y, z, h = r.px4 if r.px4 is not None else (0.0, 0.0, 0.0, 0.0)
        age = PX4_AGE_NONE if r.px4_age_ms is None else r.px4_age_ms
        out += RECORD.pack(r.esp_time_us, x, y, z, h, age, len(r.frame)) + r.frame
    return bytes(out)


@dataclass
class LinkStats:
    """Transport-level bookkeeping for a stream of packets."""
    packets: int = 0
    bad_packets: int = 0
    lost: int = 0                  # sequence numbers never seen
    out_of_order: int = 0
    records: int = 0
    first: Optional[UwbDebugPacket] = None
    last: Optional[UwbDebugPacket] = None
    _next_seq: Optional[int] = field(default=None, repr=False)

    def add(self, pkt: UwbDebugPacket) -> None:
        self.packets += 1
        self.records += len(pkt.records)
        if self.first is None:
            self.first = pkt
        if self._next_seq is not None:
            if pkt.seq > self._next_seq:
                self.lost += pkt.seq - self._next_seq
            elif pkt.seq < self._next_seq:
                self.out_of_order += 1
        if self._next_seq is None or pkt.seq >= self._next_seq:
            self._next_seq = pkt.seq + 1
        self.last = pkt

    def esp_delta(self, name: str) -> int:
        """How much an ESP32 counter grew between the first and last packet."""
        if self.first is None or self.last is None:
            return 0
        return self.last.counters[name] - self.first.counters[name]
