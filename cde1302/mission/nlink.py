"""
Nooploop NLink protocol (LinkTrack UWB): frame finding, checksum and decoding.

Field layout from UWB_documentation/NLink_V1.4.pdf.  Pure Python, no serial
port: feed it bytes from anywhere (USB, UART adapter, a capture file).

    parser = NLinkParser()
    for frame in parser.feed(chunk):          # complete, checksum-checked frames
        if frame.kind == "TAG_FRAME0":
            tag = decode_tag_frame0(frame.data)
            print(tag.pos, tag.dis, tag.voltage)
    parser.stats                               # good / bad / skipped counts

Byte order: the PDF does not state it.  Fields are decoded little-endian;
check on the real module that `voltage` reads the supply (about 5 V) and
`local_time` counts up in ms.  If they look like garbage, the order is wrong.

Checksum (all frames except ANCHOR_FRAME0): the sum of every previous byte,
modulo 256.  The PDF lists ANCHOR_FRAME0's last byte as "Value = 0xEE" instead,
so that frame is accepted on either rule.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Optional

# (header, function mark) -> (name, fixed length or None = uint16 length at bytes 2..3)
FRAME_TYPES: dict[tuple[int, int], tuple[str, Optional[int]]] = {
    (0x55, 0x00): ("ANCHOR_FRAME0", 896),
    (0x55, 0x01): ("TAG_FRAME0", 128),
    (0x55, 0x02): ("NODE_FRAME0", None),
    (0x55, 0x03): ("NODE_FRAME1", None),
    (0x55, 0x04): ("NODE_FRAME2", None),
    (0x55, 0x05): ("NODE_FRAME3", None),
    (0x55, 0x06): ("NODE_FRAME4", None),
    (0x55, 0x08): ("NODE_FRAME5", None),
    (0x55, 0x09): ("NODE_FRAME6", None),
    (0x54, 0x00): ("SETTING_FRAME0", 128),
    (0x54, 0xFA): ("ERROR_FRAME0", None),
    (0x52, 0x00): ("SYSTEM_COMMON_FRAME0", 32),
}
HEADERS = {h for h, _ in FRAME_TYPES}
# Sanity bounds for length-field frames.  1024 B = NODE_FRAME2 with 69 anchors;
# the drone's C parser (main/nlink_parser.c) uses the same numbers.
MIN_VAR_LEN, MAX_VAR_LEN = 7, 1024

ROLES = {0: "NODE", 1: "ANCHOR", 2: "TAG", 3: "CONSOLE", 4: "MASTER", 5: "SLAVE"}
INVALID_COORD = -8388.608               # int24 minimum / 1000: the docs' "invalid" -8388
EOP_NO_FIX = 2.55                       # eop byte 255 / 100: the field's maximum

# Observed on a real LinkTrack P-A tag with no anchors (2026-10-29, NODE_FRAME2
# at 50 Hz): it still streams, but reports pos = (1, 1, 1) with eop = 2.55 m on
# every axis.  So "no position fix" is recognised by eop at its maximum, not by
# a special coordinate value.


def has_fix(pos: tuple[float, ...], eop: tuple[float, ...]) -> bool:
    """True if the module claims a real position (see EOP_NO_FIX above)."""
    return (all(math.isfinite(v) and v != INVALID_COORD for v in pos)
            and max(eop) < EOP_NO_FIX)


# ---------------------------------------------------------------------------
# Low-level field readers (little-endian, see module docstring)
# ---------------------------------------------------------------------------

def int24(b: bytes, i: int) -> int:
    """Signed 24-bit little-endian integer at offset i."""
    v = b[i] | (b[i + 1] << 8) | (b[i + 2] << 16)
    return v - (1 << 24) if v & 0x800000 else v


def u16(b: bytes, i: int) -> int:
    return b[i] | (b[i + 1] << 8)


def u32(b: bytes, i: int) -> int:
    return struct.unpack_from("<I", b, i)[0]


def floats(b: bytes, i: int, n: int) -> tuple[float, ...]:
    return struct.unpack_from(f"<{n}f", b, i)


def checksum(b: bytes) -> int:
    """Sum of all bytes except the last, modulo 256."""
    return sum(b[:-1]) & 0xFF


def checksum_ok(name: str, frame: bytes) -> bool:
    if checksum(frame) == frame[-1]:
        return True
    return name == "ANCHOR_FRAME0" and frame[-1] == 0xEE


# ---------------------------------------------------------------------------
# Stream parser
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    kind: str                 # e.g. "TAG_FRAME0"
    data: bytes               # whole frame, header to checksum


@dataclass
class ParserStats:
    frames: dict[str, int] = field(default_factory=dict)   # good frames per kind
    bad_checksum: int = 0     # complete frames whose checksum failed
    bad_length: int = 0       # length field out of bounds
    skipped_bytes: int = 0    # bytes thrown away while searching for a header
    total_bytes: int = 0

    @property
    def good(self) -> int:
        return sum(self.frames.values())


class NLinkParser:
    """Turns an arbitrary byte stream into checksum-checked frames.

    Resynchronises after noise or a bad frame by dropping one byte and
    searching for the next known header + function mark.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.stats = ParserStats()

    def feed(self, chunk: bytes) -> list[Frame]:
        self._buf += chunk
        self.stats.total_bytes += len(chunk)
        out: list[Frame] = []
        buf = self._buf
        while True:
            start = self._find_header(buf)
            if start < 0:                        # keep a lone trailing header byte
                keep = 1 if buf and buf[-1] in HEADERS else 0
                self.stats.skipped_bytes += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if start:
                self.stats.skipped_bytes += start
                del buf[:start]
            if len(buf) < 2:
                break
            name, length = FRAME_TYPES[(buf[0], buf[1])]
            if length is None:
                if len(buf) < 4:
                    break
                length = u16(buf, 2)
                if not MIN_VAR_LEN <= length <= MAX_VAR_LEN:
                    self.stats.bad_length += 1
                    self.stats.skipped_bytes += 1
                    del buf[:1]
                    continue
            if len(buf) < length:
                break                            # wait for the rest
            frame = bytes(buf[:length])
            if checksum_ok(name, frame):
                out.append(Frame(name, frame))
                self.stats.frames[name] = self.stats.frames.get(name, 0) + 1
                del buf[:length]
            else:
                self.stats.bad_checksum += 1
                self.stats.skipped_bytes += 1
                del buf[:1]                      # maybe a false header: resync
        return out

    @staticmethod
    def _find_header(buf: bytearray) -> int:
        """Index of the first known (header, mark) pair, or -1."""
        for i in range(len(buf) - 1):
            if (buf[i], buf[i + 1]) in FRAME_TYPES:
                return i
        return -1


# ---------------------------------------------------------------------------
# Decoded frames (only those a TAG can output: Tag_Frame0, Node_Frame2/3)
# ---------------------------------------------------------------------------

def _vec24(b: bytes, i: int, scale: float, n: int = 3) -> tuple[float, ...]:
    return tuple(int24(b, i + 3 * k) / scale for k in range(n))


@dataclass(frozen=True)
class TagFrame0:
    """NLink_LinkTrack_Tag_Frame0 (0x55 0x01, 128 bytes)."""
    id: int
    role: int
    pos: tuple[float, float, float]        # m
    vel: tuple[float, float, float]        # m/s
    dis: tuple[float, ...]                 # m, to anchors A0..A7
    gyro: tuple[float, float, float]       # rad/s
    acc: tuple[float, float, float]        # m/s^2
    angle: tuple[float, float, float]      # deg (Euler)
    quat: tuple[float, float, float, float]
    local_time: int                        # ms
    system_time: int                       # ms
    eop: tuple[float, float, float]        # m, estimated position precision
    voltage: float                         # V, interface supply

    @property
    def pos_valid(self) -> bool:
        return has_fix(self.pos, self.eop)


def decode_tag_frame0(b: bytes) -> TagFrame0:
    if len(b) != 128 or b[0] != 0x55 or b[1] != 0x01:
        raise ValueError("not a 128-byte TAG_FRAME0")
    return TagFrame0(
        id=b[2], role=b[3],
        pos=_vec24(b, 4, 1000.0), vel=_vec24(b, 13, 10000.0),
        dis=_vec24(b, 22, 1000.0, 8),
        gyro=floats(b, 46, 3), acc=floats(b, 58, 3),
        angle=tuple(v / 100.0 for v in struct.unpack_from("<3h", b, 82)),
        quat=floats(b, 88, 4),
        local_time=u32(b, 108), system_time=u32(b, 112),
        eop=tuple(v / 100.0 for v in b[117:120]),
        voltage=u16(b, 120) / 1000.0,
    )


@dataclass(frozen=True)
class AnchorRange:
    role: int
    id: int
    dis: float          # m
    fp_rssi: float      # dB, first-path power
    rx_rssi: float      # dB, received power


@dataclass(frozen=True)
class NodeFrame2:
    """NLink_LinkTrack_Node_Frame2 (0x55 0x04): tag pose + per-anchor ranges/RSSI."""
    role: int
    id: int
    system_time: int
    eop: tuple[float, float, float]
    pos: tuple[float, float, float]
    vel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    acc: tuple[float, float, float]
    angle: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    local_time: int
    voltage: float
    nodes: tuple[AnchorRange, ...]

    @property
    def pos_valid(self) -> bool:
        return has_fix(self.pos, self.eop)


def _ranges(b: bytes, start: int, count: int, block: int) -> tuple[AnchorRange, ...]:
    out = []
    for k in range(count):
        i = start + k * block
        if i + block > len(b) - 1:                  # whole block must fit before the checksum
            break
        out.append(AnchorRange(role=b[i], id=b[i + 1], dis=int24(b, i + 2) / 1000.0,
                               fp_rssi=b[i + 5] / -2.0, rx_rssi=b[i + 6] / -2.0))
    return tuple(out)


def decode_node_frame2(b: bytes) -> NodeFrame2:
    if b[:2] != b"\x55\x04" or len(b) < 120 or u16(b, 2) != len(b):
        raise ValueError("not a NODE_FRAME2")
    return NodeFrame2(
        role=b[4], id=b[5], system_time=u32(b, 6),
        eop=tuple(v / 100.0 for v in b[10:13]),
        pos=_vec24(b, 13, 1000.0), vel=_vec24(b, 22, 10000.0),
        gyro=floats(b, 40, 3), acc=floats(b, 52, 3),
        angle=tuple(v / 100.0 for v in struct.unpack_from("<3h", b, 76)),
        quat=floats(b, 82, 4),
        local_time=u32(b, 102), voltage=u16(b, 116) / 1000.0,
        nodes=_ranges(b, 119, b[118], 13),          # role, id, dis(3), fp, rx, 6 reserved
    )


@dataclass(frozen=True)
class NodeFrame3:
    """NLink_LinkTrack_Node_Frame3 (0x55 0x05): per-anchor ranges/RSSI only."""
    role: int
    id: int
    local_time: int
    system_time: int
    voltage: float
    nodes: tuple[AnchorRange, ...]


def decode_node_frame3(b: bytes) -> NodeFrame3:
    if b[:2] != b"\x55\x05" or len(b) < 22 or u16(b, 2) != len(b):
        raise ValueError("not a NODE_FRAME3")
    return NodeFrame3(role=b[4], id=b[5], local_time=u32(b, 6), system_time=u32(b, 10),
                      voltage=u16(b, 18) / 1000.0,
                      nodes=_ranges(b, 21, b[20], 7))   # role, id, dis(3), fp, rx


DECODERS = {"TAG_FRAME0": decode_tag_frame0, "NODE_FRAME2": decode_node_frame2,
            "NODE_FRAME3": decode_node_frame3}


def decode(frame: Frame):
    """Decoded dataclass for frames a TAG outputs; None for other or malformed frames."""
    fn = DECODERS.get(frame.kind)
    if fn is None:
        return None
    try:
        return fn(frame.data)
    except (ValueError, struct.error):
        return None
