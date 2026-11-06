"""
Drone pose at the moment a camera frame was captured, sent inside the JPEG.

The ESP32 (main/camera_stream.c, pose_com_t) pairs each frame with PX4's
position on its own clock and puts it in a JPEG COM segment right after SOI.
Both times are on the drone, so Wi-Fi delay does not shift the pairing.
Decoders skip COM segments, so the laptop/ viewers still show the frame.

    fp = parse_frame_pose(jpeg)      # FramePose, or None (old firmware / no pose)
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

MAGIC = b"CDEP"
VERSION = 1
POSE_VALID = 0x01
_SEG_FMT = "<4sBBHffff"                       # after FF FE len_hi len_lo
_SEG_SIZE = struct.calcsize(_SEG_FMT)        # 24
SEGMENT_SIZE = 4 + _SEG_SIZE                 # 28, = sizeof(pose_com_t)


@dataclass(frozen=True)
class FramePose:
    x: float              # PX4 local NED north (m)
    y: float              # east (m)
    heading: float        # rad [-pi, pi), 0 = North, clockwise positive
    yaw_rate: float       # rad/s
    gap_ms: int           # capture time -> nearest PX4 sample used


def parse_frame_pose(jpeg: bytes) -> Optional[FramePose]:
    """The pose segment of one frame, or None if absent, unknown or not valid."""
    if len(jpeg) < 2 + SEGMENT_SIZE or jpeg[:4] != b"\xff\xd8\xff\xfe":
        return None
    seg_len = (jpeg[4] << 8) | jpeg[5]
    if seg_len != SEGMENT_SIZE - 2:
        return None
    magic, version, flags, gap_ms, x, y, heading, rate = struct.unpack_from(_SEG_FMT, jpeg, 6)
    if magic != MAGIC or version != VERSION or not flags & POSE_VALID:
        return None
    if not all(math.isfinite(v) for v in (x, y, heading, rate)):
        return None
    return FramePose(x, y, heading, rate, gap_ms)


def build_frame_pose(jpeg: bytes, pose: Optional[FramePose]) -> bytes:
    """Insert a pose segment the way the firmware does (for tests and the sim)."""
    assert jpeg[:2] == b"\xff\xd8"
    if pose is None:
        body = struct.pack(_SEG_FMT, MAGIC, VERSION, 0, 0, 0.0, 0.0, 0.0, 0.0)
    else:
        body = struct.pack(_SEG_FMT, MAGIC, VERSION, POSE_VALID, pose.gap_ms,
                           pose.x, pose.y, pose.heading, pose.yaw_rate)
    return jpeg[:2] + b"\xff\xfe" + struct.pack(">H", SEGMENT_SIZE - 2) + body + jpeg[2:]
