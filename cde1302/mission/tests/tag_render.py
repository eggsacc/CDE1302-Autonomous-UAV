"""
Synthetic drone camera frames with tag16h5 tags lying on the floor.

Ray-casts every pixel of the drone's 320x240 camera (firmware intrinsics,
45 deg tilt and mount offset from apriltag_host.py) onto the floor plane,
then looks up the tag pattern there, so a test knows exactly where each tag
is in PX4 local NED and can check the whole detect -> body -> NED chain.

    gray = render_view([FloorTag(3, 0.8, 0.3)], drone=(0.2, 0.3, 0.0))
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import laptop_path  # noqa: F401
from apriltag_host import CAM_POS_M, FIRMWARE_INTRINSICS, R_BODY_CAM, TAG_SIZE_M

_TAG16H5_C = (Path(laptop_path.LAPTOP_DIR).parent / "components" / "esp-apriltag"
              / "apriltag" / "tag16h5.c")

# bit i of the code (MSB first) -> data cell (bit_x, bit_y), from tag16h5.c
_BIT_X = [1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3, 1, 1, 1, 2]
_BIT_Y = [1, 1, 1, 2, 1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3]


def _codes() -> list[int]:
    """The (uncommented) tag16h5 codes compiled into the detector."""
    codes = []
    for line in _TAG16H5_C.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("0x") and s.endswith("UL,"):
            codes.append(int(s[:-3], 16))
    return codes


CODES = _codes()


def tag_bitmap(tag_id: int) -> np.ndarray:
    """8x8 tag image, 1.0 = white, as apriltag_to_image() draws it.

    Outer ring white, next ring black (the border TAG_SIZE is measured on),
    inner 4x4 data cells white where the code bit is set.
    """
    code = CODES[tag_id]
    im = np.zeros((8, 8))
    im[0, :] = im[-1, :] = im[:, 0] = im[:, -1] = 1.0
    for i in range(16):
        if code & (1 << (15 - i)):
            im[_BIT_Y[i] + 1, _BIT_X[i] + 1] = 1.0
    return im


@dataclass(frozen=True)
class FloorTag:
    tag_id: int
    north: float              # tag centre, PX4 local NED (m)
    east: float
    yaw: float = 0.0          # rotation of the printed tag on the floor (rad)
    size: float = TAG_SIZE_M  # black-border edge length (m)


def render_view(tags: list[FloorTag], drone: tuple[float, float, float],
                altitude: float = 0.5, width: int = 320, height: int = 240,
                supersample: int = 4, floor_level: int = 200, black_level: int = 30,
                noise: float = 2.0, jpeg_quality: int | None = 60,
                seed: int = 0) -> np.ndarray:
    """Grayscale uint8 image the drone camera would see.

    Args:
        drone: (north, east, heading rad) in PX4 local NED.
        altitude: drone body origin height above the floor (m).
        jpeg_quality: round-trip through JPEG like the real stream (None = off).
    """
    k = FIRMWARE_INTRINSICS
    off = (np.arange(supersample) + 0.5) / supersample
    # AprilTag pixel convention: pixel i spans [i, i+1).
    u = (np.arange(width)[:, None] + off[None, :]).ravel()
    v = (np.arange(height)[:, None] + off[None, :]).ravel()
    uu, vv = np.meshgrid(u, v)                             # (H*ss, W*ss)
    dc = np.stack([(uu - k.cx) / k.fx, (vv - k.cy) / k.fy, np.ones_like(uu)], -1)
    db = dc @ np.asarray(R_BODY_CAM).T                     # rays in body frame
    o = np.asarray(CAM_POS_M)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (altitude - o[2]) / db[..., 2]
    hits = db[..., 2] > 1e-6
    f = o[0] + s * db[..., 0]
    r = o[1] + s * db[..., 1]
    x, y, h = drone
    c, sn = math.cos(h), math.sin(h)
    north = x + f * c - r * sn
    east = y + f * sn + r * c

    img = np.where(hits, 1.0, 0.6)                         # floor white-ish, wall grey
    for t in tags:
        cell = t.size / 6.0
        dn, de = north - t.north, east - t.east
        cy, sy = math.cos(t.yaw), math.sin(t.yaw)
        a = dn * cy + de * sy                              # along tag "up"
        b = -dn * sy + de * cy                             # along tag "right"
        col = np.floor(b / cell + 4).astype(int)
        row = np.floor(-a / cell + 4).astype(int)
        inside = hits & (col >= 0) & (col < 8) & (row >= 0) & (row < 8)
        bm = tag_bitmap(t.tag_id)
        img = np.where(inside, bm[row.clip(0, 7), col.clip(0, 7)], img)

    img = img.reshape(height, supersample, width, supersample).mean(axis=(1, 3))
    gray = black_level + (floor_level - black_level) * img
    gray += np.random.default_rng(seed).normal(0.0, noise, gray.shape)
    gray = np.clip(np.rint(gray), 0, 255).astype(np.uint8)
    if jpeg_quality is not None:
        ok, buf = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        assert ok
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    return np.ascontiguousarray(gray)


def jpeg_bytes(gray: np.ndarray, quality: int = 60) -> bytes:
    ok, buf = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf.tobytes()
