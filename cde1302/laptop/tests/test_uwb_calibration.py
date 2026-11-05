"""simple_uwb_calibration.py: frame decoding, packet parsing and the fit."""

import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simple_uwb_calibration as cal  # noqa: E402


def node_frame2(x, y, eop=(0.05, 0.05, 0.10)):
    b = bytearray(120)
    b[0], b[1] = 0x55, 0x04
    struct.pack_into("<H", b, 2, len(b))
    b[10:13] = bytes(round(e * 100) for e in eop)
    for k, v in enumerate((x, y, -1.0)):
        raw = round(v * 1000) & 0xFFFFFF
        b[13 + 3 * k:16 + 3 * k] = raw.to_bytes(3, "little")
    b[-1] = sum(b[:-1]) & 0xFF
    return bytes(b)


def debug_packet(records):
    out = bytearray(cal.DBG_HEADER.pack(b"UWBD", 2, 2, len(records), 0, 0, *([0] * 12)))
    for px, py, age, frame in records:
        out += cal.DBG_RECORD.pack(0, px, py, -0.5, 0.0, age, len(frame)) + frame
    return bytes(out)


def test_decode_node_frame2():
    x, y, e = cal.decode_node_frame2(node_frame2(1.234, -5.678, (0.07, 0.12, 2.55)))
    assert (x, y, e) == (pytest.approx(1.234), pytest.approx(-5.678), pytest.approx(0.12))


def test_no_xy_estimate_or_bad_checksum_is_rejected():
    assert cal.decode_node_frame2(node_frame2(1, 2, (2.55, 2.55, 2.55))) is None
    f = bytearray(node_frame2(1, 2))
    f[-1] ^= 1
    assert cal.decode_node_frame2(bytes(f)) is None


def test_debug_packet_round_trip():
    pkt = debug_packet([(1.0, 2.0, 20, node_frame2(3, 4)), (0.0, 0.0, cal.PX4_AGE_NONE, b"\x55")])
    recs = list(cal.parse_debug_packet(pkt))
    assert recs[0][:3] == (1.0, 2.0, 20) and recs[1][2] is None


def square_pairs(theta_deg, mirror, tx, ty, noise=0.0, seed=1):
    rng = np.random.default_rng(seed)
    p = cal.square_path(1.0, 0.15, "right")[:, :2]
    th = math.radians(theta_deg)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    M = np.diag([1.0, -1.0]) if mirror else np.eye(2)
    u = (p - [tx, ty]) @ R @ M.T                     # inverse of p = R M u + t, rows
    u += rng.normal(0, noise, u.shape)
    return [(a, b, c, d) for (a, b), (c, d) in zip(u, p)]


@pytest.mark.parametrize("mirror", [True, False])
def test_calibration_recovers_the_transform(mirror):
    r = cal.calibrate(square_pairs(-84.0, mirror, 4.1, 3.8, noise=0.05), "auto")
    assert r["mirror"] is mirror
    assert r["theta_deg"] == pytest.approx(-84.0, abs=2.0)
    assert (r["tx"], r["ty"]) == (pytest.approx(4.1, abs=0.05), pytest.approx(3.8, abs=0.05))
    assert r["rms_m"] < 0.08 and r["other_mirror_rms_m"] > 3 * r["rms_m"]


def test_square_ends_where_it_started():
    pts = cal.square_path(1.2, 0.15, "left")
    assert np.allclose(pts[0], 0) and np.allclose(pts[-1], 0)
    assert pts[:, 1].min() == pytest.approx(-1.2)
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert steps.max() <= 0.15 * cal.DT_S + 1e-9
