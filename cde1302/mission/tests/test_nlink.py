"""NLink parser/decoder (mission/nlink.py) and the uwb_sniff.py checker.

Frames are built here byte by byte from the offset tables in
UWB_documentation/NLink_V1.4.pdf, independently of the decoder.
"""

import struct
import sys
from pathlib import Path

import pytest

from nlink import (INVALID_COORD, NLinkParser, checksum, decode, decode_node_frame2,
                   decode_node_frame3, decode_tag_frame0, int24)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import uwb_sniff  # noqa: E402


def put_i24(b, i, v):
    v = int(round(v)) & 0xFFFFFF
    b[i:i + 3] = bytes((v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF))


def finish(b):
    b[-1] = sum(b[:-1]) & 0xFF
    return bytes(b)


def tag_frame0(pos=(1.234, -0.567, 0.089), vel=(0.1, -0.2, 0.0),
               dis=(1.5, 2.25, 0, 3.125, 0, 0, 0, 0), voltage=5.012, local_time=123456,
               system_time=654321, eop=(0.05, 0.06, 0.12), angle=(1.5, -2.5, 179.99),
               tag_id=0, role=2):
    b = bytearray(128)
    b[0], b[1], b[2], b[3] = 0x55, 0x01, tag_id, role
    for k in range(3):
        put_i24(b, 4 + 3 * k, pos[k] * 1000)
        put_i24(b, 13 + 3 * k, vel[k] * 10000)
    for k in range(8):
        put_i24(b, 22 + 3 * k, dis[k] * 1000)
    struct.pack_into("<3f", b, 46, 0.01, 0.02, 0.03)        # gyro
    struct.pack_into("<3f", b, 58, 0.0, 0.0, 9.81)          # acc
    struct.pack_into("<3h", b, 82, *(round(a * 100) for a in angle))
    struct.pack_into("<4f", b, 88, 1.0, 0.0, 0.0, 0.0)
    struct.pack_into("<I", b, 108, local_time)
    struct.pack_into("<I", b, 112, system_time)
    b[117:120] = bytes(round(e * 100) for e in eop)
    struct.pack_into("<H", b, 120, round(voltage * 1000))
    return finish(b)


def node_frame2(anchors=((1, 0, 2.5, 80, 75), (1, 3, 4.0, 90, 85)), pos=(0.5, 0.25, -0.1),
                voltage=4.9, local_time=1000):
    n = len(anchors)
    b = bytearray(119 + 13 * n + 1)
    b[0], b[1] = 0x55, 0x04
    struct.pack_into("<H", b, 2, len(b))
    b[4], b[5] = 2, 0
    struct.pack_into("<I", b, 6, 777)
    b[10:13] = bytes((5, 5, 10))
    for k in range(3):
        put_i24(b, 13 + 3 * k, pos[k] * 1000)
    struct.pack_into("<I", b, 102, local_time)
    struct.pack_into("<H", b, 116, round(voltage * 1000))
    b[118] = n
    for j, (role, aid, dis, fp, rx) in enumerate(anchors):
        i = 119 + 13 * j
        b[i], b[i + 1] = role, aid
        put_i24(b, i + 2, dis * 1000)
        b[i + 5], b[i + 6] = fp, rx                          # stored as -2 x dB
    return finish(b)


def node_frame3(anchors=((1, 2, 1.75, 70, 60),)):
    n = len(anchors)
    b = bytearray(21 + 7 * n + 1)
    b[0], b[1] = 0x55, 0x05
    struct.pack_into("<H", b, 2, len(b))
    b[4], b[5] = 2, 0
    struct.pack_into("<I", b, 6, 42)
    struct.pack_into("<I", b, 10, 43)
    struct.pack_into("<H", b, 18, 5000)
    b[20] = n
    for j, (role, aid, dis, fp, rx) in enumerate(anchors):
        i = 21 + 7 * j
        b[i], b[i + 1] = role, aid
        put_i24(b, i + 2, dis * 1000)
        b[i + 5], b[i + 6] = fp, rx
    return finish(b)


# ---------------------------------------------------------------------------
# Field decoding
# ---------------------------------------------------------------------------

def test_int24_sign():
    assert int24(bytes((0xFF, 0xFF, 0x7F)), 0) == 8388607
    assert int24(bytes((0x00, 0x00, 0x80)), 0) == -8388608
    assert int24(bytes((0xFF, 0xFF, 0xFF)), 0) == -1


def test_tag_frame0_round_trip():
    t = decode_tag_frame0(tag_frame0())
    assert t.id == 0 and t.role == 2
    assert t.pos == pytest.approx((1.234, -0.567, 0.089))
    assert t.vel == pytest.approx((0.1, -0.2, 0.0))
    assert t.dis == pytest.approx((1.5, 2.25, 0, 3.125, 0, 0, 0, 0))
    assert t.angle == pytest.approx((1.5, -2.5, 179.99))
    assert t.gyro == pytest.approx((0.01, 0.02, 0.03)) and t.acc[2] == pytest.approx(9.81)
    assert t.quat == pytest.approx((1, 0, 0, 0))
    assert (t.local_time, t.system_time) == (123456, 654321)
    assert t.eop == pytest.approx((0.05, 0.06, 0.12))
    assert t.voltage == pytest.approx(5.012)
    assert t.pos_valid


def test_invalid_coordinate_is_flagged():
    t = decode_tag_frame0(tag_frame0(pos=(INVALID_COORD, INVALID_COORD, INVALID_COORD)))
    assert not t.pos_valid


def test_node_frame2_ranges_and_rssi():
    n = decode_node_frame2(node_frame2())
    assert n.pos == pytest.approx((0.5, 0.25, -0.1)) and n.voltage == pytest.approx(4.9)
    assert [(a.id, a.dis, a.fp_rssi, a.rx_rssi) for a in n.nodes] == \
        [(0, pytest.approx(2.5), -40.0, -37.5), (3, pytest.approx(4.0), -45.0, -42.5)]


def test_node_frame3_ranges():
    n = decode_node_frame3(node_frame3())
    assert (n.local_time, n.system_time, n.voltage) == (42, 43, pytest.approx(5.0))
    assert n.nodes[0].id == 2 and n.nodes[0].dis == pytest.approx(1.75)


def test_decoders_reject_wrong_frames():
    with pytest.raises(ValueError):
        decode_tag_frame0(node_frame2())
    with pytest.raises(ValueError):
        decode_node_frame2(tag_frame0())


# ---------------------------------------------------------------------------
# Stream parser
# ---------------------------------------------------------------------------

def test_frames_split_into_single_bytes():
    p = NLinkParser()
    stream = tag_frame0() + node_frame2() + node_frame3() + tag_frame0(local_time=9)
    out = []
    for b in stream:
        out += p.feed(bytes([b]))
    assert [f.kind for f in out] == ["TAG_FRAME0", "NODE_FRAME2", "NODE_FRAME3", "TAG_FRAME0"]
    assert p.stats.bad_checksum == 0 and p.stats.skipped_bytes == 0
    assert decode(out[-1]).local_time == 9


def test_garbage_and_a_false_header_are_skipped():
    p = NLinkParser()
    noise = b"\x00\x13\x55\x01\x99" + bytes(range(30))        # contains a fake 55 01 header
    out = p.feed(noise + tag_frame0() + b"\xAA\xBB" + tag_frame0(local_time=2))
    assert [decode(f).local_time for f in out] == [123456, 2]
    assert p.stats.skipped_bytes > 0


def test_corrupted_frame_is_counted_and_the_next_one_still_found():
    bad = bytearray(tag_frame0())
    bad[50] ^= 0xFF                                           # flip bits in the payload
    p = NLinkParser()
    out = p.feed(bytes(bad) + tag_frame0(local_time=7))
    assert p.stats.bad_checksum >= 1
    assert [decode(f).local_time for f in out] == [7]


def test_bad_length_field_resyncs():
    p = NLinkParser()
    out = p.feed(b"\x55\x04\x01\x00" + node_frame3())         # length 1: impossible
    assert p.stats.bad_length == 1 and [f.kind for f in out] == ["NODE_FRAME3"]


def test_incomplete_frame_waits_for_the_rest():
    f = tag_frame0()
    p = NLinkParser()
    assert p.feed(f[:100]) == []
    assert [x.kind for x in p.feed(f[100:])] == ["TAG_FRAME0"]


def test_anchor_frame0_accepts_the_documented_0xEE_end_byte():
    b = bytearray(896)
    b[0], b[1] = 0x55, 0x00
    b[-1] = 0xEE
    p = NLinkParser()
    assert checksum(bytes(b)) == 0x55                         # sum rule would reject it...
    assert [f.kind for f in p.feed(bytes(b))] == ["ANCHOR_FRAME0"]   # ...0xEE rule accepts


# ---------------------------------------------------------------------------
# uwb_sniff.py on recorded data
# ---------------------------------------------------------------------------

def test_sniff_replay_of_good_capture_passes(tmp_path, capsys):
    cap = tmp_path / "good.bin"
    frames = b"".join(tag_frame0(local_time=1000 + 20 * k) for k in range(50))
    cap.write_bytes(b"\x01\x02" + frames)
    assert uwb_sniff.main(["--replay", str(cap)]) == 0
    out = capsys.readouterr().out
    assert "TAG_FRAME0: 50" in out
    assert "supply voltage 5.012-5.012 V  OK" in out
    assert "median 20 ms (= 50.0 Hz)" in out
    assert "VERDICT: frames OK" in out


def test_sniff_reports_checksum_errors(tmp_path, capsys):
    bad = bytearray(tag_frame0())
    bad[60] ^= 0x01
    cap = tmp_path / "bad.bin"
    cap.write_bytes(tag_frame0() + bytes(bad) + tag_frame0(local_time=200000))
    assert uwb_sniff.main(["--replay", str(cap)]) == 1
    assert "checksum errors" in capsys.readouterr().out


def test_sniff_hints_for_wrong_baud_and_nmea(tmp_path, capsys):
    noise = tmp_path / "noise.bin"
    noise.write_bytes(bytes((i * 37) % 256 for i in range(5000)).replace(b"\x55", b"\x56"))
    assert uwb_sniff.main(["--replay", str(noise)]) == 1
    assert "wrong --baud" in capsys.readouterr().out
    nmea = tmp_path / "nmea.bin"
    nmea.write_bytes(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n" * 50)
    assert uwb_sniff.main(["--replay", str(nmea)]) == 1
    assert "NMEA" in capsys.readouterr().out


def test_sniff_empty_capture(tmp_path, capsys):
    empty = tmp_path / "empty.bin"
    empty.write_bytes(b"")
    assert uwb_sniff.main(["--replay", str(empty)]) == 1
    assert "no bytes at all" in capsys.readouterr().out


def test_sniff_list_ports_does_not_crash(capsys):
    assert uwb_sniff.main(["--list"]) == 0


# ---------------------------------------------------------------------------
# Real hardware: 10 s from a LinkTrack P-A tag over USB, no anchors (2026-10-29)
# ---------------------------------------------------------------------------

REAL_CAPTURE = Path(__file__).parent / "data" / "linktrack_pa_no_anchors_node_frame2.bin"


def test_real_pa_capture_decodes_cleanly():
    p = NLinkParser()
    frames = p.feed(REAL_CAPTURE.read_bytes())
    assert p.stats.frames == {"NODE_FRAME2": 512}
    assert p.stats.bad_checksum == 0 and p.stats.skipped_bytes == 0
    decoded = [decode(f) for f in frames]
    times = [d.local_time for d in decoded]
    steps = {b - a for a, b in zip(times, times[1:])}
    assert steps <= {19, 20, 21}                    # 50 Hz, as configured
    assert all(4.9 < d.voltage < 5.0 for d in decoded)   # USB 5 V: little-endian is right
    for d in decoded:                               # no anchors -> placeholder, no fix
        assert d.role == 2 and d.id == 0 and d.nodes == ()
        assert d.pos == pytest.approx((1.0, 1.0, 1.0)) and d.eop == pytest.approx((2.55,) * 3)
        assert not d.pos_valid


def test_eop_below_max_means_a_fix():
    from nlink import has_fix
    assert has_fix((1.0, 1.0, 1.0), (0.05, 0.05, 0.10))
    assert not has_fix((1.0, 1.0, 1.0), (0.05, 2.55, 0.10))
