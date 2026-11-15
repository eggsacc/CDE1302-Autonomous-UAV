"""UWB debug stream: C packer (main/uwb_debug_pack.c) <-> Python reader
(mission/uwb_link.py), and uwb_sniff.py --udp end to end over loopback."""

import csv
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from nlink import NLinkParser
from test_nlink import REAL_CAPTURE
from test_nlink_c import CDE, _compiler
from uwb_link import (COUNTER_NAMES, PX4_AGE_NONE, LinkStats, UwbRecord, build_packet,
                      parse_packet)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import uwb_sniff  # noqa: E402

MAX_PACKET = 1400                          # UWB_DBG_MAX_PACKET


@pytest.fixture(scope="session")
def debug_host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler")
    exe = tmp_path_factory.mktemp("uwb_dbg") / ("uwb_debug_host.exe" if os.name == "nt" else "uwb_debug_host")
    srcs = [CDE / "main" / "nlink_parser.c", CDE / "main" / "uwb_debug_pack.c",
            Path(__file__).parent / "host_c" / "uwb_debug_host_main.c"]
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, srcs), "-o", str(exe), "-lm"]
    if os.name == "nt":
        cmd.insert(1, "-static")
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def read_packets(path: Path) -> list[bytes]:
    data, out, i = path.read_bytes(), [], 0
    while i < len(data):
        (n,) = struct.unpack_from("<H", data, i)
        out.append(data[i + 2:i + 2 + n])
        i += 2 + n
    return out


# ---------------------------------------------------------------------------
# C packer vs Python reader
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("per_flush", [1, 5, 50, 512])
def test_c_packets_decode_in_python(debug_host, tmp_path, per_flush):
    out = tmp_path / "packets.bin"
    subprocess.run([str(debug_host), str(REAL_CAPTURE), str(out), str(per_flush)], check=True)
    raw = read_packets(out)
    pkts = [parse_packet(p) for p in raw]
    assert all(len(p) <= MAX_PACKET for p in raw)
    assert [p.seq for p in pkts] == list(range(len(pkts)))             # no gaps
    records = [r for p in pkts for r in p.records]
    expected = [f.data for f in NLinkParser().feed(REAL_CAPTURE.read_bytes())]
    assert [r.frame for r in records] == expected                    # 512 identical frames
    for i, r in enumerate(records):
        assert r.esp_time_us == 1_000_000 + 20_000 * i
        if i % 4 == 0:
            assert r.px4 is None and r.px4_age_ms is None
        else:
            assert r.px4 == pytest.approx((0.1 * i, -0.2 * i, -0.5, 0.01 * i), rel=1e-6)
            assert r.px4_age_ms == i % 1000
    assert pkts[0].drone_id == 2
    assert pkts[0].counters == dict(zip(COUNTER_NAMES, (1000, 0, 7, 1, 2, 3, 4, 5, 6, 8, 9, 10)))


def test_python_packer_matches_the_layout():
    rec = UwbRecord(123456789, (1.5, -2.5, -0.5, 0.25), 42, b"\x55\x04abc")
    rec_none = UwbRecord(5, None, None, b"")
    data = build_packet(7, 99, 4321, {"bytes": 10, "records_dropped": 3}, [rec, rec_none])
    assert len(data) == 64 + 28 + 5 + 28
    assert data[:6] == b"UWBD\x02\x07"
    p = parse_packet(data)
    assert (p.drone_id, p.seq, p.esp_time_ms) == (7, 99, 4321)
    assert p.counters["bytes"] == 10 and p.counters["records_dropped"] == 3
    assert p.records[0] == rec and p.records[1] == rec_none
    assert struct.unpack_from("<H", data, 64 + 24)[0] == 42
    assert struct.unpack_from("<H", data, 64 + 28 + 5 + 24)[0] == PX4_AGE_NONE


@pytest.mark.parametrize("data, msg", [
    (b"UWBD", "too short"),
    (b"XXXX" + bytes(60), "bad magic"),
    (b"UWBD\x01" + bytes(59), "version 1"),
])
def test_bad_packets_rejected(data, msg):
    with pytest.raises(ValueError, match=msg):
        parse_packet(data)


def test_truncated_and_trailing_bytes_rejected():
    good = build_packet(1, 0, 0, {}, [UwbRecord(1, None, None, b"\x01\x02\x03")])
    with pytest.raises(ValueError, match="truncated"):
        parse_packet(good[:-1])
    with pytest.raises(ValueError, match="trailing"):
        parse_packet(good + b"\x00")


def test_link_stats_counts_gaps_and_reordering():
    st = LinkStats()
    for seq in (0, 1, 2, 5, 6, 4, 7):
        st.add(parse_packet(build_packet(1, seq, 0, {"frames_decoded": seq * 10}, [])))
    assert st.packets == 7 and st.lost == 2 and st.out_of_order == 1
    assert st.esp_delta("frames_decoded") == 70


# ---------------------------------------------------------------------------
# uwb_sniff.py --udp, end to end on loopback
# ---------------------------------------------------------------------------

def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def drone_packets(skip_seq=None, uart_frame_errors=0):
    """What a healthy drone sends: the real capture, 5 frames per 100 ms packet,
    counters that add up, PX4 position present."""
    frames = [f.data for f in NLinkParser().feed(REAL_CAPTURE.read_bytes())][:100]
    out, seq, decoded = [], 0, 0
    for i in range(0, len(frames), 5):
        batch = frames[i:i + 5]
        decoded += len(batch)                  # counted on the ESP32 before sending
        recs = [UwbRecord(10**6 + 20000 * (i + k), (0.1, 0.2, -0.5, 0.0), 30, f)
                for k, f in enumerate(batch)]
        counters = {"bytes": 120 * decoded, "frames_decoded": decoded,
                    "uart_frame_errors": uart_frame_errors if i else 0}
        if seq != skip_seq:
            out.append(build_packet(2, seq, 1000 + i, counters, recs))
        seq += 1
    return out


def run_sniff_with(packets, tmp_path, capsys, extra=()):
    port = free_port()
    result = {}
    csv_path = tmp_path / "bench.csv"
    args = ["--udp", str(port), "--bind", "127.0.0.1", "--duration", "1.5",
            "--interval", "5", "--csv", str(csv_path), *extra]
    th = threading.Thread(target=lambda: result.update(rc=uwb_sniff.main(args)))
    th.start()
    time.sleep(0.3)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for p in packets:
            s.sendto(p, ("127.0.0.1", port))
            time.sleep(0.005)
    th.join(10)
    return result["rc"], capsys.readouterr().out, csv_path


def test_sniff_udp_healthy_chain(tmp_path, capsys):
    rc, out, csv_path = run_sniff_with(drone_packets(), tmp_path, capsys)
    assert "stream from 127.0.0.1, drone 2" in out
    for line in ("1. module -> ESP32 UART     OK", "2. ESP32 decoding           OK",
                 "3. Wi-Fi ESP32 -> laptop    OK", "4. laptop re-decode         OK",
                 "5. ESP32 vs laptop count    OK", "6. PX4 position pairing     OK"):
        assert line in out, out
    assert "VERDICT: whole chain OK" in out and rc == 0
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    assert len(rows) == 100
    assert rows[0]["kind"] == "NodeFrame2" and rows[0]["has_fix"] == "0"
    assert rows[0]["px4_x"] == "0.1000" and rows[0]["px4_age_ms"] == "30"


def test_sniff_udp_flags_lost_packets_and_uart_errors(tmp_path, capsys):
    rc, out, _ = run_sniff_with(drone_packets(skip_seq=7, uart_frame_errors=3), tmp_path, capsys)
    assert rc == 1
    assert "1. module -> ESP32 UART     PROBLEM" in out and "framing errors 3" in out
    assert "3. Wi-Fi ESP32 -> laptop    PROBLEM" in out and "1 lost" in out
    assert "5. ESP32 vs laptop count    NOT TESTED" in out


def test_sniff_udp_no_packets_explains(tmp_path, capsys):
    rc, out, _ = run_sniff_with([], tmp_path, capsys)
    assert rc == 1 and "HOST_IPV4_ADDR" in out and "firewall" in out


def test_sniff_udp_without_px4(tmp_path, capsys):
    pkts = [build_packet(2, 0, 0, {"bytes": 1, "frames_decoded": 0}, []),
            build_packet(2, 1, 100, {"bytes": 121, "frames_decoded": 1},
                         [UwbRecord(1, None, None,
                                    NLinkParser().feed(REAL_CAPTURE.read_bytes()[:120])[0].data)])]
    rc, out, _ = run_sniff_with(pkts, tmp_path, capsys)
    assert "6. PX4 position pairing     NOT TESTED" in out
