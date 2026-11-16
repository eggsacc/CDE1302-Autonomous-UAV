"""The drone's C parser (main/nlink_parser.c) must agree with mission/nlink.py.

pytest compiles main/nlink_parser.c with the laptop's gcc (the same compiler
apriltag_host.py uses) and a small driver (tests/host_c/nlink_host_main.c),
runs both parsers over the same bytes, fed in different chunk sizes, and
compares every frame, every decoded value and every error counter.
"""

import json
import os
import random
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from nlink import NLinkParser, NodeFrame2, NodeFrame3, TagFrame0, decode
from test_nlink import REAL_CAPTURE, finish, node_frame2, node_frame3, tag_frame0

CDE = Path(__file__).resolve().parents[2]                  # cde1302/
C_MAX_ANCHORS = 16                                         # NLINK_MAX_ANCHORS in nlink_parser.h
SOURCES = [CDE / "main" / "nlink_parser.c", Path(__file__).parent / "host_c" / "nlink_host_main.c"]


def _compiler():
    conda_gcc = Path(sys.prefix) / "Library" / "bin" / "gcc.exe"
    for cc in (os.environ.get("CC"), str(conda_gcc) if conda_gcc.is_file() else None, "gcc", "cc"):
        if cc and shutil.which(cc):
            return shutil.which(cc)
    return None


@pytest.fixture(scope="session")
def nlink_host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler (install the conda gcc: conda install -c conda-forge gcc)")
    exe = tmp_path_factory.mktemp("nlink_c") / ("nlink_host.exe" if os.name == "nt" else "nlink_host")
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, SOURCES), "-o", str(exe), "-lm"]
    if os.name == "nt":
        cmd.insert(1, "-static")
    # A conda gcc finds its assembler/linker via PATH; without `conda activate`
    # its folder may be missing, and gcc then fails silently.
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def run_c(exe, data: bytes, chunk: int, tmp_path) -> tuple[list[dict], dict]:
    f = tmp_path / f"stream_{chunk}.bin"
    f.write_bytes(data)
    out = subprocess.run([str(exe), str(f), str(chunk)], capture_output=True, text=True,
                         check=True).stdout.splitlines()
    rows = [json.loads(line) for line in out]
    return rows[:-1], rows[-1]["stats"]


def run_py(data: bytes, chunk: int) -> tuple[list[dict], dict]:
    """The same JSON shape as the C driver, produced by mission/nlink.py."""
    p = NLinkParser()
    rows = []
    for i in range(0, len(data), chunk):
        for fr in p.feed(data[i:i + chunk]):
            row = {"kind": fr.kind, "len": len(fr.data)}
            obj = decode(fr)
            row["decoded"] = obj is not None
            if obj is not None:
                d = fr.data
                row.update(role=obj.role, id=getattr(obj, "id", None),
                           local_time=obj.local_time, system_time=obj.system_time,
                           voltage=obj.voltage)
                if isinstance(obj, (TagFrame0, NodeFrame2)):
                    row.update(has_pose=True, has_fix=obj.pos_valid, pos=list(obj.pos),
                               vel=list(obj.vel), eop=list(obj.eop))
                else:
                    row.update(has_pose=False, has_fix=False)
                if isinstance(obj, TagFrame0):
                    row["n_anchors_total"] = 8
                    row["anchors"] = [[255, k, dis, None, None] for k, dis in enumerate(obj.dis)]
                else:
                    row["n_anchors_total"] = d[118] if isinstance(obj, NodeFrame2) else d[20]
                    row["anchors"] = [[a.role, a.id, a.dis, a.fp_rssi, a.rx_rssi]
                                      for a in obj.nodes][:C_MAX_ANCHORS]   # C storage cap
            rows.append(row)
    st = p.stats
    return rows, {"frames": dict(st.frames), "bad_checksum": st.bad_checksum,
                  "bad_length": st.bad_length, "skipped_bytes": st.skipped_bytes,
                  "total_bytes": st.total_bytes}


def same(a, b):
    """Recursive equality, floats to float32 precision."""
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        return a == pytest.approx(b, rel=1e-6, abs=1e-6)
    return a == b


def assert_parity(exe, data, chunk, tmp_path):
    c_rows, c_stats = run_c(exe, data, chunk, tmp_path)
    py_rows, py_stats = run_py(data, chunk)
    assert c_stats == py_stats
    assert len(c_rows) == len(py_rows)
    for i, (c, p) in enumerate(zip(c_rows, py_rows)):
        assert same(c, p), f"frame {i} differs:\nC:  {c}\nPy: {p}"
    return c_rows, c_stats


# ---------------------------------------------------------------------------

@pytest.mark.parametrize("chunk", [1, 7, 64, 4096])
def test_real_pa_capture_identical(nlink_host, tmp_path, chunk):
    rows, stats = assert_parity(nlink_host, REAL_CAPTURE.read_bytes(), chunk, tmp_path)
    assert stats["frames"] == {"NODE_FRAME2": 512} and stats["bad_checksum"] == 0
    assert all(r["decoded"] and not r["has_fix"] for r in rows)        # no anchors
    assert rows[0]["pos"] == [1.0, 1.0, 1.0]


def _awkward_stream() -> bytes:
    bad = bytearray(tag_frame0(local_time=5))
    bad[40] ^= 0x10                                         # checksum failure
    short_n2 = bytearray(30)                                # checksum-valid but too short
    short_n2[0], short_n2[1], short_n2[2] = 0x55, 0x04, 30
    anchor = bytearray(896)
    anchor[0], anchor[-1] = 0x55, 0xEE                      # documented 0xEE end byte
    many = node_frame2(anchors=tuple((1, k, 1.0 + k / 10, 2 * k, 2 * k + 1) for k in range(20)))
    return (b"\x00\x55\x01\x13" + tag_frame0() + bytes(bad) + b"\x55\x04\x01\x00"
            + node_frame2() + finish(short_n2) + node_frame3() + bytes(anchor)
            + many + b"\x52" + tag_frame0(pos=(-8388.608,) * 3, local_time=9) + b"\x55")


@pytest.mark.parametrize("chunk", [1, 3, 100, 4096])
def test_awkward_stream_identical(nlink_host, tmp_path, chunk):
    rows, stats = assert_parity(nlink_host, _awkward_stream(), chunk, tmp_path)
    assert stats["bad_checksum"] >= 1 and stats["bad_length"] >= 1
    kinds = [r["kind"] for r in rows]
    assert kinds.count("NODE_FRAME2") == 3 and "ANCHOR_FRAME0" in kinds
    short = [r for r in rows if r["len"] == 30]
    assert short and not short[0]["decoded"]                 # rejected, not crashed
    big = [r for r in rows if r.get("n_anchors_total") == 20]
    assert len(big[0]["anchors"]) == 16                      # C storage cap respected


@pytest.mark.parametrize("seed", range(12))
def test_random_corruption_identical(nlink_host, tmp_path, seed):
    rng = random.Random(seed)
    parts = []
    for _ in range(60):
        r = rng.random()
        if r < 0.4:
            f = bytearray(node_frame2(anchors=tuple((1, k, rng.uniform(0, 9), 60, 70)
                                                    for k in range(rng.randrange(5)))))
        elif r < 0.6:
            f = bytearray(tag_frame0(local_time=rng.randrange(1 << 32)))
        elif r < 0.7:
            f = bytearray(node_frame3())
        else:
            f = bytearray(rng.randbytes(rng.randrange(1, 60)))
        if rng.random() < 0.2 and len(f) > 4:
            f[rng.randrange(len(f))] ^= 1 << rng.randrange(8)   # bit flip
        parts.append(bytes(f))
    data = b"".join(parts)
    assert_parity(nlink_host, data, rng.choice([1, 5, 64, 999, 4096]), tmp_path)
