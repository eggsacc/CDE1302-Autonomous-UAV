"""The drone's main/uwb_px4.c must agree with mission/uwb_align.py and uwb_link.py.

Compiled with the laptop's gcc, like test_nlink_c.py: the laptop builds the
CMD_UWB_ALIGN datagram, the C code decodes it and applies it, and the result
must match the Python model that the alignment was fitted with.
"""

import math
import os
import random
import re
import struct
import subprocess
from pathlib import Path

import pytest

from test_nlink_c import CDE, _compiler
from uwb_align import Alignment
from uwb_link import ALIGN_CMD, CMD_UWB_ALIGN, PKT_CMD, build_align_clear, build_align_command

SOURCES = [CDE / "main" / "uwb_px4.c", Path(__file__).parent / "host_c" / "uwb_px4_host_main.c"]
TOLERANCE_M = 2e-3            # float32 at a few metres


@pytest.fixture(scope="session")
def host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler (install the conda gcc: conda install -c conda-forge gcc)")
    exe = tmp_path_factory.mktemp("uwb_px4_c") / ("uwb_px4_host.exe" if os.name == "nt" else "uwb_px4_host")
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, SOURCES), "-o", str(exe), "-lm"]
    if os.name == "nt":
        cmd.insert(1, "-static")
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def run(exe, lines):
    out = subprocess.run([str(exe)], input="\n".join(lines) + "\n", capture_output=True,
                         text=True, check=True).stdout
    return out.splitlines()


@pytest.mark.parametrize("mirror", [False, True])
def test_c_applies_the_alignment_like_python(host, mirror):
    rng = random.Random(4)
    lines, expected = [], []
    for _ in range(40):
        a = Alignment(rng.uniform(-math.pi, math.pi), mirror,
                      rng.uniform(-8, 8), rng.uniform(-8, 8))
        lines.append("A " + build_align_command(a.theta, a.mirror, a.tx, a.ty).hex())
        expected.append(None)
        for _ in range(5):
            ux, uy = rng.uniform(-10, 10), rng.uniform(-10, 10)
            lines.append(f"P {ux:.6f} {uy:.6f}")
            expected.append(a.to_px4(ux, uy))
    for got, want in zip(run(host, lines), expected):
        if want is None:
            assert got == "1 1"
        else:
            assert tuple(map(float, got.split())) == pytest.approx(want, abs=TOLERANCE_M)


def test_clear_turns_the_alignment_off(host):
    assert run(host, ["A " + build_align_clear().hex()]) == ["1 0"]


def test_c_refuses_bad_datagrams(host):
    good = build_align_command(0.5, False, 1.0, 2.0)
    nan = ALIGN_CMD.pack(PKT_CMD, CMD_UWB_ALIGN, 1, float("nan"), 0.0, 0.0)
    inf = ALIGN_CMD.pack(PKT_CMD, CMD_UWB_ALIGN, 1, 0.0, float("inf"), 0.0)
    assert run(host, ["A " + good[:-1].hex(), "A " + nan.hex(), "A " + inf.hex()]) == ["0"] * 3


@pytest.mark.parametrize("line,expected", [
    ("U 1 1 0.10 0.10 0.10 0.30", "1"),
    ("U 1 1 0.30 0.30 2.50 0.30", "1"),          # z precision does not matter
    ("U 1 0 0.10 0.10 0.10 0.30", "0"),          # no fix
    ("U 0 1 0.10 0.10 0.10 0.30", "0"),          # no position in the frame
    ("U 1 1 0.31 0.10 0.10 0.30", "0"),
    ("U 1 1 0.10 0.31 0.10 0.30", "0"),
])
def test_usable_reading(host, line, expected):
    assert run(host, [line]) == [expected]


def test_constants_match_the_firmware():
    wifi = (CDE / "main" / "wifi_task.h").read_text(encoding="utf-8")
    assert int(re.search(r"#define CMD_UWB_ALIGN\s+(0x[0-9A-Fa-f]+)", wifi).group(1), 16) == CMD_UWB_ALIGN
    px4 = (CDE / "main" / "uwb_px4.h").read_text(encoding="utf-8")
    assert int(re.search(r"#define UWB_ALIGN_CMD_SIZE\s+(\d+)", px4).group(1)) == ALIGN_CMD.size


def test_layout_is_little_endian():
    d = build_align_command(1.5, True, -2.0, 3.0)
    assert d[:3] == bytes((PKT_CMD, CMD_UWB_ALIGN, 3))
    assert struct.unpack_from("<fff", d, 3) == (1.5, -2.0, 3.0)
