"""main/pose_hist.c: the drone's pose at a camera frame's capture time.

Compiled with the laptop's gcc, like test_traj_gate_c.py.
"""

import math
import os
import subprocess
from pathlib import Path

import pytest

from test_nlink_c import CDE, _compiler

SOURCES = [CDE / "main" / "pose_hist.c", Path(__file__).parent / "host_c" / "pose_hist_host_main.c"]
T0, DT = 1_000_000, 20_000          # first sample, spacing (us); 200 samples, 128 kept


@pytest.fixture(scope="session")
def host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler (install the conda gcc: conda install -c conda-forge gcc)")
    exe = tmp_path_factory.mktemp("pose_hist_c") / ("pose_hist.exe" if os.name == "nt" else "pose_hist")
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, SOURCES), "-o", str(exe), "-lm"]
    if os.name == "nt":
        cmd.insert(1, "-static")
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def at(exe, *times):
    out = subprocess.run([str(exe), *map(str, times)], capture_output=True, text=True,
                         check=True).stdout.splitlines()
    return [None if line == "0" else tuple(map(float, line.split()[1:])) for line in out]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_interpolates_between_samples(host):
    i = 180.5                                          # halfway between samples 180 and 181
    (x, y, hd, rate, gap), = at(host, int(T0 + i * DT))
    assert x == pytest.approx(0.02 * i, abs=1e-4) and y == pytest.approx(-0.01 * i, abs=1e-4)
    assert hd == pytest.approx(wrap(2.0 + 0.01 * i), abs=1e-4)
    assert rate == pytest.approx(0.01 / 0.02, rel=1e-3)        # 0.5 rad/s
    assert gap == DT / 2


def test_heading_crosses_pi_the_short_way(host):
    i = 114.5                       # 3.14 -> 3.15 (= -3.133): halfway is 3.145 = -3.138
    (_, _, hd, rate, _), = at(host, int(T0 + i * DT))
    assert hd == pytest.approx(wrap(2.0 + 0.01 * i), abs=1e-4)
    assert rate == pytest.approx(0.5, rel=1e-3)                 # never a -2 pi jump


def test_keeps_only_the_newest_samples_and_ignores_old_ones(host):
    oldest = 200 - 128
    assert at(host, T0 + (oldest - 1) * DT) == [None]
    (x, *_), = at(host, T0 + oldest * DT)
    assert x == pytest.approx(0.02 * oldest, abs=1e-4)        # the late (99, 99) is not there


def test_holds_the_newest_briefly_then_gives_up(host):
    newest = T0 + 199 * DT
    (x, _, _, _, gap), = at(host, newest + 50_000)
    assert x == pytest.approx(0.02 * 199, abs=1e-4) and gap == 50_000
    assert at(host, newest + 150_000) == [None]
