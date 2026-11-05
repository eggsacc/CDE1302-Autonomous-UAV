"""main/traj_gate.c: the target stays a fixed distance ahead of the drone.

Compiled with the laptop's gcc, like test_nlink_c.py, and flown against a simple
PX4 model: a P position loop with a speed limit and a short lag.
"""

import os
import subprocess
from collections import namedtuple
from pathlib import Path

import pytest

from test_nlink_c import CDE, _compiler

SOURCES = [CDE / "main" / "traj_gate.c", Path(__file__).parent / "host_c" / "traj_gate_host_main.c"]
Run = namedtuple("Run", "done_s k x y end_err corner_err speed speed_std max_gap")
GAIN = 0.95                          # PX4 default MPC_XY_P


@pytest.fixture(scope="session")
def host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler (install the conda gcc: conda install -c conda-forge gcc)")
    exe = tmp_path_factory.mktemp("traj_gate_c") / ("traj_gate.exe" if os.name == "nt" else "traj_gate")
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, SOURCES), "-o", str(exe), "-lm"]
    if os.name == "nt":
        cmd.insert(1, "-static")
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def fly(exe, lookahead=0.25, corner=0.15, max_speed=0.5, noise=0.0, seconds=60.0,
        leg=2.0, leg2=0.0, freeze=None):
    args = [lookahead, corner, GAIN, max_speed, noise, seconds, leg, leg2]
    if freeze is not None:
        args.append(freeze)
    out = subprocess.run([str(exe), *map(str, args)], capture_output=True, text=True,
                         check=True).stdout.split()
    return Run(*map(float, out))


def test_speed_is_about_gain_times_lookahead(host):
    assert fly(host, lookahead=0.25).speed == pytest.approx(GAIN * 0.25, rel=0.1)
    assert fly(host, lookahead=0.15).speed == pytest.approx(GAIN * 0.15, rel=0.1)


def test_speed_is_steady(host):
    assert fly(host).speed_std < 0.01


def test_position_noise_barely_changes_the_speed(host):
    # 5 cm wobble: the commanded speed varies by the wobble x gain, not by more.
    r = fly(host, noise=0.05)
    assert r.speed == pytest.approx(GAIN * 0.25, rel=0.2)
    assert r.speed_std < 0.08


def test_a_speed_limit_only_lengthens_the_flight(host):
    r = fly(host, max_speed=0.08, seconds=60)
    assert r.done_s > 0 and r.end_err < 0.16
    assert r.speed == pytest.approx(0.08, rel=0.1)


def test_finishes_only_at_the_last_point(host):
    r = fly(host)
    assert r.done_s > 0 and r.end_err < 0.16 and r.x == pytest.approx(2.0, abs=0.16)


def test_corner_is_reached_before_moving_on(host):
    r = fly(host, leg=1.0, leg2=0.5, corner=0.10, seconds=60)
    assert r.done_s > 0
    assert 0 <= r.corner_err <= 0.10 + 0.02


def test_never_finishes_if_the_drone_cannot_move(host):
    r = fly(host, max_speed=0.0, seconds=120)
    assert r.done_s < 0 and r.x == 0.0
    assert r.k * 0.02 == pytest.approx(0.25, abs=0.02)      # target waits a lookahead away


def test_stale_position_stops_the_target(host):
    healthy = fly(host, seconds=20)
    frozen = fly(host, seconds=20, freeze=2.0)
    assert frozen.done_s < 0 and frozen.k < healthy.k / 3


def test_clock_mode_finishes_on_time_whatever_the_drone_does(host):
    r = fly(host, lookahead=0.0, max_speed=0.05, seconds=25)      # 2 m at 0.1 m/s = 20 s
    assert r.done_s == pytest.approx(20.0, abs=0.5)
    assert r.x < 1.4
