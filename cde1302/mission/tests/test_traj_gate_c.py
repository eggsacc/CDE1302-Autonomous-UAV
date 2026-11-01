"""main/traj_gate.c: the setpoint waits for the drone instead of running on the clock.

Compiled with the laptop's gcc, like test_nlink_c.py, and run against a simple
drone that has a speed limit.
"""

import os
import subprocess
from pathlib import Path

import pytest

from test_nlink_c import CDE, _compiler

SOURCES = [CDE / "main" / "traj_gate.c", Path(__file__).parent / "host_c" / "traj_gate_host_main.c"]
MOVE, WAIT, DONE = 0, 1, 2
TICK_S = 0.05
POINTS, SAMPLE_S = 51, 0.2           # 10 s at 0.1 m/s = 1.0 m


@pytest.fixture(scope="session")
def host(tmp_path_factory):
    cc = _compiler()
    if cc is None:
        pytest.skip("no C compiler (install the conda gcc: conda install -c conda-forge gcc)")
    exe = tmp_path_factory.mktemp("traj_gate_c") / ("traj_gate.exe" if os.name == "nt" else "traj_gate")
    cmd = [cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
           f"-I{CDE / 'main'}", *map(str, SOURCES), "-o", str(exe)]
    if os.name == "nt":
        cmd.insert(1, "-static")
    env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert proc.returncode == 0, f"C build failed:\n{proc.stderr}"
    return exe


def fly(exe, leash, max_speed, seconds, *extra):
    out = subprocess.run([str(exe), str(POINTS), str(SAMPLE_S), str(leash), str(max_speed),
                          str(int(seconds / TICK_S)), *map(str, extra)],
                         capture_output=True, text=True, check=True).stdout.split()
    return float(out[0]), float(out[1]), float(out[2]), int(out[3]), int(out[4])


def fly_corner(exe, leash, corner_m, corner_at, max_speed, seconds):
    out = subprocess.run([str(exe), str(POINTS), str(SAMPLE_S), str(leash), str(max_speed),
                          str(int(seconds / TICK_S)), "-1", str(corner_m), str(corner_at)],
                         capture_output=True, text=True, check=True).stdout.split()
    return float(out[1]), int(out[3]), int(out[4]), int(out[5]), float(out[6])


def test_fast_drone_takes_the_planned_time(host):
    k, drone, _, result, done_tick = fly(host, 0.25, 1.0, 12)
    assert result == DONE
    assert done_tick * TICK_S == pytest.approx(10.0, abs=0.3)
    assert drone == pytest.approx(1.0, abs=0.03) and k == POINTS - 1


def test_slow_drone_still_reaches_the_end(host):
    # Half the commanded speed: the setpoint waits, and the drone flies all of it.
    _, drone, max_err, result, done_tick = fly(host, 0.25, 0.05, 40)
    assert result == DONE
    assert drone >= 1.0 - 0.25
    assert max_err <= 0.25 + 0.02                # never got further ahead than the leash
    assert done_tick * TICK_S > 12.0             # took as long as it took


def test_the_clock_runs_away_from_a_slow_drone(host):
    # leash 0 = the old behaviour: "done" with the drone only halfway.
    _, drone, _, result, done_tick = fly(host, 0.0, 0.05, 12)
    assert result == DONE and done_tick * TICK_S == pytest.approx(10.0, abs=0.3)
    assert drone < 0.65


def test_waits_forever_rather_than_guessing(host):
    # A drone that cannot move: the setpoint stops at the leash and is never done.
    k, drone, max_err, result, done_tick = fly(host, 0.25, 0.0, 300)
    assert drone == 0.0 and result in (MOVE, WAIT) and done_tick == -1
    assert k * SAMPLE_S * 0.1 == pytest.approx(0.25, abs=0.02)


def test_setpoint_stops_on_a_corner_until_the_drone_is_close(host):
    # Slow drone: up to 0.4 m behind on the lane, but within 5 cm at the corner.
    drone, result, done, passed_tick, err_there = fly_corner(host, 0.4, 0.05, 25, 0.05, 40)
    assert passed_tick > 0 and err_there <= 0.05
    assert result == DONE
    assert drone == pytest.approx(1.0, abs=0.06)       # the last point is a corner too


def test_a_corner_adds_only_a_short_wait_for_a_fast_drone(host):
    plain = fly(host, 0.4, 1.0, 16)[4]
    with_corner = fly_corner(host, 0.4, 0.05, 25, 1.0, 16)[2]
    assert with_corner - plain < int(2.0 / TICK_S)


def test_stale_position_stops_the_setpoint(host):
    k_frozen = fly(host, 0.25, 1.0, 30, 40)[0]         # position stale from 2 s
    assert k_frozen == pytest.approx(40 * TICK_S / SAMPLE_S, abs=1.0)
    assert fly(host, 0.25, 1.0, 30, 40)[3] == WAIT
