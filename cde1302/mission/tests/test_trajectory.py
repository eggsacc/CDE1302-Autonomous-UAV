"""Trajectory building and upload (mission/trajectory.py, DroneLink.send_trajectory)."""

import math
import struct

import numpy as np
import pytest

from drone_link import DroneLink
from protocol import CMD_TRAJ_DATA, CMD_TRAJ_START, PKT_CMD, TRAJ_CHUNK_PTS
from settings import load_settings
from test_drone_link import FakeClock, FakeComms
from trajectory import build_trajectory, shift_for_camera


# ---------------------------------------------------------------------------
# build_trajectory
# ---------------------------------------------------------------------------

def test_constant_speed_through_every_waypoint():
    wps = [(1.0, 0.0), (1.0, 0.5), (0.0, 0.5)]           # 1 + 0.5 + 1 = 2.5 m
    t = build_trajectory((0.0, 0.0), wps, speed=0.1, dt=0.05)
    assert t.dt_ms == 50
    assert t.duration == pytest.approx(25.0)
    assert len(t.points) == 501
    assert t.wp_times == pytest.approx((10.0, 15.0, 25.0))
    for wp, wt in zip(wps, t.wp_times):                  # on the waypoint at its time
        assert t.points[round(wt / 0.05)] == pytest.approx(wp)
    steps = np.hypot(*np.diff(t.points, axis=0).T)
    assert steps.max() == pytest.approx(0.005)           # 0.1 m/s x 0.05 s
    assert t.max_step_speed == pytest.approx(0.1)


def test_offsets_are_relative_to_the_start_with_zero_z():
    t = build_trajectory((2.0, -1.0), [(2.5, -1.0)], speed=0.25, dt=0.1)
    off = t.offsets
    assert off.shape == (len(t.points), 3)
    assert off[0] == pytest.approx((0, 0, 0)) and off[-1] == pytest.approx((0.5, 0, 0))
    assert np.all(off[:, 2] == 0.0)


def test_last_point_is_exactly_the_last_waypoint():
    t = build_trajectory((0.0, 0.0), [(0.333, 0.777)], speed=0.13, dt=0.05)
    assert tuple(t.points[-1]) == (0.333, 0.777)


def test_long_paths_get_a_longer_sample_period_instead_of_too_many_points():
    t = build_trajectory((0.0, 0.0), [(30.0, 0.0)], speed=0.1, dt=0.05)   # 300 s
    assert len(t.points) <= 2400
    assert t.dt_ms % 10 == 0 and t.dt_ms > 50
    assert t.max_step_speed == pytest.approx(0.1, rel=1e-6)
    with pytest.raises(ValueError, match="does not fit"):
        build_trajectory((0.0, 0.0), [(1000.0, 0.0)], speed=0.1, dt=0.05)


def test_zero_length_segments_are_harmless():
    t = build_trajectory((0.0, 0.0), [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0)], 0.1, 0.05)
    assert t.wp_times == pytest.approx((0.0, 10.0, 10.0))
    assert t.duration == pytest.approx(10.0)


def test_bad_inputs():
    with pytest.raises(ValueError):
        build_trajectory((0, 0), [], 0.1, 0.05)
    with pytest.raises(ValueError):
        build_trajectory((0, 0), [(1, 0)], 0.0, 0.05)


@pytest.mark.parametrize("heading_deg, expected", [
    (0, (0.4, 0.5)),      # facing North: path moves South
    (90, (1.0, -0.1)),    # facing East:  path moves West
    (180, (1.6, 0.5)),
    (-90, (1.0, 1.1)),
])
def test_shift_for_camera_moves_the_path_backwards(heading_deg, expected):
    out = shift_for_camera([(1.0, 0.5)], math.radians(heading_deg), 0.6)
    assert out[0] == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# DroneLink.send_trajectory: the bytes that go to the drone
# ---------------------------------------------------------------------------

class RawComms(FakeComms):
    def __init__(self):
        super().__init__()
        self.raw = []

    def send_raw(self, drone_id, data):
        self.raw.append(data)
        return True


def test_upload_matches_send_trajectory_py():
    s = load_settings()
    comms, sleeps = RawComms(), []
    link = DroneLink(s, comms=comms, clock=FakeClock(), sleep=sleeps.append)
    t = build_trajectory((1.0, 1.0), [(1.0, 1.0 + 0.1 * 200 * 0.05)], 0.1, 0.05)  # 201 pts
    assert link.send_trajectory(t.offsets, t.dt_ms)
    *chunks, start = comms.raw
    assert len(chunks) == math.ceil(len(t.points) / TRAJ_CHUNK_PTS) == 3
    assert sleeps == [s.flight.traj_upload_gap_s] * 3
    # chunk header: pkt, cmd, id, total, offset, n
    pkt, cmd, tid, total, off, n = struct.unpack_from("<BBBHHB", chunks[1])
    assert (pkt, cmd, total, off, n) == (PKT_CMD, CMD_TRAJ_DATA, 201, 80, 80)
    first = struct.unpack_from("<fff", chunks[0], 8)
    assert first == (0.0, 0.0, 0.0)
    # start: pkt, cmd, id, dt_ms -- same id as the chunks
    assert struct.unpack("<BBBH", start) == (PKT_CMD, CMD_TRAJ_START, tid, 50)


def test_each_upload_gets_a_new_id_and_dry_run_sends_nothing():
    s = load_settings()
    comms = RawComms()
    link = DroneLink(s, comms=comms, clock=FakeClock(), sleep=lambda _: None)
    t = build_trajectory((0, 0), [(0.5, 0)], 0.1, 0.05)
    ids = set()
    for _ in range(5):
        link.send_trajectory(t.offsets, t.dt_ms)
        ids.add(comms.raw[-1][2])
    assert len(ids) > 1
    dry_comms = RawComms()
    dry = DroneLink(s, comms=dry_comms, dry_run=True, clock=FakeClock(), sleep=lambda _: None)
    assert dry.send_trajectory(t.offsets, t.dt_ms) is False
    assert dry_comms.raw == []
