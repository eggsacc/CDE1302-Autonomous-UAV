"""simple_uwb_frame_goal.py: the UWB-frame alignment and the snap check."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simple_uwb_frame_goal as fg  # noqa: E402
from simple_uwb_set_goal import to_px4  # noqa: E402


def test_nose_along_x_keeps_x_and_flips_y():
    al = fg.frame_alignment(0.0)
    assert to_px4(al, 2.0, 1.5) == pytest.approx((2.0, -1.5))


def test_nose_direction_is_forward():
    # Whatever UWB direction the nose pointed at power-up becomes PX4 +x (forward).
    for mirror in (True, False):
        for nose in (0.0, 90.0, 180.0, -90.0, 37.0):
            al = fg.frame_alignment(nose, mirror)
            a = to_px4(al, 0.0, 0.0)
            b = to_px4(al, fg.math.cos(fg.math.radians(nose)), fg.math.sin(fg.math.radians(nose)))
            assert (b[0] - a[0], b[1] - a[1]) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_mirror_decides_which_side_is_right():
    # Nose along UWB -y.  Mirror yes: UWB +x is on the drone's left (PX4 -y).
    # Mirror no: UWB +x is on its right (PX4 +y).
    assert to_px4(fg.frame_alignment(-90.0, True), 1.0, 0.0) == pytest.approx((0.0, -1.0), abs=1e-9)
    assert to_px4(fg.frame_alignment(-90.0, False), 1.0, 0.0) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_snap_error():
    al = fg.frame_alignment(0.0)
    assert fg.snap_error(al, (2.0, 1.5, 2.0, -1.5)) == pytest.approx(0.0)
    assert fg.snap_error(al, (2.0, 1.5, 0.0, 0.0)) == pytest.approx(2.5)   # PX4 not snapped yet


def test_reach_needs_an_average_within_tolerance():
    r = fg.Reach((1.0, 2.0), 0.15, n=10, need=5)
    for _ in range(4):
        r.add((1.05, 2.0))
    assert not r.reached                       # too few readings yet
    r.add((1.05, 2.0))
    assert r.reached and r.err == pytest.approx(0.05)
    r = fg.Reach((1.0, 2.0), 0.15)
    for _ in range(10):
        r.add((1.5, 2.0))
    assert not r.reached and r.err == pytest.approx(0.5)


class FakeUwb:
    def __init__(self, xy):
        self.latest = (*xy, 0.0, 0.0)


class FakeDrone:
    def __init__(self):
        self.sent = []

    def waypoint(self, x, y, yaw):
        self.sent.append((x, y, yaw))


def test_fly_to_sends_the_px4_waypoint_and_waits_for_uwb():
    al = fg.frame_alignment(-90.0)
    drone = FakeDrone()
    ok, err = fg.fly_to(drone, FakeUwb((3.73, 4.40)), al, (3.73, 4.40), 1.2, 0.15, 3.0, "t")
    assert ok and err == pytest.approx(0.0)
    x, y, yaw = drone.sent[0]
    assert (x, y) == pytest.approx(to_px4(al, 3.73, 4.40)) and yaw == 1.2
    drone = FakeDrone()
    ok, _ = fg.fly_to(drone, FakeUwb((0.0, 0.0)), al, (3.73, 4.40), 1.2, 0.15, 1.5, "t")
    assert not ok and len(drone.sent) == 2      # resent every second while waiting


def test_waypoint_packet():
    import struct
    import simple_uwb_set_goal as sg

    class Comms:
        def on_telemetry(self, cb):
            pass

        def send_raw(self, drone_id, data):
            self.data = (drone_id, data)

    c = Comms()
    sg.Drone(c, 2).waypoint(-4.4, -2.73, -1.57)
    drone_id, data = c.data
    assert drone_id == 2 and len(data) == 14
    pkt, cmd, x, y, yaw = struct.unpack("<BBfff", data)
    assert (pkt, cmd) == (0x02, 0x0B)
    assert (x, y, yaw) == pytest.approx((-4.4, -2.73, -1.57), abs=1e-6)
