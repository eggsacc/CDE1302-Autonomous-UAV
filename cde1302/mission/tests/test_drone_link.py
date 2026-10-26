"""Telemetry tracking, pose history and commands (mission/drone_link.py)."""

import dataclasses
import math
import socket
import struct
import time

import pytest

from drone_link import DroneLink, Pose, PoseBuffer, TelemetryTracker, pose_from_packet
from geometry import angle_diff, lerp_angle, wrap_angle
from protocol import (CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, NAV_FLYING,
                      PKT_CMD, CMD_SET_NAV_TAGS, TelemetryPacket, VFH_BINS)
from settings import load_settings

DRONE = 2


def packet(x=0.0, y=0.0, heading=0.0, nav=0, drone_id=DRONE) -> TelemetryPacket:
    return TelemetryPacket(drone_id=drone_id, ned_x=x, ned_y=y, heading_rad=heading,
                           nav_state=nav, tag_id=-1, tag_dist_m=0.0,
                           vfh_blocked=[False] * VFH_BINS, is_stuck=False,
                           reloc_age_s=0xFFFF)


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeComms:
    """Records what DroneLink does to its CommsNode."""

    def __init__(self, ip="192.168.1.202"):
        self.calls = []
        self.sent = []
        self.ip = ip
        self.callbacks = []

    def set_nav_tags(self, tags, starts):
        self.calls.append(("set_nav_tags", list(tags), dict(starts)))

    def on_telemetry(self, cb):
        self.callbacks.append(cb)

    def start(self):
        self.calls.append(("start",))

    def stop(self):
        self.calls.append(("stop",))

    def send_nav_tags(self, drone_id=None):
        self.calls.append(("send_nav_tags", drone_id))

    def known_drones(self):
        return {DRONE: self.ip} if self.ip else {}

    def send_command(self, drone_id, cmd):
        if self.ip is None:
            return False
        self.sent.append((drone_id, cmd.cmd_type, cmd.goal_x, cmd.goal_y))
        return True


@pytest.fixture
def settings():
    return load_settings()


@pytest.fixture
def clock():
    return FakeClock()


def make_link(settings, clock, **kw):
    comms = kw.pop("comms", FakeComms())
    sleeps = []
    link = DroneLink(settings, comms=comms, clock=clock, sleep=sleeps.append, **kw)
    return link, comms, sleeps


# ---------------------------------------------------------------------------
# Angle helpers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("a, expected", [
    (0.0, 0.0), (math.pi, -math.pi), (-math.pi, -math.pi), (3 * math.pi / 2, -math.pi / 2),
    (2 * math.pi, 0.0), (-3 * math.pi / 2, math.pi / 2), (6.2, 6.2 - 2 * math.pi),
])
def test_wrap_angle(a, expected):
    assert wrap_angle(a) == pytest.approx(expected)


def test_angle_diff_takes_the_short_way():
    assert angle_diff(math.radians(-170), math.radians(170)) == pytest.approx(math.radians(20))
    assert angle_diff(math.radians(170), math.radians(-170)) == pytest.approx(math.radians(-20))


def test_lerp_angle_across_pi():
    mid = lerp_angle(math.radians(170), math.radians(-170), 0.5)
    assert abs(angle_diff(mid, math.pi)) < 1e-9


# ---------------------------------------------------------------------------
# PoseBuffer
# ---------------------------------------------------------------------------

def P(t, x=0.0, y=0.0, h=0.0, nav=0):
    return Pose(t, x, y, h, nav)


def test_interpolates_position_linearly():
    b = PoseBuffer()
    b.append(P(10.0, 0.0, 0.0))
    b.append(P(10.1, 1.0, -2.0, nav=NAV_FLYING))
    p = b.pose_at(10.025)
    assert (p.x, p.y) == (pytest.approx(0.25), pytest.approx(-0.5))
    assert p.t == 10.025
    assert p.nav_state == 0                      # from the earlier sample


def test_exact_sample_time_returns_that_sample():
    b = PoseBuffer()
    for i in range(3):
        b.append(P(10.0 + i * 0.1, float(i), 0.0))
    assert b.pose_at(10.1).x == pytest.approx(1.0)
    assert b.pose_at(10.0).x == 0.0


@pytest.mark.parametrize("h0_deg, h1_deg, expected_deg", [
    (170, -170, 180),       # wrap at +-pi: must pass through 180, not 0
    (-170, 170, 180),
    (350, 10, 0),           # firmware [0, 2pi) range wrapping through North
    (10, 350, 0),
    (80, 100, 90),          # plain case
])
def test_heading_interpolation_is_wrap_safe(h0_deg, h1_deg, expected_deg):
    b = PoseBuffer()
    b.append(P(0.0, h=wrap_angle(math.radians(h0_deg))))
    b.append(P(1.0, h=wrap_angle(math.radians(h1_deg))))
    h = b.pose_at(0.5).heading
    assert abs(angle_diff(h, math.radians(expected_deg))) < 1e-9
    assert -math.pi <= h < math.pi


def test_outside_history_returns_none_or_newest_within_tolerance():
    b = PoseBuffer(tolerance_s=0.25)
    assert b.pose_at(1.0) is None                # empty
    b.append(P(1.0, 1.0))
    b.append(P(1.1, 2.0))
    assert b.pose_at(0.99) is None               # before the oldest
    assert b.pose_at(1.3).x == 2.0               # just after newest -> newest
    assert b.pose_at(1.36) is None               # too far past newest


def test_out_of_order_pose_is_dropped():
    b = PoseBuffer()
    assert b.append(P(2.0))
    assert not b.append(P(1.5))
    assert len(b) == 1


def test_history_is_trimmed():
    b = PoseBuffer(history_s=1.0)
    for i in range(30):
        b.append(P(i * 0.1))
    snap = b.snapshot()
    assert snap[-1].t == pytest.approx(2.9)
    assert snap[0].t >= 2.9 - 1.0 - 1e-9


def test_yaw_rate_including_across_the_wrap():
    b = PoseBuffer()
    b.append(P(0.0, h=math.radians(170)))
    b.append(P(0.5, h=math.radians(-170)))       # +20 deg in 0.5 s, through 180
    assert b.yaw_rate_at(0.25) == pytest.approx(math.radians(40))
    assert b.yaw_rate_at(0.5) == pytest.approx(math.radians(40))    # on a sample
    assert b.yaw_rate_at(0.6) == pytest.approx(math.radians(40))    # past newest
    assert PoseBuffer().yaw_rate_at(0.0) is None


# ---------------------------------------------------------------------------
# TelemetryTracker
# ---------------------------------------------------------------------------

def test_tracker_wraps_firmware_heading_and_ignores_other_drones(clock):
    t = TelemetryTracker(DRONE, PoseBuffer(), clock)
    seen = []
    t.on_pose(seen.append)
    t.callback(packet(1.0, 2.0, heading=3 * math.pi / 2, drone_id=9), "1.2.3.4")
    assert t.latest().packet is None and not seen
    t.callback(packet(1.0, 2.0, heading=3 * math.pi / 2), "192.168.1.202")
    assert t.packets == 1
    assert seen[0].heading == pytest.approx(-math.pi / 2)
    assert t.poses.latest() == seen[0]
    assert seen[0].t == clock.t


def test_pose_from_packet():
    p = pose_from_packet(packet(0.5, -0.25, 0.1, NAV_FLYING), 7.0)
    assert p == Pose(7.0, 0.5, -0.25, pytest.approx(0.1), NAV_FLYING)
    assert p.nav_state_name == "FLYING"


# ---------------------------------------------------------------------------
# DroneLink
# ---------------------------------------------------------------------------

def test_start_resets_offset_before_listening(settings, clock):
    link, comms, _ = make_link(settings, clock)
    link.start()
    assert comms.calls[0] == ("set_nav_tags", [], {})
    assert comms.calls[1] == ("start",)
    assert comms.callbacks == [link.tracker.callback]
    link.reset_offset()
    assert comms.calls[-1] == ("send_nav_tags", DRONE)


def test_staleness(settings, clock):
    link, comms, _ = make_link(settings, clock)
    link.start()
    assert not link.connected
    assert link.is_stale()                       # never heard from it
    comms.callbacks[0](packet(), "192.168.1.202")
    assert link.connected and not link.is_stale()
    clock.t += settings.safety.stale_timeout_s - 0.01
    assert not link.is_stale()
    clock.t += 0.02
    assert link.is_stale()
    assert link.telemetry_age() == pytest.approx(settings.safety.stale_timeout_s + 0.01)


def test_land_and_start_are_repeated(settings, clock):
    events = []
    link, comms, sleeps = make_link(settings, clock,
                                    on_command=lambda n, d: events.append((n, d)))
    assert link.send_land()
    assert [c[1] for c in comms.sent] == [CMD_LAND] * 3
    assert sleeps == [settings.flight.repeat_gap_s] * 2
    assert link.send_start()
    assert [c[1] for c in comms.sent[3:]] == [CMD_START] * settings.flight.start_repeats
    assert events[0] == ("CMD_LAND", "x3")


def test_goto_and_hold(settings, clock):
    link, comms, _ = make_link(settings, clock)
    assert link.send_goto(1.25, -0.5)
    assert link.send_hold()
    assert comms.sent == [(DRONE, CMD_GOTO, 1.25, -0.5), (DRONE, CMD_HOLD, 0.0, 0.0)]


def test_unknown_ip_returns_false_without_raising(settings, clock):
    events = []
    link, comms, _ = make_link(settings, clock, comms=FakeComms(ip=None),
                               on_command=lambda n, d: events.append((n, d)))
    assert link.send_land() is False
    assert events == [("CMD_LAND", "x3 FAILED")]


def test_send_exception_is_swallowed(settings, clock):
    comms = FakeComms()
    comms.send_command = lambda *a: (_ for _ in ()).throw(OSError("net down"))
    link, _, _ = make_link(settings, clock, comms=comms)
    assert link.send_land() is False


def test_dry_run_never_sends_flight_commands(settings, clock):
    events = []
    link, comms, _ = make_link(settings, clock, dry_run=True,
                               on_command=lambda n, d: events.append(n))
    link.start()
    for send in (link.send_start, link.send_hold, link.send_land):
        assert send() is False
    assert link.send_goto(1.0, 1.0) is False
    assert comms.sent == []
    assert events == ["CMD_START", "CMD_HOLD", "CMD_LAND", "CMD_GOTO"]
    assert ("set_nav_tags", [], {}) in comms.calls      # offset reset still allowed


# ---------------------------------------------------------------------------
# Real CommsNode over UDP loopback (no drone needed)
# ---------------------------------------------------------------------------

def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _telemetry_bytes(x, y, heading, nav) -> bytes:
    return struct.pack("<BBfffBbf32sBH", 0x01, DRONE, x, y, heading, nav, -1, 0.0,
                       b"\x00" * 32, 0, 0xFFFF)


def test_real_comms_node_round_trip(settings):
    telem_port, cmd_port = _free_udp_port(), _free_udp_port()
    s = dataclasses.replace(settings, network=dataclasses.replace(
        settings.network, telem_port=telem_port, cmd_port=cmd_port))
    drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # plays the ESP32
    drone.bind(("127.0.0.1", cmd_port))
    drone.settimeout(2.0)
    link = DroneLink(s)
    link.start()
    try:
        drone.sendto(_telemetry_bytes(1.5, -0.5, 4.0, NAV_FLYING), ("127.0.0.1", telem_port))
        deadline = time.monotonic() + 2.0
        while not link.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.connected
        assert link.esp_ip == "127.0.0.1"
        pose = link.latest_pose()
        assert (pose.x, pose.y) == (pytest.approx(1.5), pytest.approx(-0.5))
        assert pose.heading == pytest.approx(4.0 - 2 * math.pi, abs=1e-6)

        # First contact -> CommsNode sends the nav-tag packet with offset (0, 0).
        nav = drone.recv(1024)
        assert nav[:2] == bytes([PKT_CMD, CMD_SET_NAV_TAGS])
        assert struct.unpack_from("<Bff", nav, 2) == (0, 0.0, 0.0)

        assert link.send_land()
        cmds = [drone.recv(1024) for _ in range(3)]
        assert all(c[:2] == bytes([PKT_CMD, CMD_LAND]) for c in cmds)
    finally:
        link.stop()
        drone.close()
