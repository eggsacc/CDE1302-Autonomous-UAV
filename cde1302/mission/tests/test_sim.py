"""Fake drone and fake camera (mission/sim.py)."""

import dataclasses
import inspect
import math

import numpy as np
import pytest

from drone_link import DroneLink
from protocol import NAV_ARRIVED, NAV_FLYING, NAV_IDLE, NAV_ROTATING
from settings import TrueTag, load_settings
from sim import ManualClock, ScaledClock, SimDrone, SimTagSource
from tag_map import TagMap
from tag_pipeline import R_YAW_RATE, TagPipeline


@pytest.fixture
def settings():
    return load_settings()


@pytest.fixture
def clock():
    return ManualClock(100.0)


def run(clock, drone, seconds, dt=0.1, cam=None, until=None):
    """Advance sim time; returns the telemetry nav states seen."""
    states = []
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        p = drone.latest_pose()
        if cam is not None:
            cam.pump()
        if p is not None and (not states or states[-1] != p.nav_state):
            states.append(p.nav_state)
        if until is not None and until():
            break
    return states


def airborne(settings, clock, **kw):
    d = SimDrone(settings, clock, **kw)
    d.start()
    run(clock, d, 0.5)
    assert d.send_start()
    run(clock, d, 10.0, until=lambda: d.phase == "FLYING")
    assert d.phase == "FLYING"
    return d


def with_sim(settings, **kw):
    return dataclasses.replace(settings, sim=dataclasses.replace(settings.sim, **kw))


# ---------------------------------------------------------------------------
# Interfaces match the real classes
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("real, fake", [(DroneLink, SimDrone), (TagPipeline, SimTagSource)])
def test_sim_has_the_real_public_interface(real, fake):
    public = {n for n, _ in inspect.getmembers(real) if not n.startswith("_")}
    missing = public - {n for n, _ in inspect.getmembers(fake)}
    assert missing == set()


# ---------------------------------------------------------------------------
# SimDrone
# ---------------------------------------------------------------------------

def test_clocks():
    c = ManualClock(5.0)
    assert c() == 5.0 and c.advance(0.5) == 5.5
    real = [10.0]
    s = ScaledClock(4.0, base=lambda: real[0])
    real[0] = 11.0
    assert s() == pytest.approx(14.0)


def test_telemetry_is_10_hz_and_starts_after_start(settings, clock):
    d = SimDrone(settings, clock)
    assert not d.connected and d.esp_ip is None
    d.start()
    run(clock, d, 10.0)
    assert d.connected and d.esp_ip == "127.0.0.1"
    assert 99 <= len(d.poses) <= 101
    assert d.telemetry_age() < 0.11


def test_commands_need_telemetry(settings, clock):
    d = SimDrone(settings, clock)
    assert d.send_start() is False


def test_takeoff_sequence_and_goto_ignored_until_flying(settings, clock):
    d = SimDrone(settings, clock)
    d.start()
    run(clock, d, 0.5)
    d.send_start()
    assert d.phase == "ARMING"
    assert d.send_goto(1.0, 0.0)                 # sent, but firmware ignores it
    run(clock, d, settings.sim.arm_delay_s + 0.2)
    assert d.armed and d.phase == "CLIMBING"
    run(clock, d, 10.0, until=lambda: d.phase == "FLYING")
    assert d.alt == pytest.approx(0.5)
    assert (d.x, d.y) == (0.0, 0.0)              # the early GOTO did nothing
    assert d.latest_pose().nav_state == NAV_IDLE


def test_second_start_is_ignored(settings, clock):
    d = airborne(settings, clock)
    d.send_start()
    assert d.phase == "FLYING"


def test_goto_rotates_then_flies_at_cruise_speed(settings, clock):
    d = airborne(settings, clock)
    t0 = clock()
    d.send_goto(0.0, 1.0)                        # due East: 90 deg turn, 1 m
    states = run(clock, d, 30.0, until=lambda: d.latest_pose().nav_state == NAV_ARRIVED)
    assert states[:3] == [NAV_ROTATING, NAV_FLYING, NAV_ARRIVED]
    turn = 90 / settings.sim.yaw_rate_deg_s
    fly = (1.0 - 0.25) / settings.flight.cruise_speed_mps
    assert clock() - t0 == pytest.approx(turn + fly, abs=0.6)
    assert d.heading == pytest.approx(math.pi / 2, abs=0.1)
    run(clock, d, 5.0)                           # settles onto the goal
    assert math.hypot(d.x - 0.0, d.y - 1.0) < 0.01


def test_hold_stops_mid_leg(settings, clock):
    d = airborne(settings, clock)
    d.send_goto(2.0, 0.0)
    run(clock, d, 6.0)
    assert d.latest_pose().nav_state == NAV_FLYING
    d.send_hold()
    x = d.x
    run(clock, d, 3.0)
    assert d.x == pytest.approx(x) and d.latest_pose().nav_state == NAV_IDLE


def test_land_descends_disarms_and_returns_to_ready(settings, clock):
    d = airborne(settings, clock)
    d.send_land()
    assert d.phase == "LANDING"
    run(clock, d, 5.0)
    assert d.alt == 0.0 and not d.armed and d.phase == "READY"
    assert [c[1] for c in d.commands].count("CMD_LAND") == 3


def test_land_during_arming_aborts(settings, clock):
    d = SimDrone(settings, clock)
    d.start()
    run(clock, d, 0.5)
    d.send_start()
    d.send_land()
    run(clock, d, 5.0)
    assert d.phase == "READY" and not d.armed and d.alt == 0.0


def test_paused_telemetry_goes_stale(settings, clock):
    d = airborne(settings, clock)
    d.pause_telemetry()
    run(clock, d, settings.safety.stale_timeout_s + 0.2)
    assert d.is_stale()
    d.pause_telemetry(False)
    run(clock, d, 0.2)
    assert not d.is_stale()


def test_command_loss_drops_some_copies(settings, clock):
    d = airborne(with_sim(settings, command_loss=0.5), clock)
    for _ in range(40):
        d.send_hold()
    n = [c[1] for c in d.commands].count("CMD_HOLD")
    assert 8 < n < 32


def test_dry_run_sim_never_moves(settings, clock):
    hooks = []
    d = SimDrone(settings, clock, dry_run=True, on_command=lambda n, _: hooks.append(n))
    d.start()
    run(clock, d, 0.5)
    assert d.send_start() is False
    run(clock, d, 10.0)
    assert d.phase == "READY" and hooks == ["CMD_START"]


def test_heading_reported_in_minus_pi_to_pi(settings, clock):
    d = airborne(settings, clock)
    d.send_goto(-1.0, -0.05)                     # turn to face ~South-ish (180 deg)
    run(clock, d, 8.0)
    assert all(-math.pi <= p.heading < math.pi for p in d.poses.snapshot())


def test_pose_listeners_get_every_telemetry_pose(settings, clock):
    d = SimDrone(settings, clock)
    got = []
    d.on_pose(got.append)
    d.start()
    run(clock, d, 2.0)
    assert len(got) == len(d.poses)


# ---------------------------------------------------------------------------
# SimTagSource
# ---------------------------------------------------------------------------

def one_tag(settings, x, y, **sim_kw):
    return with_sim(settings, true_tags=(TrueTag(4, x, y),), **sim_kw)


def test_tag_in_view_is_seen_at_camera_rate_with_latency(settings, clock):
    s = one_tag(settings, 0.6, 0.0)                       # 0.6 m ahead of start
    d = airborne(s, clock)
    got = []
    cam = SimTagSource(s, d, got.append, clock=clock, threaded=False)
    cam.start()
    run(clock, d, 5.0, cam=cam)
    assert 40 <= len(got) <= 46                           # ~9 fps x 5 s
    for g in got:
        assert g.tag_id == 4
        assert g.t_detect - g.t_capture == pytest.approx(s.sim.latency_s)
        assert math.dist((g.x, g.y), (0.6, 0.0)) < 5 * s.sim.sighting_noise_m
    assert cam.stats().accepted == len(got)


def test_tag_outside_footprint_or_on_ground_not_seen(settings, clock):
    s = with_sim(settings, true_tags=(TrueTag(4, -0.6, 0.0), TrueTag(5, 0.6, 0.0)))
    d = SimDrone(s, clock)
    d.start()
    got = []
    cam = SimTagSource(s, d, got.append, clock=clock, threaded=False)
    cam.start()
    run(clock, d, 3.0, cam=cam)                           # on the ground
    assert got == []
    d.send_start()
    run(clock, d, 10.0, cam=cam, until=lambda: d.phase == "FLYING")
    run(clock, d, 2.0, cam=cam)
    assert {g.tag_id for g in got} == {5}                 # tag 4 is behind


def test_turning_sightings_are_rejected(settings, clock):
    # Tag ends up in view only while rotating fast -> yaw-rate filter drops it.
    # The 180 deg turn goes anticlockwise (North -> West -> South), sweeping
    # the camera past a tag 0.6 m West of the drone.
    s = one_tag(settings, 0.0, -0.6, yaw_rate_deg_s=90.0)
    d = airborne(s, clock)
    got = []
    cam = SimTagSource(s, d, got.append, clock=clock, threaded=False)
    cam.start()
    d.send_goto(-1.0, 0.0)
    run(clock, d, 1.5, cam=cam)
    assert cam.stats().rejected[R_YAW_RATE] > 0
    assert got == []


def test_sim_sightings_confirm_in_tag_map(settings, clock):
    s = one_tag(settings, 0.7, 0.1, false_positive_rate=0.3)
    d = airborne(s, clock)
    tm = TagMap.from_settings(s.tags)
    cam = SimTagSource(s, d, tm.add, clock=clock, threaded=False)
    cam.start()
    run(clock, d, 5.0, cam=cam)
    confirmed = tm.estimates(confirmed_only=True)
    assert [e.tag_id for e in confirmed] == [4]
    assert math.dist((confirmed[0].x, confirmed[0].y), (0.7, 0.1)) < 0.03


def test_threaded_source_delivers(settings, clock):
    import time
    s = one_tag(settings, 0.6, 0.0)
    d = airborne(s, clock)
    got = []
    cam = SimTagSource(s, d, got.append, clock=clock)    # pumps on its own thread
    cam.start()
    assert cam.running
    deadline = time.monotonic() + 3.0
    while not got and time.monotonic() < deadline:
        clock.advance(0.1)
        time.sleep(0.01)
    cam.stop()
    assert got and not cam.running


def test_sim_camera_view_for_the_live_map(settings, clock):
    s = one_tag(settings, 0.6, 0.0)
    d = airborne(s, clock)
    cam = SimTagSource(s, d, lambda _: None, clock=clock, threaded=False)
    assert cam.latest_view() is None                      # no frame yet
    cam.start()
    run(clock, d, 1.0, cam=cam)
    view = cam.latest_view()
    assert view.gray.shape == (240, 320) and view.gray.dtype == np.uint8
    assert [(m.tag_id, m.reason) for m in view.marks] == [(4, None)]
    u = [c[0] for c in view.marks[0].corners]
    v = [c[1] for c in view.marks[0].corners]
    assert 100 < sum(u) / 4 < 220 and 0 < min(v) and max(v) < 240   # straight ahead
    # The black tag is really drawn under its outline.
    cu, cv = int(sum(u) / 4), int(sum(v) / 4)
    assert view.gray[cv - 4:cv + 5, cu - 4:cu + 5].min() < 100
    assert cam.latest_view() is view                      # rendered once per frame


def test_sim_camera_view_marks_rejections(settings, clock):
    s = one_tag(settings, 0.0, -0.6, yaw_rate_deg_s=90.0)
    d = airborne(s, clock)
    cam = SimTagSource(s, d, lambda _: None, clock=clock, threaded=False)
    cam.start()
    d.send_goto(-1.0, 0.0)                                # turn fast past the tag
    reasons = set()
    for _ in range(15):
        run(clock, d, 0.1, cam=cam)
        view = cam.latest_view()
        if view is not None:
            reasons |= {m.reason for m in view.marks}
    assert R_YAW_RATE in reasons
