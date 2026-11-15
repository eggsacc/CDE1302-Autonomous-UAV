"""UwbFeed: debug stream -> paired samples -> alignment, and the mission around it."""

import dataclasses
import math
import random
import socket
import time

import pytest

from mission import Phase
from settings import load_settings
from test_mission import Rig, configure
from test_nlink import node_frame2
from uwb_align import AlignResult, Alignment, AlignSample
from uwb_feed import UwbFeed
from uwb_link import UwbRecord, build_packet

TRUE = Alignment(math.radians(25.0), True, 1.2, -0.7)


def datagram(i, ux, uy, seq=None, drone_id=2, px4=True, age=20, align_received=0, pos=None):
    px, py = TRUE.to_px4(ux, uy)
    rec = UwbRecord(1_000_000 + 20_000 * i, (px, py, -0.5, 0.0) if px4 else None,
                    age if px4 else None,
                    node_frame2(pos=pos if pos is not None else (ux, uy, -1.0)))
    return build_packet(drone_id, seq if seq is not None else i, 100 + i,
                        {"align_received": align_received, "px4_fed": i}, [rec])


def l_path(n=400):
    """A walk with a corner, so a mirrored axis can be told apart."""
    half = n // 2
    return ([(0.01 * i, 0.0) for i in range(half)]
            + [(0.01 * half, 0.01 * i) for i in range(half)])


def fed(path, **kw):
    feed = UwbFeed(load_settings())
    feed.begin()
    for i, (ux, uy) in enumerate(path):
        feed.feed_datagram(datagram(i, ux, uy, **kw))
    return feed


def test_recovers_the_alignment_from_the_stream():
    r = fed(l_path()).solve()
    assert r.ok, r.reason
    a = r.alignment
    assert a.mirror is True
    assert (a.theta, a.tx, a.ty) == pytest.approx((TRUE.theta, TRUE.tx, TRUE.ty), abs=0.01)


def test_nothing_is_collected_before_begin_or_after_finish():
    feed = UwbFeed(load_settings())
    feed.feed_datagram(datagram(0, 1.0, 1.0))
    assert feed.estimator.samples == []
    feed.begin()
    feed.feed_datagram(datagram(1, 1.0, 1.0))
    feed.finish()
    feed.feed_datagram(datagram(2, 1.0, 1.0))
    assert len(feed.estimator.samples) == 1


def test_begin_forgets_earlier_samples():
    feed = fed(l_path(50))
    assert feed.estimator.samples
    feed.begin()
    assert feed.estimator.samples == []


@pytest.mark.parametrize("kw", [{"px4": False}, {"age": 201}, {"drone_id": 7}])
def test_unusable_records_are_skipped(kw):
    assert fed(l_path(20), **kw).estimator.samples == []


def test_corrupt_datagram_is_counted_not_raised():
    feed = UwbFeed(load_settings())
    feed.feed_datagram(b"garbage")
    assert feed.link.bad_packets == 1 and feed.latest() is None


def test_counters_come_from_the_newest_packet():
    feed = UwbFeed(load_settings())
    assert feed.align_received == 0
    feed.feed_datagram(datagram(3, 1.0, 1.0, align_received=4))
    assert feed.align_received == 4 and feed.counter("px4_fed") == 3


def test_reads_real_udp_datagrams():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:      # find a free port
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    s = load_settings()
    feed = UwbFeed(dataclasses.replace(s, uwb=dataclasses.replace(s.uwb, debug_port=port)))
    feed.start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            tx.sendto(datagram(0, 1.0, 2.0, align_received=2), ("127.0.0.1", port))
            deadline = time.monotonic() + 2.0
            while feed.link.packets == 0 and time.monotonic() < deadline:
                time.sleep(0.01)
        assert feed.align_received == 2 and feed.latest().uy == pytest.approx(2.0)
    finally:
        feed.stop()


def test_noisy_stream_still_fits():
    rng = random.Random(1)
    path = [(x + rng.gauss(0, 0.03), y + rng.gauss(0, 0.03)) for x, y in l_path(600)]
    feed = UwbFeed(load_settings())
    feed.begin()
    for i, (ux, uy) in enumerate(path):
        clean_x, clean_y = l_path(600)[i]
        px, py = TRUE.to_px4(clean_x, clean_y)
        rec = UwbRecord(i, (px, py, -0.5, 0.0), 20, node_frame2(pos=(ux, uy, -1.0)))
        feed.feed_datagram(build_packet(2, i, i, {}, [rec]))
    r = feed.solve()
    assert r.ok, r.reason
    assert r.rms_m < 0.1


# ---------------------------------------------------------------------------
# The mission around it
# ---------------------------------------------------------------------------

class FakeFeed:
    """What Mission uses of UwbFeed, with the answers set by the test."""

    def __init__(self, ok=True, latest=None, ack=True):
        self.result = (AlignResult(True, "", TRUE, rms_m=0.04, n_used=300) if ok
                       else AlignResult(False, "12/50 usable samples"))
        px, py = TRUE.to_px4(1.0, 2.0)
        self.sample = latest or AlignSample(0.0, 1.0, 2.0, px, py)
        self.ack = ack
        self.align_received = 0
        self.calls = []

    def start(self):
        self.calls.append("start")

    def stop(self):
        self.calls.append("stop")

    def begin(self):
        self.calls.append("begin")

    def finish(self):
        self.calls.append("finish")

    def solve(self):
        self.calls.append("solve")
        return self.result

    def latest(self):
        return self.sample


def rig(tmp_path, feed):
    r = Rig(configure(tmp_path, flight={"stop_when_all_found": False},
                      uwb={"send_to_px4": True}), uwb=feed)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    return r


def fly_until_exploring(r):
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 60)


def test_alignment_is_sent_during_the_sweep_and_then_stops(tmp_path):
    feed = FakeFeed()
    r = rig(tmp_path, feed)
    assert "start" in feed.calls and "solve" not in feed.calls
    fly_until_exploring(r)
    assert "begin" in feed.calls
    assert r.until(lambda: r.drone.uwb_alignments, 5)
    assert r.drone.uwb_alignments[0] == TRUE and "finish" in feed.calls
    assert r.events("UWB_ALIGNED")

    # UDP may be lost: resent until the ESP32's counter shows it arrived.
    assert r.until(lambda: len(r.drone.uwb_alignments) >= 3, 10)
    feed.align_received = 1
    assert r.until(lambda: r.events("UWB_ALIGN_CONFIRMED"), 5)
    sent = len(r.drone.uwb_alignments)
    r.finish()
    assert len(r.drone.uwb_alignments) == sent
    assert r.m.exit_code == 0 and "stop" in feed.calls


def test_a_sweep_with_no_alignment_flies_on_without_uwb(tmp_path):
    feed = FakeFeed(ok=False)
    r = rig(tmp_path, feed)
    fly_until_exploring(r)
    r.finish()
    assert r.m.exit_code == 0 and r.drone.uwb_alignments == []
    assert len(r.events("UWB_ALIGN_WAITING")) == 1        # logged once, not every second


def test_alignment_that_would_jump_px4_is_refused(tmp_path):
    px, py = TRUE.to_px4(1.0, 2.0)
    feed = FakeFeed(latest=AlignSample(0.0, 1.0, 2.0, px + 0.5, py))
    r = rig(tmp_path, feed)
    fly_until_exploring(r)
    assert r.until(lambda: r.events("UWB_ALIGN_REFUSED"), 5)
    assert r.drone.uwb_alignments == [] and r.m.uwb_alignment is None
    r.m.command("land")
    r.finish()


def test_a_stale_ack_from_an_earlier_alignment_does_not_count(tmp_path):
    feed = FakeFeed()
    feed.align_received = 5                                # from a previous run
    r = rig(tmp_path, feed)
    fly_until_exploring(r)
    assert r.until(lambda: r.drone.uwb_alignments, 5)
    assert not r.events("UWB_ALIGN_CONFIRMED")
    feed.align_received = 6
    assert r.until(lambda: r.events("UWB_ALIGN_CONFIRMED"), 5)
    r.m.command("land")
    r.finish()


def test_recording_only_never_sends_an_alignment(tmp_path):
    feed = FakeFeed()
    r = Rig(configure(tmp_path, flight={"stop_when_all_found": False},
                      uwb={"send_to_px4": False, "record": True}), uwb=feed)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    fly_until_exploring(r)
    r.finish()
    assert r.drone.uwb_alignments == [] and "solve" not in feed.calls
    assert "begin" not in feed.calls and not r.events("UWB_COLLECTING")


def test_every_reading_is_handed_to_the_recorder():
    got = []
    feed = UwbFeed(load_settings(), on_reading=lambda **f: got.append(f))
    feed.feed_datagram(datagram(0, 1.0, 2.0))
    feed.feed_datagram(datagram(1, 1.5, 2.5, px4=False))          # recorded, not paired
    assert [g["uwb_x"] for g in got] == [pytest.approx(1.0), pytest.approx(1.5)]
    assert got[0]["px4_x"] is not None and got[1]["px4_x"] is None
    assert got[0]["fix"] == 1 and len(feed.estimator.samples) == 0   # not collecting


def test_without_a_feed_nothing_changes(tmp_path):
    r = rig(tmp_path, None)
    fly_until_exploring(r)
    r.finish()
    assert r.drone.uwb_alignments == [] and not r.events("UWB_ALIGNED")
