"""Camera frames -> Sightings in PX4 local NED (mission/tag_pipeline.py).

Uses synthetic frames from tag_render.py, detected with the course's own
firmware detector (apriltag_host.Detector), so the whole chain
image -> detection -> t_body -> capture-time pose -> NED is exercised.
"""

import dataclasses
import math
import socket
import struct
import time

import numpy as np
import pytest

from apriltag_host import CAM_POS_M, R_BODY_CAM, Detection, Detector
from drone_link import Pose, PoseBuffer
from geometry import body_to_ned
from protocol import CMD_CAMERA_STREAM, PKT_CMD
from settings import load_settings
from tag_pipeline import (R_HEIGHT, R_GATE, R_NO_DRONE, R_NO_POSE, R_POSE_ERR,
                          R_RANGE, R_YAW_RATE, TagFilter, TagPipeline,
                          capture_time, process_gray)
from synth_camera import FloorTag, floor_tag_corners_px, jpeg_bytes, render_view

TOL_M = 0.03          # "within a few cm"


@pytest.fixture(scope="module")
def detector():
    with Detector() as d:
        yield d


@pytest.fixture
def settings():
    return load_settings()


@pytest.fixture
def tag_filter(settings):
    return TagFilter(settings.tags)


class StillPoses:
    """Pose source that always returns one pose (drone hovering)."""

    def __init__(self, x, y, h, rate=0.0):
        self.pose, self.rate = (x, y, h), rate

    def pose_at(self, t):
        return Pose(t, *self.pose, 2)

    def yaw_rate_at(self, t):
        return self.rate


def tag_ahead(drone, forward, right, tag_id=3, yaw=0.4):
    n, e = body_to_ned(forward, right, *drone)
    return FloorTag(tag_id, n, e, yaw)


# ---------------------------------------------------------------------------
# Offline: synthetic frames with a known pose
# ---------------------------------------------------------------------------

def test_capture_time():
    assert capture_time(100.0, 130) == pytest.approx(99.87)
    assert capture_time(100.0, 130, bias_s=0.02) == pytest.approx(99.85)


@pytest.mark.parametrize("heading_deg", [0, 90, 180, -90, -135, 30])
@pytest.mark.parametrize("forward, right", [(0.6, 0.0), (0.5, 0.25), (0.8, -0.3)])
def test_synthetic_tag_lands_within_a_few_cm(detector, tag_filter, heading_deg,
                                             forward, right):
    drone = (0.4, -0.2, math.radians(heading_deg))
    tag = tag_ahead(drone, forward, right)
    gray = render_view([tag], drone)
    got, rejected = process_gray(gray, detector, tag_filter, StillPoses(*drone), 5.0, 5.1)
    assert rejected == []
    assert [s.tag_id for s in got] == [3]
    s = got[0]
    assert math.dist((s.x, s.y), (tag.north, tag.east)) < TOL_M
    assert s.body[2] == pytest.approx(0.5, abs=0.03)
    assert (s.t_capture, s.t_detect) == (5.0, 5.1)
    assert s.pose_err is not None and s.decision_margin > 55


def test_several_tags_in_one_frame(detector, tag_filter):
    drone = (1.0, 0.5, math.radians(60))
    tags = [tag_ahead(drone, 0.55, -0.3, 1), tag_ahead(drone, 0.6, 0.0, 9),
            tag_ahead(drone, 0.75, 0.35, 12)]
    got, _ = process_gray(render_view(tags, drone), detector, tag_filter,
                          StillPoses(*drone), 0.0, 0.0)
    assert sorted(s.tag_id for s in got) == [1, 9, 12]
    for s in got:
        t = next(t for t in tags if t.tag_id == s.tag_id)
        assert math.dist((s.x, s.y), (t.north, t.east)) < TOL_M


def test_uses_pose_at_capture_time_not_latest(detector, tag_filter):
    # Drone flies North at 0.1 m/s; the frame was captured at t=10.5 (x=0.05).
    buf = PoseBuffer()
    buf.append(Pose(10.0, 0.0, 0.0, 0.0, 2))
    buf.append(Pose(11.0, 0.1, 0.0, 0.0, 2))
    drone_then = (0.05, 0.0, 0.0)
    tag = tag_ahead(drone_then, 0.6, 0.1)
    got, _ = process_gray(render_view([tag], drone_then), detector, tag_filter,
                          buf, 10.5, 11.0)
    assert got[0].drone.x == pytest.approx(0.05)
    assert math.dist((got[0].x, got[0].y), (tag.north, tag.east)) < TOL_M


def test_wrong_tag_size_is_caught_by_the_floor_check(detector, settings):
    # 90 mm tag with the detector told 120 mm: everything looks 4/3 too far.
    floor_only = TagFilter(dataclasses.replace(settings.tags, tags_on_floor=True))
    drone = (0.0, 0.0, 0.0)
    n, e = body_to_ned(0.6, 0.0, *drone)
    gray = render_view([FloorTag(3, n, e, 0.2, size=0.09)], drone)
    got, rejected = process_gray(gray, detector, floor_only, StillPoses(*drone), 0, 0)
    assert got == [] and rejected == [(3, R_HEIGHT)]


def test_marks_outline_every_detection(detector, settings):
    # One accepted tag and one rejected (too high off the floor in floor mode).
    floor_only = TagFilter(dataclasses.replace(settings.tags, tags_on_floor=True))
    drone = (0.0, 0.0, 0.0)
    good = tag_ahead(drone, 0.6, -0.2, tag_id=3)
    n, e = body_to_ned(0.6, 0.25, *drone)
    small = FloorTag(9, n, e, 0.3, size=0.09)        # looks too far -> height reject
    marks = []
    process_gray(render_view([good, small], drone), detector, floor_only,
                 StillPoses(*drone), 0, 0, marks)
    by_id = {m.tag_id: m for m in marks}
    assert by_id[3].reason is None and by_id[9].reason == R_HEIGHT
    # Detected corners match the projection the sim uses for its outlines.
    want = floor_tag_corners_px(good, drone)
    for c in by_id[3].corners:
        assert min(math.dist(c, w) for w in want) < 1.5


def test_empty_floor_gives_nothing(detector, tag_filter):
    got, rejected = process_gray(render_view([], (0, 0, 0)), detector, tag_filter,
                                 StillPoses(0, 0, 0), 0, 0)
    assert got == [] and rejected == []


# ---------------------------------------------------------------------------
# Filters, one reason at a time
# ---------------------------------------------------------------------------

def make_det(body=(0.6, 0.1, 0.5), margin=200.0, hamming=0, pose_err=1e-4,
             pose=True, tag_id=4) -> Detection:
    """A Detection whose t_body equals `body` (camera frame solved backwards)."""
    R = np.asarray(R_BODY_CAM)
    t = tuple(R.T @ (np.asarray(body) - np.asarray(CAM_POS_M)))
    return Detection(id=tag_id, hamming=hamming, decision_margin=margin,
                     centre=(160.0, 120.0), corners=((0, 0),) * 4, pose_valid=pose,
                     t=t if pose else None, R=tuple(np.eye(3).ravel()) if pose else None,
                     pose_err=pose_err if pose else None)


def test_make_det_round_trips():
    assert make_det((0.6, 0.1, 0.5)).t_body == pytest.approx((0.6, 0.1, 0.5))


def test_accepts_a_good_detection(tag_filter):
    s, reason = tag_filter.evaluate(make_det(), 1.0, 1.1, StillPoses(1.0, 0.0, math.pi / 2))
    assert reason is None
    assert (s.x, s.y) == (pytest.approx(0.9), pytest.approx(0.6))   # 0.6 E, 0.1 S
    assert s.cam_R is not None and s.decision_margin == 200.0


@pytest.mark.parametrize("det, poses, reason", [
    (make_det(margin=40.0), StillPoses(0, 0, 0), R_GATE),
    (make_det(hamming=2), StillPoses(0, 0, 0), R_GATE),
    (make_det(pose=False), StillPoses(0, 0, 0), R_NO_POSE),
    (make_det(pose_err=0.9), StillPoses(0, 0, 0), R_POSE_ERR),
    (make_det(body=(1.6, 0.0, 0.5)), StillPoses(0, 0, 0), R_RANGE),
    (make_det(body=(0.6, 0.0, 0.8)), StillPoses(0, 0, 0), R_HEIGHT),
    (make_det(), PoseBuffer(), R_NO_DRONE),
    (make_det(), StillPoses(0, 0, 0, rate=math.radians(45)), R_YAW_RATE),
    (make_det(), StillPoses(0, 0, 0, rate=-math.radians(45)), R_YAW_RATE),
])
def test_rejections(tag_filter, det, poses, reason):
    assert tag_filter.evaluate(det, 1.0, 1.0, poses) == (None, reason)


@pytest.mark.parametrize("down, on_floor, ok", [
    (0.0, False, True),      # wall tag level with the drone
    (-0.4, False, True),     # wall tag 0.4 m above the drone
    (-0.6, False, False),    # implausibly high
    (0.5, False, True),      # floor tag, wall mode still accepts it
    (0.7, False, False),     # below the floor
    (0.5, True, True),
    (0.3, True, False),      # floor mode is strict
])
def test_height_check(settings, down, on_floor, ok):
    f = TagFilter(dataclasses.replace(settings.tags, tags_on_floor=on_floor))
    s, reason = f.evaluate(make_det(body=(0.6, 0.0, down)), 0, 0, StillPoses(0, 0, 0))
    assert (reason is None) == ok
    assert ok or reason == R_HEIGHT


def test_unknown_yaw_rate_is_not_a_rejection(tag_filter):
    class NoRate(StillPoses):
        def yaw_rate_at(self, t):
            return None
    s, reason = tag_filter.evaluate(make_det(), 0, 0, NoRate(0, 0, 0))
    assert reason is None


# ---------------------------------------------------------------------------
# Threaded pipeline over UDP loopback (a fake ESP32 streaming JPEG chunks)
# ---------------------------------------------------------------------------

HEADER = "<4sBBBBIIIHHHHH"     # main/camera_stream.c, version 2


def camera_datagrams(frame_id, jpeg, age_ms=120, chunk=1400):
    def pkt(flags, offset, size, payload):
        return struct.pack(HEADER, b"ECAM", 2, flags, 2, 7, frame_id, offset, size,
                           320, 240, len(payload), age_ms, 0) + payload
    out = [pkt(0, off, 0, jpeg[off:off + chunk]) for off in range(0, len(jpeg), chunk)]
    return out + [pkt(0x02, len(jpeg), len(jpeg), b"")]


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_pipeline_thread_end_to_end(settings):
    cam_port, cmd_port = free_port(), free_port()
    s = dataclasses.replace(settings, network=dataclasses.replace(
        settings.network, camera_port=cam_port, cmd_port=cmd_port))
    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # the fake drone
    esp.bind(("127.0.0.1", cmd_port))
    esp.settimeout(3.0)

    drone = (0.3, 0.2, math.radians(-90))
    tag = tag_ahead(drone, 0.6, 0.05, tag_id=7)
    poses = StillPoses(*drone)
    got = []
    pipe = TagPipeline(s, poses, esp_ip=lambda: "127.0.0.1", on_sighting=got.append,
                       bind_host="127.0.0.1")
    assert pipe.latest_view() is None
    pipe.start()
    try:
        keepalive = esp.recv(64)                             # it asks for video
        assert keepalive[:3] == bytes([PKT_CMD, CMD_CAMERA_STREAM, 1])
        jpeg = jpeg_bytes(render_view([tag], drone, jpeg_quality=None))
        for frame_id in (1, 2):
            for d in camera_datagrams(frame_id, jpeg):
                esp.sendto(d, ("127.0.0.1", cam_port))
            time.sleep(0.05)
        deadline = time.monotonic() + 3.0
        while len(got) < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert [g.tag_id for g in got] == [7, 7]
        assert math.dist((got[0].x, got[0].y), (tag.north, tag.east)) < TOL_M
        now = time.monotonic()
        assert now - 1.0 < got[0].t_capture < now           # first_rx - 0.12 s
        st = pipe.stats()
        assert st.frames == 2 and st.accepted == 2 and st.last_age_ms == 120
        view = pipe.latest_view()                            # for the camera panel
        assert view.gray.shape == (240, 320) and view.age_ms == 120
        assert [(m.tag_id, m.reason) for m in view.marks] == [(7, None)]
        assert all(0 <= u <= 320 and 0 <= v <= 240 for u, v in view.marks[0].corners)
    finally:
        pipe.stop()
    # stop() tells the drone to stop streaming (after any queued keepalives)
    msgs = []
    try:
        while True:
            msgs.append(esp.recv(64))
    except socket.timeout:
        pass
    finally:
        esp.close()
    assert msgs and msgs[-1][:3] == bytes([PKT_CMD, CMD_CAMERA_STREAM, 0])
    assert not pipe.running


def test_port_in_use_fails_loudly(settings):
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("127.0.0.1", 0))
    port = blocker.getsockname()[1]
    s = dataclasses.replace(settings, network=dataclasses.replace(
        settings.network, camera_port=port))
    pipe = TagPipeline(s, StillPoses(0, 0, 0), lambda: None, lambda _: None,
                       bind_host="127.0.0.1")
    try:
        with pytest.raises(OSError, match="close tag_stream.py"):
            pipe.start()
    finally:
        blocker.close()
