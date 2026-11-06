"""Drone pose carried inside each camera frame (main/camera_stream.c -> frame_pose.py),
and the camera's position on the drone (tags.camera_forward_m / right_m / down_m)."""

import dataclasses
import math
import socket
import time

import cv2
import numpy as np
import pytest

from frame_pose import FramePose, build_frame_pose, parse_frame_pose
from tag_pipeline import TagFilter, TagPipeline
from synth_camera import jpeg_bytes, render_view
from test_tag_pipeline import (TOL_M, StillPoses, camera_datagrams, free_port, make_det,
                               settings, tag_ahead)  # noqa: F401  (settings is a fixture)

POSE = FramePose(1.5, -0.4, math.radians(30), 0.1, 12)


def test_round_trip_and_the_image_still_decodes():
    gray = (np.arange(320 * 240) % 251).astype(np.uint8).reshape(240, 320)
    jpeg = bytes(cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 90])[1])
    with_pose = build_frame_pose(jpeg, POSE)
    got = parse_frame_pose(with_pose)
    assert dataclasses.astuple(got) == pytest.approx(dataclasses.astuple(POSE))
    a = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    b = cv2.imdecode(np.frombuffer(with_pose, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(a, b)                     # viewers see the same picture


def test_no_pose_from_old_firmware_or_a_pose_marked_invalid():
    jpeg = bytes(cv2.imencode(".jpg", np.zeros((240, 320), np.uint8))[1])
    assert parse_frame_pose(jpeg) is None
    assert parse_frame_pose(build_frame_pose(jpeg, None)) is None
    assert parse_frame_pose(b"") is None


def test_frame_pose_replaces_the_laptop_history(settings):
    f = TagFilter(settings.tags)
    late = StillPoses(9.0, 9.0, 0.0, rate=math.radians(90))   # stale laptop pose, turning
    s, reason = f.evaluate(make_det((0.6, 0.0, 0.5)), 1.0, 1.1, late, POSE)
    assert reason is None                            # yaw rate is the frame's (0.1 rad/s)
    assert (s.drone.x, s.drone.y, s.drone.heading) == pytest.approx(
        (POSE.x, POSE.y, POSE.heading))
    fwd = 0.6 + settings.tags.camera_forward_m - 0.02          # make_det assumes 0.02
    expect = (POSE.x + fwd * math.cos(POSE.heading), POSE.y + fwd * math.sin(POSE.heading))
    assert (s.x, s.y) == pytest.approx(expect, abs=1e-6)
    assert s.drone.nav_state == 2                    # still from the laptop telemetry


def test_camera_offset_moves_the_tag(settings):
    det = make_det((0.6, 0.0, 0.5))                  # the same image both times
    base = TagFilter(settings.tags).evaluate(det, 1, 1, StillPoses(0, 0, 0))[0]
    moved = dataclasses.replace(settings.tags,
                                camera_forward_m=settings.tags.camera_forward_m + 0.10,
                                camera_right_m=settings.tags.camera_right_m + 0.03)
    s = TagFilter(moved).evaluate(det, 1, 1, StillPoses(0, 0, 0))[0]
    assert (s.x - base.x, s.y - base.y) == pytest.approx((0.10, 0.03))


@pytest.mark.parametrize("gap_ms, used", [(12, True), (500, False)])
def test_pipeline_uses_the_frame_pose(settings, gap_ms, used):
    cam_port, cmd_port = free_port(), free_port()
    s = dataclasses.replace(settings, network=dataclasses.replace(
        settings.network, camera_port=cam_port, cmd_port=cmd_port))
    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", cmd_port))
    esp.settimeout(3.0)
    drone = (0.3, 0.2, math.radians(-90))
    tag = tag_ahead(drone, 0.1, 0.05, tag_id=7)
    laptop = StillPoses(drone[0] + 0.3, drone[1], drone[2])   # Wi-Fi-late: 30 cm off
    got = []
    pipe = TagPipeline(s, laptop, esp_ip=lambda: "127.0.0.1", on_sighting=got.append,
                       bind_host="127.0.0.1")
    pipe.start()
    try:
        esp.recv(64)
        jpeg = jpeg_bytes(render_view([tag], drone, jpeg_quality=None))
        jpeg = build_frame_pose(jpeg, FramePose(*drone, 0.0, gap_ms))
        for d in camera_datagrams(1, jpeg):
            esp.sendto(d, ("127.0.0.1", cam_port))
        deadline = time.monotonic() + 3.0
        while not got and time.monotonic() < deadline:
            time.sleep(0.02)
        assert got
        err = math.dist((got[0].x, got[0].y), (tag.north, tag.east))
        if used:
            assert err < TOL_M and pipe.stats().frame_pose == 1
        else:                                         # too far from a PX4 sample: laptop's
            assert err == pytest.approx(0.3, abs=TOL_M) and pipe.stats().frame_pose == 0
    finally:
        pipe.stop()
        esp.close()
