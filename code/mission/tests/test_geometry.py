"""Body-frame -> PX4 local NED localization (mission/geometry.py)."""

import math
import random

import pytest

from apriltag_host import camera_to_body
from geometry import body_to_ned, ned_to_body


def approx_xy(x, y, abs=1e-9):
    return (pytest.approx(x, abs=abs), pytest.approx(y, abs=abs))


def test_hand_checked_case_facing_east():
    # Drone at (1, 0) facing East sees a tag 0.5 m straight ahead.
    assert body_to_ned(0.5, 0.0, 1.0, 0.0, math.pi / 2) == approx_xy(1.0, 0.5)


# Drone at (2, 3); tag 1 m ahead and 0.5 m to the right.
@pytest.mark.parametrize("heading_deg, expected", [
    (0,    (3.0, 3.5)),     # facing North: ahead = +x, right = +y (East)
    (90,   (1.5, 4.0)),     # facing East:  ahead = +y, right = -x (South)
    (180,  (1.0, 2.5)),     # facing South: ahead = -x, right = -y (West)
    (-90,  (2.5, 2.0)),     # facing West:  ahead = -y, right = +x (North)
    (270,  (2.5, 2.0)),     # same as -90 (firmware reports [0, 2pi))
])
def test_four_headings(heading_deg, expected):
    got = body_to_ned(1.0, 0.5, 2.0, 3.0, math.radians(heading_deg))
    assert got == approx_xy(*expected)


def test_forty_five_degrees():
    # Facing North-East, a tag 1 m ahead is at (cos45, sin45) from the drone.
    h = math.radians(45)
    assert body_to_ned(1.0, 0.0, 0.0, 0.0, h) == approx_xy(math.sqrt(0.5), math.sqrt(0.5))


def test_round_trip_and_distance_preserved():
    rng = random.Random(1)
    for _ in range(200):
        f, r = rng.uniform(-3, 3), rng.uniform(-3, 3)
        x, y, h = rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-10, 10)
        n, e = body_to_ned(f, r, x, y, h)
        assert math.hypot(n - x, e - y) == pytest.approx(math.hypot(f, r))
        assert ned_to_body(n, e, x, y, h) == approx_xy(f, r)


def test_from_camera_frame_through_to_ned():
    # apriltag_host's documented example: a tag 1 m ahead of and 0.5 m below
    # the lens is seen at t = (0, -0.354, 1.061) and is (1.02, 0, 0.5) in the
    # body frame.  With the drone at (0.3, 0.2) facing West it is 1.02 m West.
    f, r, d = camera_to_body((0.0, -0.5 * math.sqrt(0.5) * 1.0, 1.5 * math.sqrt(0.5)))
    assert (f, r, d) == (pytest.approx(1.02, abs=1e-3), pytest.approx(0.0, abs=1e-9),
                         pytest.approx(0.5, abs=1e-3))
    assert body_to_ned(f, r, 0.3, 0.2, -math.pi / 2) == approx_xy(0.3, 0.2 - 1.02, abs=1e-3)
