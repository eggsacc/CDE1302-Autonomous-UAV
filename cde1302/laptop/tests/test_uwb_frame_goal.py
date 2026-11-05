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
    for nose in (0.0, 90.0, 180.0, -90.0, 37.0):
        al = fg.frame_alignment(nose)
        a = to_px4(al, 0.0, 0.0)
        b = to_px4(al, fg.math.cos(fg.math.radians(nose)), fg.math.sin(fg.math.radians(nose)))
        assert (b[0] - a[0], b[1] - a[1]) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_snap_error():
    al = fg.frame_alignment(0.0)
    assert fg.snap_error(al, (2.0, 1.5, 2.0, -1.5)) == pytest.approx(0.0)
    assert fg.snap_error(al, (2.0, 1.5, 0.0, 0.0)) == pytest.approx(2.5)   # PX4 not snapped yet
