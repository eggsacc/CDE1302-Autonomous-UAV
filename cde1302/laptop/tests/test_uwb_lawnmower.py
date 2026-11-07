"""simple_uwb_lawnmower.py: the sweep corners and the reach check."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simple_uwb_lawnmower as lm  # noqa: E402


def test_left_first_when_facing_minus_y():
    # Facing -y, +x is the drone's left: left, forward, right, back, right, forward.
    assert lm.lawnmower(-0.5, 0.5, -3.0, 0.0, 0.5) == [
        (0.5, 0.0), (0.5, -3.0), (0.0, -3.0), (0.0, 0.0), (-0.5, 0.0), (-0.5, -3.0)]


def test_first_lane_xmin_and_near_y_edge():
    assert lm.lawnmower(-0.5, 1.0, 0.0, 3.0, 0.5, "xmin") == [
        (-0.5, 0.0), (-0.5, 3.0), (0.0, 3.0), (0.0, 0.0),
        (0.5, 0.0), (0.5, 3.0), (1.0, 3.0), (1.0, 0.0)]


def test_last_lane_is_on_the_far_edge():
    xs = [x for x, _ in lm.lawnmower(0.0, 1.2, 0.0, 1.0, 0.5)]
    assert sorted(set(xs)) == pytest.approx([0.0, 0.2, 0.7, 1.2])     # starts on xmax
    xs = [x for x, _ in lm.lawnmower(0.0, 1.2, 0.0, 1.0, 0.5, "xmin")]
    assert sorted(set(xs)) == pytest.approx([0.0, 0.5, 1.0, 1.2])


def test_bad_bounds():
    with pytest.raises(ValueError):
        lm.lawnmower(1.0, 0.0, 0.0, 1.0, 0.5)
    with pytest.raises(ValueError):
        lm.lawnmower(0.0, 1.0, 0.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        lm.lawnmower(0.0, 1.0, 0.0, 1.0, 0.5, "middle")
