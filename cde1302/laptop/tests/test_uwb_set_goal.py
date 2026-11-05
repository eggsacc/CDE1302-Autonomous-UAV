"""simple_uwb_set_goal.py: coordinate conversion, the goal path and argument handling."""

import json
import math
import sys
from argparse import Namespace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simple_uwb_set_goal as goal  # noqa: E402

AL = {"theta_deg": -84.0, "mirror": True, "tx": 4.1, "ty": 3.8}


def test_conversion_round_trips():
    for u in [(0, 0), (1.5, -2.0), (6.2, 7.1)]:
        assert goal.to_uwb(AL, *goal.to_px4(AL, *u)) == pytest.approx(u, abs=1e-9)


def test_theta_zero_with_mirror_flips_only_y():
    # Nose along UWB x at power-up, takeoff spot at UWB (2, 3): t = (-2, +3).
    al = {"theta_deg": 0.0, "mirror": True, "tx": -2.0, "ty": 3.0}
    assert goal.to_px4(al, 2.0, 3.0) == pytest.approx((0.0, 0.0))     # takeoff = PX4 origin
    assert goal.to_px4(al, 3.0, 3.0) == pytest.approx((1.0, 0.0))     # +UWB x = forward
    assert goal.to_px4(al, 2.0, 4.0) == pytest.approx((0.0, -1.0))    # +UWB y = to the left


def test_matches_the_calibration_fit():
    import simple_uwb_calibration as cal
    p = cal.square_path(1.0, 0.15, "right")[:, :2]
    u = np.array([goal.to_uwb(AL, *q) for q in p])
    r = cal.calibrate([(a, b, c, d) for (a, b), (c, d) in zip(u, p)], "yes")
    assert r["theta_deg"] == pytest.approx(AL["theta_deg"], abs=0.01)
    assert (r["tx"], r["ty"]) == (pytest.approx(AL["tx"]), pytest.approx(AL["ty"]))


def test_line_path_ends_on_the_goal():
    pts = goal.line_path((0.5, -0.2), (2.5, 1.3), 0.15)
    assert np.allclose(pts[0], 0) and pts[-1][:2] == pytest.approx((2.0, 1.5))
    assert np.linalg.norm(np.diff(pts, axis=0), axis=1).max() <= 0.15 * goal.DT_S + 1e-9


def args(**kw):
    base = dict(theta=None, mirror=None, tx=None, ty=None, calibration=None)
    return Namespace(**{**base, **kw})


def test_alignment_sources(tmp_path):
    assert goal.alignment_from_args(args()) is None                     # auto-calibrate
    a = goal.alignment_from_args(args(theta=10.0, mirror="no", tx=1.0, ty=2.0))
    assert a == {"theta_deg": 10.0, "mirror": False, "tx": 1.0, "ty": 2.0}
    f = tmp_path / "cal.json"
    f.write_text(json.dumps({**AL, "rms_m": 0.03}))
    assert goal.alignment_from_args(args(calibration=str(f))) == AL
    with pytest.raises(SystemExit):
        goal.alignment_from_args(args(theta=1.0))                       # incomplete
