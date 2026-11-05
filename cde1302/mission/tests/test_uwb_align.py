"""UWB -> PX4 alignment estimator (mission/uwb_align.py), on synthetic flights.

PX4 paths come from the real planner + trajectory builder; the UWB positions
are those paths seen from a hidden "true" anchor frame (rotation, optional
mirror, offset) plus noise.  The estimator must recover the hidden frame.
"""

import dataclasses
import math

import numpy as np
import pytest

import planner
from settings import ConfigError, load_settings, settings_from_dict
from trajectory import build_trajectory
from uwb_align import AlignmentEstimator, AlignSample, Alignment, fit, path_shape


def sweep_path(n_lanes=3):
    """PX4 positions along the start of a lawnmower sweep, 50 Hz at 0.1 m/s."""
    wps = planner.lawnmower(0.0, 1.5, 0.0, 1.0, 0.6, 0.2)[: 2 * n_lanes]
    return build_trajectory((0.0, 0.0), wps, 0.1, 0.02).points


def straight_path(length=1.5):
    return build_trajectory((0.0, 0.0), [(length, 0.0)], 0.1, 0.02).points


def samples_for(px4_path, truth: Alignment, noise=0.03, seed=0, eop=0.05, outliers=0.0):
    rng = np.random.default_rng(seed)
    out = []
    for i, (px, py) in enumerate(px4_path):
        ux, uy = truth.to_uwb(px, py)
        ux, uy = ux + rng.normal(0, noise), uy + rng.normal(0, noise)
        if rng.random() < outliers:
            ux, uy = ux + rng.uniform(-1, 1), uy + rng.uniform(-1, 1)
        out.append(AlignSample(i * 0.02, ux, uy, px, py, True, eop))
    return out


def estimator(**kw):
    return AlignmentEstimator(**{"mirror": "auto", **kw})


def solve(samples, **kw):
    est = estimator(**kw)
    for s in samples:
        est.add(s)
    return est.solve()


def angle_err_deg(a, b):
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


# ---------------------------------------------------------------------------

def test_transform_round_trip():
    a = Alignment(theta=0.7, mirror=True, tx=1.2, ty=-3.4)
    for ux, uy in [(0.0, 0.0), (2.5, -1.0), (-4.0, 7.5)]:
        assert a.to_uwb(*a.to_px4(ux, uy)) == (pytest.approx(ux), pytest.approx(uy))


def test_exact_data_gives_exact_fit():
    truth = Alignment(theta=math.radians(30), mirror=False, tx=0.5, ty=-0.2)
    u = np.array([[0, 0], [1, 0], [1, 1], [0, 2.0]])
    p = np.array([truth.to_px4(*x) for x in u])
    al, res = fit(u, p, np.ones(4), mirror=False)
    assert al.theta == pytest.approx(truth.theta) and res.max() < 1e-9
    assert (al.tx, al.ty) == (pytest.approx(0.5), pytest.approx(-0.2))


@pytest.mark.parametrize("theta_deg", [0, 37, 180, -120])
@pytest.mark.parametrize("mirror", [False, True])
def test_recovers_rotation_mirror_and_offset_from_a_sweep(theta_deg, mirror):
    truth = Alignment(math.radians(theta_deg), mirror, tx=-2.3, ty=1.7)
    path = sweep_path()
    r = solve(samples_for(path, truth, noise=0.03, seed=theta_deg % 360 + mirror))
    assert r.ok, r.reason
    assert r.alignment.mirror == mirror
    assert angle_err_deg(r.alignment.theta, truth.theta) < 1.0
    assert r.rms_m < 0.05
    # Mapped UWB positions land on the PX4 path within a few cm.
    for (px, py) in path[::50]:
        ux, uy = truth.to_uwb(px, py)
        assert math.dist(r.alignment.to_px4(ux, uy), (px, py)) < 0.03
    assert r.other_rms_m > 2 * r.rms_m                    # the mirror choice was clear


def test_straight_line_cannot_decide_the_mirror():
    truth = Alignment(math.radians(50), True, 1.0, 1.0)
    r = solve(samples_for(straight_path(), truth))
    assert not r.ok and "too straight" in r.reason
    assert r.spread_ratio < 0.05


@pytest.mark.parametrize("mirror", [False, True])
def test_straight_line_is_enough_once_the_mirror_is_configured(mirror):
    truth = Alignment(math.radians(50), mirror, 1.0, 1.0)
    r = solve(samples_for(straight_path(), truth), mirror="yes" if mirror else "no")
    assert r.ok, r.reason
    assert angle_err_deg(r.alignment.theta, truth.theta) < 1.0


def test_wrong_configured_mirror_fits_badly():
    truth = Alignment(math.radians(10), True, 0.0, 0.0)
    r = solve(samples_for(sweep_path(), truth), mirror="no")
    assert not r.ok and "fit too poor" in r.reason


def test_not_enough_samples_or_distance():
    truth = Alignment(0.0, False, 0.0, 0.0)
    r = solve(samples_for(sweep_path()[:20], truth))
    assert not r.ok and "20/50 usable samples" in r.reason
    r = solve(samples_for(straight_path(0.5), truth))       # 3 cm noise must not inflate it
    assert not r.ok and "path only 0.5" in r.reason and r.span_m < 0.55


def test_outliers_are_trimmed():
    truth = Alignment(math.radians(-75), False, 3.0, 2.0)
    r = solve(samples_for(sweep_path(), truth, outliers=0.05, seed=3))
    assert r.ok, r.reason
    assert angle_err_deg(r.alignment.theta, truth.theta) < 1.5
    assert r.n_used < r.n_valid


def test_samples_without_fix_or_with_poor_precision_are_ignored():
    truth = Alignment(math.radians(20), False, 0.0, 0.0)
    good = samples_for(sweep_path(), truth)
    junk = [dataclasses.replace(s, ux=s.ux + 5, has_fix=False) for s in good[:200]]
    junk += [dataclasses.replace(s, uy=s.uy - 5, eop=2.55) for s in good[200:400]]
    r = solve(good + junk)
    assert r.ok and r.n_valid == len(good)


def test_noisy_data_is_rejected():
    truth = Alignment(0.3, False, 0.0, 0.0)
    r = solve(samples_for(sweep_path(), truth, noise=0.3), mirror="no")
    assert not r.ok and "fit too poor" in r.reason


def test_small_px4_drift_is_tolerated():
    # PX4 position drifts 2 % of distance flown (flow odometry); still aligns.
    truth = Alignment(math.radians(65), False, -1.0, 0.5)
    path = sweep_path()
    flown = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    drifted = path + np.column_stack([0.02 * flown, 0.01 * flown])
    samples = samples_for(path, truth)                           # UWB sees the truth...
    samples = [dataclasses.replace(s, px=d[0], py=d[1]) for s, d in zip(samples, drifted)]
    r = solve(samples)                                            # ...PX4 reports drift
    assert r.ok, r.reason
    assert angle_err_deg(r.alignment.theta, truth.theta) < 3.0


def test_path_shape():
    span, spread = path_shape(straight_path(1.5))
    assert span == pytest.approx(1.5, rel=0.03) and spread < 1e-6   # 1-99 % extent
    span, spread = path_shape(sweep_path())
    assert spread > 0.05


def test_settings_section():
    s = load_settings()
    est = AlignmentEstimator.from_settings(s.uwb)
    assert est.mirror == s.uwb.mirror and est.min_samples == s.uwb.align_min_samples
    raw = dict(s.raw, uwb=dict(s.raw["uwb"], mirror="maybe"))
    with pytest.raises(ConfigError, match="uwb.mirror"):
        settings_from_dict(raw)
