"""Sighting grouping, confirmation and robust estimates (mission/tag_map.py)."""

import math
import threading

import numpy as np
import pytest

from drone_link import Pose
from sighting import Sighting
from tag_map import TagMap, robust_estimate
from settings import load_settings

TRUE = (0.8, 0.3)


def sighting(tag_id, x, y, t=0.0) -> Sighting:
    return Sighting(tag_id=tag_id, t_capture=t - 0.13, t_detect=t, x=x, y=y,
                    body=(0.6, 0.0, 0.5), drone=Pose(t, 0.2, 0.3, 0.0, 2))


def noisy(rng, n, centre=TRUE, sigma=0.03):
    return [tuple(p) for p in rng.normal(centre, sigma, size=(n, 2))]


@pytest.fixture
def rng():
    return np.random.default_rng(42)


@pytest.fixture
def confirmed():
    return []


@pytest.fixture
def tm(confirmed):
    return TagMap(min_sightings=5, confirm_radius_m=0.20, outlier_radius_m=0.30,
                  on_confirm=confirmed.append)


def test_from_settings_uses_config():
    t = TagMap.from_settings(load_settings().tags)
    assert (t.min_sightings, t.confirm_radius_m, t.outlier_radius_m) == (5, 0.20, 0.30)


def test_noisy_sightings_converge(tm, rng):
    for i, (x, y) in enumerate(noisy(rng, 60)):
        est = tm.add(sighting(3, x, y, t=float(i)))
    assert est.confirmed
    assert math.dist((est.x, est.y), TRUE) < 0.015      # 3 cm noise -> ~5 mm error
    assert est.n_sightings == est.n_inliers == 60
    assert 0.0 < est.spread_m < 0.06


def test_one_wild_outlier_is_ignored(tm, rng):
    pts = noisy(rng, 10)
    for i, (x, y) in enumerate(pts[:5]):
        tm.add(sighting(3, x, y, t=float(i)))
    tm.add(sighting(3, 5.0, -4.0, t=5.0))              # wild
    for i, (x, y) in enumerate(pts[5:]):
        est = tm.add(sighting(3, x, y, t=6.0 + i))
    clean = np.median(np.array(pts), axis=0)
    assert (est.x, est.y) == (pytest.approx(clean[0]), pytest.approx(clean[1]))
    assert est.n_sightings == 11 and est.n_inliers == 10


def test_single_spurious_sighting_never_confirmed(tm, rng, confirmed):
    tm.add(sighting(9, 1.0, 1.0))
    for i, (x, y) in enumerate(noisy(rng, 50)):     # lots of traffic on other IDs
        tm.add(sighting(3, x, y, t=float(i)))
    assert tm.get(9).confirmed is False
    assert [e.tag_id for e in confirmed] == [3]
    assert [e.tag_id for e in tm.estimates(confirmed_only=True)] == [3]
    assert [e.tag_id for e in tm.estimates()] == [3, 9]


def test_scattered_false_positives_never_confirm(tm, confirmed):
    # Many wrong-ID detections, all far apart: no cluster, no confirmation.
    for i in range(20):
        tm.add(sighting(11, 0.5 * i, -0.4 * i, t=float(i)))
    assert not tm.get(11).confirmed and confirmed == []


def test_confirms_exactly_once_on_the_min_sightings_th(tm, rng, confirmed):
    pts = noisy(rng, 12)
    for i, (x, y) in enumerate(pts):
        est = tm.add(sighting(3, x, y, t=10.0 + i))
        assert est.confirmed == (i >= 4)
    assert len(confirmed) == 1
    first = confirmed[0]
    assert first.confirmed_t == 14.0 and first.support == 5
    assert first.first_seen_t == 10.0
    assert est.confirmed_t == 14.0                  # stays at the first time
    assert (est.confirmed_x, est.confirmed_y) == (first.x, first.y)


def test_early_false_cluster_does_not_drag_the_estimate(tm, rng, confirmed):
    # 3 wrong sightings arrive first, elsewhere; then the real tag.
    for i, (x, y) in enumerate(noisy(rng, 3, centre=(2.0, 2.0))):
        tm.add(sighting(5, x, y, t=float(i)))
    for i, (x, y) in enumerate(noisy(rng, 6)):
        est = tm.add(sighting(5, x, y, t=3.0 + i))
    assert len(confirmed) == 1
    assert math.dist((confirmed[0].x, confirmed[0].y), TRUE) < 0.05
    assert math.dist((est.x, est.y), TRUE) < 0.05
    assert est.n_inliers == 6


def test_ids_are_independent(tm, rng):
    for i, ((x1, y1), (x2, y2)) in enumerate(zip(noisy(rng, 8), noisy(rng, 8, (1.4, -0.2)))):
        tm.add(sighting(3, x1, y1, t=float(i)))
        tm.add(sighting(7, x2, y2, t=float(i)))
    e3, e7 = tm.get(3), tm.get(7)
    assert math.dist((e3.x, e3.y), TRUE) < 0.05
    assert math.dist((e7.x, e7.y), (1.4, -0.2)) < 0.05


def test_final_rows(tm, rng):
    for i, (x, y) in enumerate(noisy(rng, 6)):
        tm.add(sighting(3, x, y, t=20.0 + i))
    tm.add(sighting(4, 0.0, 0.0, t=30.0))           # unconfirmed, left out
    rows = tm.final_rows()
    assert len(rows) == 1
    assert rows[0]["tag_id"] == 3 and rows[0]["n_sightings"] == 6
    assert rows[0]["first_seen_t"] == 20.0 and rows[0]["confirmed_t"] == 24.0


def test_history_is_capped():
    t = TagMap(min_sightings=2, max_sightings_per_tag=10)
    for i in range(25):
        est = t.add(sighting(3, 1.0, 1.0, t=float(i)))
    assert est.n_sightings == 10


def test_robust_estimate_single_point():
    est, inliers, support = robust_estimate(np.array([[1.0, 2.0]]), 0.2, 0.3)
    assert tuple(est) == (1.0, 2.0) and inliers.tolist() == [True] and support == 1


def test_bad_radii_rejected():
    with pytest.raises(ValueError):
        TagMap(confirm_radius_m=0.5, outlier_radius_m=0.2)


def test_callback_failure_does_not_break_adding(rng):
    def boom(_):
        raise RuntimeError("display broke")
    t = TagMap(min_sightings=2, on_confirm=boom)
    for i, (x, y) in enumerate(noisy(rng, 3)):
        est = t.add(sighting(3, x, y, t=float(i)))
    assert est.confirmed


def test_concurrent_adds(tm, rng):
    pts = noisy(rng, 400)

    def worker(k):
        for i in range(k, 400, 4):
            tm.add(sighting(3, *pts[i], t=float(i)))

    threads = [threading.Thread(target=worker, args=(k,)) for k in range(4)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    assert tm.get(3).n_sightings == 400


def test_sighting_log_fields_match_detection_columns():
    from mission_log import DETECTION_FIELDS
    s = Sighting(3, 0.87, 1.0, 0.8, 0.3, (0.6, 0.1, 0.5), Pose(1.0, 0.2, 0.2, 0.1, 2),
                 cam_t=(0.1, -0.2, 0.7), cam_R=tuple(range(9)), pose_err=0.01,
                 decision_margin=120.0)
    fields = s.log_fields()
    assert set(fields) <= set(DETECTION_FIELDS)
    assert fields["R12"] == 5 and fields["cam_tz"] == 0.7 and fields["tag_x"] == 0.8
