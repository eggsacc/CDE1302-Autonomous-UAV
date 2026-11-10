"""tags.home_tag_id: measure the optical-flow drift on a tag at home, land on it,
and correct the search tags.  The sim drifts PX4's position away from the truth
in proportion to the distance flown, like real optical flow."""

import math

import pytest

from home_tag import DistanceFlown, drift_at
from mission import Phase
from settings import TrueTag, load_settings
from test_mission import Rig, configure

HOME_ID = 29
PER_M = (0.008, -0.006)            # drift per metre flown


def rig(tmp_path, home_tag=True, **tags):
    s = load_settings()
    true_tags = tuple(s.sim.true_tags) + ((TrueTag(HOME_ID, 0.0, 0.0),) if home_tag else ())
    s = configure(tmp_path, flight={"stop_when_all_found": False},
                  tags={"home_tag_id": HOME_ID, **tags}, sim={"true_tags": true_tags})
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    return r


def fly(r, per_m=PER_M):
    """Fly the whole mission; PX4's error grows with the distance flown."""
    r.m.command("takeoff")
    while r.m.phase != Phase.DONE:
        d = r.m.flown.at(r.drone.now()) if r.m.home_ref is not None else 0.0
        r.drone.set_estimate_error(per_m[0] * d, per_m[1] * d)
        if not r.step():
            break
    r.m.shutdown()


def true_xy(r):
    ex, ey = r.drone._est_err
    return r.drone.x - ex, r.drone.y - ey


def test_lands_on_the_real_home_and_corrects_the_tags(tmp_path):
    r = rig(tmp_path)
    fly(r)
    m = r.m
    assert m.exit_code == 0 and r.events("HOME_TAG_REF") and r.events("HOME_TAG_DRIFT")
    flown = m.drift[2] - m.home_ref[2]
    expected = (PER_M[0] * flown, PER_M[1] * flown)
    assert math.hypot(*expected) > 0.15                       # a real drift was simulated
    assert m.drift[0] == pytest.approx(expected[0], abs=0.04)
    assert m.drift[1] == pytest.approx(expected[1], abs=0.04)
    assert r.events("HOME_TAG_MOVE")
    # Landed on the real takeoff spot, not where PX4 thought home was.
    assert math.hypot(*true_xy(r)) < 0.06
    # Search tags: corrected positions are much closer to the truth than the raw ones.
    truth = {t.id: (t.x, t.y) for t in m.settings.sim.true_tags if t.id != HOME_ID}
    raw_err, cor_err = [], []
    for e, cx, cy in m.corrected_tags():
        raw_err.append(math.dist((e.x, e.y), truth[e.tag_id]))
        cor_err.append(math.dist((cx, cy), truth[e.tag_id]))
    assert len(cor_err) == len(truth)
    assert max(cor_err) < 0.05 and max(cor_err) < max(raw_err)
    assert HOME_ID not in [e.tag_id for e in m.tag_map.estimates()]   # not a search tag
    rows = (m.log.folder / "tags_final.csv").read_text().splitlines()
    assert rows[0].startswith("tag_id,x,y,x_raw,y_raw")


def test_no_home_tag_seen_before_the_sweep_flies_as_before(tmp_path):
    r = rig(tmp_path, home_tag=False)
    fly(r, per_m=(0.0, 0.0))
    assert r.m.exit_code == 0 and r.events("HOME_TAG_MISSING")
    assert not r.events("HOME_TAG_DRIFT") and r.m.drift is None
    assert "cm from home" in r.events("LAND_CMD")[0]["details"]


def test_drift_too_big_for_the_camera_lands_anyway(tmp_path):
    r = rig(tmp_path, home_tag_wait_s=3.0)
    fly(r, per_m=(0.05, 0.05))                                # ~1 m+: tag out of view at home
    assert r.m.exit_code == 0 and r.events("HOME_TAG_NOT_SEEN")
    assert r.drone.alt == 0.0


def test_drift_model():
    f = DistanceFlown()
    for i in range(11):
        f.add(float(i), 0.1 * i, 0.0)                        # 1 m in 10 s
    assert f.at(5.0) == pytest.approx(0.5)
    assert drift_at(0.5, 0.0, 1.0, (0.2, -0.1)) == pytest.approx((0.1, -0.05))
    assert drift_at(2.0, 0.0, 1.0, (0.2, -0.1)) == pytest.approx((0.2, -0.1))
