"""Localisation tags: drift fit, precision landing, offline re-run (mission/localisation.py)."""

import csv
import dataclasses
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

from localisation import (Fix, LocSighting, LocTags, Transform2D, correct, find_fixes,
                          fit_transform, solve_fixes)
from mission import Phase
import settings as settings_mod  # noqa: E402
import yaml  # noqa: E402
from settings import ConfigError, load_settings, settings_from_dict
from test_mission import Rig, configure, required

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import relocalise as tool  # noqa: E402

DRIFT = {"drift_scale": 0.05, "drift_rot_deg": 3.0, "drift_x_mps": 0.0004,
         "drift_y_mps": -0.0004}


@pytest.fixture
def lo():
    return load_settings().localisation


def fixes_from(transform: Transform2D, refs, lo_visit=1):
    """Fixes whose PX4 positions are the inverse of `transform` applied to refs."""
    inv = np.linalg.inv(transform.matrix)
    out = []
    for i, (tag_id, rx, ry) in enumerate(refs):
        px = inv @ (np.array([rx, ry]) - np.array([transform.tx, transform.ty]))
        out.append(Fix(tag_id, lo_visit, float(i), float(i) + 1, 20,
                       (float(px[0]), float(px[1])), (rx, ry), 0.01))
    return out


REFS = [(29, 0.0, 0.0), (27, 3.3, -0.3), (28, 3.3, 0.8)]


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("model,truth", [
    ("similarity", Transform2D("similarity", 0.95, math.radians(-4.0), 0.05, -0.03)),
    ("rigid", Transform2D("rigid", 1.0, math.radians(2.5), -0.02, 0.04)),
    ("translation", Transform2D("translation", 1.0, 0.0, 0.07, -0.01)),
])
def test_fit_recovers_a_known_transform(model, truth):
    src = np.array([[0.0, 0.0], [3.4, -0.2], [3.5, 0.9], [1.0, 0.5]])
    dst = truth.apply_many(src)
    got = fit_transform(src, dst, model)
    assert got.scale == pytest.approx(truth.scale, abs=1e-9)
    assert got.theta == pytest.approx(truth.theta, abs=1e-9)
    assert (got.tx, got.ty) == pytest.approx((truth.tx, truth.ty), abs=1e-9)


def test_rotation_sign_matches_ned_yaw():
    # +90 deg turns North (x) into East (y), like PX4 yaw.
    t = Transform2D("rigid", theta=math.pi / 2)
    assert t.apply(1.0, 0.0) == pytest.approx((0.0, 1.0))


def test_solve_with_all_three_tags_uses_the_configured_model(lo):
    truth = Transform2D("similarity", 0.96, math.radians(-3.0), -0.03, 0.02)
    fit = solve_fixes(fixes_from(truth, REFS), lo)
    assert fit.ok and fit.transform.model == "similarity"
    assert fit.transform.scale == pytest.approx(0.96, abs=1e-6)
    assert fit.rms_m == pytest.approx(0.0, abs=1e-9)


def test_only_the_home_tag_falls_back_to_a_shift(lo):
    fixes = [Fix(29, 1, 0, 1, 30, (0.02, -0.01), (0.0, 0.0), 0.01),
             Fix(29, 2, 90, 99, 30, (0.08, -0.05), (0.0, 0.0), 0.01)]
    fit = solve_fixes(fixes, lo)
    assert fit.ok and fit.transform.model == "translation"
    assert (fit.transform.tx, fit.transform.ty) == pytest.approx((-0.05, 0.03))
    assert "min_span_m" in fit.reason


def test_a_wild_fix_is_trimmed(lo):
    truth = Transform2D("similarity", 1.02, math.radians(1.0), 0.01, 0.0)
    fixes = fixes_from(truth, REFS + [(29, 0.0, 0.0)])
    bad = fixes[-1]
    fixes[-1] = dataclasses.replace(bad, px4=(bad.px4[0] + 0.5, bad.px4[1]), visit=2)
    fit = solve_fixes(fixes, lo)
    assert fit.ok and fit.used == (True, True, True, False)
    assert fit.transform.scale == pytest.approx(1.02, abs=1e-6)
    assert "trimmed tag 29 visit 2" in fit.reason


def test_an_impossible_fit_reports_no_correction(lo):
    truth = Transform2D("similarity", 1.0, math.radians(60.0), 0.0, 0.0)   # 60 deg turn
    fit = solve_fixes(fixes_from(truth, REFS), lo)
    assert not fit.ok
    assert "max_rotation_deg" in fit.reason and "translation: rms" in fit.reason
    tag = type("E", (), {"tag_id": 3, "x": 1.0, "y": 0.5, "n_inliers": 9})()
    assert correct(fit, [tag])[0].x == 1.0              # raw positions kept


def test_no_fixes(lo):
    fit = solve_fixes([], lo)
    assert not fit.ok and "seen often enough" in fit.reason


# ---------------------------------------------------------------------------
# Visits and the live store
# ---------------------------------------------------------------------------

def test_visits_split_on_time_gaps_and_need_enough_sightings(lo):
    rows = ([LocSighting(29, 1.0 + 0.1 * i, 0.01, 0.0) for i in range(30)]       # takeoff
            + [LocSighting(29, 50.0 + 0.1 * i, 0.3, 0.3) for i in range(3)]      # too few
            + [LocSighting(29, 120.0 + 0.1 * i, 0.05, -0.02) for i in range(10)]  # landing
            + [LocSighting(27, 40.0 + 0.1 * i, 3.4, -0.2) for i in range(8)]
            + [LocSighting(5, 41.0, 1.0, 1.0)] * 10)                             # not a loc tag
    fixes = find_fixes(rows, lo, 0.2, 0.3)
    assert [(f.tag_id, f.visit, f.n) for f in fixes] == [(29, 1, 30), (27, 1, 8), (29, 2, 10)]
    assert fixes[2].px4 == pytest.approx((0.05, -0.02))
    assert fixes[1].ref == (3.3, -0.3)


def test_recent_estimate_uses_only_fresh_sightings():
    loc = LocTags(load_settings())
    for i in range(20):
        loc.add(type("S", (), {"tag_id": 29, "t_capture": float(i), "x": 0.5, "y": 0.0})())
    for i in range(6):
        loc.add(type("S", (), {"tag_id": 29, "t_capture": 100.0 + 0.2 * i,
                               "x": 0.08, "y": -0.04})())
    assert loc.recent_estimate(29, since_t=99.0, window_s=2.0, min_n=5)[:2] == \
        pytest.approx((0.08, -0.04))
    assert loc.recent_estimate(29, since_t=99.0, window_s=2.0, min_n=7) is None
    assert loc.recent_estimate(28, since_t=0.0, window_s=2.0, min_n=1) is None
    assert loc.wants(29) and not loc.wants(5)


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

def _with_loc(**kw):
    s = load_settings()
    raw = {**s.raw, "localisation": {**s.raw["localisation"], **kw}}
    return settings_from_dict(raw)


def test_default_config_uses_the_back_of_tag16h5():
    lo = load_settings().localisation
    assert lo.enabled and lo.ids == {27, 28, 29} and lo.home_tag_id == 29


@pytest.mark.parametrize("kw,msg", [
    ({"tags": [{"id": 29, "x": 0, "y": 0}, {"id": 29, "x": 1, "y": 0}]}, "duplicate"),
    ({"tags": [{"id": 30, "x": 0, "y": 0}]}, "outside the tag16h5 ids"),
    ({"home_tag_id": 26}, "not in localisation.tags"),
    ({"model": "affine"}, "localisation.model"),
    ({"tags": [{"id": 29, "x": 0, "y": 0}, {"id": 12, "x": 1, "y": 0}]}, "sim.true_tags uses"),
    ({"bogus": 1}, "unknown key"),
])
def test_bad_localisation_config_is_rejected(kw, msg):
    with pytest.raises(ConfigError, match=msg):
        _with_loc(**kw)


# ---------------------------------------------------------------------------
# Whole mission in the simulator
# ---------------------------------------------------------------------------

def fly(tmp_path, flight=None, **kw):
    r = Rig(configure(tmp_path, flight={"stop_when_all_found": False, **(flight or {})}, **kw))
    assert r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    assert r.m.exit_code == 0
    return r


def real_position(r):
    d = r.drone
    x, y, _ = d.real_from_estimate(d.x, d.y, d.heading)
    return x, y


def test_drift_is_corrected_and_the_drone_lands_on_the_home_tag(tmp_path):
    # The sim's drift leaves the tag ~6 cm off: below the default 8 cm "close
    # enough", so ask for 3 cm to see the correction move.
    r = fly(tmp_path, sim=DRIFT, flight={"land_radius_m": 0.03})
    m, s = r.m, r.m.settings
    truth = {t.id: (t.x, t.y) for t in s.sim.true_tags}
    tags = m.tag_map.estimates(confirmed_only=True)
    assert {e.tag_id for e in tags} == set(truth)            # loc tags kept out of the map
    # The fit undoes the sim's drift: estimate = 1.05 R(3 deg) real + creep.
    fit = m.loc_fit
    assert fit.ok and fit.transform.model == "similarity"
    assert fit.transform.scale == pytest.approx(1 / 1.05, abs=0.01)
    assert math.degrees(fit.transform.theta) == pytest.approx(-3.0, abs=0.5)
    assert {f.tag_id for f in fit.fixes} == {27, 28, 29}
    raw = [math.dist((e.x, e.y), truth[e.tag_id]) for e in tags]
    fixed = [math.dist(fit.transform.apply(e.x, e.y), truth[e.tag_id]) for e in tags]
    assert max(raw) > 0.06 and max(fixed) < 0.035 and np.mean(fixed) < np.mean(raw) / 2
    # Precision landing: moved over the tag, so it really lands near (0, 0).
    assert r.events("HOME_TAG") and r.events("LAND_TARGET")
    assert re.search("cm from the home tag", r.events("LAND_CMD")[0]["details"])
    assert math.hypot(*real_position(r)) < 0.03
    # Logs and the brief's required events are unchanged.
    assert required(r.lines) == ["TAKEOFF_CMD"] + ["TAG_CONFIRMED"] * 6 + ["LAND_CMD"]
    assert r.events("LOC_FIT")
    with open(m.log.folder / "tags_corrected.csv", newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 6
    row = next(x for x in rows if x["tag_id"] == "5")
    assert math.dist((float(row["x"]), float(row["y"])), truth[5]) < 0.035
    with open(m.log.folder / "localisation.csv", newline="", encoding="utf-8") as fh:
        assert {x["tag_id"] for x in csv.DictReader(fh)} == {"27", "28", "29"}
    assert any("CORRECTED TAG REPORT" in ln for ln in r.lines)


def test_without_precision_landing_drift_lands_it_off_the_tag(tmp_path):
    r = fly(tmp_path, sim=DRIFT, localisation={"precision_landing": False})
    assert not r.events("LAND_TARGET")
    assert math.hypot(*real_position(r)) > 0.05


def test_no_drift_means_almost_no_correction(tmp_path):
    r = fly(tmp_path)
    t = r.m.loc_fit.transform
    assert r.m.loc_fit.ok and abs(t.scale - 1) < 0.01 and abs(math.degrees(t.theta)) < 0.5
    assert math.hypot(t.tx, t.ty) < 0.02
    assert math.hypot(*real_position(r)) <= r.m.settings.flight.land_radius_m + 0.01


def test_disabled_localisation_routes_nothing_and_writes_no_files(tmp_path):
    s = configure(tmp_path, flight={"stop_when_all_found": False},
                  localisation={"enabled": False})
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    assert r.m.loc_fit is None and r.m.loc.count() == 0
    assert not (r.m.log.folder / "tags_corrected.csv").exists()


def test_offline_tool_reproduces_the_flight_fit(tmp_path):
    r = fly(tmp_path, sim=DRIFT)
    run = r.m.log.folder
    fit, tags = tool.relocalise(r.m.settings, tool.read_detections(run / "detections.csv"))
    assert fit.ok
    assert fit.transform.scale == pytest.approx(r.m.loc_fit.transform.scale, abs=0.005)
    assert {c.tag_id for c in tags} == {t.id for t in r.m.settings.sim.true_tags}
    assert tool.main([str(run), "--write"]) == 0
    assert (run / "tags_corrected.csv").is_file()


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------

def test_detector_decodes_the_localisation_ids():
    from apriltag_host import EXPECTED_NCODES, Detector
    from synth_camera import FloorTag, render_view
    with Detector() as d:
        assert d.ncodes == EXPECTED_NCODES == 30
        for tag_id in (27, 28, 29):
            got = d.detect(render_view([FloorTag(tag_id, 0.0, 0.0)], (0.0, 0.0, 0.0)))
            assert [det.id for det in got] == [tag_id]


# ---------------------------------------------------------------------------
# Alignment to the home tag (localisation.align_to_home_tag)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("theta_deg", [-10.0, -3.5, 3.5, 10.0])
def test_tag_yaw_sign_through_the_real_detector(theta_deg):
    # A drone really turned +theta over a square tag, PX4 thinking it faces 0:
    # the arena x axis is then at -theta in PX4's frame.
    from apriltag_host import Detector
    from localisation import quarter_mean, tag_yaw
    from synth_camera import FloorTag, render_view
    with Detector() as d:
        im = render_view([FloorTag(29, 0.0, 0.0)], (0.0, 0.0, math.radians(theta_deg)), altitude=0.5)
        det = next(x for x in d.detect(im) if x.id == 29)
    a, _ = quarter_mean([tag_yaw(det.R, 90.0, 0.0)])
    assert math.degrees(a) == pytest.approx(-theta_deg, abs=0.5)


def test_quarter_mean_wraps_at_90_degrees():
    from localisation import quarter_mean
    m, spread = quarter_mean(np.radians([44.0, -44.0, 46.0, 134.0]))
    assert abs(abs(math.degrees(m)) - 45.0) < 2.0 and math.degrees(spread) < 2.0


def _arena_error(r):
    """Largest distance between where the uploaded sweep really is (sim truth)
    and the sweep as written in the arena frame."""
    from mission import plan_waypoints
    planned = plan_waypoints(r.m.settings)
    real = [r.drone.real_from_estimate(x, y, 0.0)[:2] for x, y in r.m.waypoints]
    return max(math.dist(a, b) for a, b in zip(real, planned))


def test_alignment_squares_the_sweep_to_the_home_tag(tmp_path):
    # PX4's frame turned 4 deg, as if the drone was put down crooked.
    r = fly(tmp_path, sim={"drift_rot_deg": 4.0})
    ev = r.events("ALIGNED")
    assert ev and float(re.search(r"turned ([+-][\d.]+) deg", ev[0]["details"]).group(1)) \
        == pytest.approx(4.0, abs=0.3)
    assert _arena_error(r) < 0.02                     # really flies the arena lanes
    assert {e.tag_id for e in r.m.tag_map.estimates(confirmed_only=True)} == \
        {t.id for t in r.m.settings.sim.true_tags}


def test_without_alignment_a_crooked_drone_sweeps_crooked(tmp_path):
    r = fly(tmp_path, sim={"drift_rot_deg": 4.0}, localisation={"align_to_home_tag": False})
    assert not r.events("ALIGNED")
    assert _arena_error(r) > 0.25                     # 3.5 m x sin 4 deg


def test_alignment_refuses_a_big_turn(tmp_path):
    r = fly(tmp_path, sim={"drift_rot_deg": 20.0})
    assert r.events("ALIGN_REFUSED") and not r.events("ALIGNED")


def test_offline_tool_plots_and_corrects_the_route(tmp_path):
    r = fly(tmp_path, sim=DRIFT)
    run = r.m.log.folder
    truth = " ".join(f"{t.id}:{t.x},{t.y}" for t in r.m.settings.sim.true_tags)
    assert tool.main([str(run), "--truth", truth, "--plot", "--write"]) == 0
    assert (run / "correction.png").stat().st_size > 20_000
    with open(run / "trajectory_corrected.csv", newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) > 100
    # The corrected route ends back on the home tag's true spot (0, 0), within a few cm
    # (the sim drone itself lands within ~3 cm; the route is corrected in time).
    assert math.hypot(float(rows[-1]["x"]), float(rows[-1]["y"])) < 0.08
    fit, tags = tool.relocalise(r.m.settings, tool.read_detections(run / "detections.csv"))
    errs = tool.tag_errors(tags, tool.parse_truth(truth))
    assert all(e["corr_err"] < e["raw_err"] for e in errs if e["raw_err"] and e["raw_err"] > 0.05)
    assert tool.parse_truth("2:1.3,-1.35;4:2.76,0.92") == {2: (1.3, -1.35), 4: (2.76, 0.92)}


def test_precision_landing_uses_the_tag_seen_on_the_way_home(tmp_path):
    # Big sideways creep: back "home" the tag is out of the camera's view, but
    # the return leg passed over it (flight 20261110_220437, 0.6 m to the left).
    r = fly(tmp_path, sim={"drift_y_mps": 0.005})
    ev = r.events("LAND_TARGET")
    assert ev, "no correction: the return-leg sightings were ignored"
    seen_cm = float(re.search(r"seen (\d+) cm", ev[0]["details"]).group(1))
    assert seen_cm > 40                                # the tag really was far away
    assert math.hypot(*real_position(r)) < 0.06        # but it landed on it


def test_mission_saves_the_correction_picture_after_landing(tmp_path):
    r = fly(tmp_path, sim=DRIFT, localisation={"truth_tags": tuple(r_ for r_ in
                                               load_settings().sim.true_tags)})
    folder = r.m.log.folder
    assert (folder / "correction.png").stat().st_size > 20_000
    with open(folder / "trajectory_corrected.csv", newline="", encoding="utf-8") as fh:
        assert len(list(csv.DictReader(fh))) > 100
    names = [e["event"] for e in r.events()]
    assert names.index("CORRECTION_SAVED") > names.index("LAND_CMD")   # never before landing
    assert any("TAG ERRORS vs TRUE POSITIONS" in ln for ln in r.lines)


# ---------------------------------------------------------------------------
# Drift that changes during the flight (localisation.drift_model)
# ---------------------------------------------------------------------------

from localisation import DriftCorrection, correct_with, drift_correction  # noqa: E402

GRID = [(29, 0.0, 0.0), (23, 1.38, 0.92), (22, 2.30, -0.46), (24, 3.22, -1.38),
        (27, 4.60, -1.38), (28, 4.60, 0.92), (25, 3.68, 0.0)]


def creeping_fixes(rate=(0.004, -0.003), turn_deg=0.0):
    """Visits over 90 s while PX4's position creeps (and optionally its frame is turned)."""
    p = math.radians(turn_deg)
    fx = []
    for i, (tid, x, y) in enumerate(GRID + [(29, 0.0, 0.0)]):
        t = 5.0 + 12.0 * i
        rx, ry = math.cos(p) * x - math.sin(p) * y, math.sin(p) * x + math.cos(p) * y
        fx.append(Fix(tid, 1, t, t + 2.0, 20, (rx + rate[0] * t, ry + rate[1] * t), (x, y), 0.01))
    return fx


def _lo(**kw):
    return dataclasses.replace(load_settings().localisation, **kw)


def test_auto_picks_time_for_a_creeping_drift():
    fixes = creeping_fixes()
    corr = drift_correction(solve_fixes(fixes, _lo()), _lo())
    assert corr.model == "time"
    loo = dict(corr.loo_cm)
    assert loo["time"] < loo["global"] / 3          # the end visits can only be extrapolated
    # a tag seen at t=50 s at its drifted spot comes out at its true spot
    true = (2.0, 0.5)
    seen = (true[0] + 0.004 * 51, true[1] - 0.003 * 51)
    assert corr.apply(51.0, *seen) == pytest.approx(true, abs=0.005)


def test_auto_picks_a_transform_for_a_turned_frame():
    fixes = creeping_fixes(rate=(0.0, 0.0), turn_deg=6.0)
    corr = drift_correction(solve_fixes(fixes, _lo()), _lo())
    assert corr.model in ("global", "hybrid") and dict(corr.loo_cm)["global"] < 1.0
    assert corr.apply(30.0, 4.0 * math.cos(math.radians(6)), 4.0 * math.sin(math.radians(6))) \
        == pytest.approx((4.0, 0.0), abs=0.01)


def test_drift_model_can_be_forced():
    fixes = creeping_fixes()
    corr = drift_correction(solve_fixes(fixes, _lo(drift_model="global")), _lo(drift_model="global"))
    assert corr.model == "global" and corr.loo_cm == ()


def test_each_sighting_is_corrected_at_its_own_time():
    corr = DriftCorrection("time", knot_t=(0.0, 100.0), knot_dx=(0.0, -0.4), knot_dy=(0.0, 0.0))
    tag = type("E", (), {"tag_id": 4, "x": 1.2, "y": 0.0, "n_inliers": 2, "first_seen_t": 0.0})()
    # seen twice: at t=0 (no drift) and t=100 (+0.4 m drift): both are really at x = 1.0
    samples = {4: [(0.0, 1.0, 0.0)] * 3 + [(100.0, 1.4, 0.0)] * 3}
    c = correct_with(corr, [tag], samples)[0]
    assert (c.x, c.y) == pytest.approx((1.0, 0.0), abs=1e-6)


def test_bad_drift_model_is_rejected():
    raw = yaml.safe_load(settings_mod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    raw["localisation"]["drift_model"] = "magic"
    with pytest.raises(ConfigError, match="drift_model"):
        settings_from_dict(raw)


def test_mission_reports_the_drift_correction(tmp_path):
    r = fly(tmp_path, sim=DRIFT)
    assert r.m.loc_drift is not None and r.m.loc_drift.ok
    assert r.events("LOC_DRIFT") and any("drift correction:" in ln for ln in r.lines)
    truth = {t.id: (t.x, t.y) for t in r.m.settings.sim.true_tags}
    errs = [math.dist((c.x, c.y), truth[c.tag_id]) for c in r.m.corrected_tags]
    assert max(errs) < 0.04
