"""mission/tools/cam_bench.py: what the camera bench fit can tell."""

import dataclasses
import math
import socket
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import cam_bench as B  # noqa: E402
from apriltag_host import CAM_POS_M, FIRMWARE_INTRINSICS, Detector  # noqa: E402
from settings import DEFAULT_CONFIG_PATH, load_settings  # noqa: E402
from synth_camera import FloorTag, render_view  # noqa: E402

SPOTS = [(0.0, 0.0), (0.30, 0.0), (-0.25, 0.0), (0.0, 0.35), (0.0, -0.35), (0.25, 0.30),
         (-0.2, -0.3)]


def synthetic_stations(kr, kf, kz, yaw_deg, lens_fwd, lens_right, depth=0.8):
    """Exact readings from a camera with these flaws (camera looking down)."""
    p = math.radians(yaw_deg)
    A = np.diag([kr, kf]) @ np.array([[math.cos(p), -math.sin(p)], [math.sin(p), math.cos(p)]])
    out = []
    for fwd, right in SPOTS:
        mr, mf = A @ np.array([right - lens_right, fwd - lens_fwd])
        out.append(B.Station(fwd, right, depth, (float(mr), float(-mf), kz * depth), 20, 0.003, 4))
    return out


def test_fit_recovers_known_camera_flaws():
    st = synthetic_stations(kr=1.05, kf=0.97, kz=1.08, yaw_deg=3.0, lens_fwd=0.065, lens_right=-0.01)
    f = B.fit_stations(st, 0.12, FIRMWARE_INTRINSICS)
    assert (f.k_right, f.k_fwd, f.k_depth) == pytest.approx((1.05, 0.97, 1.08), abs=1e-6)
    assert f.yaw_deg == pytest.approx(3.0, abs=1e-4)
    assert (f.lens_fwd, f.lens_right) == pytest.approx((0.065, -0.01), abs=1e-6)
    assert f.rms_m == pytest.approx(0.0, abs=1e-9)
    # s = sqrt(kr kf): the tag size ratio; the rest is the lens model
    assert f.tag_size_m == pytest.approx(0.12 / math.sqrt(1.05 * 0.97), rel=1e-6)


def test_a_line_of_captures_is_not_enough():
    st = [s for s in synthetic_stations(1, 1, 1, 0, 0.02, 0) if s.right == 0.0]
    with pytest.raises(ValueError, match="in a line"):
        B.fit_stations(st, 0.12, FIRMWARE_INTRINSICS)


@pytest.mark.parametrize("cfg_size,cfg_fx", [(0.12, FIRMWARE_INTRINSICS.fx),   # all right
                                            (0.13, FIRMWARE_INTRINSICS.fx),   # wrong tag size
                                            (0.12, 175.0)])                   # wrong fx
def test_real_detector_on_synthetic_frames_finds_the_truth(cfg_size, cfg_fx):
    # The synthetic camera: tag 0.12 m, the course's lens model, lens 2 cm ahead.
    intr = dataclasses.replace(FIRMWARE_INTRINSICS, fx=cfg_fx)
    stations = []
    with Detector(intrinsics=intr, tag_size_m=cfg_size) as d:
        for fwd, right in SPOTS:
            gray = render_view([FloorTag(4, fwd, right)], (0.0, 0.0, 0.0), altitude=0.8)
            dets = [x for x in d.detect(gray) if x.id == 4 and x.t is not None]
            assert dets, f"tag at {fwd, right} not detected"
            stations.append(B.Station(fwd, right, 0.8, tuple(dets[0].t), 1, 0.0, 4))
    f = B.fit_stations(stations, cfg_size, intr)
    assert f.tag_size_m == pytest.approx(0.12, abs=0.002)
    assert f.fx == pytest.approx(FIRMWARE_INTRINSICS.fx, rel=0.03)
    assert f.fy == pytest.approx(FIRMWARE_INTRINSICS.fy, rel=0.03)
    assert f.lens_fwd == pytest.approx(CAM_POS_M[0], abs=0.01)
    assert abs(f.lens_right) < 0.01 and abs(f.yaw_deg) < 1.0 and f.rms_m < 0.01


def test_apply_writes_the_fit_and_keeps_a_backup(tmp_path):
    cfg = tmp_path / "config.yaml"
    text = DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    cfg.write_text(text, encoding="utf-8")
    fit = B.fit_stations(synthetic_stations(1.02, 1.02, 1.05, 0.5, 0.06, 0.005), 0.12,
                         FIRMWARE_INTRINSICS)
    before = load_settings(cfg)
    B.apply_fit(cfg, fit)
    s = load_settings(cfg)
    assert s.tags.tag_size_m == pytest.approx(round(before.tags.tag_size_m * fit.size_scale, 4))
    assert (s.tags.camera_forward_m, s.tags.camera_right_m) == (0.06, 0.005)
    assert (s.camera.fx, s.camera.fy) == (round(fit.fx, 2), round(fit.fy, 2))
    assert (tmp_path / "config.yaml.bak").read_text(encoding="utf-8") == text
    assert "# lens focal lengths" in cfg.read_text(encoding="utf-8")       # comments kept


def test_the_mission_detector_uses_the_calibrated_lens():
    from tag_pipeline import TagPipeline
    s = load_settings()
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    s = dataclasses.replace(s, camera=dataclasses.replace(s.camera, fx=170.0, fy=160.0),
                            network=dataclasses.replace(s.network, camera_port=port))
    seen = {}

    def factory(**kw):
        seen.update(kw)
        return Detector(**kw)

    p = TagPipeline(s, poses=None, esp_ip=lambda: None, on_sighting=lambda x: None,
                    detector_factory=factory, bind_host="127.0.0.1")
    p.start()
    p.stop()
    assert (seen["intrinsics"].fx, seen["intrinsics"].fy) == (170.0, 160.0)
    assert seen["intrinsics"].cx == FIRMWARE_INTRINSICS.cx


# ---------------------------------------------------------------------------
# Localisation tags printed at a different size (localisation.tag_size_m)
# ---------------------------------------------------------------------------

class _Still:
    def pose_at(self, t):
        from drone_link import Pose
        return Pose(t, 0.0, 0.0, 0.0, 0)

    def yaw_rate_at(self, t):
        return 0.0


def _read(tag_id, printed_m, loc_size_m, spot=(0.25, -0.2)):
    from tag_pipeline import TagFilter
    s = load_settings()
    s = dataclasses.replace(s, tags=dataclasses.replace(s.tags, camera_forward_m=CAM_POS_M[0],
                                                            max_down_m=1.5),
                            localisation=dataclasses.replace(s.localisation, tag_size_m=loc_size_m))
    with Detector(tag_size_m=s.tags.tag_size_m) as d:
        gray = render_view([FloorTag(tag_id, spot[0], spot[1], size=printed_m)], (0.0, 0.0, 0.0),
                           altitude=0.8)
        det = next(x for x in d.detect(gray) if x.id == tag_id)
    sighting, reason = TagFilter(s.tags, s.localisation).evaluate(det, 1.0, 1.0, _Still())
    assert reason is None
    return sighting


def test_a_smaller_printed_localisation_tag_lands_on_its_spot():
    # Loc tag 29 printed at 100 mm while the detector assumes the search tags' 120 mm.
    right = _read(29, printed_m=0.10, loc_size_m=0.10)
    assert math.dist((right.x, right.y), (0.25, -0.2)) < 0.02
    assert right.body[2] == pytest.approx(0.8, abs=0.03)
    wrong = _read(29, printed_m=0.10, loc_size_m=0.0)          # not told: reads 20 % too far
    assert wrong.body[2] == pytest.approx(0.8 * 0.12 / 0.10, abs=0.04)


def test_search_tags_keep_their_own_size():
    s = _read(4, printed_m=0.12, loc_size_m=0.10)               # tag 4 is not a loc tag
    assert math.dist((s.x, s.y), (0.25, -0.2)) < 0.02


def test_apply_corrects_the_kind_of_tag_that_was_captured(tmp_path):
    cfg = tmp_path / "config.yaml"
    cfg.write_text(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    before = load_settings(cfg)
    fit = B.fit_stations(synthetic_stations(1.05, 1.05, 1.05, 0.0, 0.08, 0.0), 0.12,
                         FIRMWARE_INTRINSICS)                   # reads 5 % too far
    B.apply_fit(cfg, fit, {"loc"})
    after = load_settings(cfg)
    assert after.tags.tag_size_m == before.tags.tag_size_m      # search tags untouched
    loc_before = before.localisation.tag_size_m or before.tags.tag_size_m
    assert after.localisation.tag_size_m == pytest.approx(loc_before / 1.05, abs=1e-4)


def test_negative_localisation_tag_size_is_rejected():
    from settings import ConfigError, settings_from_dict
    import yaml
    raw = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    raw["localisation"]["tag_size_m"] = -0.1
    with pytest.raises(ConfigError, match="localisation.tag_size_m"):
        settings_from_dict(raw)
