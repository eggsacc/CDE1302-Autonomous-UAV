"""Live map (mission/live_map.py), drawn headless with the Agg backend."""

import math

import matplotlib

matplotlib.use("Agg")                     # no window in tests

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from drone_link import Pose  # noqa: E402
from live_map import LiveMap, MapRunner  # noqa: E402
from mission import Phase, build, parse_args  # noqa: E402
from mission_state import StateSnapshot  # noqa: E402
from sim import ManualClock  # noqa: E402
from tag_map import TagEstimate  # noqa: E402
from test_mission import configure  # noqa: E402

TICK = 0.1


@pytest.fixture(autouse=True)
def close_figures():
    yield
    plt.close("all")


def estimate(tag_id, x, y, confirmed):
    return TagEstimate(tag_id, x, y, 6, 6, 6, 0.01, confirmed, 0.0,
                       1.0 if confirmed else None, x if confirmed else None,
                       y if confirmed else None)


class MapRig:
    """Sim mission + map, with the GUI timer callback called by hand."""

    def __init__(self, settings, camera=False):
        self.clock = ManualClock(0.0)
        self.m = build(settings, sim=True, auto=True, clock=self.clock,
                       threaded_source=False, echo=lambda _: None)
        self.view = LiveMap(settings, self.m.waypoints)
        self.view.draw = lambda: None          # skip pixel rendering (map.png still renders)
        if not camera:                         # skip rendering sim camera images
            self.m.source.latest_view = lambda: None
        self.runner = MapRunner(self.m, self.view, period_s=TICK)
        self.m.start()

    def tick(self, n=1):
        for _ in range(n):
            if self.runner.finished:
                return
            self.clock.advance(TICK)
            self.m.source.pump()
            self.runner.on_timer()
            if self.m.phase == Phase.READY and not self.m.start_sent:
                self.m.command("takeoff")

    def until(self, cond, max_s=900):
        for _ in range(int(max_s / TICK)):
            if cond() or self.runner.finished:
                return cond()
            self.tick()
        return cond()


# ---------------------------------------------------------------------------
# Drawing from snapshots
# ---------------------------------------------------------------------------

def test_candidates_hollow_confirmed_labelled(tmp_path):
    view = LiveMap(configure(tmp_path), [(0.2, 0.2), (1.3, 0.2)])
    view.update(StateSnapshot(phase="EXPLORING", tags=(
        estimate(3, 0.8, 0.3, True), estimate(9, 1.0, 0.6, False))))
    assert view._conf.get_offsets().tolist() == [[0.3, 0.8]]       # (east, north)
    assert view._cand.get_offsets().tolist() == [[0.6, 1.0]]
    assert set(view.tag_labels) == {3}
    assert "3" in view.tag_labels[3].get_text() and "(0.80, 0.30)" in view.tag_labels[3].get_text()
    assert view._title.get_text().endswith("tags 1/6")


def test_label_follows_the_improving_estimate(tmp_path):
    view = LiveMap(configure(tmp_path), [(0.2, 0.2)])
    view.update(StateSnapshot(tags=(estimate(3, 0.80, 0.30, True),)))
    view.update(StateSnapshot(tags=(estimate(3, 0.82, 0.31, True),)))
    assert len(view.tag_labels) == 1
    assert view.tag_labels[3].xy == (0.31, 0.82)
    assert "(0.82, 0.31)" in view.tag_labels[3].get_text()


@pytest.mark.parametrize("heading_deg, east, north", [
    (0, 0.0, 1.0), (90, 1.0, 0.0), (180, 0.0, -1.0), (-90, -1.0, 0.0)])
def test_heading_arrow_points_the_right_way(tmp_path, heading_deg, east, north):
    s = configure(tmp_path)
    view = LiveMap(s, [(0.2, 0.2)])
    view.update(StateSnapshot(pose=Pose(0.0, 0.5, 0.4, math.radians(heading_deg), 2)))
    L = s.live_map.arrow_m
    assert view._arrow.U[0] == pytest.approx(east * L, abs=1e-9)
    assert view._arrow.V[0] == pytest.approx(north * L, abs=1e-9)
    assert view._drone.get_data()[0][0] == 0.4 and view._drone.get_data()[1][0] == 0.5


def test_view_grows_to_include_far_tags_and_trail(tmp_path):
    view = LiveMap(configure(tmp_path), [(0.2, 0.2), (1.3, 0.8)])
    view.update(StateSnapshot(trajectory=((0.0, 0.0), (-1.0, 2.5)),
                              tags=(estimate(5, 3.0, -1.0, True),)))
    x0, x1 = view.ax.get_xlim()
    y0, y1 = view.ax.get_ylim()
    assert x0 < -1.0 and x1 > 2.5           # east axis covers the trail and the tag
    assert y0 < -1.0 and y1 > 3.0           # north axis too


def test_next_waypoint_ring(tmp_path):
    wps = [(0.2, 0.2), (1.3, 0.2)]
    view = LiveMap(configure(tmp_path), wps)
    view.update(StateSnapshot(phase="EXPLORING", waypoints=tuple(wps), wp_index=1))
    assert np.allclose(view._next.get_data(), ([0.2], [1.3]))
    view.update(StateSnapshot(phase="RETURNING", waypoints=tuple(wps), home=(0.0, 0.0)))
    assert np.allclose(view._next.get_data(), ([0.0], [0.0]))
    view.update(StateSnapshot(phase="LANDING"))
    assert len(view._next.get_data()[0]) == 0


# ---------------------------------------------------------------------------
# Running the mission from the map timer
# ---------------------------------------------------------------------------

def test_full_sim_mission_through_the_map(tmp_path):
    r = MapRig(configure(tmp_path))
    r.until(lambda: r.runner.finished)
    assert r.m.phase == Phase.DONE and r.m.exit_code == 0
    assert set(r.view.tag_labels) == {t.id for t in r.m.settings.sim.true_tags}
    assert len(r.view._trail.get_data()[0]) > 300
    assert r.view._home.get_data()[0][0] == pytest.approx(0.0)
    assert (r.m.log.folder / "map.png").stat().st_size > 10_000
    assert r.view._title.get_text().startswith("DONE")


def test_redraw_rate_is_lower_than_tick_rate(tmp_path):
    r = MapRig(configure(tmp_path))
    assert r.runner.redraw_every == 2                    # 10 Hz ticks, 5 Hz redraw
    calls = []
    r.view.update = lambda snap, _orig=r.view.update: (calls.append(1), _orig(snap))
    r.tick(10)
    assert len(calls) == 5


def test_closing_the_window_mid_flight_lands(tmp_path):
    r = MapRig(configure(tmp_path))
    assert r.until(lambda: r.m.phase == Phase.EXPLORING)
    r.runner.on_close()
    assert r.m.land_sent and r.m.exit_code == 130 and r.runner.finished
    assert r.m.drone.commands[-1][1] == "CMD_LAND"


def test_ctrl_c_mid_flight_lands_on_next_tick(tmp_path):
    r = MapRig(configure(tmp_path))
    assert r.until(lambda: r.m.phase == Phase.EXPLORING)
    r.runner.request_stop("Ctrl+C", 130)
    assert not r.m.land_sent                             # acted on at the next tick
    r.tick()
    assert r.m.land_sent and r.m.exit_code == 130 and r.runner.finished


def test_exception_in_tick_lands(tmp_path, monkeypatch):
    r = MapRig(configure(tmp_path))
    assert r.until(lambda: r.m.phase == Phase.EXPLORING)

    def boom():
        raise RuntimeError("bug")
    monkeypatch.setattr(r.m, "tick", boom)
    r.runner.on_timer()
    assert r.m.land_sent and r.m.exit_code == 1 and r.runner.finished


def test_close_after_finish_is_harmless(tmp_path):
    r = MapRig(configure(tmp_path))
    r.until(lambda: r.runner.finished)
    r.runner.on_close()
    assert r.m.exit_code == 0


def test_cli_map_flag():
    a = parse_args(["--map", "--sim", "--auto"])
    assert a.map and a.sim and a.auto
    assert not parse_args([]).map


# ---------------------------------------------------------------------------
# Camera panel
# ---------------------------------------------------------------------------

def cam_view(t=10.0, marks=()):
    from tag_pipeline import CameraView
    gray = np.full((240, 320), 200, np.uint8)
    return CameraView(gray, t, tuple(marks), 130)


def test_camera_panel_waits_then_shows_frames_and_outlines(tmp_path):
    from tag_pipeline import PipelineStats, TagMark
    view = LiveMap(configure(tmp_path), [(0.2, 0.2)])
    view.update_camera(None, 0.0)
    assert view._cam_title.get_text() == "camera: waiting for video"
    marks = [TagMark(3, ((100, 100), (130, 100), (130, 130), (100, 130))),
             TagMark(9, ((200, 50), (220, 50), (220, 70), (200, 70)), "height")]
    view.update_camera(cam_view(10.0, marks), 10.1, PipelineStats(frames=50, dropped=10))
    view.update_camera(cam_view(11.0, marks), 11.1, PipelineStats(frames=59, dropped=10))
    assert np.array_equal(view._cam_img.get_array(), np.full((240, 320), 200))
    texts = [a.get_text() for a in view._cam_artists if hasattr(a, "get_text")]
    assert texts == ["3", "9: height"]
    title = view._cam_title.get_text()
    assert "9.0 fps" in title and "dropped 14%" in title and "latency 130 ms" in title
    view.update_camera(cam_view(11.0), 11.2)              # outlines replaced, not piled up
    assert view._cam_artists == []


def test_camera_panel_says_no_video_when_frames_stop(tmp_path):
    s = configure(tmp_path)
    view = LiveMap(s, [(0.2, 0.2)])
    view.update_camera(cam_view(10.0), 10.0 + s.live_map.camera_stale_s + 0.5)
    assert view._cam_title.get_text().startswith("camera: NO VIDEO")


def test_camera_panel_can_be_turned_off(tmp_path):
    import dataclasses
    s = configure(tmp_path)
    s = dataclasses.replace(s, live_map=dataclasses.replace(s.live_map, show_camera=False))
    view = LiveMap(s, [(0.2, 0.2)])
    assert view.cam_ax is None
    view.update_camera(cam_view(), 10.0)                  # harmless no-op


def test_sim_mission_fills_the_camera_panel(tmp_path):
    r = MapRig(configure(tmp_path), camera=True)
    assert r.until(lambda: r.m.phase == Phase.EXPLORING and r.m.wp_index >= 1)
    r.tick(2)
    assert r.view._cam_img.get_array().std() > 5          # a real (rendered) image
    assert "fps" in r.view._cam_title.get_text()
    r.runner.on_close()
