"""Whole-mission behaviour in the simulator (mission/mission.py)."""

import csv
import dataclasses
import io
import math
import types
import re

import pytest

import mission as M
from mission import AIRBORNE, Phase, build, geofence, parse_args, run, validate_plan
from settings import Area, ConfigError, load_settings
from sim import ManualClock

TICK = 0.1


def configure(tmp_path, *, flight=None, safety=None, sim=None, tags=None, sweep=None,
              uwb=None, localisation=None):
    """config.yaml (with the test arena, conftest.TEST_ARENA) with overrides.
    sweep.start_at_home is off unless asked for: the sim's true tags are laid
    out in the area exactly as the test arena writes it."""
    s = load_settings()
    s = dataclasses.replace(s, logging=dataclasses.replace(s.logging, log_dir=str(tmp_path)))
    sweep = {"start_at_home": False, "start_in_middle": False, **(sweep or {})}
    for name, kw in (("flight", flight), ("safety", safety), ("sim", sim), ("tags", tags),
                     ("sweep", sweep), ("uwb", uwb), ("localisation", localisation)):
        if kw:
            s = dataclasses.replace(s, **{name: dataclasses.replace(getattr(s, name), **kw)})
    return s


class Rig:
    """A sim mission on a manual clock, stepped one tick at a time."""

    def __init__(self, settings, auto=True, dry_run=False, uwb=None):
        self.clock = ManualClock(1000.0)
        self.lines = []
        self.m = build(settings, sim=True, auto=auto, dry_run=dry_run, clock=self.clock,
                       threaded_source=False, echo=self.lines.append, uwb=uwb)
        self.drone, self.cam = self.m.drone, self.m.source
        self.m.start()

    def step(self):
        self.clock.advance(TICK)
        self.cam.pump()
        return self.m.tick()

    def until(self, cond, max_s=600.0):
        for _ in range(int(max_s / TICK)):
            if cond():
                return True
            if not self.step():
                return cond()
        return False

    def finish(self, max_s=900.0):
        while self.m.phase != Phase.DONE and max_s > 0:
            self.step()
            max_s -= TICK
        self.m.shutdown()

    def events(self, name=None):
        with open(self.m.log.folder / "events.csv", newline="", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        return [r for r in rows if name is None or r["event"] == name]


def required(lines):
    """Event names from the '[EVENT] <date> <time>  T+  12.345 s  NAME ...' lines."""
    return [re.search(r"T\+\s*\S+ s\s+(\S+)", ln).group(1)
            for ln in lines if ln.startswith("[EVENT]")]


# ---------------------------------------------------------------------------
# Full runs
# ---------------------------------------------------------------------------

def test_full_sweep_visits_all_waypoints_returns_lands_and_confirms_all_tags(tmp_path):
    s = configure(tmp_path, flight={"stop_when_all_found": False})
    r = Rig(s)
    assert r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()

    m, d = r.m, r.drone
    assert m.exit_code == 0
    assert m.reached == list(range(len(m.waypoints))) and m.timeouts == []
    # Back home, settled within land_radius_m, and landed.
    assert math.hypot(d.x - m.home[0], d.y - m.home[1]) <= s.flight.land_radius_m + 0.01
    assert re.search(r"cm from (home|the home tag)", r.events("LAND_CMD")[0]["details"])
    assert d.alt == 0.0 and not d.armed
    # Every tag confirmed, within a few cm of the truth.
    truth = {t.id: (t.x, t.y) for t in s.sim.true_tags}
    found = {e.tag_id: e for e in m.tag_map.estimates(confirmed_only=True)}
    assert set(found) == set(truth)
    for tid, e in found.items():
        assert math.dist((e.x, e.y), truth[tid]) < 0.03
    # Required events, in order, displayed and recorded.
    assert required(r.lines) == ["TAKEOFF_CMD"] + ["TAG_CONFIRMED"] * 6 + ["LAND_CMD"]
    assert [e["event"] for e in r.events() if e["event"] in
            ("TAKEOFF_CMD", "TAG_REPORT", "LAND_CMD")] == ["TAKEOFF_CMD", "TAG_REPORT", "LAND_CMD"]
    assert any("TAG REPORT" in ln for ln in r.lines)
    # Log folder.
    folder = m.log.folder
    for name in ("events.csv", "trajectory.csv", "detections.csv", "tags_final.csv",
                 "config_used.yaml"):
        assert (folder / name).is_file()
    with open(folder / "tags_final.csv", newline="", encoding="utf-8") as fh:
        assert len(list(csv.DictReader(fh))) == 6
    with open(folder / "trajectory.csv", newline="", encoding="utf-8") as fh:
        assert len(list(csv.DictReader(fh))) > 500


def test_lands_anyway_if_it_never_settles_over_home(tmp_path):
    # The sim drone holds its setpoint perfectly, so shift the mission's idea of
    # home by 20 cm once it starts waiting: the drone then never gets within 5 cm.
    # (No precision landing: the home tag would put the landing spot back.)
    r = Rig(configure(tmp_path, flight={"land_settle_max_s": 3.0},
                      localisation={"enabled": False}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m._settle_t is not None, 600)
    r.m.land_target = (r.m.land_target[0] + 0.2, r.m.land_target[1])
    t0 = r.clock()
    assert r.until(lambda: r.m.land_sent, 10)
    assert r.clock() - t0 == pytest.approx(3.0, abs=0.3)
    assert "not settled" in r.events("LAND_CMD")[0]["details"]
    r.finish()
    assert r.m.exit_code == 0 and r.drone.alt == 0.0


def test_early_stop_when_all_tags_found(tmp_path):
    r = Rig(configure(tmp_path))                              # stop_when_all_found: true
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    assert r.events("ALL_TAGS_FOUND")
    assert len(r.m.reached) < len(r.m.waypoints)
    assert len(r.m.tag_map.estimates(confirmed_only=True)) == 6
    assert r.m.exit_code == 0 and r.drone.alt == 0.0


def test_manual_mode_waits_for_start(tmp_path):
    r = Rig(configure(tmp_path), auto=False)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.HOVERING and r.m._hover_ok, 60)
    for _ in range(50):                                      # 5 s: still hovering
        r.step()
    assert r.m.phase == Phase.HOVERING
    assert r.events("HOVER_CONFIRMED") and r.events("FLYING_CONFIRMED")
    r.m.command("start")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 1)


def test_goto_sent_during_takeoff_is_not_trusted(tmp_path):
    # Hover probe can only succeed once the firmware is FLYING.
    r = Rig(configure(tmp_path, flight={"takeoff_wait_s": 0.5}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.HOVERING, 60)
    assert r.drone.phase == "FLYING"
    assert r.events("GOTO_RESEND")                           # probe ignored at first


# ---------------------------------------------------------------------------
# Operator and safety
# ---------------------------------------------------------------------------

def test_land_mid_exploration_lands_immediately(tmp_path):
    r = Rig(configure(tmp_path))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING and r.m.wp_index >= 1, 300)
    x, y = r.drone.x, r.drone.y
    r.m.command("land")
    r.step()
    assert r.m.phase == Phase.ABORT_LANDING and r.m.land_sent
    assert required(r.lines)[-1] == "LAND_CMD"
    r.finish()
    assert r.drone.alt == 0.0
    assert math.hypot(r.drone.x - x, r.drone.y - y) < 0.1   # landed where it was
    assert r.m.exit_code == 0


def test_stale_telemetry_triggers_abort_land(tmp_path):
    s = configure(tmp_path)
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 300)
    r.drone.pause_telemetry()
    t0 = r.clock()
    assert r.until(lambda: r.m.phase == Phase.ABORT_LANDING, 15)
    assert r.clock() - t0 == pytest.approx(s.safety.stale_timeout_s, abs=0.3)
    assert "telemetry stale" in r.events("ABORT")[0]["details"]
    assert r.m.land_sent and r.m.exit_code == 1
    r.finish()
    assert r.drone.alt == 0.0                                # LAND still got through


def test_geofence_triggers_abort_land(tmp_path):
    r = Rig(configure(tmp_path))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 300)
    x_max = geofence(r.m.area, r.m.home, r.m.settings.safety.geofence_margin_m,
                     r.m.waypoints)[1]
    r.drone.ekf_jump(x_max + 0.3 - r.drone.x, 0.0)      # just past the fence, any config
    assert r.until(lambda: r.m.phase == Phase.ABORT_LANDING, 2)
    assert "geofence" in r.events("ABORT")[0]["details"]


def test_time_limit_returns_and_lands(tmp_path):
    r = Rig(configure(tmp_path, safety={"mission_time_limit_s": 30.0}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    assert r.events("TIME_LIMIT")
    assert any("time limit" in e["details"] for e in r.events("PHASE"))
    assert r.m.exit_code == 0 and r.drone.alt == 0.0
    assert math.hypot(r.drone.x - r.m.home[0], r.drone.y - r.m.home[1]) <= 0.25


def test_lost_goto_after_arrival_is_not_mistaken_for_arrival(tmp_path):
    # After waypoint 1 the firmware still says ARRIVED.  If every later GOTO
    # is lost, the mission must resend, then time out -- never "arrive".
    r = Rig(configure(tmp_path, flight={"nav_mode": "waypoints", "waypoint_timeout_margin_s": 0.0}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.reached == [0], 300)
    r.drone.settings = dataclasses.replace(
        r.drone.settings, sim=dataclasses.replace(r.drone.settings.sim, command_loss=1.0))
    # (the GOTO already sent in the same tick may still get through)
    assert r.until(lambda: r.m.timeouts, 120)
    first_lost = r.m.timeouts[0]
    assert first_lost in (1, 2) and r.m.reached == list(range(first_lost))
    assert r.events("GOTO_RESEND") and r.events("WAYPOINT_TIMEOUT")


def test_single_lost_goto_is_resent(tmp_path):
    r = Rig(configure(tmp_path, flight={"nav_mode": "waypoints", "stop_when_all_found": False}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.reached == [0], 300)
    sim = r.drone.settings.sim
    r.drone.settings = dataclasses.replace(r.drone.settings,
                                           sim=dataclasses.replace(sim, command_loss=1.0))
    r.until(lambda: r.events("GOTO_RESEND"), 10)          # first copy of GOTO 2 lost
    r.drone.settings = dataclasses.replace(r.drone.settings, sim=sim)
    assert r.until(lambda: 1 in r.m.reached, 120)
    assert r.m.timeouts == []


def driven_by_run(settings, interrupt_when=None, max_ticks=6000):
    """Build a sim mission and drive it through mission.run(), as main() does.

    Types `takeoff` once READY; raises KeyboardInterrupt when interrupt_when()
    becomes true.  Fails instead of hanging if it never finishes.
    """
    clock, lines = ManualClock(0.0), []
    m = build(settings, sim=True, auto=True, clock=clock, threaded_source=False,
              echo=lines.append)
    ticks = [0]

    def before():
        ticks[0] += 1
        assert ticks[0] < max_ticks, f"mission stuck in {m.phase}"
        clock.advance(TICK)
        m.source.pump()
        if m.phase == Phase.READY and not m.start_sent:
            m.command("takeoff")
        if interrupt_when is not None and interrupt_when(m):
            raise KeyboardInterrupt

    code = run(m, period_s=0.0, sleep=lambda _: None, before_tick=before)
    return m, lines, code


def test_ctrl_c_lands(tmp_path):
    m, lines, code = driven_by_run(configure(tmp_path),
                                   interrupt_when=lambda m: m.phase == Phase.EXPLORING)
    assert code == 130
    assert m.land_sent and required(lines)[-1] == "LAND_CMD"
    assert m.drone.commands[-1][1] == "CMD_LAND"


def test_ctrl_c_on_the_ground_sends_nothing(tmp_path):
    m, _, code = driven_by_run(configure(tmp_path),
                               interrupt_when=lambda m: m.phase == Phase.READY)
    assert code == 130 and not m.start_sent and not m.land_sent
    assert m.drone.commands == []


def test_exception_after_start_lands(tmp_path, monkeypatch):
    def boom(self, now):
        raise RuntimeError("bug in exploring")
    monkeypatch.setattr(M.Mission, "_tick_exploring", boom)
    m, lines, code = driven_by_run(configure(tmp_path))
    assert code == 1
    assert m.land_sent and required(lines)[-1] == "LAND_CMD"
    assert m.drone.commands[-1][1] == "CMD_LAND"


def test_full_run_through_main_loop(tmp_path):
    m, lines, code = driven_by_run(configure(tmp_path))
    assert code == 0 and m.phase == Phase.DONE
    assert required(lines)[0] == "TAKEOFF_CMD" and required(lines)[-1] == "LAND_CMD"


def test_operator_commands_in_wrong_states_are_harmless(tmp_path):
    r = Rig(configure(tmp_path))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    for cmd in ("land", "start", "status", "bogus", "help"):
        r.m.command(cmd)
    r.step()
    assert r.m.phase == Phase.READY and not r.m.land_sent and r.drone.commands == []
    r.m.command("takeoff")
    r.step()
    r.m.command("takeoff")                                   # second one ignored
    r.m.command("quit")                                      # refused while airborne
    r.step()
    assert r.m.phase == Phase.TAKING_OFF
    assert [c[1] for c in r.drone.commands].count("CMD_START") == 3
    r.m.command("land")
    r.finish()


def test_quit_on_ground(tmp_path):
    r = Rig(configure(tmp_path))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("quit")
    assert r.step() is False
    r.m.shutdown()
    assert not r.m.land_sent


def test_dry_run_never_flies_but_detects(tmp_path):
    r = Rig(configure(tmp_path), dry_run=True)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    for _ in range(30):
        r.step()
    assert r.m.phase == Phase.READY and not r.m.start_sent
    assert r.drone.commands == [] and r.drone.alt == 0.0
    r.m.command("quit")
    r.step()
    r.m.shutdown()


def test_connect_timeout(tmp_path):
    s = configure(tmp_path)
    s = dataclasses.replace(s, network=dataclasses.replace(s.network, connect_timeout_s=2.0))
    clock = ManualClock(0.0)
    m = build(s, sim=True, clock=clock, threaded_source=False, echo=lambda _: None)
    m.drone.pause_telemetry()
    m.start()
    for _ in range(40):
        clock.advance(TICK)
        if not m.tick():
            break
    assert m.phase == Phase.DONE and m.exit_code == 1 and not m.start_sent
    m.shutdown()


def test_state_snapshot_for_live_map(tmp_path):
    r = Rig(configure(tmp_path))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.until(lambda: r.m.phase == Phase.EXPLORING and r.m.wp_index >= 2, 300)
    snap = r.m.state.snapshot()
    assert snap.phase == "EXPLORING" and snap.home is not None
    assert len(snap.waypoints) == len(r.m.waypoints) and snap.wp_index >= 2
    assert len(snap.trajectory) > 100 and snap.pose is not None
    assert snap.mission_s > 0
    r.m.command("land")
    r.finish()


# ---------------------------------------------------------------------------
# Startup checks and CLI
# ---------------------------------------------------------------------------

def test_validate_plan():
    area = Area(0.0, 1.0, 0.0, 1.0)
    validate_plan([(0.5, 0.5)], area)
    with pytest.raises(ConfigError, match="outside the search area"):
        validate_plan([(0.5, 0.5), (1.2, 0.5)], area)
    with pytest.raises(ConfigError, match="no waypoints"):
        validate_plan([], area)


def test_cli_modes():
    a = parse_args(["--sim", "--auto"])
    assert a.sim and a.auto and not a.dry_run
    with pytest.raises(SystemExit):
        parse_args(["--sim", "--dry-run"])


def test_bad_config_exits_2(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("drone_id: 2\n", encoding="utf-8")
    assert M.main(["--config", str(bad), "--sim"]) == 2
    assert "config error" in capsys.readouterr().err


def test_operator_input_thread(tmp_path):
    r = Rig(configure(tmp_path))
    th = M.start_operator_input(r.m, io.StringIO("status\nquit\n"))
    th.join(1.0)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    assert r.step() is False                                 # quit processed
    r.m.shutdown()


# ---------------------------------------------------------------------------
# Trajectory mode (flight.nav_mode: trajectory, the default)
# ---------------------------------------------------------------------------

def test_trajectory_sweep_keeps_the_heading_and_covers_the_area(tmp_path):
    s = configure(tmp_path, flight={"stop_when_all_found": False})
    assert s.flight.nav_mode == "trajectory"
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 60)
    h0 = r.drone.latest_pose().heading
    r.finish()
    m = r.m
    assert m.exit_code == 0 and r.drone.alt == 0.0
    # The heading never changed after takeoff.
    assert max(abs(p.heading - h0) for p in r.drone.poses.snapshot()) < 1e-6
    # The camera looks straight down, so the plan stays where it is.
    assert not r.events("PLAN_SHIFTED")
    assert m.waypoints[0][0] == pytest.approx(s.area.x_min + s.area.margin)
    # One upload each for the sweep and the return, both played to the end.
    assert [e["details"].split(":")[0] for e in r.events("TRAJ_END")] == ["sweep", "return home"]
    assert all("done" in e["details"] for e in r.events("TRAJ_END"))
    assert m.reached == list(range(len(m.waypoints))) and m.timeouts == []
    assert math.hypot(r.drone.x - m.home[0], r.drone.y - m.home[1]) < 0.05
    # Every tag, including those near the area edge behind the camera.
    found = {e.tag_id for e in m.tag_map.estimates(confirmed_only=True)}
    assert found == {t.id for t in s.sim.true_tags}


@pytest.mark.parametrize("nav_mode", ["trajectory", "waypoints"])
def test_start_at_home_moves_plan_and_area_so_waypoint_1_is_home(tmp_path, nav_mode):
    s = configure(tmp_path, sweep={"start_at_home": True},
                  flight={"nav_mode": nav_mode, "stop_when_all_found": False})
    r = Rig(s)
    r.drone.ekf_jump(0.7, -0.4)          # takeoff spot is not where the area's corner is
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 60)
    m, a0 = r.m, s.area
    assert m.waypoints[0] == pytest.approx(m.home, abs=1e-3)
    # Same size, moved by the same vector as the plan.
    assert m.area.x_max - m.area.x_min == pytest.approx(a0.x_max - a0.x_min)
    assert m.area.y_max - m.area.y_min == pytest.approx(a0.y_max - a0.y_min)
    dx, dy = m.area.x_min - a0.x_min, m.area.y_min - a0.y_min
    assert m.home[1] == pytest.approx(a0.y_min + a0.margin + dy, abs=1e-3)
    if nav_mode == "waypoints":                # no camera shift: plain translation
        assert m.home[0] == pytest.approx(a0.x_min + a0.margin + dx, abs=1e-3)
    assert r.events("PLAN_MOVED") and m.state.snapshot().area == m.area
    # The whole sweep flies inside the moved geofence (no abort) and lands at home.
    r.finish()
    assert m.exit_code == 0 and not r.events("ABORT")
    tol = 0.05 if nav_mode == "trajectory" else 0.3      # GOTO arrive radius 0.25 m
    assert math.hypot(r.drone.x - m.home[0], r.drone.y - m.home[1]) < tol


@pytest.mark.parametrize("lanes_along", ["x", "y"])
def test_start_in_middle_centres_the_lanes_across_home(tmp_path, lanes_along):
    s = configure(tmp_path, sweep={"start_at_home": True, "start_in_middle": True,
                                   "lanes_along": lanes_along},
                  flight={"stop_when_all_found": False})
    r = Rig(s, uwb=None)
    r.drone.ekf_jump(0.7, -0.4)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 60)
    m = r.m
    xs, ys = zip(*m.waypoints)
    along, across = (0, 1) if lanes_along == "x" else (1, 0)
    # Waypoint 1 is level with home, and home is halfway across the lanes.
    assert m.waypoints[0][along] == pytest.approx(m.home[along], abs=1e-3)
    assert m.waypoints[0][across] != pytest.approx(m.home[across], abs=0.05)
    mid = (min(ys) + max(ys)) / 2 if lanes_along == "x" else (min(xs) + max(xs)) / 2
    assert mid == pytest.approx(m.home[across], abs=2e-3)
    # The area box moved with the plan, so home is on its middle line too.
    box = (m.area.y_min + m.area.y_max, m.area.x_min + m.area.x_max)[across == 0]
    assert box / 2 == pytest.approx(m.home[across], abs=2e-3)
    r.finish()
    assert m.exit_code == 0 and not r.events("ABORT")


def test_start_at_home_off_keeps_the_configured_area(tmp_path):
    r = Rig(configure(tmp_path), auto=False)
    r.drone.ekf_jump(0.7, -0.4)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.HOVERING and r.m._hover_ok, 60)
    r.m.command("start")
    r.step()
    assert r.m.area == r.m.settings.area and not r.events("PLAN_MOVED")
    r.m.command("land")
    r.finish()


def test_camera_offset_moves_the_sweep_backwards(tmp_path):
    s = configure(tmp_path, sweep={"camera_offset_m": 0.6})
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.EXPLORING, 60)
    assert r.events("PLAN_SHIFTED")                        # drone faces North -> South
    assert r.m.waypoints[0][0] == pytest.approx(s.area.x_min + s.area.margin - 0.6)
    r.m.command("land")
    r.finish()


def test_trajectory_start_is_retried_when_the_upload_is_lost(tmp_path):
    r = Rig(configure(tmp_path), auto=False)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.HOVERING and r.m._hover_ok, 60)
    lossless = r.drone.settings
    r.drone.settings = dataclasses.replace(
        lossless, sim=dataclasses.replace(lossless.sim, command_loss=1.0))
    r.m.command("start")
    assert r.until(lambda: len(r.events("TRAJECTORY")) == 2, 10)    # first one lost
    r.drone.settings = lossless
    assert r.until(lambda: r.events("TRAJ_STARTED"), 10)
    assert "attempt 3" in r.events("TRAJ_STARTED")[0]["details"]
    r.m.command("land")
    r.finish()


def test_a_trajectory_that_never_finishes_times_out(tmp_path):
    s = configure(tmp_path, flight={"traj_end_margin_s": 2.0, "traj_timeout_factor": 3.0})
    r = Rig(s)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.events("TRAJ_STARTED"), 80)
    r.m.job.traj = types.SimpleNamespace(duration=1.0)       # still playing after 3 x 1 + 2 s
    start = r.clock()
    assert r.until(lambda: r.events("TRAJ_END"), 8)
    assert "timeout" in r.events("TRAJ_END")[0]["details"]
    assert r.clock() - start == pytest.approx(5.0, abs=2.0)
    r.finish()


def test_a_wifi_gap_does_not_use_up_trajectory_attempts(tmp_path):
    r = Rig(configure(tmp_path), auto=False)
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.HOVERING and r.m._hover_ok, 60)
    uploads = []
    send = r.drone.send_trajectory
    r.drone.send_trajectory = lambda offsets, dt_ms: uploads.append(1) or send(offsets, dt_ms)
    r.drone.pause_telemetry(True)
    r.m.command("start")
    r.step()
    assert r.m.phase == Phase.EXPLORING
    for _ in range(50):                                     # 5 s of silence, longer than the
        r.step()                                            # 1.5 s start timeout
    assert len(uploads) == 1 and r.m.phase == Phase.EXPLORING
    r.drone.pause_telemetry(False)
    assert r.until(lambda: r.events("TRAJ_STARTED"), 5)
    r.m.command("land")
    r.finish()


def test_trajectory_refused_every_time_abort_lands(tmp_path):
    r = Rig(configure(tmp_path, flight={"traj_start_attempts": 3}))
    r.drone.send_trajectory = lambda offsets, dt_ms: True     # sent, never plays
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.ABORT_LANDING, 120)
    assert len(r.events("TRAJECTORY")) == 3 and r.events("TRAJ_FAILED")
    assert r.m.land_sent and r.m.exit_code == 1
    r.finish()
    assert r.drone.alt == 0.0


def test_early_stop_cancels_the_sweep_and_flies_home_by_trajectory(tmp_path):
    r = Rig(configure(tmp_path))                              # stop_when_all_found: true
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    assert r.events("ALL_TAGS_FOUND")
    cmds = [c[1] for c in r.drone.commands]
    i = cmds.index("CMD_HOLD")                                # sweep cancelled...
    assert cmds[i + 1] == "CMD_TRAJ"                          # ...then the return trajectory
    ends = r.events("TRAJ_END")
    assert ends[-1]["details"].startswith("return home: done")
    assert r.m.exit_code == 0 and r.drone.alt == 0.0
