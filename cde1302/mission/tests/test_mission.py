"""Whole-mission behaviour in the simulator (mission/mission.py)."""

import csv
import dataclasses
import io
import math
import re

import pytest

import mission as M
from mission import AIRBORNE, Phase, build, parse_args, run, validate_plan
from settings import Area, ConfigError, load_settings
from sim import ManualClock

TICK = 0.1


def configure(tmp_path, *, flight=None, safety=None, sim=None, tags=None):
    s = load_settings()
    s = dataclasses.replace(s, logging=dataclasses.replace(s.logging, log_dir=str(tmp_path)))
    for name, kw in (("flight", flight), ("safety", safety), ("sim", sim), ("tags", tags)):
        if kw:
            s = dataclasses.replace(s, **{name: dataclasses.replace(getattr(s, name), **kw)})
    return s


class Rig:
    """A sim mission on a manual clock, stepped one tick at a time."""

    def __init__(self, settings, auto=True, dry_run=False):
        self.clock = ManualClock(1000.0)
        self.lines = []
        self.m = build(settings, sim=True, auto=auto, dry_run=dry_run, clock=self.clock,
                       threaded_source=False, echo=self.lines.append)
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
    # Back home and landed.
    assert math.hypot(d.x - m.home[0], d.y - m.home[1]) <= s.flight.arrival_radius_m
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
    assert r.until(lambda: r.m.phase == Phase.ABORT_LANDING, 10)
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
    r.drone.x += 3.0                                         # e.g. an EKF jump
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
    r = Rig(configure(tmp_path, flight={"waypoint_timeout_margin_s": 0.0}))
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
    r = Rig(configure(tmp_path, flight={"stop_when_all_found": False}))
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
