#!/usr/bin/env python3
"""
Autonomous AprilTag search mission for the CDE1302 mini project.

Run from the cde1302/ folder:

    python mission/mission.py --sim --auto        # simulated drone, no hardware
    python mission/mission.py --dry-run           # real drone: telemetry + tags, NEVER flies
    python mission/mission.py --auto              # real flight, no pause after hover
    python mission/mission.py                     # real flight, type `start` after hover
    python mission/mission.py --config other.yaml
    add --map to any of these for a live map window (live_map.py)

Operator commands (type + Enter, any time):
    takeoff   arm, take off and hover            (READY)
    start     begin the search                   (after hover; not needed with --auto)
    land      land immediately                   (any airborne state)
    status    print one status line
    quit      exit                               (only while on the ground)
Ctrl+C lands the drone (if it took off) and exits.

State machine (every transition goes to events.csv):

    CONNECTING -> READY -> TAKING_OFF -> HOVERING -> EXPLORING -> RETURNING
               -> LANDING -> DONE,      plus ABORT_LANDING from any airborne state

Hover is confirmed by (a) a probe GOTO to the current position that the
firmware answers with nav_state ARRIVED, which it can only do once it is in
its FLYING phase (altitude reached), and (b) xy staying within
hover_radius_m for hover_settle_s.  Telemetry has no altitude, so this is
the best signal available.

Navigation (flight.nav_mode):
    trajectory (default)  The sweep, and then the return home, are each uploaded
        as one constant-speed trajectory (flight.cruise_speed_mps) that the
        firmware plays itself, as laptop/send_trajectory.py does.  The heading
        stays whatever it was at takeoff, so the sweep is first shifted back by
        sweep.camera_offset_m so the camera's view covers the area.  Start is
        confirmed by nav_state TRAJ (re-uploaded if not), end by ARRIVED.  The
        firmware's setpoint waits whenever the drone falls behind, so a sweep
        takes as long as it takes (flight.traj_timeout_factor limits that).
    waypoints  One GOTO per waypoint; the drone turns to face each one.

With sweep.start_at_home the plan and the area are then moved together so the
first waypoint is home (the takeoff spot); the area's size and shape stay.
sweep.start_in_middle centres the lanes across home instead.

With uwb.send_to_px4, UWB/PX4 position pairs are collected during the sweep
(uwb_feed.py).  Once they give a good alignment it is sent to the drone, which
feeds PX4 from then on.  A sweep with no alignment simply flies on without UWB.

The main loop is a fixed-rate, non-blocking Mission.tick(), so a GUI timer
(the future live map) can drive it instead of run().
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Iterable, Optional

if __package__ in (None, ""):                  # `python mission/mission.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import laptop_path  # noqa: F401
from protocol import NAV_ARRIVED, NAV_FLYING, NAV_IDLE, NAV_ROTATING, NAV_STUCK, NAV_TRAJ

import planner
from drone_link import DroneLink, Pose
from interfaces import DroneInterface, SightingSource
from mission_log import MissionLog, format_wall
from mission_state import MissionState
from settings import Area, ConfigError, DEFAULT_CONFIG_PATH, Settings, load_settings
from sighting import Sighting
from tag_map import TagEstimate, TagMap
from trajectory import Trajectory, build_trajectory, shift_for_camera

log = logging.getLogger("mission")

# A telemetry packet received this soon after a command may still describe
# the drone's state from before the command (10 Hz telemetry + Wi-Fi delay).
NAV_ACK_DELAY_S = 0.15
MOVING_STATES = (NAV_ROTATING, NAV_FLYING, NAV_STUCK)


class Phase(str, Enum):
    CONNECTING = "CONNECTING"
    READY = "READY"
    TAKING_OFF = "TAKING_OFF"
    HOVERING = "HOVERING"
    EXPLORING = "EXPLORING"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    ABORT_LANDING = "ABORT_LANDING"
    DONE = "DONE"


# States where the drone may be in the air: safety checks run, `land` works.
AIRBORNE = (Phase.TAKING_OFF, Phase.HOVERING, Phase.EXPLORING, Phase.RETURNING)

HELP = ("commands: takeoff | start | land | status | quit   "
        "(Ctrl+C lands and exits)")


# ---------------------------------------------------------------------------
# Plan helpers
# ---------------------------------------------------------------------------

def plan_waypoints(settings: Settings) -> list[tuple[float, float]]:
    """Sweep waypoints from planner.py, without its final return to (0, 0):
    the mission returns to the recorded home pose instead."""
    return planner.plan_from_config(settings.raw)[:-1]


def validate_plan(waypoints: list[tuple[float, float]], area: Area) -> None:
    """Raise ConfigError if the plan is empty or any waypoint is outside the area."""
    if not waypoints:
        raise ConfigError("the sweep plan has no waypoints")
    eps = 1e-6
    for i, (x, y) in enumerate(waypoints, 1):
        if not (area.x_min - eps <= x <= area.x_max + eps
                and area.y_min - eps <= y <= area.y_max + eps):
            raise ConfigError(f"waypoint {i} ({x:.2f}, {y:.2f}) is outside the search area "
                              f"x {area.x_min}..{area.x_max}, y {area.y_min}..{area.y_max}")


def geofence(area: Area, home: Optional[tuple[float, float]], margin: float,
             path: Iterable[tuple[float, float]] = ()) -> tuple[float, float, float, float]:
    """(x_min, x_max, y_min, y_max): the area, home and planned path, grown by margin."""
    xs, ys = [area.x_min, area.x_max], [area.y_min, area.y_max]
    for p in ([home] if home is not None else []) + list(path):
        xs.append(p[0])
        ys.append(p[1])
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


# ---------------------------------------------------------------------------
# One GOTO leg
# ---------------------------------------------------------------------------

@dataclass
class Leg:
    """A GOTO in progress, with resend and timeout bookkeeping."""
    name: str
    goal: tuple[float, float]
    sent_t: float
    last_sent_t: float
    deadline: float
    probe: bool = False            # hover probe: done only on firmware ARRIVED
    saw_moving: bool = False
    resends: int = 0


@dataclass
class TrajJob:
    """A trajectory being uploaded / played (flight.nav_mode: trajectory)."""
    name: str
    waypoints: list[tuple[float, float]]
    traj: Trajectory
    sent_t: float
    attempts: int = 1
    started_t: Optional[float] = None    # first telemetry showing TRAJ
    result: Optional[str] = None         # see Mission._update_traj


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

class Mission:
    """The search mission state machine.  Call tick() at flight.tick_hz."""

    def __init__(self, settings: Settings, drone: DroneInterface, source: SightingSource,
                 tag_map: TagMap, mlog: MissionLog, *,
                 waypoints: list[tuple[float, float]], auto: bool = False,
                 echo: Optional[Callable[[str], None]] = None,
                 uwb=None) -> None:
        validate_plan(waypoints, settings.area)
        self.settings = settings
        self.drone = drone
        self.source = source
        self.tag_map = tag_map
        self.log = mlog
        self.uwb = uwb                           # uwb_feed.UwbFeed or None
        self.uwb_alignment = None                # uwb_align.Alignment once decided
        self._uwb_next = 0.0
        self._uwb_reason = ""
        self._uwb_acks = 0                       # ESP32's align_received when we first sent
        self._uwb_confirmed = False
        self.waypoints = list(waypoints)
        self.auto = auto
        self.dry_run = drone.dry_run
        self._echo = echo or (lambda line: print(line, flush=True))
        self._cmds: "queue.SimpleQueue[str]" = queue.SimpleQueue()

        self.phase = Phase.CONNECTING
        self._phase_t = drone.now()
        self.area = settings.area                # moved by sweep.start_at_home
        self.state = MissionState(phase=self.phase.value, waypoints=tuple(waypoints),
                                  area=self.area, dry_run=self.dry_run)
        self.home: Optional[tuple[float, float]] = None
        self.start_sent = False
        self.land_sent = False
        self.exit_code = 0
        self.wp_index = 0
        self.leg: Optional[Leg] = None
        self.job: Optional[TrajJob] = None       # trajectory mode
        self._takeoff_t = 0.0
        self._start_requested = False
        self._hover_ok = False
        self._recent: deque[tuple[float, float, float]] = deque()
        self._next_status = 0.0
        self.reached: list[int] = []            # waypoint indices reached
        self.timeouts: list[int] = []           # waypoint indices that timed out
        self._started = False

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Start the drone link and the tag source (may raise, e.g. port busy)."""
        self._started = True
        self.drone.on_pose(self._on_pose)
        self.drone.start()
        self.source.start()
        if self.uwb is not None:
            self.uwb.start()
        self.log.event("PROGRAM_START", f"mode={'dry-run' if self.dry_run else 'flight'} "
                       f"auto={self.auto} waypoints={len(self.waypoints)} "
                       f"drone={self.settings.drone_id}")

    def shutdown(self) -> None:
        """Fallback LAND if needed, then stop everything.  Safe to call twice."""
        if self.start_sent and not self.land_sent:
            log.warning("Shutting down while possibly airborne: sending LAND")
            self._send_land("shutdown fallback")
        if self._started:
            self._started = False
            stops = [self.source.stop, self.drone.stop]
            if self.uwb is not None:
                stops.insert(0, self.uwb.stop)
            for stop in stops:
                try:
                    stop()
                except Exception:
                    log.exception("error while stopping")
        self.log.event("PROGRAM_END", f"phase={self.phase.value} exit_code={self.exit_code}")
        self.log.close()

    def _on_pose(self, p: Pose) -> None:
        self.log.trajectory(p.t, p.x, p.y, p.heading, p.nav_state_name)
        self.state.add_point(p.x, p.y)

    # -- operator -----------------------------------------------------------

    def command(self, line: str) -> None:
        """Queue an operator command (thread-safe; handled on the next tick)."""
        line = line.strip().lower()
        if line:
            self._cmds.put(line)

    def _handle_commands(self, now: float) -> None:
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            self.log.event("OPERATOR", cmd)
            if cmd == "land":
                if self.phase in AIRBORNE:
                    self.abort("operator land", exit_code=0)
                else:
                    log.warning("land: not flying (phase %s)", self.phase.value)
            elif cmd == "takeoff":
                if self.dry_run:
                    log.warning("takeoff: --dry-run never sends flight commands")
                elif self.phase == Phase.READY:
                    self._takeoff(now)
                else:
                    log.warning("takeoff: only possible in READY (phase %s)", self.phase.value)
            elif cmd == "start":
                if self.phase in (Phase.TAKING_OFF, Phase.HOVERING):
                    self._start_requested = True
                    log.info("start: search begins as soon as hover is confirmed")
                else:
                    log.warning("start: only after takeoff (phase %s)", self.phase.value)
            elif cmd == "status":
                log.info(self.status_line(now))
            elif cmd in ("quit", "exit"):
                if self.phase in AIRBORNE:
                    log.warning("quit: drone may be airborne, use `land` first")
                else:
                    self._set_phase(Phase.DONE, "operator quit")
            else:
                log.info(HELP)

    # -- tick ---------------------------------------------------------------

    def tick(self) -> bool:
        """Advance the state machine once.  Returns False when the mission is DONE."""
        now = self.drone.now()
        self._handle_commands(now)
        if self.phase in AIRBORNE:
            self._safety(now)
        handler = {
            Phase.CONNECTING: self._tick_connecting,
            Phase.READY: self._tick_ready,
            Phase.TAKING_OFF: self._tick_taking_off,
            Phase.HOVERING: self._tick_hovering,
            Phase.EXPLORING: self._tick_exploring,
            Phase.RETURNING: self._tick_returning,
            Phase.LANDING: self._tick_landing,
            Phase.ABORT_LANDING: self._tick_landing,
            Phase.DONE: lambda _now: None,
        }[self.phase]
        handler(now)
        self._tick_uwb(now)
        self.state.set(phase=self.phase.value, pose=self.drone.latest_pose(),
                       home=self.home, wp_index=self.wp_index,
                       tags=tuple(self.tag_map.estimates()),
                       mission_s=self.log.mission_elapsed(now))
        return self.phase != Phase.DONE

    def _set_phase(self, new: Phase, reason: str = "") -> None:
        old, self.phase = self.phase, new
        self._phase_t = self.drone.now()
        self.log.event("PHASE", f"{old.value} -> {new.value}" + (f" ({reason})" if reason else ""))
        self.state.set(phase=new.value, message=reason)

    def _safety(self, now: float) -> None:
        sf = self.settings.safety
        if self.drone.is_stale(now):
            self.abort(f"telemetry stale ({self.drone.telemetry_age(now):.1f} s)")
            return
        pose = self.drone.latest_pose()
        if pose is not None:
            x0, x1, y0, y1 = geofence(self.area, self.home, sf.geofence_margin_m,
                                      self.waypoints)
            if not (x0 <= pose.x <= x1 and y0 <= pose.y <= y1):
                self.abort(f"geofence: drone at ({pose.x:.2f}, {pose.y:.2f})")

    # -- phases -------------------------------------------------------------

    def _tick_connecting(self, now: float) -> None:
        if self.drone.connected:
            p = self.drone.latest_pose()
            self.log.event("CONNECTED", f"drone {self.settings.drone_id} at {self.drone.esp_ip}, "
                           f"pose ({p.x:.2f}, {p.y:.2f}) heading {math.degrees(p.heading):.0f} deg")
            self._set_phase(Phase.READY)
            if self.dry_run:
                log.info("DRY RUN: watching telemetry and tags, no flight commands. "
                         "Type `quit` or Ctrl+C to stop.")
            else:
                log.info("Ready. Type `takeoff` to begin%s.  %s",
                         " (search follows automatically)" if self.auto else "", HELP)
        elif now - self._phase_t > self.settings.network.connect_timeout_s:
            self.log.event("ERROR", f"no telemetry from drone {self.settings.drone_id} "
                           f"within {self.settings.network.connect_timeout_s:.0f} s")
            self.exit_code = 1
            self._set_phase(Phase.DONE, "connect timeout")

    def _tick_ready(self, now: float) -> None:
        if now >= self._next_status:
            self._next_status = now + self.settings.logging.status_period_s
            if self.dry_run:
                log.info(self.status_line(now))

    def _takeoff(self, now: float) -> None:
        pose = self.drone.latest_pose()
        if pose is None or self.drone.is_stale(now):
            log.error("takeoff refused: no fresh telemetry")
            return
        self.home = (pose.x, pose.y)
        self.log.event("HOME", f"({pose.x:.3f}, {pose.y:.3f}) heading "
                       f"{math.degrees(pose.heading):.1f} deg (PX4 local NED)")
        t_cmd = self.drone.now()                   # timestamp = when sending began
        if not self.drone.send_start():
            self.log.event("ERROR", "CMD_START could not be sent")
            return
        self.start_sent = True
        self.log.takeoff_sent(t=t_cmd)
        self._takeoff_t = t_cmd
        self._set_phase(Phase.TAKING_OFF)

    def _tick_taking_off(self, now: float) -> None:
        f = self.settings.flight
        elapsed = now - self._takeoff_t
        if elapsed < f.takeoff_wait_s:
            return
        if elapsed > f.takeoff_wait_s + f.hover_timeout_s:
            self.abort("takeoff not confirmed (probe GOTO never answered)")
            return
        pose = self.drone.latest_pose()
        if self.leg is None:
            self.leg = self._new_leg("hover probe", (pose.x, pose.y), now, probe=True)
            return
        if self._update_leg(now) == "arrived":
            self.leg = None
            self.log.event("FLYING_CONFIRMED", "firmware answered the probe GOTO with ARRIVED")
            self._recent.clear()
            self._set_phase(Phase.HOVERING)

    def _tick_hovering(self, now: float) -> None:
        f = self.settings.flight
        if not self._hover_ok:
            pose = self.drone.latest_pose()
            self._recent.append((now, pose.x, pose.y))
            while self._recent and now - self._recent[0][0] > f.hover_settle_s:
                self._recent.popleft()
            span = self._recent[-1][0] - self._recent[0][0]
            mx = sum(p[1] for p in self._recent) / len(self._recent)
            my = sum(p[2] for p in self._recent) / len(self._recent)
            still = all(math.hypot(p[1] - mx, p[2] - my) <= f.hover_radius_m
                        for p in self._recent)
            if still and span >= f.hover_settle_s - 1e-6:
                self._hover_ok = True
                self.log.event("HOVER_CONFIRMED", f"at ({mx:.2f}, {my:.2f}), "
                               f"{now - self._takeoff_t:.1f} s after takeoff command")
                if not (self.auto or self._start_requested):
                    log.info("Hovering. Type `start` to begin the search, or `land`.")
            elif now - self._phase_t > f.hover_timeout_s:
                self.abort(f"hover not stable within {f.hover_radius_m} m")
                return
        if self._hover_ok and (self.auto or self._start_requested):
            self.wp_index = 0
            self._set_phase(Phase.EXPLORING, "auto" if self.auto else "operator start")
            p = self.drone.latest_pose()
            if self.use_trajectory:
                self._shift_plan_for_camera(p.heading)
            self._move_plan_to_home()
            if self.uwb is not None:
                self.uwb.begin()
                self.log.event("UWB_COLLECTING", "pairing UWB and PX4 positions")
            if self.use_trajectory:
                self.job = self._new_traj("sweep", (p.x, p.y), self.waypoints, now)
            else:
                self._start_waypoint(now)

    def _shift_plan_for_camera(self, heading: float) -> None:
        """Trajectory mode: the heading is now fixed, so move the sweep back so
        the camera's view covers the area (sweep.camera_offset_m)."""
        off = self.settings.sweep.camera_offset_m
        if off <= 0:
            return
        self.waypoints = shift_for_camera(self.waypoints, heading, off)
        self.state.set(waypoints=tuple(self.waypoints))
        self.log.event("PLAN_SHIFTED", f"{off:.2f} m backwards from heading "
                       f"{math.degrees(heading):.0f} deg so the camera view covers the area")

    def _move_plan_to_home(self) -> None:
        """sweep.start_at_home: translate the plan and the area together so the
        sweep starts at home, instead of where config.yaml's area puts it.

        Waypoint 1 lands on home, or with sweep.start_in_middle the lanes are
        centred across home, with waypoint 1 level with it.
        """
        sweep = self.settings.sweep
        if not sweep.start_at_home or self.home is None:
            return
        dx = self.home[0] - self.waypoints[0][0]
        dy = self.home[1] - self.waypoints[0][1]
        if sweep.start_in_middle:
            xs, ys = zip(*self.waypoints)
            if sweep.lanes_along == "x":
                dy = self.home[1] - (min(ys) + max(ys)) / 2.0
            else:
                dx = self.home[0] - (min(xs) + max(xs)) / 2.0
        self.waypoints = [(round(x + dx, 3), round(y + dy, 3)) for x, y in self.waypoints]
        a = self.area
        self.area = dataclasses.replace(a, x_min=a.x_min + dx, x_max=a.x_max + dx,
                                        y_min=a.y_min + dy, y_max=a.y_max + dy)
        self.state.set(waypoints=tuple(self.waypoints), area=self.area)
        where = "the lanes are centred on" if sweep.start_in_middle else "waypoint 1 is"
        self.log.event("PLAN_MOVED", f"by ({dx:+.2f}, {dy:+.2f}) m so {where} home "
                       f"({self.home[0]:.2f}, {self.home[1]:.2f}); area now "
                       f"x {self.area.x_min:.2f}..{self.area.x_max:.2f}, "
                       f"y {self.area.y_min:.2f}..{self.area.y_max:.2f}")

    def _tick_uwb(self, now: float) -> None:
        """uwb.send_to_px4: during the sweep, solve the alignment and deliver it."""
        if (self.uwb is None or self.phase != Phase.EXPLORING or self._uwb_confirmed
                or now < self._uwb_next):
            return
        self._uwb_next = now + self.settings.uwb.align_solve_period_s
        if self.uwb_alignment is None and not self._solve_uwb():
            return
        if self.uwb.align_received > self._uwb_acks:
            self._uwb_confirmed = True
            self.log.event("UWB_ALIGN_CONFIRMED", "ESP32 accepted the alignment; "
                           "UWB now goes to PX4")
            return
        self.drone.send_uwb_align(self.uwb_alignment)     # UDP: repeated until confirmed

    def _solve_uwb(self) -> bool:
        """Try to decide the alignment.  True once there is one to send."""
        r = self.uwb.solve()
        if not r.ok:
            if r.reason.split(":")[0] != self._uwb_reason:
                self._uwb_reason = r.reason.split(":")[0]
                self.log.event("UWB_ALIGN_WAITING", r.reason)
            return False
        s = self.uwb.latest()
        jump = math.hypot(*(a - b for a, b in zip(r.alignment.to_px4(s.ux, s.uy), (s.px, s.py))))
        if jump > self.settings.uwb.max_start_jump_m:
            self.log.event("UWB_ALIGN_REFUSED", f"would move PX4's position {jump:.2f} m "
                           f"(rms {r.rms_m:.3f} m)")
            return False
        a = r.alignment
        self.uwb_alignment = a
        self._uwb_acks = self.uwb.align_received
        self.uwb.finish()
        self.log.event("UWB_ALIGNED", f"theta {math.degrees(a.theta):.1f} deg, mirror {a.mirror}, "
                       f"t ({a.tx:.3f}, {a.ty:.3f}) m, rms {r.rms_m:.3f} m, "
                       f"{r.n_used} samples, jump {jump:.2f} m")
        return True

    @property
    def use_trajectory(self) -> bool:
        """flight.nav_mode: fly the sweep and the return as firmware trajectories."""
        return self.settings.flight.nav_mode == "trajectory"

    def _start_waypoint(self, now: float) -> None:
        i = self.wp_index
        self.leg = self._new_leg(f"waypoint {i + 1}/{len(self.waypoints)}",
                                 self.waypoints[i], now)

    def _tick_exploring(self, now: float) -> None:
        f, t = self.settings.flight, self.settings.tags
        if (f.stop_when_all_found and t.expected_count > 0
                and len(self.tag_map.estimates(confirmed_only=True)) >= t.expected_count):
            self.log.event("ALL_TAGS_FOUND", f"{t.expected_count} tags confirmed, "
                           f"skipping {len(self.waypoints) - self.wp_index} waypoint(s)")
            self._begin_return(now, "all tags found")
            return
        mission_s = self.log.mission_elapsed(now) or 0.0
        if mission_s > self.settings.safety.mission_time_limit_s:
            self.log.event("TIME_LIMIT", f"{mission_s:.0f} s > "
                           f"{self.settings.safety.mission_time_limit_s:.0f} s")
            self._begin_return(now, "time limit")
            return
        if self.use_trajectory:
            self._explore_by_trajectory(now)
            return
        result = self._update_leg(now)
        if result is None:
            return
        if result == "arrived":
            self.reached.append(self.wp_index)
        else:
            self.timeouts.append(self.wp_index)
            self.drone.send_hold()
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self._begin_return(now, "sweep complete")
        else:
            self._start_waypoint(now)

    def _explore_by_trajectory(self, now: float) -> None:
        result = self._update_traj(now)
        self._track_waypoints()
        if result is None:
            return
        missed = list(range(self.wp_index, len(self.waypoints)))
        if result == "failed":
            self.abort("the drone did not start the sweep trajectory")
            return
        self.timeouts += missed
        if missed:
            self.log.event("WAYPOINT_MISSED", f"{len(missed)} waypoint(s) never came within "
                           f"{self.settings.flight.arrival_radius_m} m")
        self.wp_index = len(self.waypoints)
        self._begin_return(now, "sweep complete" if result == "done"
                           else f"sweep trajectory {result}")

    def _track_waypoints(self) -> None:
        """Trajectory mode: count waypoints off, in order, as the drone passes them."""
        p = self.drone.latest_pose()
        r = self.settings.flight.arrival_radius_m
        while (p is not None and self.wp_index < len(self.waypoints)
               and math.dist((p.x, p.y), self.waypoints[self.wp_index]) <= r):
            self.reached.append(self.wp_index)
            self.log.event("WAYPOINT_PASSED", f"{self.wp_index + 1}/{len(self.waypoints)} "
                           f"at ({p.x:.3f}, {p.y:.3f})")
            self.wp_index += 1

    def _begin_return(self, now: float, reason: str) -> None:
        self._set_phase(Phase.RETURNING, reason)
        if not self.use_trajectory:
            self.leg = self._new_leg("return home", self.home, now)
            return
        if self.job is not None and self.job.result is None:
            self.drone.send_hold()               # cancels the sweep still playing
        p = self.drone.latest_pose()
        self.job = self._new_traj("return home", (p.x, p.y), [self.home], now)

    def _tick_returning(self, now: float) -> None:
        if self.use_trajectory:
            result = self._update_traj(now)
            if result is None:
                return
            if result != "done":
                self.drone.send_hold()
            self.job = None
            self._report_and_land("search complete" if result == "done"
                                  else f"return trajectory {result}, landing in place")
            self._set_phase(Phase.LANDING)
            return
        result = self._update_leg(now)
        if result is None:
            return
        if result == "timeout":
            self.drone.send_hold()
        self.leg = None
        self._report_and_land("search complete" if result == "arrived"
                              else "return timeout, landing in place")
        self._set_phase(Phase.LANDING)

    def _tick_landing(self, now: float) -> None:
        if now - self._phase_t >= self.settings.flight.land_settle_s:
            self.log.event("MISSION_COMPLETE",
                           f"mission time {self.log.mission_elapsed(now) or 0.0:.1f} s "
                           f"(takeoff command -> land + {self.settings.flight.land_settle_s:.0f} s)")
            self._set_phase(Phase.DONE)

    # -- trajectories -------------------------------------------------------

    def _new_traj(self, name: str, start: tuple[float, float],
                  waypoints: list[tuple[float, float]], now: float) -> "TrajJob":
        """Build a constant-speed trajectory from `start` and send it."""
        job = TrajJob(name, list(waypoints), self._build_traj(start, waypoints), now)
        self._send_traj(job)
        return job

    def _build_traj(self, start: tuple[float, float],
                    waypoints: list[tuple[float, float]]) -> Trajectory:
        f = self.settings.flight
        traj = build_trajectory(start, waypoints, f.cruise_speed_mps, f.traj_dt_s)
        if traj.max_step_speed > f.traj_max_speed_mps + 1e-6:   # cannot happen by
            raise ValueError(f"trajectory speed {traj.max_step_speed:.3f} m/s "  # construction
                             f"> flight.traj_max_speed_mps {f.traj_max_speed_mps}")
        return traj

    def _send_traj(self, job: "TrajJob") -> None:
        t = job.traj
        self.log.event("TRAJECTORY", f"{job.name} attempt {job.attempts}: "
                       f"{len(t.points)} pts, {t.duration:.1f} s at {t.speed:.2f} m/s, "
                       f"dt {t.dt_ms} ms, from ({t.start[0]:.3f}, {t.start[1]:.3f})")
        self.drone.send_trajectory(t.offsets, t.dt_ms)
        job.sent_t = self.drone.now()           # the upload takes a moment

    def _update_traj(self, now: float) -> Optional[str]:
        """'done', 'failed' (never started), 'stopped', 'timeout' or None (playing).

        Started = telemetry shows TRAJ; done = the firmware switched to ARRIVED
        (it holds the last point).  Not started within traj_start_timeout_s ->
        rebuilt from the current pose and re-uploaded, up to traj_start_attempts.
        """
        job, f = self.job, self.settings.flight
        pose = self.drone.latest_pose()
        if job is None or pose is None:
            return None
        if job.started_t is None:
            if pose.t > job.sent_t + NAV_ACK_DELAY_S and pose.nav_state == NAV_TRAJ:
                job.started_t = pose.t
                self.log.event("TRAJ_STARTED", f"{job.name} (attempt {job.attempts}), "
                               f"ends in about {job.traj.duration:.1f} s")
            elif now - job.sent_t >= f.traj_start_timeout_s and pose.t > job.sent_t:
                # Only fresh telemetry can show it didn't start: during a Wi-Fi
                # gap the drone may well be playing it already.
                if job.attempts >= f.traj_start_attempts:
                    self.log.event("TRAJ_FAILED", f"{job.name}: not started after "
                                   f"{job.attempts} attempts (nav {pose.nav_state_name})")
                    job.result = "failed"
                    return job.result
                job.attempts += 1
                job.traj = self._build_traj((pose.x, pose.y), job.waypoints)
                self._send_traj(job)
            return None
        if pose.t > job.started_t and pose.nav_state == NAV_ARRIVED:
            job.result = "done"
        elif pose.t > job.started_t + NAV_ACK_DELAY_S and pose.nav_state == NAV_IDLE:
            job.result = "stopped"              # cancelled on the drone (HOLD, link loss)
        elif now > (job.started_t + job.traj.duration * f.traj_timeout_factor
                    + f.traj_end_margin_s):
            job.result = "timeout"
        if job.result is not None:
            self.log.event("TRAJ_END", f"{job.name}: {job.result} after "
                           f"{now - job.started_t:.1f} s at ({pose.x:.3f}, {pose.y:.3f})")
        return job.result

    # -- legs ---------------------------------------------------------------

    def _new_leg(self, name: str, goal: tuple[float, float], now: float,
                 probe: bool = False) -> Leg:
        f = self.settings.flight
        pose = self.drone.latest_pose()
        dist = math.hypot(goal[0] - pose.x, goal[1] - pose.y) if pose else 0.0
        budget = dist / f.cruise_speed_mps + f.turn_time_s + f.waypoint_timeout_margin_s
        self.drone.send_goto(*goal)
        self.log.event("GOTO", f"{name} -> ({goal[0]:.3f}, {goal[1]:.3f}), "
                       f"{dist:.2f} m, timeout {budget:.0f} s")
        return Leg(name, goal, now, now, now + budget, probe=probe)

    def _update_leg(self, now: float) -> Optional[str]:
        """'arrived', 'timeout' or None (still going); resends GOTO if ignored."""
        leg, f = self.leg, self.settings.flight
        pose = self.drone.latest_pose()
        if leg is None or pose is None:
            return None
        fresh = pose.t > leg.last_sent_t + NAV_ACK_DELAY_S
        if fresh and pose.nav_state in MOVING_STATES:
            leg.saw_moving = True
        # The firmware keeps reporting ARRIVED from the previous goal until a
        # new GOTO gets through, so for a normal leg ARRIVED only counts after
        # this leg was seen moving.  (The probe starts from IDLE, so it is safe.)
        fw_arrived = (fresh and pose.nav_state == NAV_ARRIVED
                      and (leg.probe or leg.saw_moving))
        dist = math.hypot(leg.goal[0] - pose.x, leg.goal[1] - pose.y)
        if fw_arrived or (not leg.probe and dist <= f.arrival_radius_m):
            self.log.event("ARRIVED", f"{leg.name} at ({pose.x:.3f}, {pose.y:.3f}), "
                           f"{dist:.2f} m from goal, {now - leg.sent_t:.1f} s")
            return "arrived"
        if not leg.probe and now > leg.deadline:
            self.log.event("WAYPOINT_TIMEOUT", f"{leg.name}: still {dist:.2f} m away "
                           f"after {now - leg.sent_t:.0f} s, holding and moving on")
            log.warning("%s timed out", leg.name)
            return "timeout"
        if not leg.saw_moving and now - leg.last_sent_t >= f.goto_resend_s:
            leg.resends += 1
            leg.last_sent_t = now
            self.drone.send_goto(*leg.goal)
            self.log.event("GOTO_RESEND", f"{leg.name} (nav {pose.nav_state_name}, "
                           f"resend {leg.resends})")
        return None

    # -- ending -------------------------------------------------------------

    def _send_land(self, reason: str) -> None:
        t_cmd = self.drone.now()                   # timestamp = when sending began
        ok = self.drone.send_land()
        self.land_sent = True
        self.log.land_sent(reason if ok else f"{reason}; SEND FAILED", t=t_cmd)

    def _report_and_land(self, reason: str) -> None:
        """Report tag positions, then LAND (the brief: report, then land)."""
        self.report()
        self._send_land(reason)

    def abort(self, reason: str, exit_code: int = 1) -> None:
        """Land now (if airborne) and end the mission.  Safe from any state."""
        if self.phase in (Phase.DONE, Phase.LANDING, Phase.ABORT_LANDING) or self.land_sent:
            return
        self.exit_code = exit_code
        self.log.event("ABORT", reason)
        log.warning("ABORT: %s", reason)
        self.leg = None
        self.job = None
        if self.start_sent:
            self._report_and_land(f"abort: {reason}")
            self._set_phase(Phase.ABORT_LANDING, reason)
        else:
            self._set_phase(Phase.DONE, reason)

    def report(self) -> str:
        """Print and save the final tag table; returns the text."""
        now = self.drone.now()
        tags = self.tag_map.estimates(confirmed_only=True)
        ms = self.log.mission_elapsed(now)
        lines = [f"=== TAG REPORT  {format_wall(self.log.wall_time(now))}  "
                 f"T+{(ms or 0.0):.1f} s  "
                 f"({len(tags)} confirmed; PX4 local NED, m; home "
                 f"{'(%.2f, %.2f)' % self.home if self.home else 'n/a'}) ===",
                 f"{'ID':>4} {'x':>8} {'y':>8} {'sightings':>10} {'spread':>7}  first seen"]
        for e in tags:
            first = self.log.mission_elapsed(e.first_seen_t)
            lines.append(f"{e.tag_id:>4} {e.x:>8.3f} {e.y:>8.3f} {e.n_inliers:>10d} "
                         f"{e.spread_m:>7.3f}  T+{(first or 0.0):.1f} s "
                         f"({format_wall(self.log.wall_time(e.first_seen_t))})")
        if not tags:
            lines.append("   (no tags confirmed)")
        text = "\n".join(lines)
        self._echo(text)
        path = self.log.write_tags_final(self.tag_map.final_rows())
        self.log.event("TAG_REPORT", f"{len(tags)} tags -> {path.name}")
        return text

    # -- display ------------------------------------------------------------

    def status_line(self, now: float) -> str:
        p = self.drone.latest_pose()
        pose = (f"({p.x:.2f}, {p.y:.2f}) {math.degrees(p.heading):.0f} deg {p.nav_state_name}"
                if p else "no pose")
        tags = ", ".join(f"{e.tag_id}@({e.x:.2f},{e.y:.2f})"
                         for e in self.tag_map.estimates(confirmed_only=True)) or "none"
        cam = self.source.stats()
        return (f"[{self.phase.value}] pose {pose}, telem age "
                f"{self.drone.telemetry_age(now):.1f} s, wp {self.wp_index}/{len(self.waypoints)}, "
                f"camera {cam.frames} frames/{cam.accepted} sightings, tags: {tags}")


# ---------------------------------------------------------------------------
# Wiring and main loop
# ---------------------------------------------------------------------------

def build(settings: Settings, *, sim: bool = False, dry_run: bool = False,
          auto: bool = False, clock: Optional[Callable[[], float]] = None,
          threaded_source: bool = True,
          echo: Optional[Callable[[str], None]] = None, uwb=None) -> Mission:
    """Create the log, drone, tag map and sighting source and wire them together."""
    if sim:
        from sim import ScaledClock, SimDrone, SimTagSource
        clock = clock or ScaledClock(settings.sim.speed)
    else:
        clock = clock or time.monotonic
    mlog = MissionLog(settings.logging.log_path, settings.source,
                      config_data=None if settings.source else settings.raw,
                      clock=clock, echo=echo)
    tag_map = TagMap.from_settings(settings.tags, on_confirm=lambda e: mlog.tag_confirmed(
        e.tag_id, e.x, e.y, e.support, t=e.confirmed_t))

    def on_command(name: str, details: str) -> None:
        mlog.event(name, details)

    def on_sighting(s: Sighting) -> None:
        mlog.detection(s.t_detect, s.t_capture, **s.log_fields())
        tag_map.add(s)

    drone: DroneInterface
    source: SightingSource
    if sim:
        drone = SimDrone(settings, clock, dry_run=dry_run, on_command=on_command)
        source = SimTagSource(settings, drone, on_sighting, clock=clock,
                              threaded=threaded_source)
    else:
        from tag_pipeline import TagPipeline
        drone = DroneLink(settings, dry_run=dry_run, clock=clock, on_command=on_command)
        source = TagPipeline(settings, drone, lambda: drone.esp_ip, on_sighting, clock=clock)
        if settings.uwb.send_to_px4 and not dry_run:
            from uwb_feed import UwbFeed
            uwb = UwbFeed(settings)
    return Mission(settings, drone, source, tag_map, mlog,
                   waypoints=plan_waypoints(settings), auto=auto, echo=echo, uwb=uwb)


def run(mission: Mission, *, period_s: float,
        sleep: Callable[[float], None] = time.sleep,
        before_tick: Optional[Callable[[], None]] = None) -> int:
    """Tick the mission at a fixed real-time period until DONE.

    Ctrl+C or any exception lands the drone (if it took off), prints the tag
    report and exits; shutdown() always runs.  Returns the exit code.
    """
    try:
        mission.start()
        next_t = time.monotonic()
        while True:
            if before_tick is not None:
                before_tick()
            if not mission.tick():
                break
            next_t += period_s
            delay = next_t - time.monotonic()
            if delay > 0:
                sleep(delay)
            else:
                next_t = time.monotonic()        # fell behind: don't try to catch up
    except KeyboardInterrupt:
        log.warning("Ctrl+C")
        mission.abort("Ctrl+C", exit_code=130)
        mission.exit_code = 130
    except Exception as exc:
        log.exception("mission error")
        mission.abort(f"error: {exc}", exit_code=1)
        mission.exit_code = 1
    finally:
        mission.shutdown()
    return mission.exit_code


def start_operator_input(mission: Mission, stream=None) -> threading.Thread:
    """Read operator commands from stdin on a daemon thread."""
    stream = stream or sys.stdin

    def loop() -> None:
        try:
            for line in stream:
                mission.command(line)
        except (OSError, ValueError):            # stdin closed
            pass

    th = threading.Thread(target=loop, name="operator-input", daemon=True)
    th.start()
    return th


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Autonomous AprilTag search mission.")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG_PATH),
                    help="mission config YAML (default: mission/config.yaml)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true",
                      help="simulated drone and camera, no network")
    mode.add_argument("--dry-run", action="store_true",
                      help="real drone telemetry + camera, NEVER send flight commands")
    ap.add_argument("--auto", action="store_true",
                    help="start the search as soon as hover is confirmed (no `start`)")
    ap.add_argument("--map", action="store_true",
                    help="show a live map of the drone and the tags (live_map.py)")
    return ap.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        settings = load_settings(args.config)
        validate_plan(plan_waypoints(settings), settings.area)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2
    logging.basicConfig(level=getattr(logging, settings.logging.level.upper()),
                        format="%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    mission = build(settings, sim=args.sim, dry_run=args.dry_run, auto=args.auto)
    log.info("Run folder: %s", mission.log.folder)
    start_operator_input(mission)
    speed = settings.sim.speed if args.sim else 1.0
    period_s = 1.0 / (settings.flight.tick_hz * speed)
    if args.map:
        from live_map import run_with_map       # imports matplotlib only when asked
        return run_with_map(mission, period_s)
    return run(mission, period_s=period_s)


if __name__ == "__main__":
    sys.exit(main())
