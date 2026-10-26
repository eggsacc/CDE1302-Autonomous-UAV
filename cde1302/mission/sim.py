"""
Simulator: a fake drone and a fake camera with the same interfaces as
drone_link.DroneLink and tag_pipeline.TagPipeline, so mission.py can run
with no hardware and no network (`python mission/mission.py --sim`).

    clock = ManualClock()                 # tests: time moves only when told
    clock = ScaledClock(speed=5.0)        # --sim: real time x speed
    drone = SimDrone(settings, clock)
    cam = SimTagSource(settings, drone, on_sighting=tag_map.add, clock=clock)

SimDrone models the firmware as read from main/main.c, wifi_task.c and
nav_task.c, simplified:

    READY --START--> arming (arm_delay_s) --> climbing to 0.5 m (climb_rate_mps)
          --> +1 s --> FLYING  (only now are GOTOs accepted)
    GOTO:  ROTATING in place (yaw_rate_deg_s) until within NAV_YAW_TOL_RAD of
           the goal bearing, then FLYING straight at cruise_speed_mps;
           ARRIVED at < 0.25 m, then settles onto the goal.
    HOLD:  IDLE, holds where it is (or at the goal if it already arrived).
    LAND:  descends at climb_rate_mps, disarms, back to READY.

Telemetry (10 Hz) adds pose_noise_m; each command datagram can be lost with
probability command_loss.  pause_telemetry() makes the link go silent, for
stale-telemetry tests.  There is no EKF drift and no obstacle model.

SimTagSource makes `camera_fps` frames per second.  A true tag is "seen" when
the drone is airborne and the tag lies between view_min_m and view_max_m
ahead and within +-fov_half_angle_deg.  The body-frame position gets
sighting_noise_m noise, is wrapped in a Detection and goes through the real
TagFilter (same range / floor / yaw-rate checks as the real pipeline), and is
delivered latency_s after capture.  false_positive_rate adds wrong-ID
sightings at random spots in view.

Both are lazy: physics and frames are computed up to clock() whenever
anything is asked, so no extra threads are needed except the optional
SimTagSource thread that delivers sightings in real time.
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from typing import Callable, Optional

import numpy as np

import laptop_path  # noqa: F401
from apriltag_host import CAM_POS_M, R_BODY_CAM, Detection
from protocol import NAV_ARRIVED, NAV_FLYING, NAV_IDLE, NAV_ROTATING

from drone_link import Pose, PoseBuffer
from geometry import angle_diff, ned_to_body, wrap_angle
from settings import Settings
from sighting import Sighting
from tag_pipeline import PipelineStats, TagFilter

log = logging.getLogger(__name__)

SIM_DT = 0.02             # physics step (s)
TELEM_DT = 0.1            # telemetry period, 10 Hz like the firmware (s)
NAV_YAW_TOL_RAD = 0.1     # nav_task.h
NAV_ARRIVE_RADIUS_M = 0.25  # nav_task.h (firmware's own, not flight.arrival_radius_m)
CRUISE_ALT_M = 0.5        # main.c
ALT_TOLERANCE_M = 0.15    # main.c: "at altitude" band
TAKEOFF_SETTLE_S = 1.0    # main.c: delay after altitude before FLYING
AIRBORNE_M = 0.2          # camera model: below this the drone sees no tags
SIM_IP = "127.0.0.1"


class ManualClock:
    """Clock that only moves when advance() is called (deterministic tests)."""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


class ScaledClock:
    """Monotonic clock running `speed` times faster than real time."""

    def __init__(self, speed: float = 1.0,
                 base: Callable[[], float] = time.monotonic) -> None:
        self.speed = speed
        self._base = base
        self._b0 = base()

    def __call__(self) -> float:
        return self._b0 + (self._base() - self._b0) * self.speed


class SimDrone:
    """Fake drone + firmware with DroneLink's interface (see interfaces.py)."""

    def __init__(self, settings: Settings, clock: Callable[[], float], *,
                 dry_run: bool = False,
                 on_command: Optional[Callable[[str, str], None]] = None,
                 start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 rng: Optional[random.Random] = None) -> None:
        self.settings = settings
        self.drone_id = settings.drone_id
        self.dry_run = dry_run
        self._clock = clock
        self._on_command = on_command
        self._rng = rng or random.Random(settings.sim.seed)
        net = settings.network
        self.poses = PoseBuffer(net.pose_history_s, net.pose_lookup_tolerance_s)
        self._truth = PoseBuffer(net.pose_history_s, 0.0)   # noise-free x, y, heading
        self._alt_hist = PoseBuffer(net.pose_history_s, 0.0)  # altitude stored in .x
        self._listeners: list[Callable[[Pose], None]] = []
        self._lock = threading.RLock()

        # vehicle state
        self.x, self.y, self.heading = start_pose[0], start_pose[1], wrap_angle(start_pose[2])
        self.alt = 0.0
        self.armed = False
        self.phase = "READY"          # READY, ARMING, CLIMBING, SETTLING, FLYING, LANDING
        self._phase_t = 0.0
        self.nav = NAV_IDLE
        self._goal: Optional[tuple[float, float]] = None
        self._hold: tuple[float, float] = (self.x, self.y)

        self._t: Optional[float] = None            # sim time reached
        self._next_telem = 0.0
        self._last_telem: Optional[float] = None
        self._telem_paused = False
        self._started = False
        self.commands: list[tuple[float, str, tuple]] = []   # delivered commands (tests)

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            self._t = self._clock()
            self._next_telem = self._t
            self._phase_t = self._t
            self._record_truth()
            self._started = True
        log.info("Simulated drone %d ready at (%.2f, %.2f)%s", self.drone_id,
                 self.x, self.y, " (DRY RUN)" if self.dry_run else "")

    def stop(self) -> None:
        self._started = False

    def reset_offset(self) -> None:
        """No-op: the sim always reports PX4 local NED with no offset."""

    # -- test hooks ---------------------------------------------------------

    def pause_telemetry(self, paused: bool = True) -> None:
        """Stop (or resume) telemetry, as if the Wi-Fi link went quiet."""
        with self._lock:
            self.advance()
            self._telem_paused = paused

    def true_pose_at(self, t: float) -> Optional[tuple[float, float, float, float]]:
        """Noise-free (x, y, heading, altitude) at time t, or None."""
        self.advance()
        p, a = self._truth.pose_at(t), self._alt_hist.pose_at(t)
        if p is None or a is None:
            return None
        return p.x, p.y, p.heading, a.x

    @property
    def airborne(self) -> bool:
        return self.alt > AIRBORNE_M

    # -- DroneInterface: state ---------------------------------------------

    def now(self) -> float:
        return self._clock()

    @property
    def connected(self) -> bool:
        self.advance()
        return self._last_telem is not None

    @property
    def esp_ip(self) -> Optional[str]:
        return SIM_IP if self.connected else None

    def on_pose(self, fn: Callable[[Pose], None]) -> None:
        self._listeners.append(fn)

    def latest_pose(self) -> Optional[Pose]:
        self.advance()
        return self.poses.latest()

    def pose_at(self, t: float) -> Optional[Pose]:
        self.advance()
        return self.poses.pose_at(t)

    def yaw_rate_at(self, t: float) -> Optional[float]:
        self.advance()
        return self.poses.yaw_rate_at(t)

    def telemetry_age(self, now: Optional[float] = None) -> float:
        self.advance()
        if self._last_telem is None:
            return float("inf")
        return (self._clock() if now is None else now) - self._last_telem

    def is_stale(self, now: Optional[float] = None) -> bool:
        return self.telemetry_age(now) > self.settings.safety.stale_timeout_s

    # -- DroneInterface: commands ------------------------------------------

    def _deliver(self, name: str, repeats: int, apply: Callable[[], None],
                 details: str = "") -> bool:
        if self.dry_run:
            log.warning("DRY RUN: %s not sent %s", name, details)
            self._hook(name, f"DRY RUN, not sent {details}".strip())
            return False
        if not self.connected:
            log.error("%s not sent: no telemetry yet", name)
            self._hook(name, f"x{repeats} {details} FAILED".strip())
            return False
        with self._lock:
            self.advance()
            for _ in range(repeats):
                if self._rng.random() >= self.settings.sim.command_loss:
                    self.commands.append((self._t, name, ()))
                    apply()
        log.info("%s sent x%d %s", name, repeats, details)
        self._hook(name, f"x{repeats} {details}".strip())
        return True

    def _hook(self, name: str, details: str) -> None:
        if self._on_command is not None:
            try:
                self._on_command(name, details)
            except Exception:
                log.exception("on_command hook failed")

    def send_start(self) -> bool:
        def apply() -> None:
            if self.phase == "READY":              # firmware: one START per flight
                self._set_phase("ARMING")
        return self._deliver("CMD_START", self.settings.flight.start_repeats, apply)

    def send_goto(self, x: float, y: float) -> bool:
        def apply() -> None:
            if self.phase == "FLYING":             # firmware: ignored unless flying
                self._goal = (float(x), float(y))
                self.nav = NAV_ROTATING
        return self._deliver("CMD_GOTO", 1, apply, f"goal=({x:.3f}, {y:.3f})")

    def send_hold(self) -> bool:
        def apply() -> None:
            if self._goal is not None:             # nav_cancel() with an active goal
                self._hold = (self.x, self.y)
            self._goal = None
            self.nav = NAV_IDLE
        return self._deliver("CMD_HOLD", 1, apply)

    def send_land(self) -> bool:
        def apply() -> None:
            if self.phase in ("ARMING",):
                self.armed = False
                self._set_phase("READY")
            elif self.phase in ("CLIMBING", "SETTLING", "FLYING"):
                self._goal, self.nav = None, NAV_IDLE
                self._set_phase("LANDING")
        return self._deliver("CMD_LAND", self.settings.flight.land_repeats, apply)

    # -- physics ------------------------------------------------------------

    def _set_phase(self, phase: str) -> None:
        log.debug("sim phase %s -> %s", self.phase, phase)
        self.phase, self._phase_t = phase, self._t

    def advance(self) -> None:
        """Run physics and telemetry up to clock()."""
        with self._lock:
            if self._t is None:
                return
            now = self._clock()
            while self._t + SIM_DT <= now + 1e-9:
                self._t += SIM_DT
                self._step(SIM_DT)
                self._record_truth()
                if self._t + 1e-9 >= self._next_telem:
                    self._next_telem += TELEM_DT
                    if not self._telem_paused:
                        self._emit_telemetry()

    def _record_truth(self) -> None:
        self._truth.append(Pose(self._t, self.x, self.y, self.heading, self.nav))
        self._alt_hist.append(Pose(self._t, self.alt, 0.0, 0.0, 0))

    def _emit_telemetry(self) -> None:
        n = self.settings.sim.pose_noise_m
        pose = Pose(self._t, self.x + self._rng.gauss(0, n) if n else self.x,
                    self.y + self._rng.gauss(0, n) if n else self.y,
                    self.heading, self.nav)
        self._last_telem = self._t
        self.poses.append(pose)
        for fn in self._listeners:
            try:
                fn(pose)
            except Exception:
                log.exception("pose listener failed")

    def _move_towards(self, gx: float, gy: float, dt: float) -> None:
        dx, dy = gx - self.x, gy - self.y
        d = math.hypot(dx, dy)
        step = self.settings.flight.cruise_speed_mps * dt
        if d <= step:
            self.x, self.y = gx, gy
        elif d > 0:
            self.x += dx / d * step
            self.y += dy / d * step

    def _step(self, dt: float) -> None:
        s = self.settings.sim
        climb = s.climb_rate_mps * dt
        if self.phase == "ARMING":
            if self._t - self._phase_t >= s.arm_delay_s:
                self.armed = True
                self._set_phase("CLIMBING")
        elif self.phase == "CLIMBING":
            self.alt = min(CRUISE_ALT_M, self.alt + climb)
            if abs(self.alt - CRUISE_ALT_M) < ALT_TOLERANCE_M and self.alt >= CRUISE_ALT_M - 1e-9:
                self._hold = (self.x, self.y)
                self._set_phase("SETTLING")
        elif self.phase == "SETTLING":
            if self._t - self._phase_t >= TAKEOFF_SETTLE_S:
                self._set_phase("FLYING")
        elif self.phase == "LANDING":
            self.alt = max(0.0, self.alt - climb)
            if self.alt <= 0.0:
                self.armed = False
                self._set_phase("READY")
            return
        if self.phase != "FLYING":
            return

        if self._goal is None:                   # IDLE or ARRIVED: hold the setpoint
            self._move_towards(*self._hold, dt)
            return
        gx, gy = self._goal
        dist = math.hypot(gx - self.x, gy - self.y)
        if dist < NAV_ARRIVE_RADIUS_M:
            self.nav, self._hold, self._goal = NAV_ARRIVED, (gx, gy), None
            return
        err = angle_diff(math.atan2(gy - self.y, gx - self.x), self.heading)
        if abs(err) > NAV_YAW_TOL_RAD:
            self.nav = NAV_ROTATING
            turn = math.radians(s.yaw_rate_deg_s) * dt
            self.heading = wrap_angle(self.heading + max(-turn, min(turn, err)))
        else:
            self.nav = NAV_FLYING
            self.heading = wrap_angle(self.heading + err)   # small correction
            step = self.settings.flight.cruise_speed_mps * dt
            self.x += math.cos(self.heading) * step
            self.y += math.sin(self.heading) * step


class SimTagSource:
    """Fake camera + detector with TagPipeline's interface (see interfaces.py)."""

    def __init__(self, settings: Settings, drone: SimDrone,
                 on_sighting: Callable[[Sighting], object], *,
                 clock: Callable[[], float], threaded: bool = True,
                 rng: Optional[random.Random] = None) -> None:
        """
        Args:
            threaded: start() runs a thread that calls pump() every 20 ms real
                time; tests pass False and call pump() themselves.
        """
        self.settings = settings
        self._drone = drone
        self._on_sighting = on_sighting
        self._clock = clock
        self._threaded = threaded
        self._rng = rng or random.Random(settings.sim.seed + 1)
        self._filter = TagFilter(settings.tags)
        self._period = 1.0 / settings.sim.camera_fps
        self._next_capture: Optional[float] = None
        self._pending: list[tuple[float, Sighting]] = []   # (deliver_t, sighting)
        self._stats = PipelineStats()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._next_capture = self._clock()
        self._stop.clear()
        if self._threaded:
            self._thread = threading.Thread(target=self._run, name="sim-camera", daemon=True)
            self._thread.start()
        log.info("Simulated camera: %d true tags, %.0f fps", len(self.settings.sim.true_tags),
                 self.settings.sim.camera_fps)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        log.info("Simulated camera stopped: %s", self.stats().summary())

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stats(self) -> PipelineStats:
        with self._lock:
            return self._stats.copy()

    def _run(self) -> None:
        while not self._stop.wait(0.02):
            try:
                self.pump()
            except Exception:
                log.exception("sim camera error")

    # -- frames -------------------------------------------------------------

    def in_view(self, forward: float, right: float) -> bool:
        """Simple footprint: a wedge on the floor ahead of the drone."""
        s = self.settings.sim
        return (s.view_min_m <= forward <= s.view_max_m
                and abs(math.degrees(math.atan2(right, forward))) <= s.fov_half_angle_deg)

    def pump(self) -> int:
        """Capture every frame due up to now and deliver sightings whose
        latency has passed.  Returns the number delivered."""
        if self._next_capture is None:
            return 0
        now = self._clock()
        latency = self.settings.sim.latency_s
        while self._next_capture <= now - latency + 1e-9:
            self._capture(self._next_capture)
            self._next_capture += self._period
        due = [s for t, s in self._pending if t <= now + 1e-9]
        self._pending = [(t, s) for t, s in self._pending if t > now + 1e-9]
        for s in due:
            try:
                self._on_sighting(s)
            except Exception:
                log.exception("on_sighting callback failed")
        return len(due)

    def _detection(self, tag_id: int, body: tuple[float, float, float]) -> Detection:
        t = tuple(float(v) for v in np.asarray(R_BODY_CAM).T
                  @ (np.asarray(body) - np.asarray(CAM_POS_M)))
        return Detection(id=tag_id, hamming=0, decision_margin=200.0,
                         centre=(160.0, 120.0), corners=((0.0, 0.0),) * 4,
                         pose_valid=True, t=t, R=tuple(np.eye(3).ravel()),
                         pose_err=1e-3)

    def _capture(self, tc: float) -> None:
        sim = self.settings.sim
        truth = self._drone.true_pose_at(tc)
        with self._lock:
            self._stats.frames += 1
            self._stats.last_frame_t = tc
        if truth is None:
            return
        x, y, heading, alt = truth
        if alt <= AIRBORNE_M:
            return
        seen: list[tuple[int, float, float]] = []
        for tag in sim.true_tags:
            f, r = ned_to_body(tag.x, tag.y, x, y, heading)
            if self.in_view(f, r):
                seen.append((tag.id, f, r))
        if self._rng.random() < sim.false_positive_rate:
            f = self._rng.uniform(sim.view_min_m, sim.view_max_m)
            r = f * math.tan(math.radians(self._rng.uniform(-1, 1) * sim.fov_half_angle_deg))
            seen.append((self._rng.randrange(22), f, r))
        n = sim.sighting_noise_m
        t_detect = tc + sim.latency_s
        for tag_id, f, r in seen:
            body = (f + self._rng.gauss(0, n), r + self._rng.gauss(0, n),
                    alt + self._rng.gauss(0, n))
            s, reason = self._filter.evaluate(self._detection(tag_id, body), tc,
                                              t_detect, self._drone)
            with self._lock:
                self._stats.detections += 1
                if s is None:
                    self._stats.rejected[reason] += 1
                else:
                    self._stats.accepted += 1
            if s is not None:
                self._pending.append((t_detect, s))
