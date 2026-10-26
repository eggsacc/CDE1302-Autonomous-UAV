"""
Link to one real drone: telemetry tracking, pose history and flight commands.

    link = DroneLink(settings)               # dry_run=True never sends flight commands
    link.start()                             # resets the start offset, starts listening
    ...
    if link.connected and not link.is_stale():
        pose = link.latest_pose()            # newest telemetry pose
        then = link.pose_at(capture_time)    # interpolated pose at an earlier time
    link.send_start(); link.send_goto(1.0, 0.5); link.send_hold(); link.send_land()
    link.stop()

Positions are pure PX4 local NED: CommsNode is built WITHOUT setup.yaml and
the drone is sent a start offset of (0, 0), as simple_waypoint_mission.py
does, so the reported frame is never shifted.

All times are seconds on one monotonic clock (time.monotonic by default),
the same clock the camera pipeline stamps frames with.

The firmware gives no acknowledgement for any command.  START and LAND are
sent several times (extra copies are ignored); GOTO resending is the mission's
job, because only it knows whether the drone started moving.
"""

from __future__ import annotations

import bisect
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

import laptop_path  # noqa: F401  (puts laptop/ on sys.path)
from comms import CommsNode
from protocol import (CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, NAV_STATE_NAMES,
                      CommandPacket, TelemetryPacket)

from geometry import angle_diff, lerp_angle, wrap_angle
from settings import Settings

log = logging.getLogger(__name__)

Clock = Callable[[], float]
CommandHook = Callable[[str, str], None]


@dataclass(frozen=True)
class Pose:
    """Drone pose at monotonic time t: PX4 local NED metres, heading in [-pi, pi)."""
    t: float
    x: float
    y: float
    heading: float
    nav_state: int

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


def pose_from_packet(pkt: TelemetryPacket, t: float) -> Pose:
    """Pose from one telemetry packet received at time t.

    The firmware sends heading in [0, 2*pi); it is wrapped to [-pi, pi).
    """
    return Pose(t, float(pkt.ned_x), float(pkt.ned_y),
                wrap_angle(float(pkt.heading_rad)), int(pkt.nav_state))


class PoseBuffer:
    """Thread-safe, time-ordered pose history with interpolation."""

    def __init__(self, history_s: float = 30.0, tolerance_s: float = 0.25) -> None:
        """
        Args:
            history_s: poses older than this (relative to the newest) are dropped.
            tolerance_s: pose_at() just past the newest pose returns the newest
                pose if within this many seconds, instead of None.
        """
        self.history_s = history_s
        self.tolerance_s = tolerance_s
        self._poses: deque[Pose] = deque()
        self._times: deque[float] = deque()
        self._lock = threading.Lock()

    def __len__(self) -> int:
        with self._lock:
            return len(self._poses)

    def append(self, pose: Pose) -> bool:
        """Add a pose; returns False (and drops it) if it is older than the newest."""
        with self._lock:
            if self._times and pose.t < self._times[-1]:
                return False
            self._poses.append(pose)
            self._times.append(pose.t)
            cutoff = pose.t - self.history_s
            while self._times and self._times[0] < cutoff:
                self._poses.popleft()
                self._times.popleft()
            return True

    def latest(self) -> Optional[Pose]:
        with self._lock:
            return self._poses[-1] if self._poses else None

    def snapshot(self) -> list[Pose]:
        """A copy of every pose held, oldest first."""
        with self._lock:
            return list(self._poses)

    def _bracket(self, t: float) -> Optional[tuple[Pose, Pose]]:
        """The two poses either side of t (equal if t hits a sample); lock held."""
        if not self._poses:
            return None
        if t > self._times[-1]:
            if t - self._times[-1] <= self.tolerance_s:
                return self._poses[-1], self._poses[-1]
            return None
        if t < self._times[0]:
            return None
        i = bisect.bisect_left(self._times, t)
        if self._times[i] == t:
            return self._poses[i], self._poses[i]
        return self._poses[i - 1], self._poses[i]

    def pose_at(self, t: float) -> Optional[Pose]:
        """Pose interpolated at time t, or None if t is outside the history.

        Position is linear; heading goes the short way round (wrap-safe);
        nav_state is taken from the earlier sample.
        """
        with self._lock:
            pair = self._bracket(t)
        if pair is None:
            return None
        p0, p1 = pair
        if p1.t == p0.t:
            return Pose(t, p0.x, p0.y, p0.heading, p0.nav_state)
        a = (t - p0.t) / (p1.t - p0.t)
        return Pose(t, p0.x + a * (p1.x - p0.x), p0.y + a * (p1.y - p0.y),
                    lerp_angle(p0.heading, p1.heading, a), p0.nav_state)

    def yaw_rate_at(self, t: float) -> Optional[float]:
        """Heading rate (rad/s, clockwise positive) around time t, or None.

        Uses the two samples bracketing t; past the newest sample it uses the
        last two samples.
        """
        with self._lock:
            if len(self._poses) < 2:
                return None
            pair = self._bracket(t)
            if pair is None:
                return None
            p0, p1 = pair
            if p0 is p1:                       # on a sample or past the end
                i = self._poses.index(p0)
                p0, p1 = (self._poses[i - 1], p0) if i > 0 else (p0, self._poses[1])
        return angle_diff(p1.heading, p0.heading) / (p1.t - p0.t)


@dataclass(frozen=True)
class TelemetrySnapshot:
    packet: Optional[TelemetryPacket]
    source_ip: Optional[str]
    received_at: float           # monotonic; 0.0 = never


class TelemetryTracker:
    """Receives CommsNode telemetry for one drone and keeps its pose history."""

    def __init__(self, drone_id: int, poses: PoseBuffer, clock: Clock = time.monotonic) -> None:
        self.drone_id = drone_id
        self.poses = poses
        self.packets = 0
        self._clock = clock
        self._snap = TelemetrySnapshot(None, None, 0.0)
        self._lock = threading.Lock()
        self._listeners: list[Callable[[Pose], None]] = []

    def on_pose(self, fn: Callable[[Pose], None]) -> None:
        """Call fn(pose) for every new pose (from the receive thread)."""
        self._listeners.append(fn)

    def callback(self, pkt: TelemetryPacket, source_ip: str) -> None:
        """CommsNode telemetry callback; ignores other drones."""
        if pkt.drone_id != self.drone_id:
            return
        now = self._clock()
        pose = pose_from_packet(pkt, now)
        with self._lock:
            self._snap = TelemetrySnapshot(pkt, source_ip, now)
            self.packets += 1
        self.poses.append(pose)
        for fn in self._listeners:
            try:
                fn(pose)
            except Exception:
                log.exception("pose listener failed")

    def latest(self) -> TelemetrySnapshot:
        with self._lock:
            return self._snap

    def age(self, now: Optional[float] = None) -> float:
        """Seconds since the last packet (inf if none yet)."""
        snap = self.latest()
        if snap.packet is None:
            return float("inf")
        return (self._clock() if now is None else now) - snap.received_at


class DroneLink:
    """The mission's interface to a real drone over CommsNode."""

    def __init__(self, settings: Settings, *, dry_run: bool = False,
                 comms: Optional[CommsNode] = None, clock: Clock = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep,
                 on_command: Optional[CommandHook] = None) -> None:
        """
        Args:
            settings: mission settings (drone_id, network, flight, safety).
            dry_run: if True, START/GOTO/HOLD/LAND are logged but never sent.
            comms: a CommsNode (or test double); default builds one with NO
                config file, so no setup.yaml start offset is ever sent.
            clock / sleep: injectable for tests.
            on_command: called as on_command(name, details) after each
                flight command is handled, e.g. to add it to events.csv.
        """
        self.settings = settings
        self.drone_id = settings.drone_id
        self.dry_run = dry_run
        self._clock = clock
        self._sleep = sleep
        self._on_command = on_command
        net = settings.network
        self.comms = comms if comms is not None else CommsNode(
            listen_port=net.telem_port, cmd_port=net.cmd_port)
        self.poses = PoseBuffer(net.pose_history_s, net.pose_lookup_tolerance_s)
        self.tracker = TelemetryTracker(self.drone_id, self.poses, clock)
        self._started = False

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Register the (0, 0) start offset and start receiving telemetry.

        CommsNode sends the offset on first contact and every 20 s after.
        """
        self.comms.set_nav_tags([], {})
        self.comms.on_telemetry(self.tracker.callback)
        self.comms.start()
        self._started = True
        log.info("Listening for drone %d telemetry on UDP %d%s", self.drone_id,
                 self.settings.network.telem_port, " (DRY RUN)" if self.dry_run else "")

    def stop(self) -> None:
        if self._started:
            self._started = False
            self.comms.stop()

    def reset_offset(self) -> None:
        """Send the (0, 0) start offset now (harmless to repeat)."""
        self.comms.set_nav_tags([], {})
        self.comms.send_nav_tags(self.drone_id)

    # -- state --------------------------------------------------------------

    def now(self) -> float:
        return self._clock()

    @property
    def connected(self) -> bool:
        """True once telemetry has arrived (so commands have an IP to go to)."""
        return self.tracker.latest().packet is not None

    @property
    def esp_ip(self) -> Optional[str]:
        """Drone IP learned from telemetry, or None before first contact."""
        return self.comms.known_drones().get(self.drone_id)

    def on_pose(self, fn: Callable[[Pose], None]) -> None:
        """Call fn(pose) for every telemetry pose (from the receive thread)."""
        self.tracker.on_pose(fn)

    def latest_pose(self) -> Optional[Pose]:
        return self.poses.latest()

    def pose_at(self, t: float) -> Optional[Pose]:
        return self.poses.pose_at(t)

    def yaw_rate_at(self, t: float) -> Optional[float]:
        return self.poses.yaw_rate_at(t)

    def telemetry_age(self, now: Optional[float] = None) -> float:
        return self.tracker.age(now)

    def is_stale(self, now: Optional[float] = None) -> bool:
        """True if no telemetry for longer than safety.stale_timeout_s."""
        return self.telemetry_age(now) > self.settings.safety.stale_timeout_s

    # -- commands -----------------------------------------------------------

    def _send(self, name: str, cmd_type: int, repeats: int = 1,
              goal: tuple[float, float] = (0.0, 0.0)) -> bool:
        """Send one flight command `repeats` times; True if every copy went out.

        Never raises, so it is safe in a finally block.
        """
        details = f"goal=({goal[0]:.3f}, {goal[1]:.3f})" if cmd_type == CMD_GOTO else ""
        if self.dry_run:
            log.warning("DRY RUN: %s not sent %s", name, details)
            self._hook(name, f"DRY RUN, not sent {details}".strip())
            return False
        ok = True
        for i in range(repeats):
            if i:
                self._sleep(self.settings.flight.repeat_gap_s)
            try:
                sent = self.comms.send_command(
                    self.drone_id, CommandPacket(cmd_type, goal_x=goal[0], goal_y=goal[1]))
            except Exception:
                log.exception("%s send failed", name)
                sent = False
            ok = ok and sent
        if ok:
            log.info("%s sent x%d %s", name, repeats, details)
        else:
            log.error("%s not (fully) sent: drone %d IP unknown or socket error",
                      name, self.drone_id)
        self._hook(name, f"x{repeats} {details}".strip() + ("" if ok else " FAILED"))
        return ok

    def _hook(self, name: str, details: str) -> None:
        if self._on_command is not None:
            try:
                self._on_command(name, details)
            except Exception:
                log.exception("on_command hook failed")

    def send_start(self) -> bool:
        """CMD_START: OFFBOARD, arm, climb to cruise altitude (flight.start_repeats copies)."""
        return self._send("CMD_START", CMD_START, self.settings.flight.start_repeats)

    def send_goto(self, x: float, y: float) -> bool:
        """CMD_GOTO to PX4 local NED (x, y); the firmware ignores it unless flying."""
        return self._send("CMD_GOTO", CMD_GOTO, 1, (float(x), float(y)))

    def send_hold(self) -> bool:
        """CMD_HOLD: cancel the current goal and hold position."""
        return self._send("CMD_HOLD", CMD_HOLD, 1)

    def send_land(self) -> bool:
        """CMD_LAND: land in place (flight.land_repeats copies)."""
        return self._send("CMD_LAND", CMD_LAND, self.settings.flight.land_repeats)
