"""
Constant-speed trajectories through waypoints, for the firmware's trajectory
player (CMD_TRAJ_DATA + CMD_TRAJ_START, as in laptop/send_trajectory.py).

    traj = build_trajectory(start=(0.0, 0.0), waypoints=[(0.2, 0.2), (1.3, 0.2)],
                            speed=0.10, dt=0.05)
    traj.offsets      # (n, 3) x, y, z offsets from the start, what gets uploaded
    traj.dt_ms        # sample period to send in CMD_TRAJ_START
    traj.wp_times     # when the path reaches each waypoint (s after start)

The firmware plays the points relative to where the drone is when the
trajectory starts, at a constant altitude (z offsets are 0) and with the
heading it had at the start, so the drone never turns.  Positions are PX4
local NED, metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import laptop_path  # noqa: F401
from protocol import TRAJ_MAX_PTS


@dataclass(frozen=True)
class Trajectory:
    start: tuple[float, float]            # absolute NED start (where the drone is)
    points: np.ndarray                    # (n, 2) absolute NED x, y at every sample
    dt_ms: int                            # sample period
    wp_times: tuple[float, ...]           # time each waypoint is reached (s from start)
    speed: float                          # m/s along the path

    @property
    def offsets(self) -> np.ndarray:
        """(n, 3) x, y, z offsets from the start: the upload format."""
        rel = self.points - np.asarray(self.start)
        return np.column_stack([rel, np.zeros(len(rel))])

    @property
    def duration(self) -> float:
        return (len(self.points) - 1) * self.dt_ms / 1000.0

    @property
    def max_step_speed(self) -> float:
        """Fastest speed between consecutive samples (m/s)."""
        if len(self.points) < 2:
            return 0.0
        steps = np.hypot(*np.diff(self.points, axis=0).T)
        return float(steps.max() * 1000.0 / self.dt_ms)


def build_trajectory(start: tuple[float, float], waypoints: list[tuple[float, float]],
                     speed: float, dt: float, max_pts: int = TRAJ_MAX_PTS) -> Trajectory:
    """Sample the polyline start -> waypoints at constant speed.

    If the path needs more than max_pts samples, dt is lengthened (in 10 ms
    steps) until it fits; straight segments lose nothing, because the firmware
    interpolates between samples.

    Raises:
        ValueError: empty path, non-positive speed, or a path too long to fit
            even with 1 s samples.
    """
    if speed <= 0:
        raise ValueError("speed must be > 0")
    if not waypoints:
        raise ValueError("a trajectory needs at least one waypoint")
    start = tuple(map(float, start))
    pts = [start]
    times_at: list[int] = []                    # index into pts of each waypoint
    for w in waypoints:
        w = tuple(map(float, w))
        if math.dist(w, pts[-1]) > 1e-9:        # skip zero-length segments
            pts.append(w)
        times_at.append(len(pts) - 1)
    seg = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
    cum = np.concatenate([[0.0], np.cumsum(seg)])        # distance at each vertex
    length = float(cum[-1])

    dt_ms = max(1, round(dt * 1000))
    duration = length / speed
    while duration * 1000.0 / dt_ms + 2 > max_pts:
        dt_ms = (dt_ms // 10 + 1) * 10
        if dt_ms > 1000:
            raise ValueError(f"path of {length:.1f} m at {speed} m/s does not fit "
                             f"{max_pts} points even at 1 s per point")

    n = max(2, math.ceil(duration * 1000.0 / dt_ms) + 1)
    s = np.minimum(np.arange(n) * dt_ms / 1000.0 * speed, length)   # distance along path
    xy = np.column_stack([np.interp(s, cum, [p[0] for p in pts]),
                          np.interp(s, cum, [p[1] for p in pts])])
    xy[-1] = pts[-1]                                    # end exactly on the last waypoint
    return Trajectory(start=start, points=xy, dt_ms=dt_ms,
                      wp_times=tuple(float(cum[i] / speed) for i in times_at), speed=speed)


def shift_for_camera(waypoints: list[tuple[float, float]], heading: float,
                     offset_m: float) -> list[tuple[float, float]]:
    """Move every waypoint `offset_m` backwards along `heading`.

    With a fixed heading the camera always looks the same way, at floor
    roughly offset_m ahead of the drone.  Shifting the drone's path back by
    that much makes the camera's view (not the drone) sweep the original
    path, so the area's edges behind the view are covered too.
    """
    dx, dy = offset_m * math.cos(heading), offset_m * math.sin(heading)
    return [(round(x - dx, 3), round(y - dy, 3)) for x, y in waypoints]
