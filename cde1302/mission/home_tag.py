"""
Home tag: measure optical-flow drift with an AprilTag at the takeoff spot.

PX4 places every tag using its own (drifting) position, so a tag that never
moves appears to move by exactly PX4's position error.  With one at home:

    before the sweep   home tag seen at  T0   (PX4 has not drifted yet)
    back home          home tag seen at  T1   ->  drift  e = T1 - T0

The real takeoff spot is then at home + e in PX4's frame: fly there and land.
The search tags are corrected too, assuming the drift grew in proportion to
the distance flown (flow error mostly scales with distance), from 0 at T0 to
e at T1.

SightingHistory keeps every accepted sighting (camera thread in, mission
thread out); Mission does the rest.
"""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np

from sighting import Sighting


class SightingHistory:
    """Thread-safe (t_capture, x, y) per tag ID, in arrival order."""

    def __init__(self) -> None:
        self._by_id: dict[int, list[tuple[float, float, float]]] = {}
        self._lock = threading.Lock()

    def add(self, s: Sighting) -> None:
        with self._lock:
            self._by_id.setdefault(s.tag_id, []).append((s.t_capture, s.x, s.y))

    def points(self, tag_id: int, since: float = -np.inf) -> list[tuple[float, float, float]]:
        """Sightings of tag_id captured at or after `since`."""
        with self._lock:
            return [p for p in self._by_id.get(tag_id, ()) if p[0] >= since]

    def median(self, tag_id: int, since: float, min_n: int) -> Optional[tuple[float, float, float, int]]:
        """(x, y, t, n): median position and time of the sightings since `since`,
        or None with fewer than min_n."""
        pts = self.points(tag_id, since)
        if len(pts) < min_n:
            return None
        a = np.asarray(pts, dtype=float)
        return (float(np.median(a[:, 1])), float(np.median(a[:, 2])),
                float(np.median(a[:, 0])), len(pts))


class DistanceFlown:
    """Cumulative horizontal distance along PX4's estimated path, by time."""

    def __init__(self, min_step_m: float = 0.005) -> None:
        self.min_step_m = min_step_m          # ignore hover jitter below this
        self._t: list[float] = []
        self._d: list[float] = []
        self._last: Optional[tuple[float, float]] = None
        self._total = 0.0

    def add(self, t: float, x: float, y: float) -> None:
        if self._last is not None:
            step = float(np.hypot(x - self._last[0], y - self._last[1]))
            if step < self.min_step_m:
                return
            self._total += step
        self._last = (x, y)
        if self._t and t <= self._t[-1]:
            self._d[-1] = self._total
            return
        self._t.append(t)
        self._d.append(self._total)

    def at(self, t: float) -> float:
        """Distance flown by time t (m), interpolated."""
        if not self._t:
            return 0.0
        return float(np.interp(t, self._t, self._d))


def drift_at(d: float, d0: float, d1: float, e: tuple[float, float]) -> tuple[float, float]:
    """The drift after flying d metres, growing linearly from 0 at d0 to e at d1."""
    if d1 <= d0:
        return e
    k = min(max((d - d0) / (d1 - d0), 0.0), 1.0)
    return e[0] * k, e[1] * k
