"""
MissionState: one thread-safe snapshot of everything a display needs.

mission.py writes it every tick (and the telemetry thread appends trajectory
points); a future live_map.py only ever calls snapshot(), from its own GUI
timer, and never touches the drone or the tag map directly.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field, replace
from typing import Any, Optional

from drone_link import Pose
from tag_map import TagEstimate


@dataclass(frozen=True)
class StateSnapshot:
    phase: str = "CONNECTING"
    pose: Optional[Pose] = None
    home: Optional[tuple[float, float]] = None
    waypoints: tuple[tuple[float, float], ...] = ()
    wp_index: int = 0                          # next waypoint to reach
    trajectory: tuple[tuple[float, float], ...] = ()
    tags: tuple[TagEstimate, ...] = ()         # all candidates; .confirmed marks real ones
    mission_s: Optional[float] = None          # since the takeoff command
    dry_run: bool = False
    message: str = ""                          # last notable event, for a status line

    @property
    def confirmed_tags(self) -> tuple[TagEstimate, ...]:
        return tuple(t for t in self.tags if t.confirmed)


class MissionState:
    """Thread-safe holder of the latest StateSnapshot."""

    def __init__(self, **initial: Any) -> None:
        self._lock = threading.Lock()
        self._snap = StateSnapshot(**initial)
        self._trajectory: list[tuple[float, float]] = []

    def set(self, **fields: Any) -> None:
        """Update some fields (names as in StateSnapshot, except trajectory)."""
        with self._lock:
            self._snap = replace(self._snap, **fields)

    def add_point(self, x: float, y: float) -> None:
        """Append one trajectory point (called from the telemetry thread)."""
        with self._lock:
            self._trajectory.append((x, y))

    def snapshot(self) -> StateSnapshot:
        with self._lock:
            return replace(self._snap, trajectory=tuple(self._trajectory))
