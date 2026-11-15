"""
Tag map: turns a stream of noisy Sightings into one position per tag ID.

    tags = TagMap.from_settings(settings.tags, on_confirm=lambda est: ...)
    tags.add(sighting)               # from the camera thread
    tags.estimates()                 # snapshot for the live map / final report

Per ID, every time a sighting arrives:

1. Seed: the sighting with the most neighbours within confirm_radius_m.
   This is the densest spot, so a few early false positives somewhere else
   cannot pull the estimate the way a plain mean (or even median) can.
2. Inliers: sightings within outlier_radius_m of the seed.
3. Estimate: per-axis median of the inliers, then one refinement pass
   (inliers re-chosen around that median, median again).
4. Support: sightings within confirm_radius_m of the estimate.  The tag is
   CONFIRMED the first time support >= min_sightings; on_confirm fires once.

The estimate keeps improving after confirmation as more sightings arrive.
tag16h5 has only 22 codes, so occasional wrong-ID detections happen: a
single (or scattered) false sighting never gets enough support to confirm.
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from settings import Tags
from sighting import Sighting

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class TagEstimate:
    """Current best guess for one tag ID (PX4 local NED, metres)."""
    tag_id: int
    x: float
    y: float
    n_sightings: int          # sightings held for this ID
    n_inliers: int            # of those, used for the estimate
    support: int              # within confirm_radius_m of the estimate
    spread_m: float           # median distance of inliers from the estimate
    confirmed: bool
    first_seen_t: float       # t_detect of the first sighting
    confirmed_t: Optional[float]   # t_detect of the sighting that confirmed it
    confirmed_x: Optional[float]   # position reported at confirmation
    confirmed_y: Optional[float]

    def final_row(self) -> dict:
        """Row for MissionLog.write_tags_final()."""
        return {"tag_id": self.tag_id, "x": self.x, "y": self.y,
                "n_sightings": self.n_inliers, "first_seen_t": self.first_seen_t,
                "confirmed_t": self.confirmed_t}


def robust_estimate(points: np.ndarray, confirm_radius: float,
                    outlier_radius: float) -> tuple[np.ndarray, np.ndarray, int]:
    """Dense-seeded median of 2-D points.

    Args:
        points: (n, 2) array, n >= 1.
    Returns:
        (estimate (2,), inlier mask (n,), support count)
    """
    diff = points[:, None, :] - points[None, :, :]
    dist = np.hypot(diff[..., 0], diff[..., 1])                 # (n, n)
    seed = int(np.argmax((dist <= confirm_radius).sum(axis=1)))  # first densest
    inliers = dist[seed] <= outlier_radius
    est = np.median(points[inliers], axis=0)
    d = np.hypot(*(points - est).T)
    if (d <= outlier_radius).any():                            # refinement pass
        inliers = d <= outlier_radius
        est = np.median(points[inliers], axis=0)
        d = np.hypot(*(points - est).T)
    return est, inliers, int((d <= confirm_radius).sum())


class _Track:
    """All sightings of one ID plus its confirmation state."""

    def __init__(self, tag_id: int, maxlen: int, first_seen_t: float) -> None:
        self.tag_id = tag_id
        self.points: deque[tuple[float, float]] = deque(maxlen=maxlen)
        self.first_seen_t = first_seen_t
        self.confirmed_t: Optional[float] = None
        self.confirmed_xy: Optional[tuple[float, float]] = None
        self.estimate: Optional[TagEstimate] = None


class TagMap:
    """Thread-safe per-ID sighting store with confirmation and robust estimates."""

    def __init__(self, min_sightings: int = 5, confirm_radius_m: float = 0.20,
                 outlier_radius_m: float = 0.30, max_sightings_per_tag: int = 400,
                 on_confirm: Optional[Callable[[TagEstimate], None]] = None) -> None:
        if outlier_radius_m < confirm_radius_m:
            raise ValueError("outlier_radius_m must be >= confirm_radius_m")
        self.min_sightings = min_sightings
        self.confirm_radius_m = confirm_radius_m
        self.outlier_radius_m = outlier_radius_m
        self.max_sightings_per_tag = max_sightings_per_tag
        self._on_confirm = on_confirm
        self._tracks: dict[int, _Track] = {}
        self._lock = threading.Lock()

    @classmethod
    def from_settings(cls, tags: Tags,
                      on_confirm: Optional[Callable[[TagEstimate], None]] = None) -> "TagMap":
        return cls(tags.min_sightings, tags.confirm_radius_m, tags.outlier_radius_m,
                   tags.max_sightings_per_tag, on_confirm)

    def add(self, s: Sighting) -> TagEstimate:
        """Add one sighting and return the updated estimate for its ID.

        If this sighting confirms the tag for the first time, on_confirm is
        called (outside the lock) with the new estimate.
        """
        with self._lock:
            tr = self._tracks.get(s.tag_id)
            if tr is None:
                tr = self._tracks[s.tag_id] = _Track(
                    s.tag_id, self.max_sightings_per_tag, s.t_detect)
            tr.points.append((s.x, s.y))
            pts = np.asarray(tr.points, dtype=float)
            est, inliers, support = robust_estimate(
                pts, self.confirm_radius_m, self.outlier_radius_m)
            newly = tr.confirmed_t is None and support >= self.min_sightings
            if newly:
                tr.confirmed_t = s.t_detect
                tr.confirmed_xy = (float(est[0]), float(est[1]))
            spread = float(np.median(np.hypot(*(pts[inliers] - est).T)))
            tr.estimate = TagEstimate(
                tag_id=s.tag_id, x=float(est[0]), y=float(est[1]),
                n_sightings=len(pts), n_inliers=int(inliers.sum()), support=support,
                spread_m=spread, confirmed=tr.confirmed_t is not None,
                first_seen_t=tr.first_seen_t, confirmed_t=tr.confirmed_t,
                confirmed_x=tr.confirmed_xy[0] if tr.confirmed_xy else None,
                confirmed_y=tr.confirmed_xy[1] if tr.confirmed_xy else None)
            result = tr.estimate
        if newly:
            log.info("Tag %d confirmed at (%.3f, %.3f) from %d sightings",
                     s.tag_id, result.x, result.y, support)
            if self._on_confirm is not None:
                try:
                    self._on_confirm(result)
                except Exception:
                    log.exception("on_confirm callback failed")
        return result

    def estimates(self, confirmed_only: bool = False) -> list[TagEstimate]:
        """Current estimate per ID, sorted by ID."""
        with self._lock:
            out = [tr.estimate for _, tr in sorted(self._tracks.items())
                   if tr.estimate is not None]
        return [e for e in out if e.confirmed] if confirmed_only else out

    def get(self, tag_id: int) -> Optional[TagEstimate]:
        with self._lock:
            tr = self._tracks.get(tag_id)
            return tr.estimate if tr else None

    def final_rows(self) -> list[dict]:
        """Confirmed tags as rows for MissionLog.write_tags_final()."""
        return [e.final_row() for e in self.estimates(confirmed_only=True)]
