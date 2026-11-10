"""
Localisation tags: known-position AprilTags that correct the PX4 drift in the
REPORTED tag positions after the search, and help the drone land on the box.

The drone has no absolute position: PX4 integrates optical flow from where
EKF2 started, so its frame slowly drifts (scale error, a little yaw, creep).
A few extra tags are laid at measured spots on the floor tiles:

    home tag (29)   centre of the start box: seen at takeoff, on any lane that
                    passes it, and again while hovering before landing
    far tags (27, 28)  near the far end of the search area, one per side

Their arena positions are hard-coded in config.yaml `localisation.tags`.
Arena frame: origin = centre of the home tag, x = the way the drone faces
when placed on it, y = to its right.  With the drone placed square on the
home tag this is also PX4's frame at takeoff (no compass: PX4 starts at
heading 0), so with no drift the correction is the identity.

    loc = LocTags(settings)
    loc.add(sighting)                 # camera thread, loc ids only
    fit = loc.solve()                 # after the search: LocFit
    if fit.ok:
        north, east = fit.transform.apply(x, y)    # PX4 -> arena

Nothing here is ever sent to the drone as odometry.  The correction only
changes the numbers in the final report.  The home tag is also used, live,
for precision landing (mission.py asks recent_estimate()).

Fit
---
1. Visits: per tag, sightings sorted by time are split wherever two are more
   than visit_gap_s apart.  A visit with >= min_visit_sightings sightings is a
   FIX: its dense-seeded median (tag_map.robust_estimate) in PX4 coordinates,
   paired with the tag's known arena position.  Drift grows with time, so the
   home tag seen at takeoff and at landing are two different fixes.
2. Transform  arena = s * R(theta) * px4 + t, fitted to all fixes by least
   squares (2-D Umeyama), equal weight per fix:
       similarity   s, theta, t  (4 DOF; scale = optical-flow / height error)
       rigid        s = 1        (3 DOF)
       translation  s = 1, theta = 0
   Turn / scale need the fixes' known positions to span >= min_span_m.
3. If some fix is more than max_rms_m off, one leave-one-out pass refits
   without each fix in turn and drops the one the others agree on least, if
   it is far off (> 3x the others' rms and > max_rms_m).
4. Checks: rms <= max_rms_m, |theta| <= max_rotation_deg, |s - 1| <=
   max_scale_error.  If the configured model fails, the next simpler one is
   tried; if all fail, no correction (the raw positions are reported).

One global transform is a first-order correction: drift that keeps changing
during the flight shows up as residuals in localisation.csv.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

import numpy as np

from settings import LOC_MODELS, Localisation, Settings
from tag_map import robust_estimate

TRIM_K = 3.0                  # trim a fix whose residual is > TRIM_K x the median


@dataclass(frozen=True)
class LocSighting:
    """One sighting of a localisation tag, PX4 local NED (m)."""
    tag_id: int
    t: float                  # capture time (mission clock, s)
    x: float
    y: float


@dataclass(frozen=True)
class Transform2D:
    """arena = scale * R(theta) * px4 + (tx, ty).  theta: rad, clockwise +
    in NED (x north, y east), the same sense as PX4 yaw."""
    model: str
    scale: float = 1.0
    theta: float = 0.0
    tx: float = 0.0
    ty: float = 0.0

    @property
    def matrix(self) -> np.ndarray:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return self.scale * np.array([[c, -s], [s, c]])

    def apply(self, x: float, y: float) -> tuple[float, float]:
        ax, ay = self.matrix @ np.array([x, y]) + np.array([self.tx, self.ty])
        return float(ax), float(ay)

    def apply_many(self, pts: np.ndarray) -> np.ndarray:
        return np.asarray(pts, dtype=float) @ self.matrix.T + np.array([self.tx, self.ty])

    def describe(self) -> str:
        return (f"{self.model}: turn {math.degrees(self.theta):+.2f} deg, "
                f"scale {self.scale:.4f}, shift ({self.tx:+.3f}, {self.ty:+.3f}) m")


IDENTITY = Transform2D("identity")


@dataclass(frozen=True)
class Fix:
    """One visit to a localisation tag: where PX4 put it vs where it really is."""
    tag_id: int
    visit: int                # 1, 2, ... per tag, in time order
    t_start: float
    t_end: float
    n: int                    # sightings in the visit
    px4: tuple[float, float]  # robust median, PX4 frame (m)
    ref: tuple[float, float]  # known arena position (m)
    spread_m: float           # median distance of the visit's inliers from px4


@dataclass(frozen=True)
class LocFit:
    """Result of LocTags.solve()."""
    ok: bool
    transform: Transform2D = IDENTITY
    fixes: tuple[Fix, ...] = ()
    used: tuple[bool, ...] = ()           # per fix: kept in the final fit (not trimmed)
    residuals: tuple[float, ...] = ()     # per fix: |T(px4) - ref| (m)
    rms_m: float = math.nan               # over the fixes used
    reason: str = ""                      # why it failed, or notes (fallbacks)

    @property
    def n_used(self) -> int:
        return sum(self.used)

    def summary(self) -> str:
        if not self.ok:
            return f"no correction: {self.reason}"
        tags = sorted({f.tag_id for f, u in zip(self.fixes, self.used) if u})
        note = f"; {self.reason}" if self.reason else ""
        return (f"{self.transform.describe()}, rms {self.rms_m:.3f} m over {self.n_used} "
                f"fix(es) of tag(s) {', '.join(map(str, tags))}{note}")


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------

def fit_transform(src: np.ndarray, dst: np.ndarray, model: str) -> Transform2D:
    """Least-squares dst ~ s R src + t (2-D Umeyama).

    Args:
        src, dst: (n, 2) arrays, n >= 1 (n >= 2 for rigid / similarity).
        model: one of settings.LOC_MODELS.
    """
    src, dst = np.asarray(src, dtype=float), np.asarray(dst, dtype=float)
    if model not in LOC_MODELS:
        raise ValueError(f"unknown model {model!r}")
    if len(src) == 0 or src.shape != dst.shape:
        raise ValueError("need matching, non-empty point sets")
    if model == "translation":
        tx, ty = (dst - src).mean(axis=0)
        return Transform2D(model, tx=float(tx), ty=float(ty))
    if len(src) < 2:
        raise ValueError(f"{model} needs at least 2 points")
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - ms, dst - md
    dot = float((a * b).sum())                                  # sum a . b
    cross = float((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum())  # sum a x b
    theta = math.atan2(cross, dot)
    scale = 1.0
    if model == "similarity":
        var = float((a * a).sum())
        if var <= 1e-12:
            raise ValueError("similarity needs points that are not all in one place")
        scale = math.hypot(dot, cross) / var
    t = md - Transform2D(model, scale, theta).matrix @ ms
    return Transform2D(model, scale, theta, float(t[0]), float(t[1]))


def _span(pts: np.ndarray) -> float:
    """Largest distance between any two points (m)."""
    if len(pts) < 2:
        return 0.0
    d = pts[:, None, :] - pts[None, :, :]
    return float(np.hypot(d[..., 0], d[..., 1]).max())


def _check_fit(t: Transform2D, rms: float, lo: Localisation) -> Optional[str]:
    if rms > lo.max_rms_m:
        return f"rms {rms:.3f} m > max_rms_m {lo.max_rms_m}"
    if abs(math.degrees(t.theta)) > lo.max_rotation_deg:
        return f"turn {math.degrees(t.theta):+.1f} deg > max_rotation_deg {lo.max_rotation_deg}"
    if abs(t.scale - 1.0) > lo.max_scale_error:
        return f"scale {t.scale:.3f} outside 1 +- max_scale_error {lo.max_scale_error}"
    return None


def _trim_one(src: np.ndarray, dst: np.ndarray, model: str,
              lo: Localisation) -> Optional[tuple[int, Transform2D]]:
    """Leave-one-out: the fix that the others agree on least, and the fit
    without it, if it is far off (> TRIM_K x the others' rms and > max_rms_m)."""
    best = None
    for i in range(len(src)):
        keep = np.arange(len(src)) != i
        if model != "translation" and _span(dst[keep]) < lo.min_span_m:
            continue
        t = fit_transform(src[keep], dst[keep], model)
        res = np.hypot(*(t.apply_many(src) - dst).T)
        rms_rest = float(np.sqrt(np.mean(res[keep] ** 2)))
        if best is None or rms_rest < best[0]:
            best = (rms_rest, i, t, float(res[i]))
    if best is None:
        return None
    rms_rest, i, t, off = best
    if off > max(TRIM_K * max(rms_rest, 0.01), lo.max_rms_m):
        return i, t
    return None


def solve_fixes(fixes: Sequence[Fix], lo: Localisation) -> LocFit:
    """Fit the PX4 -> arena transform to the fixes (see the module docstring)."""
    fixes = tuple(fixes)
    if not fixes:
        return LocFit(False, reason="no localisation tag was seen often enough "
                                    f"(min_visit_sightings {lo.min_visit_sightings})")
    src = np.array([f.px4 for f in fixes])
    dst = np.array([f.ref for f in fixes])
    notes: list[str] = []
    for model in LOC_MODELS[LOC_MODELS.index(lo.model):]:
        need = 1 if model == "translation" else 2
        if model != "translation" and _span(dst) < lo.min_span_m:
            notes.append(f"{model}: known positions span {_span(dst):.2f} m "
                         f"< min_span_m {lo.min_span_m}")
            continue
        used = np.ones(len(fixes), dtype=bool)
        t = fit_transform(src, dst, model)
        res = np.hypot(*(t.apply_many(src) - dst).T)
        if len(fixes) > need + 1 and res.max() > lo.max_rms_m:
            trimmed = _trim_one(src, dst, model, lo)            # one trimming pass
            if trimmed is not None:
                worst, t = trimmed
                used[worst] = False
                res = np.hypot(*(t.apply_many(src) - dst).T)
                f = fixes[worst]
                notes.append(f"trimmed tag {f.tag_id} visit {f.visit} "
                             f"({res[worst]:.2f} m off)")
        rms = float(np.sqrt(np.mean(res[used] ** 2)))
        problem = _check_fit(t, rms, lo)
        if problem is None:
            return LocFit(True, t, fixes, tuple(map(bool, used)), tuple(map(float, res)),
                          rms, "; ".join(notes))
        notes.append(f"{model}: {problem}")
    return LocFit(False, fixes=fixes, reason="; ".join(notes))


# ---------------------------------------------------------------------------
# Visits
# ---------------------------------------------------------------------------

def find_fixes(sightings: Iterable[LocSighting], lo: Localisation,
               confirm_radius_m: float, outlier_radius_m: float) -> list[Fix]:
    """Split each tag's sightings into visits and turn each big-enough visit
    into a Fix.  Ids not in lo.tags are ignored."""
    by_id: dict[int, list[LocSighting]] = {}
    for s in sightings:
        if lo.ref(s.tag_id) is not None:
            by_id.setdefault(s.tag_id, []).append(s)
    fixes: list[Fix] = []
    for tag_id in sorted(by_id):
        rows = sorted(by_id[tag_id], key=lambda s: s.t)
        groups: list[list[LocSighting]] = [[rows[0]]]
        for prev, cur in zip(rows, rows[1:]):
            if cur.t - prev.t > lo.visit_gap_s:
                groups.append([])
            groups[-1].append(cur)
        visit = 0
        for g in groups:
            if len(g) < lo.min_visit_sightings:
                continue
            pts = np.array([(s.x, s.y) for s in g])
            est, inliers, _ = robust_estimate(pts, confirm_radius_m, outlier_radius_m)
            visit += 1
            spread = float(np.median(np.hypot(*(pts[inliers] - est).T)))
            fixes.append(Fix(tag_id, visit, g[0].t, g[-1].t, len(g),
                             (float(est[0]), float(est[1])), lo.ref(tag_id), spread))
    fixes.sort(key=lambda f: f.t_start)
    return fixes


# ---------------------------------------------------------------------------
# Live store
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LocMark:
    """For the live map: a localisation tag's latest seen spot and its known one."""
    tag_id: int
    x: Optional[float]        # PX4 frame, latest visit (None: not seen yet)
    y: Optional[float]
    ref_x: float              # arena frame (= PX4 frame with no drift)
    ref_y: float


class LocTags:
    """Thread-safe store of localisation-tag sightings (camera thread adds,
    mission thread reads)."""

    def __init__(self, settings: Settings) -> None:
        self.cfg = settings.localisation
        self.confirm_radius_m = settings.tags.confirm_radius_m
        self.outlier_radius_m = settings.tags.outlier_radius_m
        self._rows: list[LocSighting] = []
        self._lock = threading.Lock()

    @property
    def ids(self) -> frozenset[int]:
        return self.cfg.ids

    def wants(self, tag_id: int) -> bool:
        """True for a localisation tag id (always False when disabled)."""
        return tag_id in self.cfg.ids

    def add(self, s) -> None:
        """Store a Sighting (or anything with tag_id, t_capture, x, y)."""
        with self._lock:
            self._rows.append(LocSighting(s.tag_id, s.t_capture, s.x, s.y))

    def sightings(self, tag_id: Optional[int] = None) -> list[LocSighting]:
        with self._lock:
            return [s for s in self._rows if tag_id is None or s.tag_id == tag_id]

    def count(self, tag_id: Optional[int] = None) -> int:
        return len(self.sightings(tag_id))

    def recent_estimate(self, tag_id: int, since_t: float, window_s: float,
                        min_n: int) -> Optional[tuple[float, float, int]]:
        """Robust (x, y, n) from sightings captured after since_t and within
        window_s of the newest one, or None if there are fewer than min_n."""
        rows = [s for s in self.sightings(tag_id) if s.t >= since_t]
        if not rows:
            return None
        newest = max(s.t for s in rows)
        rows = [s for s in rows if s.t >= newest - window_s]
        if len(rows) < min_n:
            return None
        est, _, _ = robust_estimate(np.array([(s.x, s.y) for s in rows]),
                                    self.confirm_radius_m, self.outlier_radius_m)
        return float(est[0]), float(est[1]), len(rows)

    def marks(self) -> tuple[LocMark, ...]:
        """Latest visit of every localisation tag, for the live map."""
        out = []
        for ref in self.cfg.tags if self.cfg.enabled else ():
            rows = self.sightings(ref.id)
            x = y = None
            if rows:
                newest = rows[-1].t
                recent = [s for s in rows if s.t >= newest - self.cfg.visit_gap_s][-60:]
                est, _, _ = robust_estimate(np.array([(s.x, s.y) for s in recent]),
                                            self.confirm_radius_m, self.outlier_radius_m)
                x, y = float(est[0]), float(est[1])
            out.append(LocMark(ref.id, x, y, ref.x, ref.y))
        return tuple(out)

    def fixes(self) -> list[Fix]:
        return find_fixes(self.sightings(), self.cfg, self.confirm_radius_m,
                          self.outlier_radius_m)

    def solve(self) -> LocFit:
        return solve_fixes(self.fixes(), self.cfg)


# ---------------------------------------------------------------------------
# Applying it
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CorrectedTag:
    tag_id: int
    x: float                  # arena frame (m); = raw when there is no fit
    y: float
    raw_x: float              # PX4 frame (m), as in tags_final.csv
    raw_y: float
    n_sightings: int

    @property
    def shift_m(self) -> float:
        return math.hypot(self.x - self.raw_x, self.y - self.raw_y)


def correct(fit: LocFit, tags: Iterable) -> list[CorrectedTag]:
    """Apply the fit to TagEstimates (anything with tag_id, x, y, n_inliers)."""
    t = fit.transform if fit.ok else IDENTITY
    out = []
    for e in tags:
        x, y = t.apply(e.x, e.y)
        out.append(CorrectedTag(e.tag_id, x, y, e.x, e.y, e.n_inliers))
    return out


LOC_FIX_FIELDS = ["tag_id", "visit", "start_mission_s", "end_mission_s", "n_sightings",
                  "px4_x", "px4_y", "ref_x", "ref_y", "fit_x", "fit_y", "residual_m",
                  "used"]

TAGS_CORRECTED_FIELDS = ["tag_id", "x", "y", "raw_x", "raw_y", "shift_m", "n_sightings"]


def fix_rows(fit: LocFit, mission_s=lambda t: t) -> list[dict]:
    """Rows for localisation.csv.  mission_s converts a fix time for display."""
    t = fit.transform if fit.ok else IDENTITY
    rows = []
    for i, f in enumerate(fit.fixes):
        fx, fy = t.apply(*f.px4)
        rows.append({"tag_id": f.tag_id, "visit": f.visit,
                     "start_mission_s": mission_s(f.t_start),
                     "end_mission_s": mission_s(f.t_end), "n_sightings": f.n,
                     "px4_x": f.px4[0], "px4_y": f.px4[1],
                     "ref_x": float(f.ref[0]), "ref_y": float(f.ref[1]),
                     "fit_x": fx, "fit_y": fy,
                     "residual_m": math.hypot(fx - f.ref[0], fy - f.ref[1]),
                     "used": (fit.used[i] if fit.used else False)})
    return rows


def corrected_rows(tags: Iterable[CorrectedTag]) -> list[dict]:
    """Rows for tags_corrected.csv."""
    return [{"tag_id": c.tag_id, "x": c.x, "y": c.y, "raw_x": c.raw_x, "raw_y": c.raw_y,
             "shift_m": c.shift_m, "n_sightings": c.n_sightings} for c in tags]


def format_report(fit: LocFit, tags: Sequence[CorrectedTag],
                  mission_s=lambda t: t) -> str:
    """Text block for the terminal: the fit, its fixes and the corrected tags."""
    lines = ["=== LOCALISATION  (PX4 frame -> arena frame from the localisation tags) ===",
             ("fit " if fit.ok else "") + fit.summary()]
    if fit.fixes:
        lines.append(f"{'tag':>4} {'visit':>5} {'T+ s':>7} {'n':>4} {'px4 x':>7} {'px4 y':>7} "
                     f"{'true x':>7} {'true y':>7} {'resid':>6}")
        for r in fix_rows(fit, mission_s):
            mark = "" if r["used"] or not fit.ok else "  (not used)"
            lines.append(f"{r['tag_id']:>4} {r['visit']:>5} {(r['start_mission_s'] or 0.0):>7.1f} "
                         f"{r['n_sightings']:>4} {r['px4_x']:>7.3f} {r['px4_y']:>7.3f} "
                         f"{r['ref_x']:>7.3f} {r['ref_y']:>7.3f} {r['residual_m']:>6.3f}{mark}")
    title = "CORRECTED TAG REPORT (arena frame, m)" if fit.ok else \
        "TAG REPORT, NOT CORRECTED (PX4 frame, m)"
    lines.append(f"=== {title} ===")
    lines.append(f"{'ID':>4} {'x':>8} {'y':>8} {'raw x':>8} {'raw y':>8} {'moved':>6}")
    for c in tags:
        lines.append(f"{c.tag_id:>4} {c.x:>8.3f} {c.y:>8.3f} {c.raw_x:>8.3f} {c.raw_y:>8.3f} "
                     f"{c.shift_m:>6.3f}")
    if not tags:
        lines.append("   (no tags confirmed)")
    return "\n".join(lines)
