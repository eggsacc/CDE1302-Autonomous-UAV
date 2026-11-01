"""
UWB -> PX4 alignment: find the 2-D transform that maps LinkTrack positions
(anchor frame: A0 origin, X toward A3) into PX4's local NED frame (origin at
takeoff, no compass, so "North" is just PX4's initial heading).

    est = AlignmentEstimator.from_settings(settings.uwb)
    est.add(AlignSample(t, uwb_x, uwb_y, px4_x, px4_y, has_fix, eop_max))  # matched pairs
    result = est.solve()
    if result.ok:
        north, east = result.alignment.to_px4(uwb_x, uwb_y)

Model:  p = R(theta) @ M @ u + t,  M = identity or a mirror (y -> -y).

Samples are UWB/PX4 positions recorded at the same moment on the drone.
The fit is a weighted 2-D Procrustes (rotation + translation, no scaling,
both in metres), with one outlier-trimming pass.

The mirror: the LinkTrack manual defines X and Z but not the handedness, so a
mirrored Y axis is possible.  A straight-line path cannot tell the two apart
(both reflections fit perfectly), so with uwb.mirror = auto the estimator
only decides once the path has a turn (spread ratio), and only if the other
choice fits clearly worse (mirror margin).  Set uwb.mirror to yes/no once
the anchor test has shown it; then a straight move is enough.

Everything here runs on the laptop; the drone only applies the result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from settings import Uwb

MIRROR = np.array([[1.0, 0.0], [0.0, -1.0]])
MIN_RESIDUAL_FLOOR_M = 0.01     # never trim/compare below 1 cm (UWB noise floor)


@dataclass(frozen=True)
class AlignSample:
    """One UWB position and the PX4 position at the same instant."""
    t: float            # drone time (s)
    ux: float           # UWB x, anchor frame (m)
    uy: float           # UWB y, anchor frame (m)
    px: float           # PX4 local x / north (m)
    py: float           # PX4 local y / east (m)
    has_fix: bool = True
    eop: float = 0.0    # largest of the module's eop.x/y/z (m)


@dataclass(frozen=True)
class Alignment:
    """p = R(theta) @ M @ u + t, with M = mirror if `mirror`."""
    theta: float                     # rad
    mirror: bool
    tx: float                        # m
    ty: float                        # m

    @property
    def matrix(self) -> np.ndarray:
        """2x2 R @ M."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        r = np.array([[c, -s], [s, c]])
        return r @ MIRROR if self.mirror else r

    def to_px4(self, ux: float, uy: float) -> tuple[float, float]:
        x, y = self.matrix @ np.array([ux, uy]) + np.array([self.tx, self.ty])
        return float(x), float(y)

    def to_uwb(self, px: float, py: float) -> tuple[float, float]:
        x, y = self.matrix.T @ (np.array([px, py]) - np.array([self.tx, self.ty]))
        return float(x), float(y)


@dataclass(frozen=True)
class FitDiag:
    mirror: bool
    rms: float                       # m, after trimming
    n_used: int


@dataclass(frozen=True)
class AlignResult:
    ok: bool
    reason: str                      # "" when ok, else why not (for logs / the map)
    alignment: Optional[Alignment] = None
    rms_m: Optional[float] = None
    n_valid: int = 0                 # samples with a fix and good precision
    n_used: int = 0                  # after outlier trimming
    span_m: float = 0.0              # path extent along its main direction
    spread_ratio: float = 0.0        # minor/major eigenvalue: 0 = straight line
    other_rms_m: Optional[float] = None   # mirror=auto: rms of the rejected choice


def fit(u: np.ndarray, p: np.ndarray, w: np.ndarray, mirror: bool) -> tuple[Alignment, np.ndarray]:
    """Weighted least-squares rotation + translation with a fixed mirror choice.

    Returns the alignment and the per-sample residual distances (m).
    """
    um = u @ MIRROR.T if mirror else u
    ws = w / w.sum()
    cu, cp = ws @ um, ws @ p
    a, b = um - cu, p - cp
    cross = np.sum(ws * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    dot = np.sum(ws * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s], [s, c]])
    t = cp - r @ cu
    al = Alignment(theta=theta, mirror=mirror, tx=float(t[0]), ty=float(t[1]))
    res = np.hypot(*(um @ r.T + t - p).T)
    return al, res


def robust_fit(u: np.ndarray, p: np.ndarray, w: np.ndarray, mirror: bool,
               k: float) -> tuple[Alignment, float, int]:
    """fit(), then drop residuals > k x median and fit again.  Returns (alignment, rms, n_used)."""
    al, res = fit(u, p, w, mirror)
    keep = res <= max(k * float(np.median(res)), MIN_RESIDUAL_FLOOR_M)
    if keep.sum() >= 3 and not keep.all():
        al, res2 = fit(u[keep], p[keep], w[keep], mirror)
        rms = float(np.sqrt(np.average(res2 ** 2, weights=w[keep])))
        return al, rms, int(keep.sum())
    return al, float(np.sqrt(np.average(res ** 2, weights=w))), len(u)


def path_shape(u: np.ndarray) -> tuple[float, float]:
    """(extent along the main direction in m, minor/major eigenvalue ratio).

    The extent is measured between the 1st and 99th percentiles, so noise or
    a single wild sample cannot make a short path look long enough.
    """
    centred = u - u.mean(axis=0)
    evals, evecs = np.linalg.eigh(np.cov(centred.T) if len(u) > 1 else np.eye(2))
    proj = centred @ evecs[:, -1]
    lo, hi = np.percentile(proj, [1, 99])
    ratio = float(evals[0] / evals[-1]) if evals[-1] > 1e-12 else 0.0
    return float(hi - lo), max(ratio, 0.0)


class AlignmentEstimator:
    """Collects matched samples and solves when the data is good enough."""

    def __init__(self, mirror: str = "auto", min_samples: int = 50, min_span_m: float = 0.8,
                 min_spread_ratio: float = 0.05, max_eop_m: float = 0.30,
                 max_rms_m: float = 0.10, mirror_margin: float = 2.0,
                 outlier_k: float = 3.0) -> None:
        if mirror not in ("auto", "yes", "no"):
            raise ValueError("mirror must be 'auto', 'yes' or 'no'")
        self.mirror = mirror
        self.min_samples = min_samples
        self.min_span_m = min_span_m
        self.min_spread_ratio = min_spread_ratio
        self.max_eop_m = max_eop_m
        self.max_rms_m = max_rms_m
        self.mirror_margin = mirror_margin
        self.outlier_k = outlier_k
        self.samples: list[AlignSample] = []

    @classmethod
    def from_settings(cls, u: Uwb) -> "AlignmentEstimator":
        return cls(u.mirror, u.align_min_samples, u.align_min_span_m,
                   u.align_min_spread_ratio, u.align_max_eop_m, u.align_max_rms_m,
                   u.align_mirror_margin, u.align_outlier_k)

    def add(self, s: AlignSample) -> None:
        self.samples.append(s)

    def clear(self) -> None:
        self.samples.clear()

    def solve(self) -> AlignResult:
        good = [s for s in self.samples
                if s.has_fix and s.eop <= self.max_eop_m
                and all(math.isfinite(v) for v in (s.ux, s.uy, s.px, s.py))]
        n = len(good)
        if n < self.min_samples:
            return AlignResult(False, f"{n}/{self.min_samples} usable samples", n_valid=n)
        u = np.array([(s.ux, s.uy) for s in good])
        p = np.array([(s.px, s.py) for s in good])
        # Weight by precision: 1 / max(eop, 2 cm)^2.
        w = 1.0 / np.maximum([s.eop for s in good], 0.02) ** 2
        span, spread = path_shape(u)
        base = dict(n_valid=n, span_m=span, spread_ratio=spread)
        if span < self.min_span_m:
            return AlignResult(False, f"path only {span:.2f} m long, need "
                               f"{self.min_span_m:.2f} m", **base)

        choices = [False, True] if self.mirror == "auto" else [self.mirror == "yes"]
        fits = sorted((robust_fit(u, p, w, m, self.outlier_k) for m in choices),
                      key=lambda f: f[1])
        best, best_rms, used = fits[0]
        base.update(alignment=best, rms_m=best_rms, n_used=used)
        if self.mirror == "auto":
            other_rms = fits[1][1]
            base["other_rms_m"] = other_rms
            if spread < self.min_spread_ratio:
                return AlignResult(False, "path too straight to tell a mirrored axis: "
                                   "needs a turn (or set uwb.mirror)", **base)
            if other_rms < self.mirror_margin * max(best_rms, MIN_RESIDUAL_FLOOR_M):
                return AlignResult(False, f"mirror unclear: {best_rms:.3f} vs "
                                   f"{other_rms:.3f} m rms", **base)
        if best_rms > self.max_rms_m:
            return AlignResult(False, f"fit too poor: {best_rms:.3f} m rms > "
                               f"{self.max_rms_m:.3f} m", **base)
        return AlignResult(True, "", **base)
