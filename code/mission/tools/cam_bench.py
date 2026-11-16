#!/usr/bin/env python3
"""
Camera bench calibration: how far off are the tag positions, and why.

    python mission/tools/cam_bench.py               # drone on, laptop on its Wi-Fi
    python mission/tools/cam_bench.py --config other.yaml

No telemetry and no flying: it only streams the camera (props off anyway; close
mission.py, tag_stream.py and other camera viewers first).  Put the drone on a
stand, camera looking down, LEVEL, and put a tag at known spots under it.  Or
hold a tag at a known spot.  At each spot, type where the tag really is; the
tool averages the last 2 s of detections.  After 4-6 spots spread over the
view (centre, forward, back, left, right, a corner), `fit` says:

    tag size      sideways offsets scale only with the tag size
    focal lengths depth also scales with the lens model (camera.fx / fx);
                  forward vs right scale differently if fx and fy disagree
    lens position where the lens really sits (tags.camera_forward_m / _right_m)
                  (each tag read at its own size: tags.tag_size_m for search tags,
                  localisation.tag_size_m for localisation tags; `fit` corrects the
                  size of whichever kind you captured)
    camera yaw    a camera twisted on its mount (reported; not corrected)

and `apply` writes the suggested tag_size_m / camera_forward_m / camera_right_m
/ camera.fx / camera.fy into config.yaml (config.yaml.bak keeps the old one).

Positions are measured from the point on the floor straight below the drone's
CENTRE (the point the mission reports positions for), in metres: forward (the
way the drone faces), right, and depth = height of the lens above the tag.
With the floor grid: stand the drone centre over a node, and `n DA DB` means
the tag is DA nodes to the right and DB nodes forward of it (0.46 m each).

Commands (Enter alone = show the current reading):
    h <depth>                 lens height above the tag plane (m), for the next captures
    c <fwd> <right> [depth]   capture: the tag is here (m); the only tag in view, or
    n <da> <db>                 the tag picked with `t <id>`; n = in grid nodes
    t <id>                    only use this tag id
    w [seconds]               watch the live reading (default 5 s)
    list | undo | fit | apply | save [file] | q
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

import laptop_path  # noqa: E402,F401
from apriltag_host import Intrinsics, passes_gate  # noqa: E402
from settings import DEFAULT_CONFIG_PATH, Settings, load_settings  # noqa: E402

NODE_M = 0.46
CAPTURE_S = 2.0          # average this much of the newest detections
MIN_SAMPLES = 5


# ---------------------------------------------------------------------------
# Stations and the fit (no camera needed: tested on its own)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Station:
    """One capture: where the tag really is, and what the camera measured."""
    fwd: float                      # true, from below the drone centre (m)
    right: float
    depth: float                    # lens height above the tag plane (m)
    cam: tuple[float, float, float]  # median Detection.t: right, down(=back), out of lens (m)
    n: int
    spread_m: float
    tag_id: int


@dataclass(frozen=True)
class BenchFit:
    n: int
    k_right: float                  # measured / true, sideways, camera right axis
    k_fwd: float                    # ... camera forward axis
    k_depth: float                  # ... depth
    yaw_deg: float                  # camera turned clockwise on its mount
    lens_fwd: float                 # where the lens really is (m), body frame
    lens_right: float
    tag_size_m: float               # real tag size implied (for the configured cfg_size)
    fx: float                       # real focal lengths implied (px)
    fy: float
    rms_m: float                    # sideways residual after the fit
    size_scale: float = 1.0         # real / configured size, for any kind of tag

    def summary(self, cfg_size: float, cfg_fwd: float, cfg_right: float,
                intr: Intrinsics) -> str:
        pct = lambda k: f"{(k - 1) * 100:+.1f} %"                    # noqa: E731
        lines = [
            f"{self.n} captures, residual {self.rms_m * 100:.1f} cm after the fit",
            f"  distances read: sideways right {pct(self.k_right)}, forward {pct(self.k_fwd)}, "
            f"depth {pct(self.k_depth)}  (0 % = right)",
            f"  tag size      {cfg_size:.4f} m in config -> {self.tag_size_m:.4f} m implied "
            f"(compare with your ruler)",
            f"  focal lengths fx {intr.fx:.1f} -> {self.fx:.1f}, fy {intr.fy:.1f} -> {self.fy:.1f} px",
            f"  lens position forward {cfg_fwd:+.3f} -> {self.lens_fwd:+.3f} m, "
            f"right {cfg_right:+.3f} -> {self.lens_right:+.3f} m",
            f"  camera yaw    {self.yaw_deg:+.1f} deg (twisted on its mount; not corrected in "
            f"software: {'re-mount the camera' if abs(self.yaw_deg) > 2 else 'fine'})",
        ]
        if self.n < 4:
            lines.append("  ! fewer than 4 captures: treat these numbers as rough")
        return "\n".join(lines)


def fit_stations(stations: list[Station], cfg_size: float,
                 intr: Intrinsics) -> BenchFit:
    """Fit measured sideways = A @ (true - lens), depth = k * true depth.

    Camera looking straight down: camera x = body right, camera -y = body
    forward, camera z = down.  A = diag(k_right, k_fwd) @ R(yaw).  With s the
    tag-size ratio (config / real) and a = fx_cfg / fx_real, b = fy_cfg / fy_real:
    k_right = s * sqrt(b / a), k_fwd = s * sqrt(a / b), k_depth = s * sqrt(a * b).
    """
    if len(stations) < 3:
        raise ValueError("need at least 3 captures, spread over the view")
    true = np.array([[st.right, st.fwd] for st in stations])
    meas = np.array([[st.cam[0], -st.cam[1]] for st in stations])
    if np.linalg.matrix_rank(np.c_[true, np.ones(len(true))]) < 3:
        raise ValueError("the captures are all in a line: add some off to the side")
    X = np.c_[true, np.ones(len(true))]                       # affine least squares
    coef, *_ = np.linalg.lstsq(X, meas, rcond=None)
    A, b = coef[:2].T, coef[2]
    lens = -np.linalg.solve(A, b)                             # (right, fwd)
    res = meas - (true @ A.T + b)
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    # A ~ diag(kr, kf) @ R: rows' lengths are the scales, the rotation from the mean angle
    k_right, k_fwd = float(np.hypot(*A[0])), float(np.hypot(*A[1]))
    yaw = math.atan2(A[1, 0] / k_fwd - A[0, 1] / k_right, A[0, 0] / k_right + A[1, 1] / k_fwd)
    depth_ratio = np.array([st.cam[2] / st.depth for st in stations])
    k_depth = float(np.median(depth_ratio))
    s = math.sqrt(k_right * k_fwd)
    ab = (k_depth / s) ** 2
    a = math.sqrt(ab * k_fwd / k_right)
    bb = math.sqrt(ab * k_right / k_fwd)
    return BenchFit(len(stations), k_right, k_fwd, k_depth, math.degrees(yaw),
                    float(lens[1]), float(lens[0]), cfg_size / s,
                    intr.fx / a, intr.fy / bb, rms, 1.0 / s)


# ---------------------------------------------------------------------------
# config.yaml
# ---------------------------------------------------------------------------

def set_config_value(text: str, section: str, key: str, value: float) -> str:
    """Rewrite `key: value` inside `section:`, keeping its comment."""
    lines = text.splitlines(keepends=True)
    start = next(i for i, ln in enumerate(lines) if ln.startswith(f"{section}:"))
    for i in range(start + 1, len(lines)):
        ln = lines[i]
        if ln.strip() and not ln.startswith((" ", "#")):
            break
        m = re.match(rf"(\s+{key}:\s*)([^#\s]+)(.*)", ln, re.S)
        if m:
            lines[i] = f"{m.group(1)}{value}{m.group(3)}"
            return "".join(lines)
    raise KeyError(f"{section}.{key} not found in the config")


def apply_fit(path: Path, fit: BenchFit, kinds: Optional[set] = None) -> Path:
    """Write the fit.  kinds: which tags were captured ("search", "loc"); each
    kind's size is scaled by fit.size_scale (default: the search tags)."""
    from settings import settings_from_dict
    import yaml
    text = path.read_text(encoding="utf-8")
    s = load_settings(path)
    kinds = kinds or {"search"}
    sizes = []
    if "search" in kinds:
        sizes.append(("tags", "tag_size_m", round(s.tags.tag_size_m * fit.size_scale, 4)))
    if "loc" in kinds:
        loc_size = s.localisation.tag_size_m or s.tags.tag_size_m
        sizes.append(("localisation", "tag_size_m", round(loc_size * fit.size_scale, 4)))
    for section, key, value in (*sizes,
                                ("tags", "camera_forward_m", round(fit.lens_fwd, 3)),
                                ("tags", "camera_right_m", round(fit.lens_right, 3)),
                                ("camera", "fx", round(fit.fx, 2)),
                                ("camera", "fy", round(fit.fy, 2))):
        text = set_config_value(text, section, key, value)
    settings_from_dict(yaml.safe_load(text))               # must still load
    shutil.copyfile(path, path.with_name(path.name + ".bak"))
    path.write_text(text, encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# Live camera
# ---------------------------------------------------------------------------

def detector_intrinsics(settings: Settings) -> Intrinsics:
    """The lens model the mission uses (tag_pipeline.camera_intrinsics)."""
    from tag_pipeline import camera_intrinsics
    return camera_intrinsics(settings)


class Collector:
    """Keeps the newest detections (time, Detection), thread-safe."""

    def __init__(self, keep_s: float = 10.0) -> None:
        self._rows: deque = deque()
        self._lock = threading.Lock()
        self._keep_s = keep_s

    def add(self, t: float, det) -> None:
        with self._lock:
            self._rows.append((t, det))
            while self._rows and t - self._rows[0][0] > self._keep_s:
                self._rows.popleft()

    def recent(self, window_s: float, tag_id: Optional[int] = None) -> tuple[Optional[int], list]:
        """(tag id, detections of it in the last window_s); with no id given,
        the only tag seen (None if several or none)."""
        now = time.monotonic()
        with self._lock:
            rows = [d for t, d in self._rows if now - t <= window_s]
        ids = {d.id for d in rows}
        if tag_id is None:
            if len(ids) != 1:
                return None, []
            tag_id = ids.pop()
        return tag_id, [d for d in rows if d.id == tag_id]


def summarise(dets: list, scales: Optional[dict] = None
              ) -> tuple[tuple[float, float, float], float]:
    """Median camera-frame position (each tag at its own printed size) and spread."""
    scales = scales or {}
    t = np.array([[v * scales.get(d.id, 1.0) for v in d.t] for d in dets])
    med = np.median(t, axis=0)
    return (float(med[0]), float(med[1]), float(med[2])), \
        float(np.median(np.linalg.norm(t - med, axis=1)))


def start_camera(settings: Settings, collector: Collector):
    """The mission's own TagPipeline, feeding raw gate-passing detections."""
    import cv2
    from apriltag_host import Detector
    from tag_pipeline import TagPipeline

    class BenchPipeline(TagPipeline):
        def _handle_frame(self, jpeg, first_rx, age_ms):
            gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                return
            with self._stats_lock:
                self._stats.frames += 1
            for det in self._detector.detect(gray):
                if det.t is not None and passes_gate(det, self.settings.tags.min_decision_margin,
                                                     self.settings.tags.max_hamming):
                    collector.add(time.monotonic(), det)

    def factory(**kw):
        return Detector(intrinsics=detector_intrinsics(settings), **kw)

    p = BenchPipeline(settings, poses=None, esp_ip=lambda: settings.esp_ip,
                      on_sighting=lambda s: None, detector_factory=factory)
    p.start()
    return p


def body_reading(cam, settings: Settings) -> tuple[float, float, float]:
    from camera_mount import camera_to_body
    t = settings.tags
    return camera_to_body(cam, t.camera_pitch_deg, t.camera_pos_m)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Camera bench calibration (see the module docs).")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    args = ap.parse_args(argv)
    path = Path(args.config)
    settings = load_settings(path)
    intr, t = detector_intrinsics(settings), settings.tags
    from tag_pipeline import size_scales
    scales = size_scales(t, settings.localisation)
    loc_ids = settings.localisation.ids
    if abs(t.camera_pitch_deg - 90.0) > 1e-6:
        print("note: tags.camera_pitch_deg is not 90; this bench assumes a camera looking "
              "straight down")
    collector = Collector()
    pipe = start_camera(settings, collector)
    print(__doc__.split("Commands")[1].strip().split("\n", 1)[1])
    print(f"\nCamera from {settings.esp_ip}; config: tag {t.tag_size_m} m, lens "
          f"fwd {t.camera_forward_m} right {t.camera_right_m}, fx {intr.fx:.1f} fy {intr.fy:.1f}")
    depth: Optional[float] = None
    only: Optional[int] = None
    stations: list[Station] = []
    fit: Optional[BenchFit] = None

    def show() -> None:
        tid, dets = collector.recent(CAPTURE_S, only)
        if not dets:
            print("  no tag (or several: pick one with `t <id>`); "
                  f"{pipe.stats().frames} frames so far")
            return
        cam, spread = summarise(dets, scales)
        f, r, d = body_reading(cam, settings)
        print(f"  tag {tid} ({'localisation' if tid in loc_ids else 'search'}): "
              f"{len(dets)} in {CAPTURE_S:.0f} s  fwd {f:+.3f}  right {r:+.3f}  "
              f"down {d:.3f} m   (spread {spread * 100:.1f} cm, margin "
              f"{np.median([x.decision_margin for x in dets]):.0f})")

    try:
        while True:
            try:
                line = input("bench> ").strip()
            except EOFError:
                break
            parts = line.split()
            cmd = parts[0].lower() if parts else ""
            try:
                if cmd == "":
                    show()
                elif cmd == "h":
                    depth = float(parts[1])
                    print(f"  depth (lens above the tag) = {depth:.3f} m")
                elif cmd == "t":
                    only = int(parts[1]) if len(parts) > 1 else None
                    print(f"  using tag {only}" if only is not None else "  any single tag")
                elif cmd == "w":
                    end = time.monotonic() + (float(parts[1]) if len(parts) > 1 else 5.0)
                    while time.monotonic() < end:
                        show()
                        time.sleep(1.0)
                elif cmd in ("c", "n"):
                    a, b_ = float(parts[1]), float(parts[2])
                    fwd, right = (b_ * NODE_M, a * NODE_M) if cmd == "n" else (a, b_)
                    d = float(parts[3]) if cmd == "c" and len(parts) > 3 else depth
                    if d is None:
                        print("  set the depth first: h <metres>")
                        continue
                    tid, dets = collector.recent(CAPTURE_S, only)
                    if len(dets) < MIN_SAMPLES:
                        print(f"  only {len(dets)} detections in the last {CAPTURE_S:.0f} s: "
                              f"hold it there a moment and try again")
                        continue
                    cam, spread = summarise(dets, scales)
                    stations.append(Station(fwd, right, d, cam, len(dets), spread, tid))
                    f, r, dd = body_reading(cam, settings)
                    print(f"  #{len(stations)} tag {tid}: true fwd {fwd:+.3f} right {right:+.3f} "
                          f"depth {d:.3f} | measured {f:+.3f} {r:+.3f} {dd:.3f} | error "
                          f"{math.hypot(f - fwd, r - right) * 100:.1f} cm sideways, "
                          f"{(dd - d) * 100:+.1f} cm depth")
                elif cmd == "list":
                    for i, st in enumerate(stations, 1):
                        f, r, dd = body_reading(st.cam, settings)
                        print(f"  #{i} tag {st.tag_id}: true ({st.fwd:+.3f}, {st.right:+.3f}, "
                              f"{st.depth:.3f}) measured ({f:+.3f}, {r:+.3f}, {dd:.3f}) n {st.n}")
                elif cmd == "undo" and stations:
                    stations.pop()
                    print(f"  removed; {len(stations)} left")
                elif cmd == "fit":
                    fit = fit_stations(stations, t.tag_size_m, intr)
                    kinds = {"loc" if st.tag_id in loc_ids else "search" for st in stations}
                    print(fit.summary(t.tag_size_m, t.camera_forward_m, t.camera_right_m, intr))
                    loc_size = settings.localisation.tag_size_m or t.tag_size_m
                    print(f"  sizes: tags read {fit.size_scale:.3f} x their configured size -> "
                          + ", ".join(
                              ([f"search {t.tag_size_m:.4f} -> {t.tag_size_m * fit.size_scale:.4f} m"]
                               if "search" in kinds else []) +
                              ([f"localisation {loc_size:.4f} -> {loc_size * fit.size_scale:.4f} m"]
                               if "loc" in kinds else [])))
                elif cmd == "apply":
                    if fit is None:
                        print("  run `fit` first")
                        continue
                    apply_fit(path, fit, kinds)
                    print(f"  written to {path} (old one: {path.name}.bak). Restart this tool "
                          f"or the mission to use it.")
                elif cmd == "save":
                    out = Path(parts[1]) if len(parts) > 1 else Path(f"cam_bench_{time.strftime('%Y%m%d_%H%M%S')}.csv")
                    with open(out, "w", newline="", encoding="utf-8") as fh:
                        w = csv.writer(fh)
                        w.writerow(["tag_id", "true_fwd", "true_right", "depth", "cam_x", "cam_y",
                                    "cam_z", "n", "spread_m"])
                        for st in stations:
                            w.writerow([st.tag_id, st.fwd, st.right, st.depth, *st.cam, st.n,
                                        st.spread_m])
                    print(f"  saved {out}")
                elif cmd in ("q", "quit", "exit"):
                    break
                else:
                    print("  commands: h, c, n, t, w, list, undo, fit, apply, save, q "
                          "(Enter = current reading)")
            except (IndexError, ValueError, KeyError) as exc:
                print(f"  ? {exc}")
    finally:
        pipe.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
