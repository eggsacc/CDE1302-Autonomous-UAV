"""
Compare flights with and without UWB, from their log folders.  From cde1302/:

    python mission/tools/compare_flights.py mission/logs/20261103_143856 mission/logs/...
    python mission/tools/compare_flights.py --last 6 --plot drift.png
    python mission/tools/compare_flights.py --last 6 --truth tags_measured.csv

For each flight it prints:

    sweep_s        how long the sweep took (planned time in brackets)
    xtrack_rms/max distance from the planned lanes, PX4's own estimate (m)
    speed, cv      lane speed (m/s) and its variation (std / mean; 0 = constant)
    corner_mean    how close the drone came to each corner, on average (m)
    return_err     distance from the takeoff spot when "return home" finished,
                   in PX4's estimate (m)
    tags, spread   tags confirmed and their mean sighting spread (m)
    drift_end/max  PX4's position against UWB at the end of the sweep and at
                   its worst (m).  Needs uwb.csv (uwb.record in config.yaml).
                   UWB is fitted to PX4 over the first --fit-s seconds of the
                   sweep, so this is how far PX4 wandered after that.
    tag_err        with --truth: distance from each tape-measured tag (m)

Everything except drift and tag_err is measured against PX4's own position,
so it shows how well the drone followed its plan, not where it really was.
In a UWB flight PX4 is pulled towards UWB, so a small drift there is partly
by construction; the drift of the non-UWB flights is the real comparison.

--truth: CSV with columns tag_id, forward, right: metres from the takeoff
spot, forward = the way the drone faced at takeoff, right = its right.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

import planner  # noqa: E402
from trajectory import shift_for_camera  # noqa: E402
from uwb_align import robust_fit  # noqa: E402

LOGS = Path(__file__).resolve().parents[1] / "logs"
MAX_EOP_M = 0.30
MAX_PX4_AGE_MS = 200


@dataclass
class Flight:
    name: str
    uwb: bool
    sweep_s: Optional[float] = None
    planned_s: Optional[float] = None
    xtrack_rms: Optional[float] = None
    xtrack_max: Optional[float] = None
    speed: Optional[float] = None
    speed_cv: Optional[float] = None
    corner_mean: Optional[float] = None
    return_err: Optional[float] = None
    tags: int = 0
    tag_spread: Optional[float] = None
    drift_end: Optional[float] = None
    drift_max: Optional[float] = None
    tag_err: Optional[float] = None
    drift_curve: Optional[tuple[np.ndarray, np.ndarray]] = field(default=None, repr=False)
    note: str = ""


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def num(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in re.findall(r"[-+]?\d+\.?\d*", text))


def planned_path(cfg: dict, events: list[dict]) -> Optional[np.ndarray]:
    """The sweep waypoints in PX4's frame, as the mission placed them."""
    shifted = [e for e in events if e["event"] == "PLAN_SHIFTED"]
    moved = [e for e in events if e["event"] == "PLAN_MOVED"]
    wps = planner.plan_from_config(cfg)[:-1]
    if shifted:
        heading = math.radians(num(shifted[0]["details"].split("heading")[1])[0])
        wps = shift_for_camera(wps, heading, cfg["sweep"]["camera_offset_m"])
    if moved:
        dx, dy = num(moved[0]["details"])[:2]
        wps = [(x + dx, y + dy) for x, y in wps]
    return np.array(wps) if wps else None


def segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from each point in p to the polyline through a -> b segments."""
    best = np.full(len(p), np.inf)
    for s, e in zip(a, b):
        d = e - s
        L2 = float(d @ d)
        t = np.clip(((p - s) @ d) / L2, 0, 1) if L2 > 0 else np.zeros(len(p))
        best = np.minimum(best, np.linalg.norm(p - (s + t[:, None] * d), axis=1))
    return best


def analyse(folder: Path, fit_s: float = 15.0,
            truth: Optional[dict[int, tuple[float, float]]] = None) -> Flight:
    events = read_csv(folder / "events.csv")
    ev = lambda name: [e for e in events if e["event"] == name]          # noqa: E731
    cfg = yaml.safe_load((folder / "config_used.yaml").read_text(encoding="utf-8"))
    f = Flight(folder.name, uwb=bool(ev("UWB_ALIGN_CONFIRMED")))

    rows = read_csv(folder / "trajectory.csv")
    t = np.array([float(r["run_s"]) for r in rows]) if rows else np.zeros(0)
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows]) if rows else np.zeros((0, 2))

    starts = [e for e in ev("TRAJ_STARTED") if e["details"].startswith("sweep")]
    ends = [e for e in ev("TRAJ_END") if e["details"].startswith("sweep")]
    if not starts or not ends or not len(t):
        f.note = "no sweep"
        return f
    t0, t1 = float(starts[0]["run_s"]), float(ends[0]["run_s"])
    f.sweep_s = t1 - t0
    f.planned_s = num(starts[0]["details"].split("about")[1])[0]
    if "done" not in ends[0]["details"]:
        f.note = ends[0]["details"].split(":")[1].strip()[:20]

    sweep = (t >= t0) & (t <= t1)
    P = xy[sweep]
    wps = planned_path(cfg, events)
    if wps is not None and len(P) > 10:
        uploads = [e for e in ev("TRAJECTORY") if e["details"].startswith("sweep")
                   and float(e["run_s"]) <= t0]
        frm = np.array(num(uploads[-1]["details"].split("from")[1])[:2]) if uploads else P[0]
        path = np.vstack([P[0], P[0] + (wps - frm)])               # firmware anchors at its start
        d = segment_distance(P, path[:-1], path[1:])
        f.xtrack_rms = float(np.sqrt(np.mean(d ** 2)))
        f.xtrack_max = float(d.max())
        f.corner_mean = float(np.mean([np.linalg.norm(P - w, axis=1).min() for w in path[2:]]))

    ts = t[sweep]
    if len(ts) > 20:
        g = np.arange(ts[0], ts[-1], 0.25)
        X, Y = np.interp(g, ts, P[:, 0]), np.interp(g, ts, P[:, 1])
        v = np.hypot(X[4:] - X[:-4], Y[4:] - Y[:-4]) / 1.0
        moving = v > 0.02
        if moving.any():
            f.speed = float(v[moving].mean())
            f.speed_cv = float(v[moving].std() / v[moving].mean())

    home = num(ev("HOME")[0]["details"])[:2] if ev("HOME") else (0.0, 0.0)
    heading = num(ev("HOME")[0]["details"].split("heading")[1])[0] if ev("HOME") else 0.0
    back = [e for e in ev("TRAJ_END") if e["details"].startswith("return home")]
    if back:
        end = num(back[0]["details"].split(" at ")[1])[:2]
        f.return_err = math.dist(end, home)

    tags = read_csv(folder / "tags_final.csv")
    dets = read_csv(folder / "detections.csv")
    f.tags = len(tags)
    spreads = []
    for tag in tags:
        pts = np.array([[float(d["tag_x"]), float(d["tag_y"])] for d in dets
                        if d["tag_id"] == tag["tag_id"]])
        if len(pts) > 2:
            spreads.append(float(np.hypot(*pts.std(axis=0))))
    f.tag_spread = float(np.mean(spreads)) if spreads else None
    if truth:
        h = math.radians(heading)
        errs = []
        for tag in tags:
            tid = int(tag["tag_id"])
            if tid in truth:
                fw, rt = truth[tid]
                tn = home[0] + fw * math.cos(h) - rt * math.sin(h)
                te = home[1] + fw * math.sin(h) + rt * math.cos(h)
                errs.append(math.dist((float(tag["x"]), float(tag["y"])), (tn, te)))
        f.tag_err = float(np.mean(errs)) if errs else None

    uwb_drift(f, folder, cfg, t0, t1, fit_s)
    return f


def uwb_drift(f: Flight, folder: Path, cfg: dict, t0: float, t1: float, fit_s: float) -> None:
    rows = [r for r in read_csv(folder / "uwb.csv")
            if r["fix"] == "1" and r["px4_x"] not in ("", None)
            and float(r["eop_max"]) <= MAX_EOP_M and float(r["px4_age_ms"]) <= MAX_PX4_AGE_MS
            and t0 <= float(r["run_s"]) <= t1]
    if len(rows) < 30:
        return
    t = np.array([float(r["run_s"]) for r in rows])
    u = np.array([[float(r["uwb_x"]), float(r["uwb_y"])] for r in rows])
    p = np.array([[float(r["px4_x"]), float(r["px4_y"])] for r in rows])
    fit = t <= t0 + fit_s
    if fit.sum() < 30 or np.ptp(p[fit], axis=0).max() < 0.3:
        f.note = (f.note + " fit window short").strip()
        fit = t <= t0 + 2 * fit_s
    mirror_cfg = str(cfg.get("uwb", {}).get("mirror", "auto"))
    choices = [mirror_cfg == "yes"] if mirror_cfg in ("yes", "no") else [False, True]
    w = np.ones(int(fit.sum()))
    al = min((robust_fit(u[fit], p[fit], w, m, 3.0) for m in choices), key=lambda r: r[1])[0]
    err = np.linalg.norm(p - (u @ al.matrix.T + np.array([al.tx, al.ty])), axis=1)
    f.drift_curve = (t - t0, err)
    end = t >= t1 - 5.0
    f.drift_end = float(np.median(err[end])) if end.any() else None
    f.drift_max = float(np.percentile(err, 95))


def fmt(v: Optional[float], d: int = 2) -> str:
    return "-" if v is None else f"{v:.{d}f}"


def print_table(flights: list[Flight]) -> None:
    head = ["flight", "UWB", "sweep_s", "xtrack_rms", "xtrack_max", "speed", "cv",
            "corner_mean", "return_err", "tags", "spread", "drift_end", "drift_max", "tag_err", "note"]
    rows = [[f.name, "yes" if f.uwb else "no",
             f"{fmt(f.sweep_s, 0)} ({fmt(f.planned_s, 0)})", fmt(f.xtrack_rms, 3), fmt(f.xtrack_max),
             fmt(f.speed, 3), fmt(f.speed_cv), fmt(f.corner_mean, 3), fmt(f.return_err, 3),
             str(f.tags), fmt(f.tag_spread, 3), fmt(f.drift_end), fmt(f.drift_max),
             fmt(f.tag_err, 3), f.note] for f in flights]
    widths = [max(len(str(r[i])) for r in [head] + rows) for i in range(len(head))]
    for r in [head] + rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))
    for label, group in (("UWB", [f for f in flights if f.uwb]),
                         ("no UWB", [f for f in flights if not f.uwb])):
        done = [f for f in group if f.xtrack_rms is not None]
        if not done:
            continue
        def mean(attr):                                     # noqa: E306
            vals = [getattr(f, attr) for f in done if getattr(f, attr) is not None]
            return (f"{np.mean(vals):.3f} (n={len(vals)})" if vals else "-")
        print(f"\n{label}: xtrack_rms {mean('xtrack_rms')}, speed cv {mean('speed_cv')}, "
              f"return_err {mean('return_err')}, drift_end {mean('drift_end')}, "
              f"tag_err {mean('tag_err')}")


def plot(flights: list[Flight], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for f in flights:
        if f.drift_curve is None:
            continue
        tt, e = f.drift_curve
        k = max(1, len(e) // 400)
        ax.plot(tt[::k], e[::k], color="#1f77b4" if f.uwb else "#d62728", alpha=0.8,
                label=f"{f.name} ({'UWB' if f.uwb else 'no UWB'})")
    ax.set_xlabel("time since sweep start (s)")
    ax.set_ylabel("PX4 position vs UWB (m)")
    ax.set_title("Position drift during the sweep")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nplot saved to {path}")


def read_truth(path: Path) -> dict[int, tuple[float, float]]:
    return {int(r["tag_id"]): (float(r["forward"]), float(r["right"])) for r in read_csv(path)}


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="*", type=Path, help="log folders to compare")
    ap.add_argument("--last", type=int, help="also take the newest N flights that started a sweep")
    ap.add_argument("--fit-s", type=float, default=15.0,
                    help="seconds of sweep used to fit UWB to PX4 for the drift (default 15)")
    ap.add_argument("--truth", type=Path, help="CSV: tag_id, forward, right (tape-measured)")
    ap.add_argument("--plot", type=Path, help="save the drift-vs-time plot to this PNG")
    args = ap.parse_args(argv)

    folders = list(args.folders)
    if args.last:
        swept = [d for d in sorted(LOGS.iterdir(), reverse=True)
                 if (d / "events.csv").exists() and "TRAJ_STARTED" in (d / "events.csv").read_text(encoding="utf-8")]
        folders += sorted(swept[:args.last])
    if not folders:
        ap.error("give log folders or --last N")
    truth = read_truth(args.truth) if args.truth else None
    flights = [analyse(d, args.fit_s, truth) for d in folders]
    print_table(flights)
    if args.plot:
        plot(flights, args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
