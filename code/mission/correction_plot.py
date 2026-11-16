"""
The before/after picture of the localisation-tag correction, and the
corrected route.  Used by mission.py (saved after every flight, after landing)
and by tools/relocalise.py (redone from a log).

    correction.png            recorded (PX4 frame, grey) vs corrected (arena
                              frame, blue) route and tags; known localisation
                              tags; true tag positions (orange) if given; every
                              tag labelled with its corrected position and
                              when it was seen
    trajectory_corrected.csv  the route mapped into the arena frame

Matplotlib's object API is used (no pyplot), so drawing this never touches the
live map's window or backend.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Callable, Optional

from localisation import IDENTITY, CorrectedTag, LocFit
from settings import Settings

TRAJ_CORRECTED_FIELDS = ["mission_s", "x_raw", "y_raw", "x", "y"]

# Colours (validated pair, dataviz palette slots 1-2): before = neutral grey,
# after = blue, truth = orange.  Text stays in ink colours.
C_BEFORE, C_AFTER, C_TRUTH = "#9a9893", "#2a78d6", "#eb6834"
C_INK, C_INK2, C_GRID, C_BG = "#0b0b0b", "#52514e", "#dcdad5", "#fcfcfb"


def _num(row: dict, *keys: str) -> Optional[float]:
    for k in keys:
        v = row.get(k, "")
        if v not in ("", None):
            return float(v)
    return None


def read_trajectory(path: Path) -> list[tuple[float, float, float]]:
    """trajectory.csv as (mission_s, x, y), from the takeoff command on (PX4 frame)."""
    out = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            t, x, y = _num(row, "mission_s"), _num(row, "x"), _num(row, "y")
            if t is not None and x is not None and y is not None:
                out.append((t, x, y))
    return out


def correct_trajectory(fit: LocFit, traj, corr=None, t_of=lambda ms: ms) -> list[dict]:
    """The recorded route mapped through the correction: corr (a
    DriftCorrection, applied at each point's time t_of(mission_s)), else the
    fit (unchanged when there is none)."""
    t = fit.transform if fit.ok else IDENTITY
    rows = []
    for ts, x, y in traj:
        cx, cy = corr.apply(t_of(ts), x, y) if corr is not None and corr.ok else t.apply(x, y)
        rows.append({"mission_s": ts, "x_raw": x, "y_raw": y, "x": cx, "y": cy})
    return rows


def write_csv(path: Path, fields: list[str], rows) -> Path:
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    return path


def tag_errors(tags: list[CorrectedTag], truth: dict) -> list[dict]:
    """Per true tag: error before and after the correction (m), None = not found."""
    found = {c.tag_id: c for c in tags}
    rows = []
    for tid, (tx, ty) in sorted(truth.items()):
        c = found.get(tid)
        rows.append({"tag_id": tid, "true_x": tx, "true_y": ty,
                     "raw_err": None if c is None else math.hypot(c.raw_x - tx, c.raw_y - ty),
                     "corr_err": None if c is None else math.hypot(c.x - tx, c.y - ty)})
    return rows


def format_errors(rows: list[dict]) -> str:
    lines = ["=== TAG ERRORS vs TRUE POSITIONS (m) ===",
             f"{'ID':>4} {'true x':>7} {'true y':>7} {'before':>7} {'after':>7}"]
    for r in rows:
        if r["raw_err"] is None:
            lines.append(f"{r['tag_id']:>4} {r['true_x']:>7.2f} {r['true_y']:>7.2f}   not found")
        else:
            lines.append(f"{r['tag_id']:>4} {r['true_x']:>7.2f} {r['true_y']:>7.2f} "
                         f"{r['raw_err']:>7.3f} {r['corr_err']:>7.3f}")
    got = [r for r in rows if r["raw_err"] is not None]
    if got:
        lines.append(f"mean {'':>15} {sum(r['raw_err'] for r in got) / len(got):>7.3f} "
                     f"{sum(r['corr_err'] for r in got) / len(got):>7.3f}")
    return "\n".join(lines)


def plot_correction(path: Path, settings: Settings, fit: LocFit, traj_rows: list[dict],
                    tags: list[CorrectedTag], truth: dict, title: str,
                    stamp: Callable[[float], str] = lambda t: f"T+{t:.1f} s",
                    corr=None) -> Path:
    """Save the before/after map.  East right / North up, like the live map.

    stamp: formats a sighting time (same clock as CorrectedTag.first_seen_t and
    Fix.t_start) for the labels.
    """
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle

    import planner

    fig = Figure(figsize=(12, 8.5), facecolor=C_BG)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0.06, 0.08, 0.60, 0.85])
    ax.set_facecolor(C_BG)
    a = settings.area
    ax.add_patch(Rectangle((a.y_min, a.x_min), a.y_max - a.y_min, a.x_max - a.x_min,
                           fill=False, ls="--", lw=1, ec=C_INK2, zorder=1))
    lanes = planner.plan_from_config(settings.raw)[:-1]
    ax.plot([p[1] for p in lanes], [p[0] for p in lanes], ":", color=C_INK2, lw=1, zorder=1)

    if traj_rows:
        ax.plot([r["y_raw"] for r in traj_rows], [r["x_raw"] for r in traj_rows], "-",
                color=C_BEFORE, lw=2, zorder=2)
        if fit.ok or (corr is not None and corr.ok):
            ax.plot([r["y"] for r in traj_rows], [r["x"] for r in traj_rows], "-",
                    color=C_AFTER, lw=2, zorder=3)

    t = fit.transform if fit.ok else IDENTITY
    fixed = (lambda f: corr.apply((f.t_start + f.t_end) / 2, *f.px4)) \
        if corr is not None and corr.ok else (lambda f: t.apply(*f.px4))
    ok = fit.ok or (corr is not None and corr.ok)
    bbox = dict(boxstyle="round,pad=0.2", fc=C_BG, ec="none", alpha=0.85)

    # Localisation tags: the known spot, each visit as recorded -> as corrected,
    # and one label per tag listing every visit's corrected position and time.
    visits: dict[int, list] = {}
    for f in fit.fixes:
        visits.setdefault(f.tag_id, []).append(f)
    for ref in settings.localisation.tags:
        ax.plot(ref.y, ref.x, "D", ms=11, mfc="none", mec=C_INK, mew=1.5, zorder=4)
        lines = [f"L{ref.id}  true ({ref.x:.2f}, {ref.y:.2f})"]
        for f in visits.get(ref.id, []):
            cx, cy = fixed(f)
            ax.plot([f.px4[1], cy], [f.px4[0], cx], "-", color=C_BEFORE, lw=1, zorder=3)
            ax.plot(f.px4[1], f.px4[0], "D", ms=6, color=C_BEFORE, mec=C_BG, mew=1, zorder=4)
            if ok:
                ax.plot(cy, cx, "D", ms=6, color=C_AFTER, mec=C_BG, mew=1, zorder=5)
            lines.append(f"  ({cx:.2f}, {cy:.2f})  {stamp(f.t_start)}")
        if not visits.get(ref.id):
            lines.append("  not seen")
        ax.annotate("\n".join(lines), (ref.y, ref.x), textcoords="offset points",
                    xytext=(10, 8), fontsize=7.5, color=C_INK, family="monospace",
                    bbox=bbox, zorder=8)

    # Search tags: recorded -> corrected, labelled at the corrected spot.
    for c in tags:
        ax.annotate("", (c.y, c.x), (c.raw_y, c.raw_x),
                    arrowprops=dict(arrowstyle="->", color=C_INK2, lw=1), zorder=4)
        ax.plot(c.raw_y, c.raw_x, "s", ms=9, color=C_BEFORE, mec=C_BG, mew=1.5, zorder=5)
        if ok:
            ax.plot(c.y, c.x, "s", ms=9, color=C_AFTER, mec=C_BG, mew=1.5, zorder=6)
        when = stamp(c.first_seen_t) if c.first_seen_t is not None else ""
        ax.annotate(f"{c.tag_id}  ({c.x:.2f}, {c.y:.2f})\n{when}", (c.y, c.x),
                    textcoords="offset points", xytext=(9, -20), fontsize=8, color=C_INK,
                    family="monospace", weight="bold", bbox=bbox, zorder=8)
    found = {c.tag_id for c in tags}
    for tid, (tx, ty) in truth.items():
        ax.plot(ty, tx, "o", ms=13, mfc="none", mec=C_TRUTH, mew=2, zorder=5)
        if tid not in found:
            ax.annotate(f"{tid} (not found)", (ty, tx), textcoords="offset points",
                        xytext=(9, -12), fontsize=8, color=C_INK2, zorder=8)

    handles = [Line2D([], [], color=C_BEFORE, lw=2, label="route as recorded (PX4 frame)"),
               Line2D([], [], color=C_AFTER, lw=2, label="route corrected (arena frame)"),
               Line2D([], [], ls="", marker="s", ms=9, color=C_BEFORE, label="tag as recorded"),
               Line2D([], [], ls="", marker="s", ms=9, color=C_AFTER,
                      label="tag corrected: (x, y), first seen"),
               Line2D([], [], ls="", marker="D", ms=9, mfc="none", mec=C_INK,
                      label="localisation tag, true spot"),
               Line2D([], [], ls="", marker="D", ms=6, color=C_BEFORE,
                      label="localisation tag visit, as recorded"),
               Line2D([], [], ls="", marker="D", ms=6, color=C_AFTER,
                      label="localisation tag visit, corrected"),
               Line2D([], [], ls="--", color=C_INK2, lw=1, label="search area + planned lanes")]
    if truth:
        handles.insert(4, Line2D([], [], ls="", marker="o", ms=11, mfc="none", mec=C_TRUTH,
                                 mew=2, label="tag, true position"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9,
              frameon=False)

    note = fit.summary() if fit.ok else f"NOT CORRECTED: {fit.reason}"
    if corr is not None:
        note += "\n" + corr.describe()
    errors = tag_errors(tags, truth)
    if errors:
        note += "\n\n" + format_errors(errors)
    fig.text(0.68, 0.08, note, fontsize=8, family="monospace", color=C_INK, va="bottom",
             wrap=True)
    ax.set_title(title, fontsize=12, color=C_INK, loc="left")
    ax.set_xlabel("East, y (m)", color=C_INK2)
    ax.set_ylabel("North, x (m)", color=C_INK2)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color=C_GRID, lw=0.6)
    ax.tick_params(colors=C_INK2)
    for s in ax.spines.values():
        s.set_color(C_GRID)
    fig.savefig(path, dpi=140, facecolor=C_BG)
    return path
