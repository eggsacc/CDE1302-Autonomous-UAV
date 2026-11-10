#!/usr/bin/env python3
"""
Redo the localisation-tag drift correction for a finished run, from its log,
and show it: recorded vs corrected route and tags, overlaid on one map.

    python mission/tools/relocalise.py                    # newest run in mission/logs
    python mission/tools/relocalise.py mission/logs/20261110_213110 --plot
    python mission/tools/relocalise.py RUN --truth "2:1.3,-1.35 4:2.76,0.92 6:4.14,0.92" --plot
    python mission/tools/relocalise.py RUN --config mission/config.yaml --write

Reads the run's detections.csv (every accepted sighting, localisation tags
included), re-clusters the mission tags exactly like the flight did
(tag_map.py), re-fits the localisation tags (localisation.py) and prints the
corrected report.  Use it after re-measuring where the localisation tags
really were, or to try another localisation.model.

    --plot [FILE]   save the before/after map (default RUN/correction.png):
                    grey = as recorded (PX4's frame), blue = corrected (arena
                    frame, the tiles), orange = true tag positions (--truth)
    --truth SPEC    true tag positions, "id:x,y id:x,y ..." or a CSV file with
                    columns tag_id,x,y (arena frame, m); adds the error table
    --write         save localisation.csv, tags_corrected.csv and
                    trajectory_corrected.csv into the run folder (the first two
                    replace the ones written in flight)

Settings come from the run's config_used.yaml, or from --config (needed for
runs flown before the `localisation` section existed).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from localisation import (IDENTITY, LOC_FIX_FIELDS, TAGS_CORRECTED_FIELDS,  # noqa: E402
                          CorrectedTag, LocFit, LocSighting, correct, corrected_rows,
                          find_fixes, fix_rows, format_report, solve_fixes)
from settings import ConfigError, Settings, load_settings  # noqa: E402
from tag_map import TagMap  # noqa: E402

LOG_ROOT = Path(__file__).resolve().parents[1] / "logs"
TRAJ_CORRECTED_FIELDS = ["mission_s", "x_raw", "y_raw", "x", "y"]

# Colours (validated pair, dataviz palette slots 1-2): before = neutral grey,
# after = blue, truth = orange.  Text stays in ink colours.
C_BEFORE, C_AFTER, C_TRUTH = "#9a9893", "#2a78d6", "#eb6834"
C_INK, C_INK2, C_GRID = "#0b0b0b", "#52514e", "#dcdad5"


def newest_run(root: Path = LOG_ROOT) -> Path:
    runs = sorted(p for p in root.glob("*") if (p / "detections.csv").is_file())
    if not runs:
        raise SystemExit(f"no runs with a detections.csv in {root}")
    return runs[-1]


def _num(row: dict, *keys: str) -> float | None:
    for k in keys:
        v = row.get(k, "")
        if v not in ("", None):
            return float(v)
    return None


def read_detections(path: Path) -> list[SimpleNamespace]:
    """detections.csv rows as (tag_id, t, x, y); t = capture mission time (s)."""
    out = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            t = _num(row, "capture_mission_s", "mission_s", "run_s")
            x, y = _num(row, "tag_x"), _num(row, "tag_y")
            if t is None or x is None or y is None:
                continue
            out.append(SimpleNamespace(tag_id=int(row["tag_id"]), t=t, x=x, y=y))
    return out


def read_trajectory(path: Path) -> list[tuple[float, float, float]]:
    """trajectory.csv as (mission_s, x, y), from takeoff on (PX4 frame)."""
    out = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            t, x, y = _num(row, "mission_s"), _num(row, "x"), _num(row, "y")
            if t is not None and x is not None and y is not None:
                out.append((t, x, y))
    return out


def relocalise(settings: Settings, detections: list[SimpleNamespace]):
    """(fit, corrected tags) from logged sightings, as the mission makes them."""
    lo = settings.localisation
    tag_map = TagMap.from_settings(settings.tags)
    loc_rows = []
    for d in sorted(detections, key=lambda d: d.t):
        if d.tag_id in lo.ids:
            loc_rows.append(LocSighting(d.tag_id, d.t, d.x, d.y))
        else:
            tag_map.add(SimpleNamespace(tag_id=d.tag_id, x=d.x, y=d.y, t_detect=d.t))
    fixes = find_fixes(loc_rows, lo, settings.tags.confirm_radius_m,
                       settings.tags.outlier_radius_m)
    fit = solve_fixes(fixes, lo)
    return fit, correct(fit, tag_map.estimates(confirmed_only=True))


def correct_trajectory(fit: LocFit, traj) -> list[dict]:
    """The recorded route mapped through the fit (unchanged when there is none)."""
    t = fit.transform if fit.ok else IDENTITY
    rows = []
    for ts, x, y in traj:
        cx, cy = t.apply(x, y)
        rows.append({"mission_s": ts, "x_raw": x, "y_raw": y, "x": cx, "y": cy})
    return rows


def parse_truth(spec: Optional[str]) -> dict[int, tuple[float, float]]:
    """'2:1.3,-1.35 4:2.76,0.92' or a CSV file (tag_id,x,y) -> {id: (x, y)}."""
    if not spec:
        return {}
    p = Path(spec)
    if p.is_file():
        with open(p, newline="", encoding="utf-8") as fh:
            return {int(r["tag_id"]): (float(r["x"]), float(r["y"])) for r in csv.DictReader(fh)}
    out = {}
    for item in spec.replace(";", " ").split():
        tid, xy = item.split(":")
        x, y = xy.split(",")
        out[int(tid)] = (float(x), float(y))
    return out


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


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_correction(path: Path, settings: Settings, fit: LocFit, traj_rows: list[dict],
                    tags: list[CorrectedTag], truth: dict, title: str,
                    errors: list[dict]) -> Path:
    """One map, East right / North up like the live map: route and tags as
    recorded (grey) and corrected (blue), arena area / lanes / known
    localisation tags as recessive references, true tags in orange."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle

    import planner

    fig, ax = plt.subplots(figsize=(10.5, 8))
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    a = settings.area
    ax.add_patch(Rectangle((a.y_min, a.x_min), a.y_max - a.y_min, a.x_max - a.x_min,
                           fill=False, ls="--", lw=1, ec=C_INK2, zorder=1))
    lanes = planner.plan_from_config(settings.raw)[:-1]
    ax.plot([p[1] for p in lanes], [p[0] for p in lanes], ":", color=C_INK2, lw=1, zorder=1)

    if traj_rows:
        ax.plot([r["y_raw"] for r in traj_rows], [r["x_raw"] for r in traj_rows], "-",
                color=C_BEFORE, lw=2, zorder=2)
        if fit.ok:
            ax.plot([r["y"] for r in traj_rows], [r["x"] for r in traj_rows], "-",
                    color=C_AFTER, lw=2, zorder=3)

    # Known localisation tags (reference) and where PX4 put each visit.
    for ref in settings.localisation.tags:
        ax.plot(ref.y, ref.x, "D", ms=10, mfc="none", mec=C_INK, mew=1.5, zorder=4)
        ax.annotate(f"L{ref.id}", (ref.y, ref.x), textcoords="offset points", xytext=(8, 6),
                    fontsize=9, color=C_INK)
    for f in fit.fixes:
        ax.plot([f.px4[1], f.ref[1]], [f.px4[0], f.ref[0]], "-", color=C_BEFORE, lw=1, zorder=3)
        ax.plot(f.px4[1], f.px4[0], "D", ms=7, color=C_BEFORE, mec="#fcfcfb", mew=1.5, zorder=4)

    # Reported tags: recorded -> corrected, labelled once at the corrected spot.
    for c in tags:
        ax.annotate("", (c.y, c.x), (c.raw_y, c.raw_x),
                    arrowprops=dict(arrowstyle="->", color=C_INK2, lw=1), zorder=4)
        ax.plot(c.raw_y, c.raw_x, "s", ms=9, color=C_BEFORE, mec="#fcfcfb", mew=1.5, zorder=5)
        if fit.ok:
            ax.plot(c.y, c.x, "s", ms=9, color=C_AFTER, mec="#fcfcfb", mew=1.5, zorder=6)
        ax.annotate(str(c.tag_id), (c.y, c.x), textcoords="offset points", xytext=(8, -12),
                    fontsize=10, color=C_INK, weight="bold")
    for tid, (tx, ty) in truth.items():
        ax.plot(ty, tx, "o", ms=12, mfc="none", mec=C_TRUTH, mew=2, zorder=5)
        if tid not in {c.tag_id for c in tags}:
            ax.annotate(f"{tid} (not found)", (ty, tx), textcoords="offset points",
                        xytext=(9, -12), fontsize=9, color=C_INK2)

    handles = [Line2D([], [], color=C_BEFORE, lw=2, label="route as recorded (PX4 frame)"),
               Line2D([], [], color=C_AFTER, lw=2, label="route corrected (arena frame)"),
               Line2D([], [], ls="", marker="s", ms=9, color=C_BEFORE, label="tag as recorded"),
               Line2D([], [], ls="", marker="s", ms=9, color=C_AFTER, label="tag corrected"),
               Line2D([], [], ls="", marker="D", ms=9, mfc="none", mec=C_INK,
                      label="localisation tag, true"),
               Line2D([], [], ls="", marker="D", ms=7, color=C_BEFORE,
                      label="localisation tag, as recorded"),
               Line2D([], [], ls="--", color=C_INK2, lw=1, label="search area + planned lanes")]
    if truth:
        handles.insert(4, Line2D([], [], ls="", marker="o", ms=11, mfc="none", mec=C_TRUTH,
                                 mew=2, label="tag, true position"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9,
              frameon=False)

    note = fit.summary() if fit.ok else f"NOT CORRECTED: {fit.reason}"
    if errors:
        note += "\n\n" + format_errors(errors)
    fig.text(0.70, 0.08, note, fontsize=8.5, family="monospace", color=C_INK, va="bottom",
             wrap=True)
    ax.set_title(title, fontsize=12, color=C_INK, loc="left")
    ax.set_xlabel("East, y (m)", color=C_INK2)
    ax.set_ylabel("North, x (m)", color=C_INK2)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color=C_GRID, lw=0.6)
    ax.tick_params(colors=C_INK2)
    for s in ax.spines.values():
        s.set_color(C_GRID)
    fig.subplots_adjust(left=0.07, right=0.68, top=0.93, bottom=0.08)
    fig.savefig(path, dpi=140, facecolor=fig.get_facecolor())
    plt.close(fig)
    return path


def _write_csv(path: Path, fields: list[str], rows) -> None:
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    print(f"wrote {path}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    ap.add_argument("run", nargs="?", help="run folder (default: newest in mission/logs)")
    ap.add_argument("--config", help="settings to use instead of the run's config_used.yaml")
    ap.add_argument("--truth", help='true tag positions: "id:x,y id:x,y" or a CSV (tag_id,x,y)')
    ap.add_argument("--plot", nargs="?", const="", default=None, metavar="FILE",
                    help="save the before/after map (default RUN/correction.png)")
    ap.add_argument("--write", action="store_true",
                    help="save localisation.csv, tags_corrected.csv, trajectory_corrected.csv")
    args = ap.parse_args(argv)

    run = Path(args.run) if args.run else newest_run()
    cfg = Path(args.config) if args.config else run / "config_used.yaml"
    try:
        settings = load_settings(cfg)
    except ConfigError as exc:
        print(f"config error: {exc}\n(an older run? try --config mission/config.yaml)",
              file=sys.stderr)
        return 2
    if not settings.localisation.ids:
        print(f"{cfg.name} has no enabled localisation tags; pass --config mission/config.yaml",
              file=sys.stderr)
        return 2
    try:
        truth = parse_truth(args.truth)
    except (ValueError, KeyError) as exc:
        print(f"bad --truth: {exc}", file=sys.stderr)
        return 2

    fit, tags = relocalise(settings, read_detections(run / "detections.csv"))
    traj = read_trajectory(run / "trajectory.csv") if (run / "trajectory.csv").is_file() else []
    traj_rows = correct_trajectory(fit, traj)
    errors = tag_errors(tags, truth)
    print(f"run {run}\nsettings {cfg}")
    print(format_report(fit, tags))
    if errors:
        print(format_errors(errors))
    if args.write:
        _write_csv(run / "localisation.csv", LOC_FIX_FIELDS, fix_rows(fit))
        _write_csv(run / "tags_corrected.csv", TAGS_CORRECTED_FIELDS, corrected_rows(tags))
        _write_csv(run / "trajectory_corrected.csv", TRAJ_CORRECTED_FIELDS, traj_rows)
    if args.plot is not None:
        out = Path(args.plot) if args.plot else run / "correction.png"
        plot_correction(out, settings, fit, traj_rows, tags, truth,
                        f"Run {run.name}: recorded vs corrected by the localisation tags",
                        errors)
        print(f"wrote {out}")
    return 0 if fit.ok else 1


if __name__ == "__main__":
    sys.exit(main())
