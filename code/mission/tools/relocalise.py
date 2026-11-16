#!/usr/bin/env python3
"""
Redo the localisation-tag drift correction for a finished run, from its log,
and show it: recorded vs corrected route and tags, overlaid on one map.

The mission already saves correction.png and trajectory_corrected.csv in the
run folder after landing (localisation.save_plot).  Use this to redo them,
e.g. after re-measuring where the localisation tags really were, or to try
another localisation.model.

    python mission/tools/relocalise.py                    # newest run in mission/logs
    python mission/tools/relocalise.py mission/logs/20261110_213110 --plot
    python mission/tools/relocalise.py RUN --truth "2:1.3,-1.35 4:2.76,0.92 6:4.14,0.92" --plot
    python mission/tools/relocalise.py RUN --config mission/config.yaml --write

Reads the run's detections.csv (every accepted sighting, localisation tags
included), re-clusters the mission tags exactly like the flight did
(tag_map.py), re-fits the localisation tags (localisation.py) and prints the
corrected report.

    --plot [FILE]   save the before/after map (default RUN/correction.png):
                    grey = as recorded (PX4's frame), blue = corrected (arena
                    frame, the tiles), orange = true tag positions; every tag
                    labelled with its corrected position and when it was seen
    --truth SPEC    true tag positions, "id:x,y id:x,y ..." or a CSV file with
                    columns tag_id,x,y (arena frame, m); default: the settings'
                    localisation.truth_tags.  Adds the error table
    --write         save localisation.csv, tags_corrected.csv and
                    trajectory_corrected.csv into the run folder (the first two
                    replace the ones written in flight)

Settings come from the run's config_used.yaml, or from --config (needed for
runs flown before the `localisation` section existed).
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from correction_plot import (TRAJ_CORRECTED_FIELDS, correct_trajectory,  # noqa: E402,F401
                             format_errors, plot_correction, read_trajectory, tag_errors,
                             write_csv)
from localisation import (LOC_FIX_FIELDS, TAGS_CORRECTED_FIELDS, LocSighting,  # noqa: E402
                          correct_with, corrected_rows, drift_correction, find_fixes,
                          fix_rows, format_report, solve_fixes)
from settings import ConfigError, Settings, load_settings  # noqa: E402
from tag_map import TagMap  # noqa: E402

LOG_ROOT = Path(__file__).resolve().parents[1] / "logs"


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
            out.append(SimpleNamespace(tag_id=int(row["tag_id"]), t=t, x=x, y=y,
                                       wall=row.get("capture_wall_time", "")))
    return out


def clock_stamp(detections: list[SimpleNamespace]) -> Callable[[float], str]:
    """'T+12.3 s  22:47:15' for a mission time, using the log's wall clock."""
    for d in detections:
        try:
            w0 = dt.datetime.strptime(d.wall, "%Y-%m-%d %H:%M:%S.%f")
        except (ValueError, TypeError):
            continue
        return lambda t: f"T+{t:.1f} s  {(w0 + dt.timedelta(seconds=t - d.t)):%H:%M:%S}"
    return lambda t: f"T+{t:.1f} s"


def relocalise(settings: Settings, detections: list[SimpleNamespace]):
    """(fit, corrected tags) from logged sightings, as the mission makes them."""
    fit, _, tags = relocalise_full(settings, detections)
    return fit, tags


def relocalise_full(settings: Settings, detections: list[SimpleNamespace]):
    """(fit, drift correction, corrected tags) from logged sightings."""
    lo, tg = settings.localisation, settings.tags
    tag_map = TagMap.from_settings(tg)
    loc_rows, samples = [], {}
    for d in sorted(detections, key=lambda d: d.t):
        if d.tag_id in lo.ids:
            loc_rows.append(LocSighting(d.tag_id, d.t, d.x, d.y))
        else:
            tag_map.add(SimpleNamespace(tag_id=d.tag_id, x=d.x, y=d.y, t_detect=d.t))
            samples.setdefault(d.tag_id, []).append((d.t, d.x, d.y))
    fixes = find_fixes(loc_rows, lo, tg.confirm_radius_m, tg.outlier_radius_m)
    fit = solve_fixes(fixes, lo)
    corr = drift_correction(fit, lo)
    tags = correct_with(corr, tag_map.estimates(confirmed_only=True), samples,
                        tg.confirm_radius_m, tg.outlier_radius_m)
    return fit, corr, tags


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
        truth = (parse_truth(args.truth) if args.truth else
                 {g.id: (g.x, g.y) for g in settings.localisation.truth_tags})
    except (ValueError, KeyError) as exc:
        print(f"bad --truth: {exc}", file=sys.stderr)
        return 2

    detections = read_detections(run / "detections.csv")
    fit, corr, tags = relocalise_full(settings, detections)
    traj = read_trajectory(run / "trajectory.csv") if (run / "trajectory.csv").is_file() else []
    traj_rows = correct_trajectory(fit, traj, corr)
    errors = tag_errors(tags, truth)
    print(f"run {run}\nsettings {cfg}")
    print(format_report(fit, tags, corr=corr))
    if errors:
        print(format_errors(errors))
    if args.write:
        for name, fields, rows in (("localisation.csv", LOC_FIX_FIELDS, fix_rows(fit)),
                                   ("tags_corrected.csv", TAGS_CORRECTED_FIELDS,
                                    corrected_rows(tags)),
                                   ("trajectory_corrected.csv", TRAJ_CORRECTED_FIELDS,
                                    traj_rows)):
            print(f"wrote {write_csv(run / name, fields, rows)}")
    if args.plot is not None:
        out = Path(args.plot) if args.plot else run / "correction.png"
        plot_correction(out, settings, fit, traj_rows, tags, truth,
                        f"Run {run.name}: recorded vs corrected", clock_stamp(detections), corr)
        print(f"wrote {out}")
    return 0 if fit.ok else 1


if __name__ == "__main__":
    sys.exit(main())
