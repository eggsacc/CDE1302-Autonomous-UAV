#!/usr/bin/env python3
"""
Redo the localisation-tag drift correction for a finished run, from its log.

    python mission/tools/relocalise.py                    # newest run in mission/logs
    python mission/tools/relocalise.py mission/logs/20261110_154735
    python mission/tools/relocalise.py RUN --config mission/config.yaml --write

Reads the run's detections.csv (every accepted sighting, localisation tags
included), re-clusters the mission tags exactly like the flight did
(tag_map.py), re-fits the localisation tags (localisation.py) and prints the
corrected report.  Use it after re-measuring where the localisation tags
really were, or to try another localisation.model.

Settings come from the run's config_used.yaml, or from --config (needed for
runs flown before the `localisation` section existed).  --write saves
localisation.csv and tags_corrected.csv into the run folder, replacing the
ones written in flight.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from localisation import (LOC_FIX_FIELDS, TAGS_CORRECTED_FIELDS, LocSighting,  # noqa: E402
                          correct, corrected_rows, find_fixes, fix_rows, format_report,
                          solve_fixes)
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
            out.append(SimpleNamespace(tag_id=int(row["tag_id"]), t=t, x=x, y=y))
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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    ap.add_argument("run", nargs="?", help="run folder (default: newest in mission/logs)")
    ap.add_argument("--config", help="settings to use instead of the run's config_used.yaml")
    ap.add_argument("--write", action="store_true",
                    help="save localisation.csv and tags_corrected.csv into the run folder")
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

    fit, tags = relocalise(settings, read_detections(run / "detections.csv"))
    print(f"run {run}\nsettings {cfg}")
    print(format_report(fit, tags))
    if args.write:
        for name, fields, rows in (("localisation.csv", LOC_FIX_FIELDS, fix_rows(fit)),
                                   ("tags_corrected.csv", TAGS_CORRECTED_FIELDS,
                                    corrected_rows(tags))):
            with open(run / name, "w", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=fields)
                w.writeheader()
                for r in rows:
                    w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v)
                                for k, v in r.items()})
            print(f"wrote {run / name}")
    return 0 if fit.ok else 1


if __name__ == "__main__":
    sys.exit(main())
