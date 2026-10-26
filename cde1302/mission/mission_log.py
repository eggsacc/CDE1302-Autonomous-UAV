"""
Timestamped mission events and per-run CSV logs.

    log = MissionLog(settings.logging.log_path, settings.source)
    log.takeoff_sent()                       # required event, starts the mission clock
    log.tag_confirmed(3, 0.81, 0.30, 5)      # required event
    log.land_sent("search complete")         # required event
    log.close()

Each run gets its own folder, mission/logs/<YYYYmmdd_HHMMSS>/:

    events.csv       every event (required ones are also printed to stdout)
    trajectory.csv   drone pose at telemetry rate
    detections.csv   every accepted raw tag sighting
    tags_final.csv   the final tag report
    config_used.yaml copy of the config this run used

Times: `run_s` counts from when the log was opened, `mission_s` from the
takeoff command (blank before it), and `wall_time` is local clock time with
milliseconds.  All times passed in are monotonic seconds from the same clock
the log was given, so the simulator can use a fake clock.
"""

from __future__ import annotations

import csv
import datetime as _dt
import logging
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

import yaml

log = logging.getLogger(__name__)

EVENT_FIELDS = ["wall_time", "run_s", "mission_s", "event", "details"]

TRAJECTORY_FIELDS = ["wall_time", "run_s", "mission_s",
                     "x", "y", "heading_rad", "nav_state"]

# R is the camera-frame rotation, row-major (apriltag_host.Detection.R).
DETECTION_FIELDS = (
    ["wall_time", "run_s", "mission_s",
     "capture_wall_time", "capture_mission_s",
     "tag_id", "body_f", "body_r", "body_d",
     "cam_tx", "cam_ty", "cam_tz"]
    + [f"R{i}{j}" for i in range(3) for j in range(3)]
    + ["pose_err", "decision_margin",
       "drone_x", "drone_y", "drone_heading_rad", "tag_x", "tag_y"]
)

TAGS_FINAL_FIELDS = ["tag_id", "x", "y", "n_sightings",
                     "first_seen_wall_time", "first_seen_mission_s",
                     "confirmed_wall_time", "confirmed_mission_s"]

# Names of the events the brief requires us to record and display.
EV_TAKEOFF = "TAKEOFF_CMD"
EV_TAG = "TAG_CONFIRMED"
EV_LAND = "LAND_CMD"


def format_wall(unix_s: float) -> str:
    """Local wall-clock time with milliseconds, e.g. '2026-10-26 14:03:12.345'."""
    d = _dt.datetime.fromtimestamp(unix_s)
    return d.strftime("%Y-%m-%d %H:%M:%S.") + f"{d.microsecond // 1000:03d}"


def _cell(value: Any) -> Any:
    """CSV cell text: floats rounded to 0.1 mm / 0.1 ms, None as blank."""
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.4f}"
    return value


class _CsvFile:
    """One CSV file, header written on open, flushed after every row."""

    def __init__(self, path: Path, fields: list[str]) -> None:
        self.path = path
        self._fh = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._fh, fieldnames=fields, restval="",
                                 extrasaction="raise")
        self._w.writeheader()
        self._fh.flush()

    def write(self, row: Mapping[str, Any]) -> None:
        self._w.writerow({k: _cell(v) for k, v in row.items()})
        self._fh.flush()       # a crash mid-flight must not lose the log

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()


class MissionLog:
    """Thread-safe event logger and CSV writer for one mission run."""

    def __init__(self, root: str | Path, config_source: str | Path | None = None,
                 *, config_data: Mapping | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 wall_clock: Callable[[], float] = time.time,
                 echo: Callable[[str], None] | None = None,
                 run_name: str | None = None) -> None:
        """Create the run folder and open every CSV.

        Args:
            root: parent folder; the run folder is created inside it.
            config_source: config file to copy into the run folder.
            config_data: used instead when there is no file (dumped as YAML).
            clock: monotonic clock all `t` arguments are measured on.
            wall_clock: Unix-time clock, read once to anchor `clock`.
            echo: where required events are displayed (default: print).
            run_name: folder name (default: the wall time, YYYYmmdd_HHMMSS).
        """
        self._clock = clock
        self._t0 = clock()
        self._wall0 = wall_clock()
        self._echo = echo or (lambda line: print(line, flush=True))
        self._lock = threading.Lock()
        self._mission_t0: float | None = None
        self._closed = False

        name = run_name or _dt.datetime.fromtimestamp(self._wall0).strftime("%Y%m%d_%H%M%S")
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        folder, n = root / name, 2
        while folder.exists():            # two runs in the same second
            folder, n = root / f"{name}_{n}", n + 1
        folder.mkdir()
        self.folder = folder

        config_copy = folder / "config_used.yaml"
        if config_source is not None:
            shutil.copyfile(config_source, config_copy)
        elif config_data is not None:
            config_copy.write_text(yaml.safe_dump(dict(config_data), sort_keys=False),
                                   encoding="utf-8")

        self._events = _CsvFile(folder / "events.csv", EVENT_FIELDS)
        self._traj = _CsvFile(folder / "trajectory.csv", TRAJECTORY_FIELDS)
        self._dets = _CsvFile(folder / "detections.csv", DETECTION_FIELDS)
        log.info("Logging this run to %s", folder)

    # -- time ---------------------------------------------------------------

    def now(self) -> float:
        """Current time on this log's monotonic clock."""
        return self._clock()

    def wall_time(self, t: float) -> float:
        """Unix time for monotonic time t."""
        return self._wall0 + (t - self._t0)

    def mission_elapsed(self, t: float | None = None) -> float | None:
        """Seconds since the takeoff command, or None before it was sent."""
        if self._mission_t0 is None:
            return None
        return (self.now() if t is None else t) - self._mission_t0

    @property
    def mission_started(self) -> bool:
        return self._mission_t0 is not None

    def _times(self, t: float) -> dict[str, Any]:
        return {"wall_time": format_wall(self.wall_time(t)),
                "run_s": t - self._t0,
                "mission_s": self.mission_elapsed(t)}

    # -- events -------------------------------------------------------------

    def event(self, name: str, details: str = "", *, required: bool = False,
              t: float | None = None) -> dict[str, Any]:
        """Record an event in events.csv and return the row written.

        Required events are also printed as one line on stdout; the others go
        to the `logging` module at INFO.
        """
        t = self.now() if t is None else t
        with self._lock:
            row = {**self._times(t), "event": name, "details": details}
            if not self._closed:
                self._events.write(row)
        if required:
            ms = row["mission_s"]
            elapsed = "T+   --.--- s" if ms is None else f"T+{ms:8.3f} s"
            self._echo(f"[EVENT] {row['wall_time']}  {elapsed}  {name:<13} {details}")
        else:
            log.info("%s %s", name, details)
        return row

    def takeoff_sent(self, details: str = "CMD_START sent",
                     t: float | None = None) -> dict[str, Any]:
        """Required event: the takeoff command went out.  Starts mission time."""
        t = self.now() if t is None else t
        with self._lock:
            if self._mission_t0 is None:
                self._mission_t0 = t
        return self.event(EV_TAKEOFF, details, required=True, t=t)

    def tag_confirmed(self, tag_id: int, x: float, y: float, n_sightings: int,
                      t: float | None = None) -> dict[str, Any]:
        """Required event: a tag ID was confirmed for the first time."""
        return self.event(EV_TAG, f"id={tag_id} x={x:.3f} y={y:.3f} "
                          f"({n_sightings} sightings)", required=True, t=t)

    def land_sent(self, reason: str, t: float | None = None) -> dict[str, Any]:
        """Required event: the land command went out."""
        return self.event(EV_LAND, f"CMD_LAND sent ({reason})", required=True, t=t)

    # -- data streams -------------------------------------------------------

    def trajectory(self, t: float, x: float, y: float, heading_rad: float,
                   nav_state: str) -> None:
        """One telemetry pose, in trajectory.csv."""
        with self._lock:
            if not self._closed:
                self._traj.write({**self._times(t), "x": x, "y": y,
                                  "heading_rad": heading_rad, "nav_state": nav_state})

    def detection(self, t: float, capture_t: float, **fields: Any) -> None:
        """One accepted tag sighting, in detections.csv.

        Args:
            t: when it was logged; capture_t: estimated frame capture time.
            fields: the remaining DETECTION_FIELDS columns (missing = blank).
        Raises:
            ValueError: for a column name not in DETECTION_FIELDS.
        """
        row = {"capture_wall_time": format_wall(self.wall_time(capture_t)),
               "capture_mission_s": self.mission_elapsed(capture_t), **fields}
        with self._lock:
            if not self._closed:
                self._dets.write({**self._times(t), **row})

    def write_tags_final(self, rows: Iterable[Mapping[str, Any]]) -> Path:
        """Write tags_final.csv (overwrites any earlier copy) and return its path.

        Each row has tag_id, x, y, n_sightings and the monotonic times
        `first_seen_t` / `confirmed_t`, converted here to wall and mission time.
        """
        path = self.folder / "tags_final.csv"
        out = _CsvFile(path, TAGS_FINAL_FIELDS)
        try:
            for r in rows:
                first, conf = r.get("first_seen_t"), r.get("confirmed_t")
                out.write({
                    "tag_id": r["tag_id"], "x": r["x"], "y": r["y"],
                    "n_sightings": r["n_sightings"],
                    "first_seen_wall_time": None if first is None else format_wall(self.wall_time(first)),
                    "first_seen_mission_s": None if first is None else self.mission_elapsed(first),
                    "confirmed_wall_time": None if conf is None else format_wall(self.wall_time(conf)),
                    "confirmed_mission_s": None if conf is None else self.mission_elapsed(conf),
                })
        finally:
            out.close()
        return path

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        """Close every file; later writes are silently dropped."""
        with self._lock:
            self._closed = True
            for f in (self._events, self._traj, self._dets):
                f.close()

    def __enter__(self) -> "MissionLog":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
