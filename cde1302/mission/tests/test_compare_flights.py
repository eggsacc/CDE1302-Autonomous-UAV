"""tools/compare_flights.py on a simulated flight, with and without a uwb.csv."""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from mission import Phase
from mission_log import UWB_FIELDS, MissionLog
from test_mission import Rig, configure

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import compare_flights  # noqa: E402


@pytest.fixture(scope="module")
def flown(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("cmp")
    r = Rig(configure(tmp, flight={"stop_when_all_found": False}))
    r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")
    r.finish()
    return r.m.log.folder


def test_sim_flight_is_measured(flown):
    f = compare_flights.analyse(flown)
    assert not f.uwb and f.note == ""
    assert f.sweep_s == pytest.approx(f.planned_s, rel=0.2)
    assert f.xtrack_rms < 0.05 and f.corner_mean < 0.1
    assert f.return_err < 0.1 and f.tags == 6
    assert f.drift_end is None                      # no uwb.csv in the sim


def write_uwb(folder, t0, t1, drift_mps):
    """UWB = truth in a rotated, mirrored anchor frame; PX4 = truth + steady drift."""
    rows = list(csv.DictReader(open(folder / "trajectory.csv", encoding="utf-8")))
    th = math.radians(40)
    with open(folder / "uwb.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=UWB_FIELDS)
        w.writeheader()
        for r in rows:
            t = float(r["run_s"])
            if not t0 <= t <= t1:
                continue
            x, y = float(r["x"]), float(r["y"])
            # anchor frame: inverse of p = R @ diag(1,-1) @ u + (2, 3)
            dx, dy = x - 2.0, y - 3.0
            ux = math.cos(th) * dx + math.sin(th) * dy
            uy = -(-math.sin(th) * dx + math.cos(th) * dy)
            drift = drift_mps * (t - t0)
            w.writerow({"run_s": t, "uwb_x": ux, "uwb_y": uy, "uwb_z": -1.0, "eop_max": 0.05,
                        "fix": 1, "px4_x": x + drift, "px4_y": y, "px4_age_ms": 20})


def sweep_times(folder):
    ev = list(csv.DictReader(open(folder / "events.csv", encoding="utf-8")))
    start = next(float(e["run_s"]) for e in ev if e["event"] == "TRAJ_STARTED"
                 and e["details"].startswith("sweep"))
    end = next(float(e["run_s"]) for e in ev if e["event"] == "TRAJ_END"
               and e["details"].startswith("sweep"))
    return start, end


@pytest.mark.parametrize("drift", [0.0, 0.01])
def test_drift_against_uwb(flown, drift, tmp_path):
    folder = tmp_path / "run"
    folder.mkdir()
    for name in ("events.csv", "trajectory.csv", "detections.csv", "tags_final.csv",
                 "config_used.yaml"):
        (folder / name).write_bytes((flown / name).read_bytes())
    t0, t1 = sweep_times(folder)
    write_uwb(folder, t0, t1, drift)
    f = compare_flights.analyse(folder, fit_s=15.0)
    # After the fit window PX4 keeps drifting at `drift` m/s.
    expected = drift * (t1 - t0 - 7.5)
    assert f.drift_end == pytest.approx(expected, abs=0.03)
    plot = tmp_path / "drift.png"
    compare_flights.plot([f], plot)
    assert plot.stat().st_size > 1000


def test_truth_gives_tag_error(flown, tmp_path):
    truth = tmp_path / "truth.csv"
    tags = list(csv.DictReader(open(flown / "tags_final.csv", encoding="utf-8")))
    with open(truth, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["tag_id", "forward", "right"])
        for t in tags:                                # heading 0 at home (0, 0)
            w.writerow([t["tag_id"], float(t["x"]) + 0.05, float(t["y"])])
    f = compare_flights.analyse(flown, truth=compare_flights.read_truth(truth))
    assert f.tag_err == pytest.approx(0.05, abs=0.02)


def test_mission_log_writes_uwb_csv(tmp_path):
    log = MissionLog(tmp_path, config_data={"a": 1}, clock=lambda: 1.0)
    log.uwb(1.0, esp_time_s=2.0, uwb_x=1, uwb_y=2, uwb_z=3, eop_max=0.1, fix=1,
            px4_x=0.5, px4_y=0.6, px4_age_ms=20)
    log.close()
    rows = list(csv.DictReader(open(log.folder / "uwb.csv", encoding="utf-8")))
    assert rows[0]["uwb_x"] == "1" and rows[0]["px4_age_ms"] == "20"


def test_no_uwb_csv_when_nothing_recorded(tmp_path):
    log = MissionLog(tmp_path, config_data={"a": 1})
    log.close()
    assert not (log.folder / "uwb.csv").exists()
