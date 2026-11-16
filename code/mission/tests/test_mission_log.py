"""Event logger and CSV writers (mission/mission_log.py)."""

import csv
import threading

import pytest

from mission_log import (DETECTION_FIELDS, EV_LAND, EV_TAG, EV_TAKEOFF,
                         MissionLog, format_wall)
from settings import DEFAULT_CONFIG_PATH

WALL0 = 1_790_000_000.0          # fixed Unix time so wall columns are predictable


class FakeClock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def mlog(tmp_path, clock):
    lines: list[str] = []
    m = MissionLog(tmp_path, DEFAULT_CONFIG_PATH, clock=clock,
                   wall_clock=lambda: WALL0, echo=lines.append, run_name="run")
    m.lines = lines
    yield m
    m.close()


def read_csv(path) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def test_run_folder_contents(mlog, tmp_path):
    assert mlog.folder == tmp_path / "run"
    for name in ("events.csv", "trajectory.csv", "detections.csv", "config_used.yaml"):
        assert (mlog.folder / name).is_file()
    assert (mlog.folder / "config_used.yaml").read_text(encoding="utf-8") == \
        DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")


def test_same_name_gets_a_new_folder(tmp_path, clock):
    a = MissionLog(tmp_path, clock=clock, run_name="r", echo=lambda _: None)
    b = MissionLog(tmp_path, clock=clock, run_name="r", echo=lambda _: None)
    assert a.folder != b.folder and b.folder.name == "r_2"
    a.close()
    b.close()


def test_config_data_is_dumped_when_there_is_no_file(tmp_path, clock):
    m = MissionLog(tmp_path, config_data={"drone_id": 2}, clock=clock, echo=lambda _: None)
    assert "drone_id: 2" in (m.folder / "config_used.yaml").read_text(encoding="utf-8")
    m.close()


def test_events_have_correct_timestamps(mlog, clock):
    clock.t = 101.0
    mlog.event("CONNECTED", "drone 2")                   # before takeoff
    clock.t = 105.25
    mlog.takeoff_sent()
    clock.t = 147.5
    mlog.tag_confirmed(3, 0.8123, 0.3049, 5)
    clock.t = 190.0
    mlog.land_sent("search complete")

    rows = read_csv(mlog.folder / "events.csv")
    assert [r["event"] for r in rows] == ["CONNECTED", EV_TAKEOFF, EV_TAG, EV_LAND]
    assert [r["run_s"] for r in rows] == ["1.0000", "5.2500", "47.5000", "90.0000"]
    assert [r["mission_s"] for r in rows] == ["", "0.0000", "42.2500", "84.7500"]
    assert rows[1]["wall_time"] == format_wall(WALL0 + 5.25)
    assert rows[2]["details"] == "id=3 x=0.812 y=0.305 (5 sightings)"
    assert rows[3]["details"] == "CMD_LAND sent (search complete)"


def test_only_required_events_are_echoed(mlog, clock):
    mlog.event("STATE", "READY -> TAKING_OFF")
    clock.t = 102.0
    mlog.takeoff_sent()
    clock.t = 112.345
    mlog.tag_confirmed(7, 1.2, -0.5, 6)
    assert len(mlog.lines) == 2
    assert mlog.lines[0].startswith(f"[EVENT] {format_wall(WALL0 + 2.0)}  T+   0.000 s  TAKEOFF_CMD")
    assert "T+  10.345 s  TAG_CONFIRMED id=7 x=1.200 y=-0.500" in mlog.lines[1]


def test_wall_time_has_milliseconds():
    assert format_wall(WALL0 + 0.0456).endswith(".045")


def test_second_takeoff_does_not_reset_mission_clock(mlog, clock):
    mlog.takeoff_sent()
    clock.t = 110.0
    mlog.takeoff_sent("resent")
    assert mlog.mission_elapsed() == pytest.approx(10.0)


def test_trajectory_and_detection_rows(mlog, clock):
    mlog.takeoff_sent()
    mlog.trajectory(100.1, 0.5, -0.25, 1.5708, "FLYING")
    clock.t = 101.0
    mlog.detection(101.0, 100.87, tag_id=3, body_f=0.6, body_r=0.1, body_d=0.5,
                   pose_err=0.01, tag_x=1.1, tag_y=0.2)

    traj = read_csv(mlog.folder / "trajectory.csv")
    assert traj == [{"wall_time": format_wall(WALL0 + 0.1), "run_s": "0.1000",
                     "mission_s": "0.1000", "x": "0.5000", "y": "-0.2500",
                     "heading_rad": "1.5708", "nav_state": "FLYING"}]

    det = read_csv(mlog.folder / "detections.csv")[0]
    assert list(det) == DETECTION_FIELDS
    assert det["tag_id"] == "3"
    assert det["capture_mission_s"] == "0.8700"
    assert det["capture_wall_time"] == format_wall(WALL0 + 0.87)
    assert det["R00"] == ""                                  # not given -> blank


def test_unknown_detection_column_is_an_error(mlog):
    with pytest.raises(ValueError):
        mlog.detection(100.0, 100.0, tag_idd=3)


def test_tags_final(mlog, clock):
    mlog.takeoff_sent()
    path = mlog.write_tags_final([
        {"tag_id": 3, "x": 0.81, "y": 0.3, "n_sightings": 12,
         "first_seen_t": 120.0, "confirmed_t": 121.5},
        {"tag_id": 7, "x": 1.2, "y": 0.8, "n_sightings": 2,
         "first_seen_t": 130.0, "confirmed_t": None},
    ])
    rows = read_csv(path)
    assert rows[0]["first_seen_mission_s"] == "20.0000"
    assert rows[0]["confirmed_wall_time"] == format_wall(WALL0 + 21.5)
    assert rows[1]["confirmed_mission_s"] == ""


def test_rows_are_on_disk_before_close(mlog):
    mlog.event("X")
    assert len(read_csv(mlog.folder / "events.csv")) == 1   # flushed per row


def test_writes_after_close_are_dropped(mlog):
    mlog.close()
    mlog.event("LATE")
    mlog.trajectory(100.0, 0, 0, 0, "IDLE")
    assert read_csv(mlog.folder / "events.csv") == []


def test_concurrent_writers_do_not_interleave(mlog):
    def worker(k):
        for i in range(200):
            mlog.event(f"E{k}", str(i))
            mlog.trajectory(100.0, k, i, 0.0, "FLYING")

    threads = [threading.Thread(target=worker, args=(k,)) for k in range(4)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    assert len(read_csv(mlog.folder / "events.csv")) == 800
    assert len(read_csv(mlog.folder / "trajectory.csv")) == 800
