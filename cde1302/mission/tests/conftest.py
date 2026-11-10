"""Make mission/ and the course's laptop/ importable from the tests."""

import sys
from pathlib import Path

import pytest
import yaml

MISSION_DIR = Path(__file__).resolve().parents[1]
LAPTOP_DIR = MISSION_DIR.parent / "laptop"
TMP_DIR = Path(__file__).resolve().parent / ".pytest_tmp"

for p in (MISSION_DIR, LAPTOP_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))


# The simulator tests fly this fixed arena at this fixed speed, whatever
# config.yaml says for demo day: the sim's true tags (config.yaml
# sim.true_tags) lie in it, under its 0.5 m lanes, with the localisation tags
# at its far end, and the timings / drift sizes the tests expect assume
# 0.15 m/s.  Everything else comes from config.yaml as it is.
TEST_ARENA = {
    "area": {"x_min": 0.0, "x_max": 3.5, "y_min": -0.5, "y_max": 1.0, "margin": 0.0},
    "sweep": {"lane_spacing": 0.5},
    "flight": {"cruise_speed_mps": 0.15},
    "localisation": {"tags": [{"id": 29, "x": 0.0, "y": 0.0},
                              {"id": 27, "x": 3.3, "y": -0.3},
                              {"id": 28, "x": 3.3, "y": 0.8}],
                     "truth_tags": []},
}


@pytest.fixture(scope="session", autouse=True)
def test_arena_config(tmp_path_factory):
    """Make load_settings() with no path load config.yaml + TEST_ARENA.

    Tests that need the shipped file itself pass settings.DEFAULT_CONFIG_PATH.
    """
    import settings
    data = yaml.safe_load(settings.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    for section, values in TEST_ARENA.items():
        data[section] = {**data[section], **values}
    path = tmp_path_factory.mktemp("config") / "config.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    old = settings.load_settings.__defaults__
    settings.load_settings.__defaults__ = (path,)
    yield path
    settings.load_settings.__defaults__ = old


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """Keep tmp_path folders in mission/tests/.pytest_tmp/ (gitignored).

    pytest's default, %TEMP%\\pytest-of-<user>, breaks on Windows when that
    folder was once created with other permissions ("Access is denied").
    pytest empties this folder at the start of every run.  Pass --basetemp
    to override.
    """
    if not config.option.basetemp:
        config.option.basetemp = str(TMP_DIR)
