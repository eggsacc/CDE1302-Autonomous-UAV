"""Config loading and validation (mission/settings.py)."""

import copy

import pytest
import yaml

import planner
from settings import DEFAULT_CONFIG_PATH, ConfigError, load_settings, settings_from_dict


@pytest.fixture
def raw() -> dict:
    """The shipped config.yaml as a plain dict, safe to mutate."""
    return copy.deepcopy(yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")))


def test_shipped_config_loads():
    s = load_settings()
    assert s.drone_id == 2
    assert s.esp_ip == "192.168.1.202"
    assert s.flight.arrival_radius_m == 0.25
    assert s.flight.cruise_speed_mps == 0.10
    assert s.tags.tags_on_floor is False          # course: some tags on walls
    assert s.tags.expected_count == 6
    assert len(s.sim.true_tags) == 6
    assert s.source == DEFAULT_CONFIG_PATH
    assert s.logging.log_path.is_absolute()


def test_planner_still_reads_the_same_config():
    wps = planner.plan_from_config(load_settings().raw)
    assert wps[-1] == (0.0, 0.0)
    assert len(wps) == 7


def test_ints_are_accepted_for_floats(raw):
    raw["flight"]["arrival_radius_m"] = 1
    s = settings_from_dict(raw)
    assert s.flight.arrival_radius_m == 1.0
    assert isinstance(s.flight.arrival_radius_m, float)


def test_optional_sections_take_defaults(raw):
    for section in ("network", "flight", "safety", "camera", "tags", "sim", "logging"):
        del raw[section]
    s = settings_from_dict(raw)
    assert s.network.telem_port == 5005
    assert s.sim.true_tags == ()


@pytest.mark.parametrize("section, key, value, message", [
    ("flight", "arrival_radius_m", -0.1, "flight.arrival_radius_m must be > 0"),
    ("flight", "tick_hz", 0, "flight.tick_hz must be > 0"),
    ("flight", "land_repeats", 2.5, "flight.land_repeats must be a whole number"),
    ("safety", "stale_timeout_s", "3", "safety.stale_timeout_s must be a number"),
    ("camera", "fps", 20, "camera.fps must be 0"),
    ("camera", "quality", 5, "camera.quality must be 0"),
    ("tags", "tags_on_floor", "yes", "tags.tags_on_floor must be true or false"),
    ("tags", "outlier_radius_m", 0.1, "outlier_radius_m (0.1) must be >="),
    ("sweep", "lanes_along", "z", "sweep.lanes_along must be 'x' or 'y'"),
    ("area", "x_max", -1.0, "area.x_max (-1.0) must be > area.x_min"),
    ("area", "margin", 0.8, "area.margin (0.8) is more than half"),
    ("logging", "level", "LOUD", "logging.level must be"),
])
def test_bad_values_are_rejected(raw, section, key, value, message):
    raw[section][key] = value
    with pytest.raises(ConfigError, match=message.replace("(", r"\(").replace(")", r"\)")):
        settings_from_dict(raw)


def test_unknown_key_is_rejected_with_its_full_name(raw):
    raw["flight"]["arival_radius_m"] = 0.3            # typo
    with pytest.raises(ConfigError, match=r"unknown key\(s\): flight\.arival_radius_m"):
        settings_from_dict(raw)


def test_unknown_section_is_rejected(raw):
    raw["estimate"] = {"cruise_speed": 0.1}           # the old section name
    with pytest.raises(ConfigError, match="unknown key.*estimate"):
        settings_from_dict(raw)


def test_missing_required_key_is_rejected(raw):
    del raw["area"]["x_max"]
    with pytest.raises(ConfigError, match="missing required key: area.x_max"):
        settings_from_dict(raw)


def test_bad_true_tag_entries(raw):
    raw["sim"]["true_tags"] = [{"id": 3, "x": 0.1, "y": 0.2},
                               {"id": 3, "x": 0.5, "y": 0.5}]
    with pytest.raises(ConfigError, match="duplicate ids"):
        settings_from_dict(raw)
    raw["sim"]["true_tags"] = [{"id": 3, "x": 0.1}]
    with pytest.raises(ConfigError, match=r"missing required key: sim.true_tags\[0\].y"):
        settings_from_dict(raw)


def test_load_settings_names_the_file(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("drone_id: 2\narea: {x_min: 0, x_max: 1, y_min: 0, y_max: 1}\n"
                   "sweep: {lane_spacing: -1}\n", encoding="utf-8")
    with pytest.raises(ConfigError, match=r"^bad\.yaml: sweep\.lane_spacing must be > 0"):
        load_settings(bad)


def test_unreadable_and_invalid_yaml(tmp_path):
    with pytest.raises(ConfigError, match="cannot read config"):
        load_settings(tmp_path / "nope.yaml")
    broken = tmp_path / "broken.yaml"
    broken.write_text("area: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="not valid YAML"):
        load_settings(broken)
