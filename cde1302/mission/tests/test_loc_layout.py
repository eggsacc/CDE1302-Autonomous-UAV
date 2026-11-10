"""mission/tools/loc_layout.py: grid nodes <-> arena metres, rewriting config.yaml."""

import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import loc_layout as L  # noqa: E402
from settings import DEFAULT_CONFIG_PATH, ConfigError, settings_from_dict  # noqa: E402


@pytest.fixture
def text():
    return DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")


def test_node_to_metres_matches_the_team_convention():
    assert L.node_to_xy((1, 0)) == (0.0, 0.46)          # a = sideways (y)
    assert L.node_to_xy((0, 1)) == (0.46, 0.0)          # b = forward (x)
    assert L.node_to_xy((-4, 2)) == (0.92, -1.84)       # play-area corner
    assert L.node_to_xy((3, 10)) == (4.6, 1.38)
    assert L.xy_to_node(4.6, -1.38) == (-3, 10)
    assert L.xy_to_node(1.30, -1.35) is None            # not on a node


def test_grid_covers_the_play_area_and_the_start():
    area = {"x_min": 0.92, "x_max": 4.6, "y_min": -1.84, "y_max": 1.38}   # nodes (-4..3, 2..10)
    cols, rows = L.grid_extent(area)
    assert (cols.start, cols[-1], rows.start, rows[-1]) == (-4, 3, 0, 10)


def test_round_trip_rewrites_only_the_tag_lines(text):
    layout = {29: (0, 0), 27: (-3, 10), 28: (2, 10), 22: (-4, 6), 23: (3, 4)}
    new = L.update_config_text(text, layout)
    s = settings_from_dict(yaml.safe_load(new))
    lo = s.localisation
    assert lo.home_tag_id == 29
    assert {t.id: (t.x, t.y) for t in lo.tags} == {
        29: (0.0, 0.0), 27: (4.6, -1.38), 28: (4.6, 0.92), 22: (2.76, -1.84), 23: (1.84, 1.38)}
    assert L.read_layout(new)[0] == layout
    # Everything outside the tag list is untouched (comments included).
    old_lines = [ln for ln in text.splitlines() if not ln.startswith("    - {id: 2")]
    new_lines = [ln for ln in new.splitlines() if not ln.startswith("    - {id: 2")]
    assert new_lines == old_lines


def test_the_home_tag_is_whichever_is_on_the_start_node(text):
    new = L.update_config_text(text, {25: (0, 0), 27: (-3, 10)})
    assert settings_from_dict(yaml.safe_load(new)).localisation.home_tag_id == 25


@pytest.mark.parametrize("layout,msg", [
    ({27: (-3, 10)}, "start node"),
    ({}, "no localisation tags"),
    ({29: (0, 0), 12: (1, 5)}, "not localisation ids"),
    ({29: (0, 0), 27: (0, 0)}, "same node"),
])
def test_bad_layouts_are_refused(text, layout, msg):
    with pytest.raises(ValueError, match=msg):
        L.update_config_text(text, layout)


def test_save_keeps_a_backup(tmp_path, text):
    cfg = tmp_path / "config.yaml"
    cfg.write_text(text, encoding="utf-8")
    L.save_layout(cfg, {29: (0, 0), 24: (-1, 8)})
    assert (tmp_path / "config.yaml.bak").read_text(encoding="utf-8") == text
    assert L.read_layout(cfg.read_text(encoding="utf-8"))[0] == {29: (0, 0), 24: (-1, 8)}


def test_a_tag_off_the_grid_is_reported(text):
    on = L.update_config_text(text, {29: (0, 0), 27: (-3, 10)})
    off = on.replace("{id: 27, x: 4.6, y: -1.38}", "{id: 27, x: 4.5, y: -1.38}")
    layout, warn = L.read_layout(off)
    assert 27 not in layout and "not on a grid node" in warn[0]
