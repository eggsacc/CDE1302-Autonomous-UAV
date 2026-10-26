"""
Typed, validated mission settings loaded from mission/config.yaml.

    from settings import load_settings
    s = load_settings()                     # or load_settings("other.yaml")
    s.flight.arrival_radius_m               # 0.25
    s.raw                                   # the plain dict, for planner.py

Each YAML section maps to a frozen dataclass below.  Loading fails with a
ConfigError naming the offending key when a key is missing, unknown (catches
typos), the wrong type, or out of range.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, get_type_hints

import yaml

MISSION_DIR = Path(__file__).resolve().parent
REPO_DIR = MISSION_DIR.parent              # cde1302/
DEFAULT_CONFIG_PATH = MISSION_DIR / "config.yaml"


class ConfigError(ValueError):
    """config.yaml is missing a key, has an unknown key, or a bad value."""


# ---------------------------------------------------------------------------
# Sections
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Area:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    margin: float = 0.0


@dataclass(frozen=True)
class Sweep:
    lane_spacing: float
    lanes_along: str = "x"


@dataclass(frozen=True)
class Network:
    telem_port: int = 5005
    cmd_port: int = 5006
    camera_port: int = 5009
    connect_timeout_s: float = 30.0
    pose_history_s: float = 30.0
    pose_lookup_tolerance_s: float = 0.25


@dataclass(frozen=True)
class Flight:
    cruise_speed_mps: float = 0.10
    turn_time_s: float = 4.0
    takeoff_wait_s: float = 8.0
    hover_radius_m: float = 0.10
    hover_settle_s: float = 2.0
    hover_timeout_s: float = 20.0
    arrival_radius_m: float = 0.25
    waypoint_timeout_margin_s: float = 10.0
    goto_resend_s: float = 2.0
    start_repeats: int = 3
    land_repeats: int = 3
    repeat_gap_s: float = 0.1
    tick_hz: float = 10.0
    stop_when_all_found: bool = True
    land_settle_s: float = 6.0


@dataclass(frozen=True)
class Safety:
    stale_timeout_s: float = 3.0
    geofence_margin_m: float = 0.5
    mission_time_limit_s: float = 600.0


@dataclass(frozen=True)
class Camera:
    fps: int = 0
    quality: int = 0
    capture_latency_bias_s: float = 0.0


@dataclass(frozen=True)
class Tags:
    tag_size_m: float = 0.12
    min_decision_margin: float = 55.0
    max_hamming: int = 1
    max_pose_err: float = 0.5
    max_detect_range_m: float = 1.5
    max_yaw_rate_deg_s: float = 30.0
    tags_on_floor: bool = False
    expected_down_m: float = 0.5
    down_tolerance_m: float = 0.15
    min_down_m: float = -0.5
    max_down_m: float = 0.65
    expected_count: int = 0
    min_sightings: int = 5
    confirm_radius_m: float = 0.20
    outlier_radius_m: float = 0.30
    max_sightings_per_tag: int = 400


@dataclass(frozen=True)
class TrueTag:
    id: int
    x: float
    y: float


@dataclass(frozen=True)
class Sim:
    speed: float = 1.0
    seed: int = 1
    arm_delay_s: float = 2.0
    climb_rate_mps: float = 0.2
    yaw_rate_deg_s: float = 45.0
    command_loss: float = 0.0
    camera_fps: float = 9.0
    false_positive_rate: float = 0.0
    pose_noise_m: float = 0.0
    sighting_noise_m: float = 0.03
    latency_s: float = 0.13
    fov_half_angle_deg: float = 44.0
    view_min_m: float = 0.1
    view_max_m: float = 1.2
    true_tags: tuple[TrueTag, ...] = ()


@dataclass(frozen=True)
class Logging:
    log_dir: str = "mission/logs"
    level: str = "INFO"
    status_period_s: float = 5.0

    @property
    def log_path(self) -> Path:
        """log_dir as an absolute path; relative paths are from cde1302/."""
        p = Path(self.log_dir)
        return p if p.is_absolute() else REPO_DIR / p


@dataclass(frozen=True)
class Settings:
    drone_id: int
    area: Area
    sweep: Sweep
    network: Network = field(default_factory=Network)
    flight: Flight = field(default_factory=Flight)
    safety: Safety = field(default_factory=Safety)
    camera: Camera = field(default_factory=Camera)
    tags: Tags = field(default_factory=Tags)
    sim: Sim = field(default_factory=Sim)
    logging: Logging = field(default_factory=Logging)
    source: Path | None = None                 # file it was loaded from
    raw: dict = field(default_factory=dict, compare=False, repr=False)

    @property
    def esp_ip(self) -> str:
        """Drone ESP32 IP by the course convention 192.168.1.(200 + id)."""
        return f"192.168.1.{200 + self.drone_id}"


# ---------------------------------------------------------------------------
# Building dataclasses from dicts
# ---------------------------------------------------------------------------

_NOT_FROM_YAML = {"source", "raw"}


def _coerce(value: Any, typ: Any, where: str) -> Any:
    """Check (and lightly convert) one YAML value against a field type."""
    if typ is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{where} must be true or false (got {value!r})")
        return value
    if typ is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{where} must be a whole number (got {value!r})")
        return value
    if typ is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{where} must be a number (got {value!r})")
        if not math.isfinite(value):
            raise ConfigError(f"{where} must be finite (got {value!r})")
        return float(value)
    if typ is str:
        if not isinstance(value, str):
            raise ConfigError(f"{where} must be text (got {value!r})")
        return value
    if typ == tuple[TrueTag, ...]:
        if value is None:
            return ()
        if not isinstance(value, list):
            raise ConfigError(f"{where} must be a list of {{id, x, y}} (got {value!r})")
        return tuple(_build(TrueTag, item, f"{where}[{i}]")
                     for i, item in enumerate(value))
    if dataclasses.is_dataclass(typ):
        return _build(typ, value, where)
    raise TypeError(f"no coercion for {typ!r}")   # programming error


def _build(cls: type, data: Any, where: str) -> Any:
    """Build dataclass `cls` from a dict, rejecting unknown and missing keys."""
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError(f"{where or 'config'} must be a mapping (got {data!r})")
    hints = get_type_hints(cls)
    fields = {f.name: f for f in dataclasses.fields(cls)
              if f.name not in _NOT_FROM_YAML}
    unknown = sorted(set(map(str, data)) - set(fields))
    if unknown:
        prefix = f"{where}." if where else ""
        raise ConfigError(f"unknown key(s): {', '.join(prefix + k for k in unknown)} "
                          f"(allowed: {', '.join(fields)})")
    kwargs = {}
    for name, f in fields.items():
        key = f"{where}.{name}" if where else name
        if name in data:
            kwargs[name] = _coerce(data[name], hints[name], key)
        elif (f.default is dataclasses.MISSING
              and f.default_factory is dataclasses.MISSING):
            raise ConfigError(f"missing required key: {key}")
    return cls(**kwargs)


# ---------------------------------------------------------------------------
# Range checks
# ---------------------------------------------------------------------------

def _check(ok: bool, msg: str) -> None:
    if not ok:
        raise ConfigError(msg)


def _positive(obj: Any, section: str, *names: str) -> None:
    for n in names:
        v = getattr(obj, n)
        _check(v > 0, f"{section}.{n} must be > 0 (got {v})")


def _non_negative(obj: Any, section: str, *names: str) -> None:
    for n in names:
        v = getattr(obj, n)
        _check(v >= 0, f"{section}.{n} must be >= 0 (got {v})")


def validate(s: Settings) -> None:
    """Raise ConfigError if any value is out of range or inconsistent."""
    _check(0 <= s.drone_id <= 54,
           f"drone_id must be 0-54 so 200 + id is a valid IP byte (got {s.drone_id})")

    a = s.area
    _check(a.x_max > a.x_min, f"area.x_max ({a.x_max}) must be > area.x_min ({a.x_min})")
    _check(a.y_max > a.y_min, f"area.y_max ({a.y_max}) must be > area.y_min ({a.y_min})")
    _non_negative(a, "area", "margin")
    _check(2 * a.margin <= min(a.x_max - a.x_min, a.y_max - a.y_min),
           f"area.margin ({a.margin}) is more than half the area's smaller side")

    _positive(s.sweep, "sweep", "lane_spacing")
    _check(s.sweep.lanes_along in ("x", "y"),
           f"sweep.lanes_along must be 'x' or 'y' (got {s.sweep.lanes_along!r})")

    n = s.network
    for name in ("telem_port", "cmd_port", "camera_port"):
        port = getattr(n, name)
        _check(1 <= port <= 65535, f"network.{name} must be 1-65535 (got {port})")
    _check(len({n.telem_port, n.camera_port}) == 2,
           "network.telem_port and network.camera_port must differ")
    _positive(n, "network", "connect_timeout_s", "pose_history_s")
    _non_negative(n, "network", "pose_lookup_tolerance_s")

    f = s.flight
    _positive(f, "flight", "cruise_speed_mps", "hover_radius_m", "hover_timeout_s",
              "arrival_radius_m", "goto_resend_s", "start_repeats",
              "land_repeats", "tick_hz")
    _non_negative(f, "flight", "turn_time_s", "takeoff_wait_s", "hover_settle_s",
                  "waypoint_timeout_margin_s", "repeat_gap_s", "land_settle_s")
    _check(f.tick_hz <= 100, f"flight.tick_hz must be <= 100 (got {f.tick_hz})")

    _positive(s.safety, "safety", "stale_timeout_s", "mission_time_limit_s")
    _non_negative(s.safety, "safety", "geofence_margin_m")

    c = s.camera
    _check(0 <= c.fps <= 15, f"camera.fps must be 0 (firmware default) or 1-15 (got {c.fps})")
    _check(c.quality == 0 or 10 <= c.quality <= 90,
           f"camera.quality must be 0 (firmware default) or 10-90 (got {c.quality})")

    t = s.tags
    _positive(t, "tags", "tag_size_m", "max_pose_err", "max_detect_range_m",
              "max_yaw_rate_deg_s", "min_sightings", "confirm_radius_m",
              "outlier_radius_m")
    _check(t.max_sightings_per_tag >= t.min_sightings,
           f"tags.max_sightings_per_tag ({t.max_sightings_per_tag}) must be >= "
           f"tags.min_sightings ({t.min_sightings})")
    _non_negative(t, "tags", "min_decision_margin", "max_hamming",
                  "expected_down_m", "down_tolerance_m", "expected_count")
    _check(t.max_down_m > t.min_down_m,
           f"tags.max_down_m ({t.max_down_m}) must be > tags.min_down_m ({t.min_down_m})")
    _check(t.outlier_radius_m >= t.confirm_radius_m,
           f"tags.outlier_radius_m ({t.outlier_radius_m}) must be >= "
           f"tags.confirm_radius_m ({t.confirm_radius_m})")

    m = s.sim
    _positive(m, "sim", "speed", "fov_half_angle_deg", "view_max_m", "climb_rate_mps",
              "yaw_rate_deg_s", "camera_fps")
    _non_negative(m, "sim", "pose_noise_m", "sighting_noise_m", "latency_s", "view_min_m",
                  "arm_delay_s")
    for name in ("command_loss", "false_positive_rate"):
        v = getattr(m, name)
        _check(0.0 <= v <= 1.0, f"sim.{name} must be between 0 and 1 (got {v})")
    _check(m.view_max_m > m.view_min_m,
           f"sim.view_max_m ({m.view_max_m}) must be > sim.view_min_m ({m.view_min_m})")
    _check(m.fov_half_angle_deg < 90,
           f"sim.fov_half_angle_deg must be < 90 (got {m.fov_half_angle_deg})")
    ids = [tag.id for tag in m.true_tags]
    _check(len(ids) == len(set(ids)), f"sim.true_tags has duplicate ids: {ids}")
    for tag in m.true_tags:
        # This repo's tag16h5 table is truncated to ids 0-21 (apriltag_host.py).
        _check(0 <= tag.id <= 21,
               f"sim.true_tags id {tag.id} is outside the detectable ids 0-21")

    _positive(s.logging, "logging", "status_period_s")
    _check(s.logging.level.upper() in ("DEBUG", "INFO", "WARNING", "ERROR"),
           f"logging.level must be DEBUG, INFO, WARNING or ERROR "
           f"(got {s.logging.level!r})")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def settings_from_dict(data: dict, source: Path | None = None) -> Settings:
    """Build and validate Settings from an already-parsed config dict."""
    s = _build(Settings, data, "")
    s = dataclasses.replace(s, source=source, raw=data)
    validate(s)
    return s


def load_settings(path: str | Path = DEFAULT_CONFIG_PATH) -> Settings:
    """Load, check and return the mission settings from a YAML file."""
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read config {path}: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    try:
        return settings_from_dict(data, source=path.resolve())
    except ConfigError as exc:
        raise ConfigError(f"{path.name}: {exc}") from None
