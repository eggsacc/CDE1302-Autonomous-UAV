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
from typing import Any, Optional, get_type_hints

import yaml

MISSION_DIR = Path(__file__).resolve().parent
REPO_DIR = MISSION_DIR.parent              # cde1302/
DEFAULT_CONFIG_PATH = MISSION_DIR / "config.yaml"

FIRMWARE_SPEED_CAP_MPS = 0.50    # MAV_CMD_SPEED_CAP_MS in main/mavlink_task.h
TAG16H5_MAX_ID = 29              # components/esp-apriltag/apriltag/tag16h5.c: ids 0-29
LOC_MODELS = ("similarity", "rigid", "translation")   # localisation.py, most to fewest DOF
DRIFT_MODELS = ("auto", "global", "time", "hybrid")   # localisation.drift_correction


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
    camera_offset_m: float = 0.6
    start_at_home: bool = False
    start_in_middle: bool = False


@dataclass(frozen=True)
class Network:
    telem_port: int = 5005
    cmd_port: int = 5006
    camera_port: int = 5009
    connect_timeout_s: float = 30.0
    pose_history_s: float = 30.0
    pose_lookup_tolerance_s: float = 0.25
    mavlink_bridge_port: int = 8888    # ESP32 port that forwards MAVLink to PX4 (CONFIG_MAVROS_BRIDGE_IN_PORT)
    mavlink_listen_port: int = 14550   # laptop port the ESP32 forwards PX4's MAVLink to (..._OUT_PORT)


@dataclass(frozen=True)
class Flight:
    nav_mode: str = "trajectory"
    cruise_speed_mps: float = 0.10
    traj_dt_s: float = 0.05
    traj_max_speed_mps: float = 0.30
    traj_upload_gap_s: float = 0.025
    traj_start_timeout_s: float = 1.5
    traj_start_attempts: int = 5
    traj_end_margin_s: float = 10.0
    traj_timeout_factor: float = 4.0
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
    land_radius_m: float = 0.05
    land_hold_s: float = 1.0
    land_settle_max_s: float = 10.0


@dataclass(frozen=True)
class Safety:
    stale_timeout_s: float = 3.0
    geofence_margin_m: float = 0.5
    mission_time_limit_s: float = 600.0
    require_gyro_cal: bool = False     # refuse `takeoff` until a `calibrate` succeeded
    gyro_cal_timeout_s: float = 30.0   # give up on a calibration after this (s)


@dataclass(frozen=True)
class Camera:
    fps: int = 0
    quality: int = 0
    capture_latency_bias_s: float = 0.0
    use_frame_pose: bool = True        # drone pose sent inside each frame (no Wi-Fi delay)
    max_frame_pose_gap_ms: int = 100   # ignore a frame pose this far from a PX4 sample
    fx: float = 0.0                    # lens focal length (px); 0 = the course's (apriltag_host)
    fy: float = 0.0


@dataclass(frozen=True)
class Tags:
    @property
    def camera_pos_m(self) -> tuple[float, float, float]:
        """Lens position in the body frame: forward, right, down (m)."""
        return (self.camera_forward_m, self.camera_right_m, self.camera_down_m)


    tag_size_m: float = 0.12
    camera_pitch_deg: float = 90.0
    camera_forward_m: float = 0.02     # lens ahead of the drone centre (m)
    camera_right_m: float = 0.0        # lens right of the drone centre (m)
    camera_down_m: float = 0.0         # lens below the drone centre (m)
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
    gyro_cal_s: float = 2.0            # fake gyro calibration time (s)
    view_half_width_m: float = 0.40
    view_min_m: float = -0.35
    view_max_m: float = 0.28
    true_tags: tuple[TrueTag, ...] = ()
    drift_scale: float = 0.0           # odometry scale error: PX4 sees (1 + this) x the real distance
    drift_rot_deg: float = 0.0         # PX4's frame turned this far from the real one (deg, cw +)
    drift_x_mps: float = 0.0           # PX4's position creeps this fast north while airborne (m/s)
    drift_y_mps: float = 0.0           # ...and east (m/s)


@dataclass(frozen=True)
class Localisation:
    """Known-position tags for drift correction and precision landing (localisation.py)."""
    enabled: bool = False
    home_tag_id: int = 29
    tags: tuple[TrueTag, ...] = ()     # arena-frame positions (m)
    model: str = "similarity"
    visit_gap_s: float = 5.0
    min_visit_sightings: int = 5
    min_span_m: float = 1.0
    max_rms_m: float = 0.15
    max_rotation_deg: float = 20.0
    max_scale_error: float = 0.25
    precision_landing: bool = True
    takeoff_home_tag: bool = True
    align_to_home_tag: bool = True
    align_max_deg: float = 15.0
    save_plot: bool = True             # correction.png + trajectory_corrected.csv after landing
    tag_size_m: float = 0.0            # printed size of the localisation tags; 0 = tags.tag_size_m
    drift_model: str = "auto"          # global | time | hybrid | auto (leave-one-out picks)
    truth_tags: tuple[TrueTag, ...] = ()   # test flights: true search-tag spots, for errors
    land_window_s: float = 2.0
    land_min_sightings: int = 5
    land_max_offset_m: float = 1.0
    land_max_corrections: int = 3

    @property
    def ids(self) -> frozenset[int]:
        return frozenset(t.id for t in self.tags) if self.enabled else frozenset()

    def ref(self, tag_id: int) -> Optional[tuple[float, float]]:
        """Arena-frame (x, y) of a localisation tag, or None."""
        for t in self.tags:
            if t.id == tag_id:
                return (t.x, t.y)
        return None


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
class LiveMap:
    redraw_hz: float = 5.0
    arrow_m: float = 0.25
    view_margin_m: float = 0.4
    save_png: bool = True
    show_camera: bool = True
    camera_stale_s: float = 2.0


@dataclass(frozen=True)
class Uwb:
    send_to_px4: bool = False
    record: bool = False
    debug_port: int = 5010
    max_start_jump_m: float = 0.30
    align_solve_period_s: float = 1.0
    align_max_px4_age_ms: int = 200
    mirror: str = "auto"
    align_min_samples: int = 50
    align_min_span_m: float = 0.8
    align_min_spread_ratio: float = 0.05
    align_max_eop_m: float = 0.30
    align_max_rms_m: float = 0.10
    align_mirror_margin: float = 2.0
    align_outlier_k: float = 3.0


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
    live_map: LiveMap = field(default_factory=LiveMap)
    uwb: Uwb = field(default_factory=Uwb)
    localisation: Localisation = field(default_factory=Localisation)
    source: Path | None = None               # file it was loaded from
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
    _non_negative(s.sweep, "sweep", "camera_offset_m")
    _check(s.sweep.lanes_along in ("x", "y"),
           f"sweep.lanes_along must be 'x' or 'y' (got {s.sweep.lanes_along!r})")

    n = s.network
    for name in ("telem_port", "cmd_port", "camera_port", "mavlink_bridge_port",
                 "mavlink_listen_port"):
        port = getattr(n, name)
        _check(1 <= port <= 65535, f"network.{name} must be 1-65535 (got {port})")
    _check(len({n.telem_port, n.camera_port, n.mavlink_listen_port}) == 3,
           "network.telem_port, camera_port and mavlink_listen_port must all differ")
    _positive(n, "network", "connect_timeout_s", "pose_history_s")
    _non_negative(n, "network", "pose_lookup_tolerance_s")

    f = s.flight
    _positive(f, "flight", "land_radius_m", "land_settle_max_s")
    _positive(f, "flight", "cruise_speed_mps", "hover_radius_m", "hover_timeout_s",
              "arrival_radius_m", "goto_resend_s", "start_repeats",
              "land_repeats", "tick_hz")
    _non_negative(f, "flight", "turn_time_s", "takeoff_wait_s", "hover_settle_s",
                  "waypoint_timeout_margin_s", "repeat_gap_s", "land_settle_s",
                  "land_hold_s")
    _check(f.tick_hz <= 100, f"flight.tick_hz must be <= 100 (got {f.tick_hz})")
    _check(f.nav_mode in ("trajectory", "waypoints"),
           f"flight.nav_mode must be 'trajectory' or 'waypoints' (got {f.nav_mode!r})")
    _positive(f, "flight", "traj_start_timeout_s", "traj_start_attempts")
    _non_negative(f, "flight", "traj_upload_gap_s", "traj_end_margin_s")
    _check(f.traj_timeout_factor >= 1.0,
           f"flight.traj_timeout_factor must be >= 1 (got {f.traj_timeout_factor})")
    _check(0.001 <= f.traj_dt_s <= 1.0,
           f"flight.traj_dt_s must be 0.001-1.0 s (got {f.traj_dt_s})")
    _check(0 < f.traj_max_speed_mps <= FIRMWARE_SPEED_CAP_MPS,
           f"flight.traj_max_speed_mps must be > 0 and <= the firmware cap "
           f"{FIRMWARE_SPEED_CAP_MPS} (got {f.traj_max_speed_mps})")
    if f.nav_mode == "trajectory":
        _check(f.cruise_speed_mps <= f.traj_max_speed_mps,
               f"flight.cruise_speed_mps ({f.cruise_speed_mps}) must be <= "
               f"flight.traj_max_speed_mps ({f.traj_max_speed_mps})")

    _positive(s.safety, "safety", "stale_timeout_s", "mission_time_limit_s",
              "gyro_cal_timeout_s")
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
    _check(all(abs(v) <= 0.5 for v in t.camera_pos_m),
           f"tags.camera_forward_m/right_m/down_m must be within 0.5 m (got {t.camera_pos_m})")
    _non_negative(c, "camera", "fx", "fy")
    _check(0 <= c.max_frame_pose_gap_ms <= 1000,
           "camera.max_frame_pose_gap_ms must be 0-1000")
    _check(0.0 <= t.camera_pitch_deg <= 90.0,
           f"tags.camera_pitch_deg must be between 0 and 90 (got {t.camera_pitch_deg})")
    _check(t.max_down_m > t.min_down_m,
           f"tags.max_down_m ({t.max_down_m}) must be > tags.min_down_m ({t.min_down_m})")
    _check(t.outlier_radius_m >= t.confirm_radius_m,
           f"tags.outlier_radius_m ({t.outlier_radius_m}) must be >= "
           f"tags.confirm_radius_m ({t.confirm_radius_m})")

    m = s.sim
    _positive(m, "sim", "speed", "view_half_width_m", "climb_rate_mps",
              "yaw_rate_deg_s", "camera_fps")
    _non_negative(m, "sim", "pose_noise_m", "sighting_noise_m", "latency_s", "arm_delay_s",
                  "gyro_cal_s")
    for name in ("command_loss", "false_positive_rate"):
        v = getattr(m, name)
        _check(0.0 <= v <= 1.0, f"sim.{name} must be between 0 and 1 (got {v})")
    _check(m.view_max_m > m.view_min_m,
           f"sim.view_max_m ({m.view_max_m}) must be > sim.view_min_m ({m.view_min_m})")
    ids = [tag.id for tag in m.true_tags]
    _check(len(ids) == len(set(ids)), f"sim.true_tags has duplicate ids: {ids}")
    for tag in m.true_tags:
        _check(0 <= tag.id <= TAG16H5_MAX_ID,
               f"sim.true_tags id {tag.id} is outside the tag16h5 ids 0-{TAG16H5_MAX_ID}")
    _check(m.drift_scale > -0.5, f"sim.drift_scale must be > -0.5 (got {m.drift_scale})")

    lo = s.localisation
    loc_ids = [tag.id for tag in lo.tags]
    _check(len(loc_ids) == len(set(loc_ids)), f"localisation.tags has duplicate ids: {loc_ids}")
    truth_ids = [tag.id for tag in lo.truth_tags]
    _check(len(truth_ids) == len(set(truth_ids)),
           f"localisation.truth_tags has duplicate ids: {truth_ids}")
    for tid in loc_ids + truth_ids:
        _check(0 <= tid <= TAG16H5_MAX_ID,
               f"localisation.tags id {tid} is outside the tag16h5 ids 0-{TAG16H5_MAX_ID}")
    _check(lo.model in LOC_MODELS,
           f"localisation.model must be one of {', '.join(LOC_MODELS)} (got {lo.model!r})")
    _positive(lo, "localisation", "visit_gap_s", "min_visit_sightings", "max_rms_m",
              "land_window_s", "land_min_sightings", "land_max_offset_m")
    _non_negative(lo, "localisation", "min_span_m", "max_rotation_deg", "max_scale_error",
                  "land_max_corrections")
    _check(0.0 < lo.align_max_deg < 45.0,
           f"localisation.align_max_deg must be between 0 and 45 (got {lo.align_max_deg})")
    _non_negative(lo, "localisation", "tag_size_m")
    _check(lo.drift_model in DRIFT_MODELS,
           f"localisation.drift_model must be one of {', '.join(DRIFT_MODELS)} "
           f"(got {lo.drift_model!r})")
    _check(lo.max_scale_error < 1.0,
           f"localisation.max_scale_error must be < 1 (got {lo.max_scale_error})")
    if lo.enabled:
        _check(bool(lo.tags), "localisation.enabled needs at least one entry in localisation.tags")
        clash = sorted(set(loc_ids) & set(ids))
        _check(not clash, f"sim.true_tags uses localisation tag id(s) {clash}")
        if lo.precision_landing or lo.takeoff_home_tag or lo.align_to_home_tag:
            _check(lo.home_tag_id in loc_ids,
                   f"localisation.home_tag_id {lo.home_tag_id} is not in localisation.tags")

    _positive(s.logging, "logging", "status_period_s")
    _positive(s.live_map, "live_map", "redraw_hz", "arrow_m", "camera_stale_s")

    u = s.uwb
    _check(u.mirror in ("auto", "yes", "no"),
           f"uwb.mirror must be 'auto', 'yes' or 'no' (got {u.mirror!r})")
    _positive(u, "uwb", "align_min_samples", "align_min_span_m", "align_max_eop_m",
              "align_max_rms_m", "align_outlier_k", "max_start_jump_m",
              "align_solve_period_s", "align_max_px4_age_ms")
    _check(1 <= u.debug_port <= 65535, f"uwb.debug_port out of range (got {u.debug_port})")
    _non_negative(u, "uwb", "align_min_spread_ratio")
    _check(u.align_mirror_margin >= 1.0,
           f"uwb.align_mirror_margin must be >= 1 (got {u.align_mirror_margin})")
    _non_negative(s.live_map, "live_map", "view_margin_m")
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
