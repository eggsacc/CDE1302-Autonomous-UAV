"""
Packet definitions for ESP32 ↔ laptop UDP protocol.

Telemetry (ESP32 → laptop, 10 Hz):
    Fixed-size packet.

Command (laptop → ESP32, event-driven):
    Fixed 22-byte mission packet, plus variable-size control packets.

Camera preview (ESP32 → laptop, port CAMERA_STREAM_PORT): see parse_camera_chunk().
"""

import math
import struct
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PKT_TELEM     = 0x01   # telemetry from drone
PKT_CMD       = 0x02   # command to drone
PKT_AT_DEBUG  = 0x04   # live AprilTag detections (debug stream)

AT_DEBUG_PORT = 5008   # UDP port the AprilTag debug stream arrives on
CAMERA_STREAM_PORT = 5009

CMD_GOTO          = 0x01   # navigate to (goal_x, goal_y)
CMD_LAND          = 0x02   # land immediately
CMD_HOLD          = 0x03   # hold position, cancel goal
CMD_SET_NAV_TAGS  = 0x04   # send navigation tag map positions to drone
CMD_START         = 0x05   # arm and take off
CMD_SET_PEERS     = 0x06   # update nearby drone positions for inter-drone avoidance
CMD_CAMERA_STREAM = 0x07   # camera preview keepalive
CMD_TRAJ_DATA     = 0x08   # trajectory chunk upload
CMD_TRAJ_START    = 0x09   # play the uploaded trajectory

VFH_BINS    = 32

# Nav state values (must match nav_state_t in nav_task.h)
NAV_IDLE       = 0
NAV_ROTATING   = 1
NAV_FLYING     = 2
NAV_ARRIVED    = 3
NAV_STUCK      = 4
NAV_TRAJ       = 5   # playing an uploaded trajectory

NAV_STATE_NAMES = {
    NAV_IDLE:       "IDLE",
    NAV_ROTATING:   "ROTATING",
    NAV_FLYING:     "FLYING",
    NAV_ARRIVED:    "ARRIVED",
    NAV_STUCK:      "STUCK",
    NAV_TRAJ:       "TRAJ",
}

# ---------------------------------------------------------------------------
# Telemetry packet
# ---------------------------------------------------------------------------

# Wire format: pkt_type, drone_id, ned_x, ned_y, heading_rad,
#              nav_state, tag_id, tag_dist_m, vfh_blocked[32], is_stuck, reloc_age_s
_TELEM_HDR_FMT  = "<BBfffBbf32sBH"
_TELEM_HDR_SIZE = struct.calcsize(_TELEM_HDR_FMT)   # 55 bytes


@dataclass
class TelemetryPacket:
    drone_id:    int
    ned_x:       float
    ned_y:       float
    heading_rad: float
    nav_state:   int
    tag_id:      int              # −1 if no tag visible
    tag_dist_m:  float
    vfh_blocked: list            # VFH_BINS bools
    is_stuck:    bool
    reloc_age_s: int              # seconds since last nav-tag fix (0xFFFF = never)
    # Optional additive trailer. Legacy 55-byte firmware yields None, never
    # fabricated armed/mode/link status. The original 55-byte prefix is intact.
    px4_hb_age_ms: int | None = None
    px4_pos_age_ms: int | None = None
    px4_armed: int | None = None
    px4_custom_main_mode: int | None = None
    px4_custom_sub_mode: int | None = None

    @property
    def px4_link_ok(self) -> bool | None:
        if self.px4_hb_age_ms is None or self.px4_pos_age_ms is None:
            return None
        return self.px4_hb_age_ms <= 1500 and self.px4_pos_age_ms <= 500

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


def parse_telemetry(data: bytes) -> Optional[TelemetryPacket]:
    """Parse a raw UDP payload into a TelemetryPacket. Returns None on error."""
    if len(data) < _TELEM_HDR_SIZE:
        return None

    fields = struct.unpack_from(_TELEM_HDR_FMT, data, 0)
    (pkt_type, drone_id, ned_x, ned_y, heading_rad,
     nav_state, tag_id, tag_dist_m,
     vfh_raw, is_stuck, reloc_age_s) = fields

    if pkt_type != PKT_TELEM:
        return None

    vfh_blocked = [bool(b) for b in vfh_raw]

    extra = {}
    if len(data) >= _TELEM_HDR_SIZE + 7:
        names = ("px4_hb_age_ms", "px4_pos_age_ms", "px4_armed",
                 "px4_custom_main_mode", "px4_custom_sub_mode")
        extra = dict(zip(names, struct.unpack_from("<HHBBB", data, _TELEM_HDR_SIZE)))
    return TelemetryPacket(
        drone_id    = drone_id,
        ned_x       = ned_x,
        ned_y       = ned_y,
        heading_rad = heading_rad,
        nav_state   = nav_state,
        tag_id      = tag_id,
        tag_dist_m  = tag_dist_m,
        vfh_blocked = vfh_blocked,
        is_stuck    = bool(is_stuck),
        reloc_age_s = reloc_age_s,
        **extra,
    )

# ---------------------------------------------------------------------------
# AprilTag debug packet  (drone → laptop, UDP port AT_DEBUG_PORT, 10 Hz)
#
# Unlike the tag_id field in telemetry (a latched mission claim that never
# clears), this stream reports every raw detection in the most recent camera
# frame — tags appear and disappear in real time.
# ---------------------------------------------------------------------------

# Header: pkt_type, drone_id, frame_ms, proc_ms, latched_id, raw_count, count
_AT_DEBUG_HDR_FMT  = "<BBIHbBB"
_AT_DEBUG_HDR_SIZE = struct.calcsize(_AT_DEBUG_HDR_FMT)   # 11 bytes

# Per detection: id, hamming, margin, cx, cy, tx, ty, tz, pose_err
_AT_DET_FMT  = "<bB7f"
_AT_DET_SIZE = struct.calcsize(_AT_DET_FMT)               # 30 bytes


@dataclass
class TagDetection:
    id:       int     # tag ID (tag16h5)
    hamming:  int     # corrected bit errors
    margin:   float   # decision margin — higher = more confident
    cx:       float   # tag centre in image pixels
    cy:       float
    tx:       float   # camera-frame translation (m): X=right, Y=down, Z=forward
    ty:       float
    tz:       float
    pose_err: float   # pose reprojection error; < 0 = pose not computed

    @property
    def has_pose(self) -> bool:
        """True if this detection passed the firmware quality gate
        (hamming ≤ 1 and margin > 55) and carries a pose estimate."""
        return self.pose_err >= 0.0

    @property
    def range_m(self) -> float:
        """Straight-line camera-to-tag distance (m)."""
        return math.sqrt(self.tx**2 + self.ty**2 + self.tz**2)


@dataclass
class AtDebugPacket:
    drone_id:   int
    frame_ms:   int    # esp_timer ms when the frame was processed (0 = none yet)
    proc_ms:    int    # frame processing time (detect + pose) in ms
    latched_id: int    # mission tag claim (telemetry tag_id), −1 = none
    raw_count:  int    # detections in frame before quality filtering
    detections: list   # of TagDetection (may be truncated to 8 by firmware)


def parse_at_debug(data: bytes) -> Optional[AtDebugPacket]:
    """Parse a raw UDP payload into an AtDebugPacket. Returns None on error."""
    if len(data) < _AT_DEBUG_HDR_SIZE:
        return None

    (pkt_type, drone_id, frame_ms, proc_ms,
     latched_id, raw_count, count) = struct.unpack_from(_AT_DEBUG_HDR_FMT, data, 0)

    if pkt_type != PKT_AT_DEBUG:
        return None

    detections = []
    offset = _AT_DEBUG_HDR_SIZE
    for _ in range(count):
        if offset + _AT_DET_SIZE > len(data):
            break
        detections.append(TagDetection(*struct.unpack_from(_AT_DET_FMT, data, offset)))
        offset += _AT_DET_SIZE

    return AtDebugPacket(
        drone_id   = drone_id,
        frame_ms   = frame_ms,
        proc_ms    = proc_ms,
        latched_id = latched_id,
        raw_count  = raw_count,
        detections = detections,
    )

# ---------------------------------------------------------------------------
# Command packet
# ---------------------------------------------------------------------------

# pkt_type, cmd_type, goal_x, goal_y, found_tag_ids[12] (int8_t, −1 = unused)
_CMD_FMT  = "<BBff12b"
_CMD_SIZE = struct.calcsize(_CMD_FMT)   # 22 bytes

MAX_FOUND_TAGS = 12


@dataclass
class CommandPacket:
    cmd_type:      int
    goal_x:        float     = 0.0
    goal_y:        float     = 0.0
    found_tag_ids: list      = field(default_factory=list)  # up to 8 tag IDs


def build_command(cmd: CommandPacket) -> bytes:
    """Serialise a CommandPacket to bytes ready to send over UDP."""
    tag_ids = (cmd.found_tag_ids + [-1] * MAX_FOUND_TAGS)[:MAX_FOUND_TAGS]
    return struct.pack(_CMD_FMT, PKT_CMD, cmd.cmd_type,
                       cmd.goal_x, cmd.goal_y, *tag_ids)


CAMERA_STREAM_MAX_FPS     = 15
CAMERA_STREAM_MIN_QUALITY = 10
CAMERA_STREAM_MAX_QUALITY = 90


def build_camera_stream_command(enabled: bool = True, max_fps: int = 0,
                                quality: int = 0) -> bytes:
    """Keepalive, sent every second: pkt, cmd, enable, max_fps, quality.

    0 means the firmware default.  The drone stops streaming 2.5 s after the
    last enable packet, or at once on enabled=False.
    """
    if not 0 <= max_fps <= CAMERA_STREAM_MAX_FPS:
        raise ValueError(f"fps must be 1-{CAMERA_STREAM_MAX_FPS}")
    if quality and not CAMERA_STREAM_MIN_QUALITY <= quality <= CAMERA_STREAM_MAX_QUALITY:
        raise ValueError(f"quality must be {CAMERA_STREAM_MIN_QUALITY}-"
                         f"{CAMERA_STREAM_MAX_QUALITY}")
    return struct.pack("<5B", PKT_CMD, CMD_CAMERA_STREAM, int(enabled),
                       max_fps, quality)


# One JPEG = data datagrams (header + payload) then an END datagram with no
# payload and offset == frame_size.  Header layout: see main/camera_stream.c.
CAMERA_MAGIC       = b"ECAM"
CAMERA_VERSION     = 2
CAMERA_FLAG_END    = 0x02
CAMERA_HEADER_FMT  = "<4sBBBBIIIHHHHH"
CAMERA_HEADER_SIZE = struct.calcsize(CAMERA_HEADER_FMT)   # 30 bytes


@dataclass
class CameraChunk:
    flags:      int
    drone_id:   int
    boot_nonce: int      # random per drone boot
    frame_id:   int
    offset:     int
    frame_size: int      # END only
    width:      int
    height:     int
    age_ms:     int      # capture -> this datagram sent, on the drone
    esp_drops:  int      # frames the drone dropped since boot (wraps)
    payload:    bytes

    @property
    def is_end(self) -> bool:
        return bool(self.flags & CAMERA_FLAG_END)


def camera_packet_version(data: bytes) -> Optional[int]:
    """Version byte of any ECAM datagram, else None (to report a mismatch)."""
    return data[4] if len(data) >= 5 and data[:4] == CAMERA_MAGIC else None


def parse_camera_chunk(data: bytes) -> Optional[CameraChunk]:
    """Parse one camera datagram; None if malformed or another version."""
    if len(data) < CAMERA_HEADER_SIZE:
        return None
    (magic, version, flags, drone_id, nonce, frame_id, offset, frame_size,
     width, height, payload_len, age_ms, esp_drops) = struct.unpack_from(
        CAMERA_HEADER_FMT, data)
    if (magic != CAMERA_MAGIC or version != CAMERA_VERSION
            or payload_len != len(data) - CAMERA_HEADER_SIZE):
        return None
    if flags & CAMERA_FLAG_END:
        if payload_len or not frame_size or offset != frame_size:
            return None
    elif frame_size or not payload_len:
        return None
    return CameraChunk(flags, drone_id, nonce, frame_id, offset, frame_size,
                       width, height, age_ms, esp_drops,
                       bytes(data[CAMERA_HEADER_SIZE:]))


# ---------------------------------------------------------------------------
# Trajectory upload  (laptop → drone)
#
# The whole trajectory is uploaded before playback; the drone then plays it
# on its own clock, so WiFi jitter cannot disturb the flight.
# ---------------------------------------------------------------------------

TRAJ_MAX_PTS   = 2400   # must match NAV_TRAJ_MAX_PTS in nav_task.h
TRAJ_CHUNK_PTS = 80     # 8 + 80*12 = 968 B, fits WIFI_CMD_BUF_SIZE (1024)


def build_traj_packets(traj_id: int, pts) -> list[bytes]:
    """CMD_TRAJ_DATA chunks: pkt, cmd, id, total, offset, n, n x (x, y, z) f32.

    pts: (x, y, z) NED offsets from the first point (m); traj_id: 1-255.
    """
    out = []
    for off in range(0, len(pts), TRAJ_CHUNK_PTS):
        chunk = pts[off:off + TRAJ_CHUNK_PTS]
        buf = struct.pack("<BBBHHB", PKT_CMD, CMD_TRAJ_DATA, traj_id,
                          len(pts), off, len(chunk))
        buf += b"".join(struct.pack("<fff", *p) for p in chunk)
        out.append(buf)
    return out


def build_traj_start(traj_id: int, dt_ms: int) -> bytes:
    """CMD_TRAJ_START: play upload traj_id, one point every dt_ms."""
    return struct.pack("<BBBH", PKT_CMD, CMD_TRAJ_START, traj_id, dt_ms)


# Tangent-yaw protocol v1. Distinct commands; legacy 0x08/0x09 are unchanged.
# Every request includes version, target drone ID, and a nonzero 32-bit transfer
# ID. A connected UDP socket receives replies on the request's source port.
PKT_TRAJ_YAW = 0x05
CMD_YAW_CAPS = 0x0A
CMD_YAW_BEGIN = 0x0B
CMD_YAW_DATA = 0x0C
CMD_YAW_START = 0x0D
CMD_YAW_STATUS = 0x0E
YAW_PROTOCOL_VERSION = 1
YAW_CHUNK_PTS = 60                  # 11 + 60*16 = 971 bytes <= 1024
YAW_MIN_DT_MS = 50                  # nav + MAVLink actually run at 20 Hz
YAW_HARD_MAX_RATE = 1.0             # must match traj_yaw.h; NOT TA approval
YAW_ALIGNMENT_RAD = math.radians(5)
YAW_CAPABLE, YAW_UPLOADING, YAW_READY, YAW_PLAYING, YAW_DONE = range(5)
YAW_BAD_PACKET, YAW_BAD_VERSION, YAW_BAD_ID, YAW_BUSY = range(10, 14)
YAW_INCOMPLETE, YAW_CORRUPT, YAW_LIMIT, YAW_NOT_FLYING, YAW_MISALIGNED = range(14, 19)
YAW_CANCELLED = 19
YAW_REPLY_FMT = "<BBBBIHHHHfff"
YAW_REPLY_SIZE = struct.calcsize(YAW_REPLY_FMT)  # 28 B


def yaw_header(cmd, drone_id, transfer_id):
    if cmd not in range(CMD_YAW_CAPS, CMD_YAW_STATUS + 1):
        raise ValueError("invalid yaw command")
    if not 1 <= drone_id <= 254 or not 1 <= transfer_id <= 0xFFFFFFFF:
        raise ValueError("invalid drone or transfer ID")
    return struct.pack("<BBBBI", PKT_CMD, cmd, YAW_PROTOCOL_VERSION,
                       drone_id, transfer_id)


def yaw_payload(points):
    if not 2 <= len(points) <= TRAJ_MAX_PTS:
        raise ValueError("yaw trajectory requires 2..2400 samples")
    out = bytearray()
    for p in points:
        if len(p) != 4 or not all(math.isfinite(float(v)) for v in p):
            raise ValueError("yaw points must be finite x,y,z,yaw")
        out.extend(struct.pack("<4f", *p))
    # Check finiteness again after conversion (overflow raises above).
    return bytes(out)


def build_yaw_begin(drone_id, transfer_id, total, dt_ms, max_rate, crc):
    if not 2 <= total <= TRAJ_MAX_PTS or not YAW_MIN_DT_MS <= dt_ms <= 1000:
        raise ValueError("yaw count or sample period outside firmware bounds")
    if not math.isfinite(max_rate) or max_rate <= 0:
        raise ValueError("max yaw rate must be finite and positive")
    return yaw_header(CMD_YAW_BEGIN, drone_id, transfer_id) + struct.pack(
        "<HHfI", total, dt_ms, max_rate, crc)


def build_yaw_packets(drone_id, transfer_id, payload):
    if len(payload) % 16 or not 32 <= len(payload) <= 16 * TRAJ_MAX_PTS:
        raise ValueError("invalid float32 yaw payload size")
    out = []
    for off in range(0, len(payload) // 16, YAW_CHUNK_PTS):
        chunk = payload[off * 16:(off + YAW_CHUNK_PTS) * 16]
        out.append(yaw_header(CMD_YAW_DATA, drone_id, transfer_id) +
                   struct.pack("<HB", off, len(chunk) // 16) + chunk)
    return out


@dataclass(frozen=True)
class YawReply:
    version: int
    drone_id: int
    command: int
    transfer_id: int
    status: int
    received: int
    max_points: int
    min_dt_ms: int
    max_yaw_rate: float
    alignment_rad: float
    heading_rad: float


def parse_yaw_reply(data):
    if len(data) != YAW_REPLY_SIZE:
        return None
    ptype, *fields = struct.unpack(YAW_REPLY_FMT, data)
    if ptype != PKT_TRAJ_YAW:
        return None
    return YawReply(*fields)


def require_yaw_reply(data, drone_id, transfer_id, command):
    reply = parse_yaw_reply(data)
    if reply is None or (reply.version, reply.drone_id, reply.transfer_id, reply.command) != (
            YAW_PROTOCOL_VERSION, drone_id, transfer_id, command):
        raise ValueError("wrong firmware/protocol version, target ID, transfer ID, or response type")
    if (reply.max_points < 2 or reply.min_dt_ms < 1 or
            not all(math.isfinite(v) for v in (reply.max_yaw_rate, reply.alignment_rad)) or
            reply.max_yaw_rate <= 0 or not 0 < reply.alignment_rad <= YAW_ALIGNMENT_RAD + 1e-6):
        raise ValueError("invalid firmware capability bounds")
    return reply


# ---------------------------------------------------------------------------
# Navigation-tag position packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_NAV_TAGS = 16

# Wire format per tag entry: int8_t id + float map_x + float map_y
_NAV_TAG_ENTRY_FMT = "<bff"
_NAV_TAG_ENTRY_SIZE = struct.calcsize(_NAV_TAG_ENTRY_FMT)  # 9 bytes


@dataclass
class NavTag:
    """A navigation AprilTag at a known map-frame position."""
    id:    int      # AprilTag ID
    map_x: float    # NED north in map frame (m)
    map_y: float    # NED east  in map frame (m)


def build_nav_tags_command(tags: list[NavTag],
                           start_x: float = 0.0,
                           start_y: float = 0.0) -> bytes:
    """
    Build a CMD_SET_NAV_TAGS packet for one specific drone.

    tags:    list of NavTag in map frame (up to MAX_NAV_TAGS).
    start_x: drone's start position in map frame (NED north, m).
    start_y: drone's start position in map frame (NED east,  m).

    Tag positions are pre-converted to the drone's odom frame
    (map_pos − start_offset) so the drone never needs to know the
    map frame directly.
    """
    count = min(len(tags), MAX_NAV_TAGS)
    # Header: pkt_type, cmd_type, count, start_map_x, start_map_y
    buf = struct.pack("<BBBff", PKT_CMD, CMD_SET_NAV_TAGS, count, start_x, start_y)
    for t in tags[:count]:
        odom_x = t.map_x - start_x
        odom_y = t.map_y - start_y
        buf += struct.pack(_NAV_TAG_ENTRY_FMT, t.id, odom_x, odom_y)
    return buf


# ---------------------------------------------------------------------------
# Peer drone positions packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_PEERS = 7


def build_peers_command(peers: list[tuple[float, float]]) -> bytes:
    """
    Build a CMD_SET_PEERS packet.

    peers: list of (map_x, map_y) positions in map frame, up to MAX_PEERS.
    Returns bytes ready to send over UDP.
    """
    count = min(len(peers), MAX_PEERS)
    buf = struct.pack("<BBB", PKT_CMD, CMD_SET_PEERS, count)
    for mx, my in peers[:count]:
        buf += struct.pack("<ff", mx, my)
    return buf
