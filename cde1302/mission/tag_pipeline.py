"""
Camera + AprilTag pipeline: drone JPEG stream -> Sightings in PX4 local NED.

    pipe = TagPipeline(settings, poses=link, esp_ip=lambda: link.esp_ip,
                       on_sighting=handle)        # handle(Sighting), camera thread
    pipe.start()          # builds the detector, binds UDP camera_port, starts thread
    ...
    pipe.stop()           # asks the drone to stop streaming, closes everything

Built from the course helpers exactly like laptop/tag_stream.py: keepalive
every KEEPALIVE_S to (esp_ip, cmd_port), FrameAssembler + drain_socket on
camera_port, cv2.imdecode, apriltag_host.Detector (the firmware's own
detector) and passes_gate.  Only one viewer can stream at a time, so
tag_stream.py must not run alongside this.

Per frame:
    capture time  = first_rx - age_ms/1000 - camera.capture_latency_bias_s
    drone pose    = poses.pose_at(capture time)   (interpolated, wrap-safe)
Per detection, rejected (and counted by reason) if it:
    fails the gate, has no pose, pose_err > max_pose_err,
    range > max_detect_range_m, is at an implausible height (floor mode:
    |down - expected_down_m| > down_tolerance_m; otherwise outside
    [min_down_m, max_down_m]),
    has no drone pose at capture time, or was taken while turning faster
    than max_yaw_rate_deg_s.
Survivors are placed in NED with geometry.body_to_ned(t_body, drone pose).
"""

from __future__ import annotations

import logging
import math
import select
import socket
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Protocol

import cv2
import numpy as np

import laptop_path  # noqa: F401  (puts laptop/ on sys.path)
from apriltag_host import Detection, Detector, passes_gate
from camera_stream import KEEPALIVE_S, FrameAssembler, drain_socket
from protocol import build_camera_stream_command

from drone_link import Pose
from geometry import body_to_ned
from settings import Settings, Tags
from sighting import Sighting

log = logging.getLogger(__name__)

STATS_LOG_S = 10.0          # how often the thread logs a one-line summary (s)
SELECT_TIMEOUT_S = 0.05     # max wait for camera data; bounds stop() latency (s)

# Rejection reasons, in the order they are checked.
R_GATE, R_NO_POSE, R_POSE_ERR, R_RANGE, R_HEIGHT, R_NO_DRONE, R_YAW_RATE = (
    "gate", "no_pose", "pose_err", "range", "height", "no_drone_pose", "yaw_rate")


class PoseSource(Protocol):
    """Anything that can give the drone pose at an earlier time (DroneLink, SimDrone)."""

    def pose_at(self, t: float) -> Optional[Pose]: ...

    def yaw_rate_at(self, t: float) -> Optional[float]: ...


def capture_time(first_rx: float, age_ms: int, bias_s: float = 0.0) -> float:
    """Approximate frame capture time on the laptop's monotonic clock.

    age_ms is capture -> send, measured on the drone; first_rx is when the
    frame's first datagram reached the laptop.
    """
    return first_rx - age_ms / 1000.0 - bias_s


class TagFilter:
    """Turns one Detection + drone pose history into a Sighting, or a reason not to."""

    def __init__(self, tags: Tags) -> None:
        self.cfg = tags
        self._max_yaw_rate = math.radians(tags.max_yaw_rate_deg_s)

    def evaluate(self, det: Detection, t_capture: float, t_detect: float,
                 poses: PoseSource) -> tuple[Optional[Sighting], Optional[str]]:
        """Returns (sighting, None) if accepted, else (None, reason)."""
        c = self.cfg
        if not passes_gate(det, c.min_decision_margin, c.max_hamming):
            return None, R_GATE
        body = det.t_body
        if body is None or det.pose_err is None:
            return None, R_NO_POSE
        if det.pose_err > c.max_pose_err:
            return None, R_POSE_ERR
        if det.range_m > c.max_detect_range_m:
            return None, R_RANGE
        if c.tags_on_floor:
            if abs(body[2] - c.expected_down_m) > c.down_tolerance_m:
                return None, R_HEIGHT
        elif not c.min_down_m <= body[2] <= c.max_down_m:
            return None, R_HEIGHT
        drone = poses.pose_at(t_capture)
        if drone is None:
            return None, R_NO_DRONE
        rate = poses.yaw_rate_at(t_capture)
        if rate is not None and abs(rate) > self._max_yaw_rate:
            return None, R_YAW_RATE
        x, y = body_to_ned(body[0], body[1], drone.x, drone.y, drone.heading)
        return Sighting(tag_id=det.id, t_capture=t_capture, t_detect=t_detect,
                        x=x, y=y, body=tuple(body), drone=drone,
                        cam_t=det.t, cam_R=det.R, pose_err=det.pose_err,
                        decision_margin=det.decision_margin), None


@dataclass(frozen=True)
class TagMark:
    """One detection as drawn on the camera panel."""
    tag_id: int
    corners: tuple[tuple[float, float], ...]   # 4 image pixels, AprilTag convention
    reason: Optional[str] = None               # None = accepted, else why rejected


@dataclass(frozen=True)
class CameraView:
    """The latest camera frame and what the detector made of it."""
    gray: np.ndarray                          # (240, 320) uint8; never modified
    t_capture: float                          # estimated capture time (mission clock)
    marks: tuple[TagMark, ...] = ()
    age_ms: Optional[int] = None              # capture -> sent, measured on the drone


def process_gray(gray: np.ndarray, detector: Detector, tag_filter: TagFilter,
                 poses: PoseSource, t_capture: float, t_detect: float,
                 marks: Optional[list[TagMark]] = None
                 ) -> tuple[list[Sighting], list[tuple[int, str]]]:
    """Detect tags in one grayscale frame.

    Returns (accepted sightings, [(tag_id, reason), ...] for rejected ones).
    If `marks` is given, a TagMark per detection is appended to it.
    """
    accepted, rejected = [], []
    for det in detector.detect(gray):
        s, reason = tag_filter.evaluate(det, t_capture, t_detect, poses)
        if s is not None:
            accepted.append(s)
        else:
            rejected.append((det.id, reason))
        if marks is not None:
            marks.append(TagMark(det.id, tuple(det.corners), reason))
    return accepted, rejected


@dataclass
class PipelineStats:
    """Counters since start(); read with TagPipeline.stats()."""
    frames: int = 0                 # frames decoded and searched
    bad_jpeg: int = 0               # frames that failed to decode
    dropped: int = 0                # frames lost/incomplete (FrameAssembler)
    detections: int = 0             # raw detections
    accepted: int = 0               # turned into Sightings
    rejected: Counter = field(default_factory=Counter)   # by reason
    last_frame_t: Optional[float] = None
    last_age_ms: Optional[int] = None

    def copy(self) -> "PipelineStats":
        return PipelineStats(self.frames, self.bad_jpeg, self.dropped, self.detections,
                             self.accepted, Counter(self.rejected), self.last_frame_t,
                             self.last_age_ms)

    def summary(self) -> str:
        rej = ", ".join(f"{k} {v}" for k, v in self.rejected.most_common()) or "none"
        return (f"{self.frames} frames ({self.dropped} dropped, {self.bad_jpeg} bad), "
                f"{self.detections} detections, {self.accepted} accepted; "
                f"rejected: {rej}")


class TagPipeline:
    """Background thread: camera stream -> detections -> Sightings."""

    def __init__(self, settings: Settings, poses: PoseSource,
                 esp_ip: Callable[[], Optional[str]],
                 on_sighting: Callable[[Sighting], Any], *,
                 clock: Callable[[], float] = time.monotonic,
                 detector_factory: Callable[..., Detector] = Detector,
                 bind_host: str = "0.0.0.0") -> None:
        """
        Args:
            settings: uses network, camera and tags sections.
            poses: pose history on the same monotonic clock (DroneLink).
            esp_ip: returns the drone's numeric IP, or None until it is known.
            on_sighting: called from the pipeline thread for each accepted sighting.
            clock: must be time.monotonic for a real stream (FrameAssembler
                stamps first_rx with it); injectable for tests.
            detector_factory: builds the detector (tests may substitute).
            bind_host: interface to receive camera datagrams on.
        """
        self.settings = settings
        self._poses = poses
        self._esp_ip = esp_ip
        self._on_sighting = on_sighting
        self._clock = clock
        self._detector_factory = detector_factory
        self._bind_host = bind_host
        self._filter = TagFilter(settings.tags)
        cam = settings.camera
        self._keepalive = build_camera_stream_command(True, cam.fps, cam.quality)
        self._stats = PipelineStats()
        self._stats_lock = threading.Lock()
        self._view: Optional[CameraView] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rx: Optional[socket.socket] = None
        self._tx: Optional[socket.socket] = None
        self._detector: Optional[Detector] = None
        self._last_ip: Optional[str] = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Build the detector, bind the camera port and start the thread.

        Raises:
            OSError: camera port already in use (is tag_stream.py running?).
            apriltag_host.ApriltagHostError: detector could not be built.
        """
        t = self.settings.tags
        self._detector = self._detector_factory(
            tag_size_m=t.tag_size_m, min_margin=t.min_decision_margin,
            max_hamming=t.max_hamming)
        port = self.settings.network.camera_port
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # No SO_REUSEADDR: a second viewer on this port must fail loudly.
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            rx.bind((self._bind_host, port))
        except OSError as exc:
            rx.close()
            self._detector.close()
            raise OSError(f"cannot listen for camera frames on UDP {port}: {exc} "
                          "(close tag_stream.py / camera_stream.py first)") from exc
        rx.setblocking(False)
        self._rx = rx
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tag-pipeline", daemon=True)
        self._thread.start()
        log.info("Tag pipeline listening on UDP %d", port)

    def stop(self) -> None:
        """Stop the thread, ask the drone to stop streaming, release resources."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._tx is not None:
            if self._last_ip:
                try:
                    self._tx.sendto(build_camera_stream_command(False),
                                    (self._last_ip, self.settings.network.cmd_port))
                except OSError:
                    pass
            self._tx.close()
            self._tx = None
        if self._rx is not None:
            self._rx.close()
            self._rx = None
        if self._detector is not None:
            self._detector.close()
            self._detector = None
        log.info("Tag pipeline stopped: %s", self.stats().summary())

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stats(self) -> PipelineStats:
        with self._stats_lock:
            return self._stats.copy()

    def latest_view(self) -> Optional[CameraView]:
        """The newest decoded frame with its tag outlines, or None before the first."""
        with self._stats_lock:
            return self._view

    # -- thread -------------------------------------------------------------

    def _send_keepalive(self, ip: str) -> None:
        try:
            self._tx.sendto(self._keepalive, (ip, self.settings.network.cmd_port))
        except OSError as exc:              # e.g. not on the drone Wi-Fi yet
            log.debug("camera keepalive failed: %s", exc)

    def _run(self) -> None:
        asm = FrameAssembler()
        next_keepalive = 0.0
        next_stats = time.monotonic() + STATS_LOG_S
        while not self._stop.is_set():
            try:
                ip = self._esp_ip()
                if ip is None:                       # no telemetry yet
                    self._stop.wait(0.2)
                    continue
                if ip != self._last_ip:
                    log.info("Requesting camera stream from %s", ip)
                    self._last_ip, asm = ip, FrameAssembler()
                now = time.monotonic()
                if now >= next_keepalive:
                    next_keepalive = now + KEEPALIVE_S
                    self._send_keepalive(ip)

                select.select([self._rx], [], [], SELECT_TIMEOUT_S)
                frame, _, bad_version = drain_socket(self._rx, asm, ip)
                asm.expire(time.monotonic())
                if bad_version is not None:
                    log.warning("drone sends camera protocol v%s: reflash or update", bad_version)
                with self._stats_lock:
                    self._stats.dropped = asm.dropped
                if frame is not None:
                    self._handle_frame(frame.jpeg, frame.first_rx, frame.age_ms)

                if time.monotonic() >= next_stats:
                    next_stats += STATS_LOG_S
                    log.info("camera: %s", self.stats().summary())
            except Exception:                       # keep the camera alive
                log.exception("tag pipeline error")
                self._stop.wait(0.1)

    def _handle_frame(self, jpeg: bytes, first_rx: float, age_ms: int) -> None:
        gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            with self._stats_lock:
                self._stats.bad_jpeg += 1
            return
        t_cap = capture_time(first_rx, age_ms, self.settings.camera.capture_latency_bias_s)
        marks: list[TagMark] = []
        accepted, rejected = process_gray(gray, self._detector, self._filter,
                                          self._poses, t_cap, self._clock(), marks)
        with self._stats_lock:
            self._view = CameraView(gray, t_cap, tuple(marks), age_ms)
            st = self._stats
            st.frames += 1
            st.detections += len(accepted) + len(rejected)
            st.accepted += len(accepted)
            st.rejected.update(reason for _, reason in rejected)
            st.last_frame_t, st.last_age_ms = first_rx, age_ms
        for tag_id, reason in rejected:
            log.debug("tag %d rejected: %s", tag_id, reason)
        for s in accepted:
            try:
                self._on_sighting(s)
            except Exception:
                log.exception("on_sighting callback failed")
