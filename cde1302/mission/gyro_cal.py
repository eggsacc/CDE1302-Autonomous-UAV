"""
Remote gyro calibration: the operator types `calibrate` in mission.py instead
of opening QGroundControl.

PX4 calibrates its gyros on MAV_CMD_PREFLIGHT_CALIBRATION (param1 = 1), from
any MAVLink sender, while disarmed and perfectly still.  The ESP32 already
bridges MAVLink over Wi-Fi for QGroundControl (QGC_MAVLINK_BRIDGE.md):

    laptop --UDP--> ESP32 :network.mavlink_bridge_port (8888) --UART--> PX4
    laptop <--UDP-- ESP32 --> laptop :network.mavlink_listen_port (14550)

so this needs no firmware change.  QGroundControl listens on 14550 too: close
it first (a busy port is reported as such).

PX4's answer: COMMAND_ACK (ACCEPTED, or e.g. TEMPORARILY_REJECTED when armed),
then STATUSTEXT lines "[cal] calibration started: ...", "[cal] calibration
done: gyro" or "[cal] calibration failed: <why>" (PX4 calibration_messages.h).

    cal = BridgeGyroCal(settings, progress=print)
    cal.start("192.168.1.202", now)
    while (result := cal.poll(now)) is None: ...     # mission.py: every tick
    ok, text = result

Non-blocking throughout: Mission.tick() calls poll() at flight.tick_hz.
"""

from __future__ import annotations

import socket
from typing import Callable, Optional

from settings import Settings

CAL_DONE = "[cal] calibration done"
CAL_FAILED = "[cal] calibration failed"
CAL_CANCELLED = "[cal] calibration cancelled"
CAL_PREFIX = "[cal]"

MAV_CMD_PREFLIGHT_CALIBRATION = 241
RESEND_S = 1.0          # resend the command if PX4 has not acknowledged it
MAX_SENDS = 3
GCS_SYSID, GCS_COMPID = 255, 190       # a ground station, like QGroundControl
PX4_SYSID, PX4_COMPID = 1, 1


class GyroCalError(RuntimeError):
    """Calibration could not even be started (e.g. the UDP port is busy)."""


class BridgeGyroCal:
    """Gyro calibration through the ESP32's MAVLink bridge (see the module docs)."""

    def __init__(self, settings: Settings,
                 progress: Optional[Callable[[str], None]] = None) -> None:
        from pymavlink.dialects.v20 import common as mavlink
        self._m = mavlink
        self._net = settings.network
        self._timeout_s = settings.safety.gyro_cal_timeout_s
        self._progress = progress or (lambda text: None)
        self._sock: Optional[socket.socket] = None
        self._target: Optional[tuple[str, int]] = None
        self._busy = False
        self._acked = False
        self._sends = 0
        self._sent_t = 0.0
        self._start_t = 0.0
        self.sends = 0                  # commands sent in the last calibration (tests)

    @property
    def busy(self) -> bool:
        return self._busy

    def start(self, target_ip: str, now: float) -> None:
        """Open the listen port and send the command.  Raises GyroCalError."""
        if self._busy:
            raise GyroCalError("a calibration is already running")
        port = self._net.mavlink_listen_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("", port))
        except OSError as exc:
            sock.close()
            raise GyroCalError(f"cannot listen on UDP {port} ({exc}); close QGroundControl, "
                               f"which uses the same port") from exc
        sock.setblocking(False)
        self._sock = sock
        self._target = (target_ip, self._net.mavlink_bridge_port)
        self._parser = self._m.MAVLink(None, srcSystem=GCS_SYSID, srcComponent=GCS_COMPID)
        self._parser.robust_parsing = True
        self._busy, self._acked, self._sends, self.sends = True, False, 0, 0
        self._start_t = now
        self._send(now)

    def _send(self, now: float) -> None:
        msg = self._parser.command_long_encode(PX4_SYSID, PX4_COMPID,
                                               MAV_CMD_PREFLIGHT_CALIBRATION, self._sends,
                                               1, 0, 0, 0, 0, 0, 0)   # param1 = gyro
        self._sock.sendto(msg.pack(self._parser), self._target)
        self._sends += 1
        self.sends = self._sends
        self._sent_t = now

    def poll(self, now: float) -> Optional[tuple[bool, str]]:
        """Handle whatever PX4 sent.  (ok, text) once finished, else None."""
        if not self._busy:
            return None
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as exc:                  # e.g. Windows ICMP "port unreachable"
                if getattr(exc, "winerror", None) == 10054:
                    continue
                return self._finish(False, f"UDP error: {exc}")
            for msg in self._parser.parse_buffer(data) or []:
                result = self._handle(msg)
                if result is not None:
                    return result
        if not self._acked and now - self._sent_t >= RESEND_S:
            if self._sends >= MAX_SENDS:
                return self._finish(False, f"no answer from PX4 after {self._sends} tries "
                                           f"(is the drone on and the ESP32 bridge running?)")
            self._send(now)
        if now - self._start_t > self._timeout_s:
            return self._finish(False, f"no result within {self._timeout_s:.0f} s")
        return None

    def _handle(self, msg) -> Optional[tuple[bool, str]]:
        m = self._m
        kind = msg.get_type()
        if kind == "COMMAND_ACK" and msg.command == MAV_CMD_PREFLIGHT_CALIBRATION:
            if msg.result in (m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_IN_PROGRESS):
                self._acked = True
                return None
            name = m.enums["MAV_RESULT"][msg.result].name if msg.result in \
                m.enums["MAV_RESULT"] else str(msg.result)
            return self._finish(False, f"PX4 refused the calibration ({name}): is it disarmed?")
        if kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode("utf-8", "replace")
            text = text.rstrip("\x00").strip()
            if not text.startswith(CAL_PREFIX):
                return None
            self._acked = True                      # a lost ACK does not matter now
            self._progress(text)
            if text.startswith(CAL_DONE):
                return self._finish(True, text)
            if text.startswith((CAL_FAILED, CAL_CANCELLED)):
                return self._finish(False, text)
        return None

    def _finish(self, ok: bool, text: str) -> tuple[bool, str]:
        self.close()
        return ok, text

    def close(self) -> None:
        self._busy = False
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class SimGyroCal:
    """Simulator stand-in: succeeds sim.gyro_cal_s after start."""

    def __init__(self, settings: Settings,
                 progress: Optional[Callable[[str], None]] = None) -> None:
        self._duration = settings.sim.gyro_cal_s
        self._progress = progress or (lambda text: None)
        self._done_t: Optional[float] = None

    @property
    def busy(self) -> bool:
        return self._done_t is not None

    def start(self, target_ip: str, now: float) -> None:
        if self.busy:
            raise GyroCalError("a calibration is already running")
        self._done_t = now + self._duration
        self._progress("[cal] calibration started: 2 gyro (sim)")

    def poll(self, now: float) -> Optional[tuple[bool, str]]:
        if self._done_t is None or now < self._done_t:
            return None
        self._done_t = None
        text = "[cal] calibration done: gyro (sim)"
        self._progress(text)
        return True, text

    def close(self) -> None:
        self._done_t = None
