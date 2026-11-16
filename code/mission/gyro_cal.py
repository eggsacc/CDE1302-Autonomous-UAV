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

How we know it worked.  PX4 answers COMMAND_ACK (ACCEPTED, or e.g.
TEMPORARILY_REJECTED when armed).  It then reports progress as STATUSTEXT
"[cal] calibration done: gyro" / "[cal] calibration failed: <why>", but on
this drone's ESP32 link PX4 sends no STATUSTEXT at all (flight 2026-11-10
23:52: 4082 messages came back, not one text).  So the result is read from
the calibration itself: the gyro offsets CAL_GYRO0_XOFF/YOFF/ZOFF are read
before the command (PARAM_REQUEST_READ), then every second from a few
seconds after the ACK; new values, the same in two reads in a row, mean
done.  A "[cal] ..." text, if one does arrive, is still honoured.

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
PRE_READ_S = 1.5        # wait this long for the offsets before sending the command anyway
FIRST_CHECK_S = 3.0     # read the offsets from this long after the ACK...
CHECK_EVERY_S = 1.0     # ...this often
OFFSET_PARAMS = ("CAL_GYRO0_XOFF", "CAL_GYRO0_YOFF", "CAL_GYRO0_ZOFF")
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
        self.sends = 0                  # commands sent in the last calibration (tests)

    @property
    def busy(self) -> bool:
        return self._busy

    # -- start --------------------------------------------------------------

    def start(self, target_ip: str, now: float) -> None:
        """Open the listen port, read the current offsets, then (in poll) send
        the command.  Raises GyroCalError."""
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
        self._busy, self.sends = True, 0
        self._phase = "pre"                     # pre -> cmd -> running
        self._acked = False
        self._start_t = now
        self._sent_t = now
        self._ack_t: Optional[float] = None
        self._next_check = 0.0
        self._before: dict[str, float] = {}
        self._latest: dict[str, float] = {}
        self._candidate: Optional[tuple[float, ...]] = None
        self._n_msgs, self._types = 0, set()     # diagnostics for a timeout
        self._read_offsets()

    def _send(self, msg) -> None:
        self._sock.sendto(msg.pack(self._parser), self._target)

    def _read_offsets(self) -> None:
        for name in OFFSET_PARAMS:
            self._send(self._parser.param_request_read_encode(
                PX4_SYSID, PX4_COMPID, name.encode(), -1))

    def _send_command(self, now: float) -> None:
        self._send(self._parser.command_long_encode(PX4_SYSID, PX4_COMPID,
                                                    MAV_CMD_PREFLIGHT_CALIBRATION, self.sends,
                                                    1, 0, 0, 0, 0, 0, 0))   # param1 = gyro
        self.sends += 1
        self._sent_t = now
        self._phase = "cmd"

    # -- poll ---------------------------------------------------------------

    def poll(self, now: float) -> Optional[tuple[bool, str]]:
        """Handle whatever came back and move on.  (ok, text) once finished."""
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
                self._n_msgs += 1
                self._types.add(msg.get_type())
                result = self._handle(msg, now)
                if result is not None:
                    return result

        if self._phase == "pre":
            if len(self._before) == len(OFFSET_PARAMS) or now - self._start_t >= PRE_READ_S:
                if len(self._before) < len(OFFSET_PARAMS):
                    self._progress("could not read the gyro offsets first; "
                                   "only PX4's own report can confirm the result")
                self._send_command(now)
        elif self._phase == "cmd" and now - self._sent_t >= RESEND_S:
            if self.sends >= MAX_SENDS:
                return self._finish(False, f"no answer from PX4 after {self.sends} tries "
                                           f"(is the drone on and the ESP32 bridge running?)")
            self._send_command(now)
        elif (self._phase == "running" and self._before
              and now - self._ack_t >= FIRST_CHECK_S and now >= self._next_check):
            self._next_check = now + CHECK_EVERY_S
            self._latest.clear()
            self._read_offsets()

        if now - self._start_t > self._timeout_s:
            return self._finish(False, self._timeout_text())
        return None

    def _handle(self, msg, now: float) -> Optional[tuple[bool, str]]:
        m = self._m
        kind = msg.get_type()
        if kind == "PARAM_VALUE":
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            name = name.rstrip("\x00")
            if name in OFFSET_PARAMS:
                (self._latest if self._phase == "running" else self._before)[name] = \
                    float(msg.param_value)
                return self._check_offsets()
            return None
        if kind == "COMMAND_ACK" and msg.command == MAV_CMD_PREFLIGHT_CALIBRATION:
            if msg.result in (m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_IN_PROGRESS):
                if not self._acked:
                    self._progress(f"PX4 acknowledged the command (result {msg.result})")
                    self._acked, self._phase, self._ack_t = True, "running", now
                return None
            name = m.enums["MAV_RESULT"][msg.result].name if msg.result in \
                m.enums["MAV_RESULT"] else str(msg.result)
            return self._finish(False, f"PX4 refused the calibration ({name}): is it disarmed?")
        if kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode("utf-8", "replace")
            text = text.rstrip("\x00").strip()
            if not text.startswith(CAL_PREFIX):
                self._progress(f"PX4 says: {text}")     # anything PX4 says meanwhile
                return None
            if not self._acked:                         # a lost ACK does not matter now
                self._acked, self._phase, self._ack_t = True, "running", now
            self._progress(text)
            if text.startswith(CAL_DONE):
                return self._finish(True, text)
            if text.startswith((CAL_FAILED, CAL_CANCELLED)):
                return self._finish(False, text)
        return None

    def _check_offsets(self) -> Optional[tuple[bool, str]]:
        """Done once a full read differs from before and matches the previous read."""
        if self._phase != "running" or len(self._latest) < len(OFFSET_PARAMS) \
                or len(self._before) < len(OFFSET_PARAMS):
            return None
        now_vals = tuple(self._latest[n] for n in OFFSET_PARAMS)
        old_vals = tuple(self._before[n] for n in OFFSET_PARAMS)
        if now_vals == old_vals:
            self._candidate = None
            return None
        if self._candidate != now_vals:                 # first read of new values: confirm
            self._candidate = now_vals
            return None
        return self._finish(True, "gyro offsets updated (rad/s): "
                                  + ", ".join(f"{n[-4:]} {o:+.5f} -> {v:+.5f}"
                                              for n, o, v in zip(OFFSET_PARAMS, old_vals,
                                                                 now_vals)))

    def _timeout_text(self) -> str:
        if not self._acked:
            return f"no result within {self._timeout_s:.0f} s; PX4 never acknowledged the command"
        if self._before and len(self._before) == len(OFFSET_PARAMS):
            return (f"no result within {self._timeout_s:.0f} s; PX4 accepted the command but the "
                    f"gyro offsets did not change: the calibration did not complete (did the "
                    f"drone move?)")
        seen = ("nothing at all came back on UDP "
                f"{self._net.mavlink_listen_port} (bridge not forwarding to this laptop?)"
                if not self._n_msgs else
                f"{self._n_msgs} MAVLink messages came back ({', '.join(sorted(self._types)[:8])}"
                f"{', ...' if len(self._types) > 8 else ''})")
        return (f"no result within {self._timeout_s:.0f} s; PX4 acknowledged the command; "
                f"{seen}, but no '[cal] ... done/failed' and the gyro offsets could not be read")

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
