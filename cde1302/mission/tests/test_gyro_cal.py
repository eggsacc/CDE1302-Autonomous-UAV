"""Remote gyro calibration (mission/gyro_cal.py) and the `calibrate` command."""

import dataclasses
import socket
import time

import pytest

pytest.importorskip("pymavlink")
from pymavlink.dialects.v20 import common as mavlink  # noqa: E402

from gyro_cal import MAX_SENDS, OFFSET_PARAMS, BridgeGyroCal, GyroCalError  # noqa: E402
from mission import Phase  # noqa: E402
from settings import load_settings  # noqa: E402
from test_mission import Rig, configure  # noqa: E402


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakePx4:
    """ESP32 bridge + PX4 on localhost.  Answers parameter reads with its gyro
    offsets; on the calibration command it ACKs (ack), says nothing on this
    link unless texts are given (like the real drone), and after cal_s
    rewrites the offsets (if new_offsets)."""

    def __init__(self, listen_port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]
        self.reply_to = ("127.0.0.1", listen_port)
        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.offsets = {"CAL_GYRO0_XOFF": 0.0021, "CAL_GYRO0_YOFF": -0.0034,
                        "CAL_GYRO0_ZOFF": 0.0019}
        self.ack = mavlink.MAV_RESULT_ACCEPTED   # None: never answer the command
        self.answer_params = True
        self.texts: list[str] = []                # sent right after the ACK
        self.new_offsets = {"CAL_GYRO0_XOFF": 0.0007, "CAL_GYRO0_YOFF": -0.0012,
                            "CAL_GYRO0_ZOFF": 0.0005}
        self.cal_s = 4.0
        self.commands = []
        self.param_reads = []                     # (t, name)
        self.silent = False
        self._done_t = None

    def send(self, msg) -> None:
        self.sock.sendto(msg.pack(self.mav), self.reply_to)

    def serve(self, t: float) -> None:
        if self._done_t is not None and t >= self._done_t and self.new_offsets:
            self.offsets = dict(self.new_offsets)
            self._done_t = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, ConnectionResetError):
                break
            for msg in self.mav.parse_buffer(data) or []:
                if self.silent:
                    continue
                kind = msg.get_type()
                if kind == "PARAM_REQUEST_READ" and self.answer_params:
                    name = msg.param_id.rstrip("\x00")
                    self.param_reads.append((t, name))
                    if name in self.offsets:
                        self.send(self.mav.param_value_encode(
                            name.encode(), self.offsets[name], mavlink.MAV_PARAM_TYPE_REAL32,
                            300, 7))
                elif kind == "COMMAND_LONG":
                    self.commands.append((t, msg))
                    if self.ack is not None:
                        self.send(self.mav.command_ack_encode(241, self.ack))
                        for text in self.texts:
                            self.send(self.mav.statustext_encode(6, text.encode()))
                        if self.ack == mavlink.MAV_RESULT_ACCEPTED and self._done_t is None:
                            self._done_t = t + self.cal_s
        # what the bridge forwards anyway
        self.send(self.mav.local_position_ned_encode(0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0))

    def close(self) -> None:
        self.sock.close()


@pytest.fixture
def rig():
    listen = free_port()
    s = load_settings()
    px4 = FakePx4(listen)
    s = dataclasses.replace(s, network=dataclasses.replace(
        s.network, mavlink_listen_port=listen, mavlink_bridge_port=px4.port))
    lines = []
    cal = BridgeGyroCal(s, progress=lines.append)
    yield cal, px4, lines
    cal.close()
    px4.close()


def run(cal, px4, step=0.1, max_s=40.0):
    """Alternate the calibrator and the fake PX4 on a simulated clock."""
    cal.start("127.0.0.1", 0.0)
    t = 0.0
    while t <= max_s:
        r = cal.poll(t)
        if r is not None:
            return r, t
        px4.serve(t)
        time.sleep(0.002)                 # let localhost UDP deliver
        t = round(t + step, 3)
    return None, t


def test_done_is_read_from_the_new_gyro_offsets(rig):
    # The real drone's link carries no STATUSTEXT: only the offsets tell.
    cal, px4, lines = rig
    (ok, text), t = run(cal, px4)
    assert ok and "gyro offsets updated" in text and "ZOFF +0.00190 -> +0.00050" in text
    t_cmd, cmd = px4.commands[0]
    assert cmd.command == 241 and cmd.param1 == 1 and cmd.param5 == 0      # gyro only
    assert (cmd.target_system, cmd.target_component) == (1, 1)
    assert {n for tr, n in px4.param_reads if tr <= t_cmd} == set(OFFSET_PARAMS)  # read first
    assert t_cmd < 1.0 and px4.cal_s < t < px4.cal_s + 4.0
    assert lines == ["PX4 acknowledged the command (result 0)"]
    assert not cal.busy


def test_px4_text_still_counts_if_it_arrives(rig):
    cal, px4, lines = rig
    px4.new_offsets = None
    px4.texts = ["[cal] calibration started: 2 gyro", "[cal] calibration done: gyro"]
    (ok, text), _ = run(cal, px4)
    assert ok and "done: gyro" in text
    assert "[cal] calibration started: 2 gyro" in lines


def test_reports_px4_failure_text(rig):
    cal, px4, _ = rig
    px4.new_offsets = None
    px4.texts = ["[cal] calibration failed: motion, hold still..."]
    (ok, text), _ = run(cal, px4)
    assert not ok and "motion" in text


def test_unchanged_offsets_mean_it_did_not_complete(rig):
    cal, px4, _ = rig
    px4.new_offsets = None                        # accepted, but nothing came of it
    (ok, text), t = run(cal, px4)
    assert not ok and "offsets did not change" in text and t > 29


def test_refused_when_armed(rig):
    cal, px4, _ = rig
    px4.ack = mavlink.MAV_RESULT_TEMPORARILY_REJECTED
    (ok, text), _ = run(cal, px4)
    assert not ok and "TEMPORARILY_REJECTED" in text


def test_resends_then_gives_up_without_an_answer(rig):
    cal, px4, lines = rig
    px4.silent = True
    (ok, text), t = run(cal, px4)
    assert not ok and "no answer" in text and cal.sends == MAX_SENDS
    assert any("could not read the gyro offsets" in ln for ln in lines)
    assert t == pytest.approx(1.5 + MAX_SENDS * 1.0, abs=0.25)


def test_a_timeout_says_what_did_come_back(rig):
    cal, px4, lines = rig
    px4.answer_params = False
    px4.new_offsets = None
    px4.texts = ["Preflight Fail: something"]     # not a [cal] line
    (ok, text), _ = run(cal, px4)
    assert not ok and "PX4 acknowledged" in text and "LOCAL_POSITION_NED" in text
    assert "could not be read" in text
    assert "PX4 says: Preflight Fail: something" in lines


def test_busy_port_names_qgroundcontrol(rig):
    cal, px4, _ = rig
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("", px4.reply_to[1]))
    try:
        with pytest.raises(GyroCalError, match="QGroundControl"):
            cal.start("127.0.0.1", 0.0)
    finally:
        blocker.close()


# ---------------------------------------------------------------------------
# The `calibrate` command (simulator)
# ---------------------------------------------------------------------------

def test_calibrate_then_takeoff(tmp_path):
    r = Rig(configure(tmp_path, safety={"require_gyro_cal": True}))
    assert r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("takeoff")                                  # refused: not calibrated
    r.step()
    assert r.m.phase == Phase.READY and not r.m.start_sent
    r.m.command("calibrate")
    assert r.until(lambda: r.m.gyro_cal_ok, 10)
    assert [e["event"] for e in r.events() if e["event"].startswith("GYRO_CAL")] == \
        ["GYRO_CAL_START", "GYRO_CAL", "GYRO_CAL", "GYRO_CAL_DONE"]
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.TAKING_OFF, 2)
    r.m.command("calibrate")                                # not in the air
    r.step()
    assert len(r.events("GYRO_CAL_START")) == 1
    r.finish()
    assert r.m.exit_code == 0


def test_takeoff_waits_for_a_running_calibration(tmp_path):
    r = Rig(configure(tmp_path))                            # require_gyro_cal off
    assert r.until(lambda: r.m.phase == Phase.READY, 5)
    r.m.command("calibrate")
    r.step()
    r.m.command("takeoff")
    r.step()
    assert r.m.phase == Phase.READY                         # still calibrating
    assert r.until(lambda: r.m.gyro_cal_ok, 10)
    r.m.command("takeoff")
    assert r.until(lambda: r.m.phase == Phase.TAKING_OFF, 2)
    r.m.shutdown()

