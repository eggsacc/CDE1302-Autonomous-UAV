"""Remote gyro calibration (mission/gyro_cal.py) and the `calibrate` command."""

import dataclasses
import socket

import pytest

pytest.importorskip("pymavlink")
from pymavlink.dialects.v20 import common as mavlink  # noqa: E402

from gyro_cal import MAX_SENDS, BridgeGyroCal, GyroCalError  # noqa: E402
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
    """A UDP endpoint standing in for ESP32 bridge + PX4."""

    def __init__(self, listen_port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self.reply_to = ("127.0.0.1", listen_port)
        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.commands = []

    def receive(self):
        data, _ = self.sock.recvfrom(4096)
        msgs = self.mav.parse_buffer(data) or []
        self.commands += [m for m in msgs if m.get_type() == "COMMAND_LONG"]
        return msgs

    def send(self, msg) -> None:
        self.sock.sendto(msg.pack(self.mav), self.reply_to)

    def ack(self, result=mavlink.MAV_RESULT_ACCEPTED) -> None:
        self.send(self.mav.command_ack_encode(241, result))

    def text(self, text: str) -> None:
        self.send(self.mav.statustext_encode(6, text.encode()))

    def chatter(self) -> None:                 # what the bridge forwards anyway
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


def run(cal, t0=0.0, step=0.1, max_s=40.0):
    t = t0
    while t < t0 + max_s:
        r = cal.poll(t)
        if r is not None:
            return r, t
        t += step
    return None, t


def test_sends_a_gyro_calibration_and_reports_done(rig):
    cal, px4, lines = rig
    cal.start("127.0.0.1", 0.0)
    px4.receive()
    cmd = px4.commands[0]
    assert cmd.command == 241 and cmd.param1 == 1 and cmd.param5 == 0      # gyro only
    assert (cmd.target_system, cmd.target_component) == (1, 1)
    px4.chatter()
    px4.ack()
    px4.text("[cal] calibration started: 2 gyro")
    px4.text("[cal] calibration done: gyro")
    (ok, text), _ = run(cal)
    assert ok and "done: gyro" in text
    assert lines == ["[cal] calibration started: 2 gyro", "[cal] calibration done: gyro"]
    assert not cal.busy


def test_reports_px4_failure_text(rig):
    cal, px4, _ = rig
    cal.start("127.0.0.1", 0.0)
    px4.receive()
    px4.ack()
    px4.text("[cal] calibration failed: motion, hold still...")
    (ok, text), _ = run(cal)
    assert not ok and "motion" in text


def test_refused_when_armed(rig):
    cal, px4, _ = rig
    cal.start("127.0.0.1", 0.0)
    px4.receive()
    px4.ack(mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
    (ok, text), _ = run(cal)
    assert not ok and "TEMPORARILY_REJECTED" in text


def test_resends_then_gives_up_without_an_answer(rig):
    cal, px4, _ = rig
    cal.start("127.0.0.1", 0.0)
    (ok, text), t = run(cal)
    assert not ok and "no answer" in text
    assert cal.sends == MAX_SENDS and t == pytest.approx(MAX_SENDS * 1.0, abs=0.15)


def test_a_lost_ack_does_not_matter_once_px4_reports(rig):
    cal, px4, _ = rig
    cal.start("127.0.0.1", 0.0)
    px4.receive()
    px4.text("[cal] calibration started: 2 gyro")          # no ACK arrived
    r, _ = run(cal, max_s=5.0)
    assert r is None and cal.sends == 1                     # still waiting, no resend
    px4.text("[cal] calibration done: gyro")
    (ok, _), _ = run(cal, t0=5.0)
    assert ok


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
