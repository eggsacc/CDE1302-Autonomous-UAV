"""Offline Python -> real navigation -> real MAVLink serialisation acceptance.

Only ESP-IDF hardware/RTOS boundaries are replaced. No socket, serial port,
autopilot, aircraft, task scheduler or flight dynamics is present in these tests.
"""
import ctypes
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
from types import SimpleNamespace
import zlib

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "laptop"))
from protocol import (
    CMD_YAW_START, CMD_YAW_STATUS, YAW_UPLOADING, YAW_READY, YAW_PLAYING,
    YAW_DONE, YAW_CANCELLED, YAW_LIMIT, YAW_NOT_FLYING, YAW_MISALIGNED,
    NAV_IDLE, NAV_TRAJ, NAV_ARRIVED, build_yaw_begin, build_yaw_packets,
    parse_yaw_reply, yaw_header, yaw_payload,
)
from trajectory_reference import read_reference, sample_reference, write_tangent_csv
from trajectory_runner import prepare


@pytest.fixture(scope="module")
def firmware(tmp_path_factory):
    compiler, zig = os.environ.get("YAW_CC"), os.environ.get("ZIG")
    command = [compiler] if compiler else ([zig, "cc"] if zig else None)
    if command is None:
        compiler = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
        if not compiler:
            pytest.skip("host C compiler unavailable; full navigation/MAVLink test not run")
        command = [compiler]
    directory = tmp_path_factory.mktemp("firmware_pipeline")
    stubs = directory / "include"
    # Forward only platform headers; all application and MAVLink headers are real.
    for name in ("sdkconfig.h", "driver/uart.h", "driver/gpio.h", "driver/i2c_master.h",
                 "freertos/FreeRTOS.h", "freertos/task.h", "freertos/semphr.h",
                 "esp_log.h", "esp_timer.h", "esp_heap_caps.h", "sys/socket.h",
                 "netinet/in.h", "arpa/inet.h", "fcntl.h", "unistd.h"):
        header = stubs / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "platform.h"\n', encoding="utf-8")
    host = Path(__file__).with_name("firmware_host")
    library = directory / ("firmware.dll" if os.name == "nt" else "firmware.so")
    cmd = command + ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared",
                     "-Wno-unused-parameter", "-Wno-unused-function",
                     "-Wno-address-of-packed-member",
                     "-I", str(stubs), "-I", str(host), "-I", str(ROOT / "main"),
                     "-I", str(ROOT / "components/mavlink/include"),
                     str(ROOT / "main/traj_yaw.c"), str(host / "io.c"),
                     str(host / "nav_wrapper.c"), str(host / "mavlink_wrapper.c"),
                     "-o", str(library)]
    if os.name != "nt":
        cmd += ["-fPIC", "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    assert result.returncode == 0, (result.stdout + result.stderr)[-6000:]
    dll = ctypes.CDLL(str(library))
    dll.host_set_time.argtypes = [ctypes.c_int64]
    dll.host_telemetry.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int] * 2 + [ctypes.c_uint32] * 3
    dll.host_nav_packet.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
    dll.host_setpoint.argtypes = [ctypes.c_void_p, ctypes.c_int]
    dll.host_legacy_put.argtypes = [ctypes.c_int] * 3 + [ctypes.c_void_p, ctypes.c_int]
    dll.host_legacy_start.argtypes = [ctypes.c_int, ctypes.c_int]
    return dll


ORIGIN = (1.25, -0.75, -1.0)
EPOCH_US = 10_000_000


def reset(fw, heading=3.13):
    fw.host_set_time(EPOCH_US)
    fw.host_mav_reset()
    fw.host_nav_reset()
    telemetry(fw, heading=heading)


def telemetry(fw, *, heading=3.13, xyz=ORIGIN, armed=1, mode=6,
              pos_age=0, att_age=0, hb_age=0):
    # Encoded simulated PX4 messages enter the actual UART telemetry parser.
    fw.host_telemetry(*xyz, heading, armed, mode, pos_age, att_age, hb_age)


def request(fw, packet, flying=True):
    reply = ctypes.create_string_buffer(28)
    assert fw.host_nav_packet(packet, len(packet), flying, reply)
    return parse_yaw_reply(reply.raw)


def upload(fw, points=None, *, ident=123, rate=0.6, dt=50):
    if points is None:
        points = [(0, 0, 0, 3.13), (-0.005, 0, 0, 3.15), (-0.01, -0.0001, 0, 3.17)]
    payload = yaw_payload(points)
    begin = build_yaw_begin(2, ident, len(points), dt, rate, zlib.crc32(payload))
    assert request(fw, begin).status == YAW_UPLOADING
    for packet in build_yaw_packets(2, ident, payload):
        reply = request(fw, packet)
    return reply


def setpoint(fw):
    """Independently decode and CRC-check actual UART bytes, MAVLink message 84."""
    output = ctypes.create_string_buffer(512)
    size = fw.host_setpoint(output, len(output))
    packet = output.raw[:size]
    assert packet[0] == 0xFD and packet[2:4] == b"\0\0"  # MAVLink 2, unsigned
    length = packet[1]
    assert packet[5:10] == bytes([1, 190, 84, 0, 0])
    assert len(packet) == 12 + length
    crc = 0xFFFF
    for byte in packet[1:10 + length] + bytes([143]):  # common.xml message 84 CRC extra
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    assert struct.unpack("<H", packet[-2:])[0] == crc
    values = struct.unpack("<I11fHBBB", packet[10:10 + length].ljust(53, b"\0"))
    assert values[13:] == (1, 1, 1)  # PX4 sysid, compid, MAV_FRAME_LOCAL_NED
    assert all(math.isnan(v) for v in values[7:10])  # ignored acceleration
    assert math.isnan(values[11])  # ignored yaw rate, yaw POSITION remains active
    return dict(time_ms=values[0], xyz=np.array(values[1:4]),
                velocity=np.array(values[4:7]), yaw=values[10], mask=values[12])


def args(path, mode):
    return SimpleNamespace(trajectory=str(path), drone_id=2, yaw_mode=mode, dt=0.05,
                           takeoff_wait=12., stale_timeout=3., dry_run=True,
                           max_yaw_rate=.6, max_speed=.30, stationary_speed=1e-4)


def test_csv_python_navigation_to_mavlink_wrap_and_completion(firmware, tmp_path):
    # A separate synthetic, feasible arc tests playback. Never substitute it for Lab 5.
    t = np.arange(101) * .01
    angle = math.pi - .02 + .4 * t
    xyz = np.column_stack([.25 * (np.sin(angle) - np.sin(angle[0])),
                           .25 * (np.cos(angle[0]) - np.cos(angle)), np.zeros_like(t)])
    source, target = tmp_path / "synthetic.csv", tmp_path / "synthetic_yaw.csv"
    np.savetxt(source, np.column_stack([t, xyz]), delimiter=",", header="t,x,y,z", comments="")
    write_tangent_csv(source, target)
    ref, dt_ms, ident, packets, start, begin = prepare(args(target, "tangent"))
    reset(firmware, ref.yaw[0])
    assert request(firmware, begin).status == YAW_UPLOADING
    for packet in packets:
        assert request(firmware, packet).status in {YAW_UPLOADING, YAW_READY}
    assert request(firmware, start).status == YAW_PLAYING
    emitted_yaw = []
    for elapsed in np.arange(41) * .025:  # endpoints AND between-sample interpolation
        firmware.host_set_time(EPOCH_US + round(elapsed * 1e6))
        telemetry(firmware, heading=ref.yaw[0])
        # Lost START ACK/retry must not reset the real playback clock.
        if elapsed == .5:
            assert request(firmware, start).status == YAW_PLAYING
        firmware.host_nav_tick()
        out = setpoint(firmware)
        expected_xyz = np.array([np.interp(elapsed, ref.t, ref.xyz[:, c]) for c in range(3)])
        np.testing.assert_allclose(out["xyz"], np.array(ORIGIN) + expected_xyz, atol=2e-6)
        desired = np.interp(elapsed, ref.t, ref.yaw)
        assert math.atan2(math.sin(out["yaw"] - desired), math.cos(out["yaw"] - desired)) == pytest.approx(0, abs=2e-6)
        emitted_yaw.append(out["yaw"])
        if elapsed < ref.t[-1]:
            i = min(int(elapsed * 1000 / dt_ms), len(ref.t) - 2)
            np.testing.assert_allclose(out["velocity"], (ref.xyz[i+1] - ref.xyz[i]) * 1000 / dt_ms, atol=2e-6)
            assert out["mask"] == 0x9C0 and firmware.host_nav_state() == NAV_TRAJ
        else:
            assert out["mask"] == 0x9F8 and np.isnan(out["velocity"]).all()
            assert firmware.host_nav_state() == NAV_ARRIVED
    assert max(emitted_yaw) > 3 and min(emitted_yaw) < -3
    assert np.max(np.abs(np.diff(np.unwrap(emitted_yaw)))) < .02
    assert request(firmware, yaw_header(CMD_YAW_STATUS, 2, ident)).status == YAW_DONE
    assert request(firmware, start).status == YAW_DONE


def test_original_fixed_ellipse_reaches_mavlink_with_constant_heading(firmware):
    ref, dt_ms, ident, *_ = prepare(args(ROOT / "trajectory/ellipse_min_snap_traj.csv", "fixed"))
    heading = .73
    reset(firmware, heading)
    data = np.asarray(ref.xyz, dtype="<f4").tobytes()
    firmware.host_legacy_put(ident, len(ref.t), 0, data, len(ref.t))
    firmware.host_legacy_start(ident, dt_ms)
    for i, elapsed in enumerate(ref.t):
        firmware.host_set_time(EPOCH_US + round(elapsed * 1e6))
        telemetry(firmware, heading=heading + .1)  # estimate changes; held heading must not
        firmware.host_nav_tick()
        out = setpoint(firmware)
        np.testing.assert_allclose(out["xyz"], np.array(ORIGIN) + ref.xyz[i], atol=2e-6)
        assert out["yaw"] == pytest.approx(heading, abs=1e-6)
        assert out["mask"] == (0x9F8 if i == len(ref.t) - 1 else 0x9C0)
    assert len(ref.t) == 681 and ref.t[-1] == 34.
    assert firmware.host_nav_state() == NAV_ARRIVED


@pytest.mark.parametrize("change", [dict(armed=0), dict(mode=3), dict(pos_age=500),
                                    dict(att_age=301), dict(hb_age=1501)])
def test_start_refuses_stale_or_nonoffboard_state(firmware, change):
    reset(firmware)
    assert upload(firmware).status == YAW_READY
    telemetry(firmware, **change)
    assert request(firmware, yaw_header(CMD_YAW_START, 2, 123)).status == YAW_NOT_FLYING
    assert firmware.host_nav_state() == NAV_IDLE
    assert math.isnan(setpoint(firmware)["yaw"])


@pytest.mark.parametrize("change", [dict(armed=0), dict(mode=3), dict(pos_age=500),
                                    dict(att_age=301), dict(hb_age=1501)])
def test_playback_cancels_to_hold_on_telemetry_loss_or_takeover(firmware, change):
    reset(firmware)
    assert upload(firmware).status == YAW_READY
    assert request(firmware, yaw_header(CMD_YAW_START, 2, 123)).status == YAW_PLAYING
    firmware.host_set_time(EPOCH_US + 25_000)
    telemetry(firmware, **change)
    firmware.host_nav_tick()
    out = setpoint(firmware)
    assert firmware.host_nav_state() == NAV_IDLE
    assert out["mask"] == 0x9F8 and math.isnan(out["yaw"])
    np.testing.assert_allclose(out["xyz"], ORIGIN)
    assert request(firmware, yaw_header(CMD_YAW_STATUS, 2, 123)).status == YAW_CANCELLED


def test_mission_and_initial_alignment_gate_do_not_change_setpoint(firmware):
    reset(firmware)
    assert upload(firmware).status == YAW_READY
    start = yaw_header(CMD_YAW_START, 2, 123)
    assert request(firmware, start, flying=False).status == YAW_NOT_FLYING
    telemetry(firmware, heading=0)
    assert request(firmware, start).status == YAW_MISALIGNED
    assert firmware.host_nav_state() == NAV_IDLE
    assert math.isnan(setpoint(firmware)["yaw"])


@pytest.mark.parametrize("axis,bad", [(0, math.nan), (1, math.inf), (2, -math.inf)])
def test_start_rejects_nonfinite_px4_origin(firmware, axis, bad):
    reset(firmware)
    assert upload(firmware).status == YAW_READY
    xyz = list(ORIGIN)
    xyz[axis] = bad
    telemetry(firmware, xyz=xyz)
    assert request(firmware, yaw_header(CMD_YAW_START, 2, 123)).status == YAW_NOT_FLYING
    assert firmware.host_nav_state() == NAV_IDLE


def test_hold_and_mavlink_watchdog_stop_velocity_feedforward(firmware):
    reset(firmware)
    upload(firmware)
    request(firmware, yaw_header(CMD_YAW_START, 2, 123))
    firmware.host_nav_tick()
    assert setpoint(firmware)["mask"] == 0x9C0
    firmware.host_set_time(EPOCH_US + 301_000)  # nav task stopped updating
    out = setpoint(firmware)
    assert out["mask"] == 0x9F8 and math.isnan(out["yaw"])
    firmware.host_nav_cancel()
    firmware.host_nav_tick()  # stale tick cannot overwrite explicit HOLD
    assert firmware.host_nav_state() == NAV_IDLE
    assert math.isnan(setpoint(firmware)["yaw"])
    assert request(firmware, yaw_header(CMD_YAW_START, 2, 123)).status == YAW_CANCELLED


def test_strict_original_ellipse_is_rejected_by_actual_firmware_too(firmware):
    ref = sample_reference(read_reference(ROOT / "trajectory/ellipse_min_snap_traj_tangent_yaw.csv", "tangent"), .05, True)
    reset(firmware, ref.yaw[0])
    # Deliberately bypass the laptop validator to exercise the independent C guard.
    assert upload(firmware, np.column_stack([ref.xyz, ref.yaw]), rate=1.).status == YAW_LIMIT
    assert request(firmware, yaw_header(CMD_YAW_START, 2, 123)).status == YAW_LIMIT
    assert firmware.host_nav_state() == NAV_IDLE
