"""Validated upload/playback orchestration. Importing this module opens no socket."""
import csv
import hashlib
import json
import math
from pathlib import Path
import secrets
import socket
import time
import logging
import zlib
from contextlib import nullcontext

import numpy as np
from protocol import (
    CommandPacket, CMD_START, CMD_LAND, CMD_HOLD, NAV_TRAJ, NAV_ARRIVED,
    TRAJ_MAX_PTS, build_traj_packets, build_traj_start,
    CMD_YAW_CAPS, CMD_YAW_START, CMD_YAW_STATUS,
    YAW_CAPABLE, YAW_UPLOADING, YAW_READY, YAW_PLAYING, YAW_DONE,
    YAW_HARD_MAX_RATE, YAW_MIN_DT_MS, YAW_ALIGNMENT_RAD,
    yaw_header, yaw_payload, build_yaw_begin, build_yaw_packets,
    parse_yaw_reply, require_yaw_reply,
)
from trajectory_reference import read_reference, sample_reference, validate_reference, rates

log = logging.getLogger("send_trajectory")


class YawLink:
    """One connected UDP endpoint. Old firmware cannot acknowledge v1 requests.

    Each request is retried idempotently; replies must echo the target, protocol,
    transfer ID and operation. Chunk ACK counts prevent accepting an old ACK.
    """
    def __init__(self, ip, port, drone_id, transfer_id):
        self.drone_id, self.transfer_id = drone_id, transfer_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((ip, port))
        self.sock.settimeout(0.5)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.sock.close()

    def request(self, packet, accepted, received_at_least=0):
        for _ in range(4):
            self.sock.send(packet)
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                try:
                    data = self.sock.recv(1025)
                except socket.timeout:
                    break
                parsed = parse_yaw_reply(data)
                # Ignore delayed ACKs for another operation, not bad versions.
                if parsed and parsed.command != packet[1]:
                    continue
                reply = require_yaw_reply(data, self.drone_id, self.transfer_id, packet[1])
                if reply.status not in accepted:
                    raise RuntimeError(f"firmware refused yaw command 0x{packet[1]:02x}, "
                                       f"status={reply.status} (see README)")
                if reply.received >= received_at_least:
                    return reply
        raise RuntimeError("no compatible yaw v1 ACK; old firmware, wrong IP, or lost link; "
                           "no fallback to legacy yaw is allowed")

    def caps(self):
        return self.request(yaw_header(CMD_YAW_CAPS, self.drone_id, self.transfer_id), {YAW_CAPABLE})


def prepare(args):
    if args.trajectory is None:
        raise ValueError("--trajectory is required except with --check-firmware")
    if not 1 <= args.drone_id <= 54:
        raise ValueError("--drone-id must be 1..54 for the 192.168.1.(200+id) network")
    if not math.isfinite(args.dt):
        raise ValueError("--dt must be finite")
    dt_ms = round(args.dt * 1000)
    if not 1 <= dt_ms <= 1000:
        raise ValueError("--dt must round to 1..1000 milliseconds")
    if not math.isfinite(args.takeoff_wait) or args.takeoff_wait <= 0:
        raise ValueError("--takeoff-wait must be finite and positive")
    if not 0 < args.stale_timeout <= 3 or not math.isfinite(args.stale_timeout):
        raise ValueError("--stale-timeout must be finite in (0,3] seconds")
    tangent = args.yaw_mode == "tangent"
    if tangent and dt_ms < YAW_MIN_DT_MS:
        raise ValueError("yaw samples may not exceed the actual 20 Hz firmware update rate")
    if tangent and not args.dry_run and args.max_yaw_rate is None:
        raise ValueError("live tangent mode requires an explicit TA-reviewed --max-yaw-rate in rad/s")
    max_yaw_rate = args.max_yaw_rate if args.max_yaw_rate is not None else 0.6
    source = read_reference(args.trajectory, args.yaw_mode)
    source_speed, source_rate = rates(source)
    log.info("Source: %d rows, %.3f s, N/E/D span %s m, peak speed %.6f m/s",
             len(source.t), source.t[-1]-source.t[0], np.ptp(source.xyz, axis=0), max(source_speed))
    if tangent:
        log.info("Source initial yaw %.3f deg; yaw rate [%.6f, %.6f] rad/s",
                 np.degrees(source.yaw[0]), min(source_rate), max(source_rate))
    # Check original 100 Hz data BEFORE resampling, so downsampling cannot hide
    # a fast rotation or a speed violation. Check outgoing float32 data too.
    validate_reference(source, args.max_speed, max_yaw_rate, args.stationary_speed)
    sampled = sample_reference(source, dt_ms / 1000, preserve_duration=tangent)
    validate_reference(sampled, args.max_speed, max_yaw_rate, args.stationary_speed,
                       check_tangent=False)
    if not 2 <= len(sampled.t) <= TRAJ_MAX_PTS:
        raise ValueError("upload requires 2..2400 points; change dt only with a reviewed timing plan")
    points = sampled.xyz if not tangent else np.column_stack([sampled.xyz, sampled.yaw])
    wire = np.asarray(points, dtype="<f4").astype(float)
    if not np.isfinite(wire).all():
        raise ValueError("trajectory cannot be represented as finite float32")
    sampled.xyz = wire[:, :3]
    if tangent:
        sampled.yaw = wire[:, 3]
        validate_reference(sampled, args.max_speed, max_yaw_rate, check_tangent=False)
        if max_yaw_rate > YAW_HARD_MAX_RATE:
            raise ValueError(f"--max-yaw-rate exceeds this firmware's {YAW_HARD_MAX_RATE} rad/s ceiling")
        if args.max_speed > 0.30 or np.max(np.abs(wire[:, :3])) > 20 or np.max(np.abs(wire[:, 3])) > 256:
            raise ValueError("trajectory or configured limit exceeds firmware speed/coordinate/yaw bounds")
    ident = secrets.randbelow(0xFFFFFFFF) + 1 if tangent else secrets.randbelow(255) + 1
    payload = yaw_payload(wire) if tangent else None
    packets = build_yaw_packets(args.drone_id, ident, payload) if tangent else build_traj_packets(ident, wire)
    start = yaw_header(CMD_YAW_START, args.drone_id, ident) if tangent else build_traj_start(ident, dt_ms)
    begin = build_yaw_begin(args.drone_id, ident, len(wire), dt_ms, max_yaw_rate,
                            zlib.crc32(payload)) if tangent else None
    if max(map(len, packets)) > 1024:
        raise ValueError("packet exceeds firmware receive buffer")
    speed, yr = rates(sampled)
    log.info("%s yaw: %d points, %.3f s, N/E/D span %s m, max speed %.6f m/s",
             args.yaw_mode, len(wire), sampled.t[-1], np.ptp(sampled.xyz, axis=0), max(speed))
    if tangent:
        log.info("Initial reference yaw %.3f deg CW from North; rate [%.6f, %.6f] rad/s; limit %.6f",
                 np.degrees(sampled.yaw[0]), min(yr), max(yr), max_yaw_rate)
        log.warning("Original +North placement is incompatible with tangent start; "
                    "no alignment manoeuvre is performed. TA must approve initial placement.")
        if args.stationary_speed != 1e-4:
            log.warning("NONDEFAULT low-speed yaw policy: %.6f m/s; TA review required", args.stationary_speed)
    log.info("Packets: %d chunks; largest %d B / 1024 B", len(packets), max(map(len, packets)))
    return sampled, dt_ms, ident, packets, start, begin


def run(args):
    if args.check_firmware:
        if args.dry_run or args.takeoff:
            raise ValueError("--check-firmware is a standalone read-only capability check")
        with YawLink(args.esp_ip or f"192.168.1.{200 + args.drone_id}", args.cmd_port,
                     args.drone_id, secrets.randbelow(0xFFFFFFFF) + 1) as link:
            reply = link.caps()
            print(json.dumps(vars(reply), indent=2))
        return
    ref, dt_ms, ident, packets, start, begin = prepare(args)
    if args.dry_run:
        print(f"DRY RUN PASS: {args.yaw_mode}, {len(ref.t)} float32 points, "
              f"{ref.t[-1]:.3f} s; packets validated; no sockets opened or commands sent.")
        return

    # Lazy import keeps offline generation and dry-runs independent of comms.
    from comms import CommsNode
    latest = {}
    owns_flight = False
    tangent = args.yaw_mode == "tangent"
    log_dir = Path(args.log_dir) / (time.strftime("%Y%m%d_%H%M%S") + f"_{args.yaw_mode}_{ident}")
    log_dir.mkdir(parents=True, exist_ok=False)
    telemetry = (log_dir / "telemetry.jsonl").open("w", encoding="utf-8")
    events = (log_dir / "events.jsonl").open("w", encoding="utf-8")

    def event(name, **fields):
        events.write(json.dumps(dict(event=name, host_monotonic_s=time.monotonic(),
                                     host_unix_s=time.time(), **fields)) + "\n")
        events.flush()

    def on_telemetry(pkt, ip):
        if pkt.drone_id == args.drone_id:
            now = time.monotonic()
            latest.update(pkt=pkt, at=now, ip=ip)
            telemetry.write(json.dumps(dict(host_monotonic_s=now, host_unix_s=time.time(),
                                           source_ip=ip, **vars(pkt))) + "\n")
            telemetry.flush()

    def fresh():
        if "at" not in latest or time.monotonic() - latest["at"] > args.stale_timeout:
            raise RuntimeError("drone telemetry stale; cannot confirm mission state")
        pkt = latest["pkt"]
        if not all(math.isfinite(v) for v in (pkt.ned_x, pkt.ned_y, pkt.heading_rad)):
            raise RuntimeError("target telemetry has nonfinite position or heading")
        return pkt

    def wait_for(condition, seconds, require_fresh=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if "pkt" in latest:
                pkt = fresh() if require_fresh else latest["pkt"]
                if condition(pkt):
                    return True
            time.sleep(0.05)
        return False

    # Tangent mode does not need (or automatically broadcast) nav-tag maps.
    comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                      config_path=None if tangent else (args.config if Path(args.config).exists() else None))
    comms.on_telemetry(on_telemetry)

    def send(command, name):
        if not comms.send_command(args.drone_id, CommandPacket(command)):
            raise RuntimeError(f"{name} not sent: unknown drone IP")
        event(name)

    try:
        comms.start()
        if not wait_for(lambda p: True, 30, require_fresh=False):
            raise RuntimeError("no target drone telemetry")
        fresh()
        ip = args.esp_ip or latest["ip"]
        if tangent and ip != latest["ip"]:
            raise RuntimeError("--esp-ip disagrees with target drone's telemetry source")
        context = YawLink(ip, args.cmd_port, args.drone_id, ident) if tangent else nullcontext(None)
        with context as link:
            if tangent:
                caps = link.caps()  # must succeed BEFORE upload/arming/takeoff
                if (len(ref.t) > caps.max_points or dt_ms < caps.min_dt_ms or
                        args.max_yaw_rate > caps.max_yaw_rate):
                    raise RuntimeError("trajectory exceeds connected firmware capabilities")
                if fresh().px4_link_ok is not True:
                    raise RuntimeError("fresh PX4 heartbeat/position link is not confirmed")
                if args.takeoff and fresh().px4_armed != 0:
                    raise RuntimeError("--takeoff requires a disarmed vehicle; review current flight state")
                mismatch = abs(math.atan2(math.sin(ref.yaw[0] - fresh().heading_rad),
                                          math.cos(ref.yaw[0] - fresh().heading_rad)))
                if mismatch > min(caps.alignment_rad, YAW_ALIGNMENT_RAD):
                    raise RuntimeError(f"initial heading mismatch {math.degrees(mismatch):.2f} deg; "
                                       "stop and obtain TA-approved placement; no rotation commanded")

            with (log_dir / "reference_sent.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["t", "x", "y", "z", "yaw_rad"])
                for i in range(len(ref.t)):
                    # Fixed yaw is captured on the ESP32 at START; do not invent
                    # it here from a potentially earlier telemetry sample.
                    writer.writerow([ref.t[i], *ref.xyz[i], "" if ref.yaw is None else ref.yaw[i]])
            metadata = dict(drone_id=args.drone_id, yaw_mode=args.yaw_mode,
                            source_sha256=hashlib.sha256(Path(args.trajectory).read_bytes()).hexdigest(),
                            dt_ms=dt_ms, transfer_id=ident, max_yaw_rate=args.max_yaw_rate,
                            stationary_speed=args.stationary_speed,
                            note="Reference and supplementary telemetry; use PX4 ULog for flight evidence")
            (log_dir / "run.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

            if args.confirm or tangent:
                token = f"{'TANGENT' if tangent else 'TRAJ'}-{args.drone_id}"
                answer = input(f"TA clearance, initial orientation and rate limit verified; type {token}: ").strip()
                if answer != token:
                    log.info("Cancelled before flight commands")
                    return
            if args.takeoff:
                send(CMD_START, "CMD_START")
                owns_flight = True
                if not wait_for(lambda p: False, args.takeoff_wait):
                    pass  # timed wait with telemetry freshness checks
            else:
                owns_flight = True  # operator explicitly adopts an already-hovering vehicle

            if tangent:
                pkt = fresh()
                if pkt.px4_link_ok is not True or pkt.px4_armed != 1 or pkt.px4_custom_main_mode != 6:
                    raise RuntimeError("fresh PX4 armed/Offboard status not confirmed; check QGC and serial output")
                # mission_task calls nav_cancel() when CMD_START begins. Upload
                # after takeoff, as in legacy Lab 5, so that reset cannot cancel
                # the pending yaw plan. Compatibility was checked before arming.
                link.request(begin, {YAW_UPLOADING, YAW_READY})
                count = 0
                for packet in packets:
                    fresh()
                    count += packet[10]
                    link.request(packet, {YAW_UPLOADING, YAW_READY}, count)
                link.request(yaw_header(CMD_YAW_STATUS, args.drone_id, ident), {YAW_READY}, len(ref.t))
                event("yaw_start_send")
                link.request(start, {YAW_PLAYING})
                event("yaw_start_ack")
            else:
                # Original fixed-yaw wire format, upload cadence and retry policy.
                for attempt in range(1, 6):
                    for packet in packets:
                        if not comms.send_raw(args.drone_id, packet):
                            raise RuntimeError("trajectory packet not sent")
                        time.sleep(0.025)
                    event("fixed_start_send", attempt=attempt)
                    comms.send_raw(args.drone_id, start)
                    if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
                        break
                else:
                    raise RuntimeError("drone refused fixed-yaw trajectory")

            end = time.monotonic() + ref.t[-1] + 10
            if tangent:
                while time.monotonic() < end:
                    fresh()
                    status = link.request(yaw_header(CMD_YAW_STATUS, args.drone_id, ident),
                                          {YAW_PLAYING, YAW_DONE})
                    if status.status == YAW_DONE:
                        break
                    time.sleep(0.2)
                else:
                    raise RuntimeError("yaw trajectory completion timeout")
            else:
                while fresh().nav_state == NAV_TRAJ and time.monotonic() < end:
                    time.sleep(0.2)
                if fresh().nav_state != NAV_ARRIVED:
                    raise RuntimeError("fixed trajectory interrupted or timed out")
            event("trajectory_complete")
            for _ in range(3 if args.finish == "land" else 1):
                send(CMD_LAND if args.finish == "land" else CMD_HOLD, args.finish.upper())
                time.sleep(0.1)
            owns_flight = False
    except (Exception, KeyboardInterrupt):
        if owns_flight:
            log.error("Mission aborted: requesting LAND; RC pilot remains responsible")
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
                time.sleep(0.1)
        raise
    finally:
        comms.stop()
        telemetry.close()
        events.close()
        log.info("Local experiment records: %s", log_dir)
