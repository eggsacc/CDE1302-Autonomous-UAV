#!/usr/bin/env python3
# ^ "shebang" line: on Linux/macOS it lets you run this file directly (./mission2.py)
#   using whichever python3 is on your PATH. Windows ignores it.
"""
Lawnmower search mission: arm/takeoff -> zig-zag sweep -> return to start -> land.

Copy of simple_waypoint_mission.py. The only change: if no --waypoint or
--waypoints-file is given, the waypoints are generated as a lawnmower sweep
over a rectangle, followed by a return to the takeoff point (0, 0).

Custom UDP commands (see protocol.py):
  - CMD_START : arm + take off to the firmware's cruise altitude
  - CMD_GOTO  : fly to a map-frame (x, y); resent until the drone acts on it
  - CMD_LAND  : land in place

Coordinates are metres relative to the takeoff point: x = the direction the
drone's nose points at takeoff (PX4 North), y = to its right (PX4 East).

Example (1.5 m x 1.0 m area, lanes at most 0.5 m apart):
    python laptop/lawnmower_mission.py --drone-id 22 --confirm \
        --x-max 1.5 --y-max 1.0 --lane-spacing 0.5

    python laptop/lawnmower_mission.py --drone-id 22 --plan-only   # just print the plan

Exit code: 0 all waypoints reached, 1 error or waypoint timeout, 130 Ctrl+C.
"""
# NOTE: the examples above say "lawnmower_mission.py" - this file is saved as
# mission2.py, so run it as:  python laptop/mission2.py --drone-id 22 ...

# Lets us write type hints like "list[float]" and "Optional[X]" without them
# being evaluated at import time (keeps older Python versions happy).
from __future__ import annotations

import argparse      # reads the command-line options (--drone-id, --x-max, ...)
import logging       # prints timestamped status messages instead of plain print()
import math          # ceil(), hypot(), dist() for the geometry
import signal        # lets us catch Ctrl+C ourselves so we can land safely
import sys           # sys.argv (the command-line words) and sys.exit()
import threading     # Condition object: safely share telemetry between threads
import time          # sleep() and monotonic() clock for timeouts
from dataclasses import dataclass   # quick way to make a simple "struct" class
from pathlib import Path            # nicer file-path handling than plain strings
from typing import Optional         # Optional[X] means "X or None"

# CommsNode handles all the UDP networking with the drone (see comms.py).
from comms import CommsNode
# Constants and packet classes that define the wire format (see protocol.py).
from protocol import (
    CMD_GOTO,        # command: fly to (x, y)
    CMD_HOLD,        # command: stop and hover where you are
    CMD_LAND,        # command: land in place
    CMD_START,       # command: arm the motors and take off
    NAV_FLYING,      # nav state reported by drone: flying forward toward goal
    NAV_ROTATING,    # nav state: turning on the spot to face the goal
    NAV_STUCK,       # nav state: can't make progress (obstacle in the way)
    CommandPacket,   # class we build and send to the drone
    TelemetryPacket, # class the drone sends back to us (position, state, ...)
)

# Set up logging once for the whole script:
logging.basicConfig(
    level=logging.INFO,                                  # show INFO, WARNING and ERROR messages
    format="%(asctime)s | %(levelname)s | %(message)s",  # e.g. "14:02:11 | INFO | CMD_START sent"
    datefmt="%H:%M:%S",                                  # time only, no date
)
# Our own named logger; log.info(...) / log.warning(...) are used everywhere below.
log = logging.getLogger("lawnmower_mission")

# If the drone still hasn't started moving after this many seconds, send the
# CMD_GOTO again (UDP packets can get lost over WiFi).
GOTO_RESEND_S = 2.0   # resend CMD_GOTO if the drone hasn't started on it
# The drone's forward speed. Only used here to print an estimated flight time;
# the real speed is set in the firmware.
CRUISE_SPEED_MS = 0.10   # NAV_CRUISE_SPEED_MS in firmware nav_task.h (for the time estimate)


# ---------------------------------------------------------------------------
# Lawnmower planner
# ---------------------------------------------------------------------------

def lane_positions(lo: float, hi: float, max_spacing: float) -> list[float]:
    """Evenly spaced lane positions from lo to hi (inclusive), no gap > max_spacing."""
    # If the range has zero (or negative) width, there is room for only one lane.
    if hi <= lo:
        return [lo]
    # How many gaps we need so that no gap is wider than max_spacing.
    # e.g. range 0.6 m, max 0.5 m -> 0.6/0.5 = 1.2 -> round UP to 2 gaps.
    n_gaps = math.ceil((hi - lo) / max_spacing)
    # Actual (equal) gap size: 0.6 m / 2 gaps = 0.3 m between lanes.
    step = (hi - lo) / n_gaps
    # n gaps means n+1 lanes: lo, lo+step, lo+2*step, ..., hi.
    return [lo + i * step for i in range(n_gaps + 1)]


def lawnmower(x_min: float, x_max: float, y_min: float, y_max: float,
              lane_spacing: float, margin: float = 0.0,
              lanes_along: str = "x") -> list[tuple[float, float]]:
    """Zig-zag waypoints covering the rectangle (shrunk inwards by margin).

    lanes_along="x": each lane runs along x (forward/back), lanes step along y.
    lanes_along="y": each lane runs along y (left/right), lanes step along x.
    Every second lane is flown in reverse. Each lane is just its two end points,
    so the drone only stops and turns at the corners.
    """
    # Shrink the rectangle inwards by "margin" on every side so the drone
    # keeps away from the walls/edges. x0..x1 and y0..y1 are the usable area.
    x0, x1 = x_min + margin, x_max - margin
    y0, y1 = y_min + margin, y_max - margin
    # If the margin ate the whole rectangle, the plan is impossible.
    if x1 < x0 or y1 < y0:
        raise ValueError("margin is larger than half the area")

    waypoints = []   # the list of (x, y) points we will build up
    if lanes_along == "x":
        # Lanes are lines of constant y; walk through each lane's y value.
        # enumerate() gives us i = 0, 1, 2, ... alongside each y.
        for i, y in enumerate(lane_positions(y0, y1, lane_spacing)):
            # The two ends of this lane: the near end (x0) and the far end (x1).
            ends = [(x0, y), (x1, y)]
            # Even lanes (i = 0, 2, 4...) go x0 -> x1; odd lanes go x1 -> x0
            # (ends[::-1] is the list reversed). That makes the zig-zag.
            waypoints += ends if i % 2 == 0 else ends[::-1]
    else:
        # Same idea, but lanes are lines of constant x running along y.
        for i, x in enumerate(lane_positions(x0, x1, lane_spacing)):
            ends = [(x, y0), (x, y1)]
            waypoints += ends if i % 2 == 0 else ends[::-1]
    # Round every coordinate to the nearest millimetre (3 decimal places) so the
    # printed plan is tidy (no 0.30000000000000004 style floating-point noise).
    return [(round(x, 3), round(y, 3)) for x, y in waypoints]


def path_length(waypoints: list[tuple[float, float]]) -> float:
    """Total distance flown from the takeoff point (0, 0) through all waypoints."""
    # Put the takeoff point (0, 0) at the front, since that's where the drone starts.
    pts = [(0.0, 0.0)] + list(waypoints)
    # zip(pts, pts[1:]) pairs each point with the next one: (p0,p1), (p1,p2), ...
    # math.dist() is the straight-line distance between two points; add them all up.
    return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))


# ---------------------------------------------------------------------------
# Telemetry tracking (the drone sends us its position ~10 times a second)
# ---------------------------------------------------------------------------

# @dataclass automatically creates __init__ etc. This is just a little box that
# holds "the most recent telemetry packet, who sent it, and when it arrived".
@dataclass
class TelemetrySnapshot:
    packet: Optional[TelemetryPacket] = None   # the latest packet (None until the first arrives)
    source_ip: Optional[str] = None            # IP address it came from
    received_at: float = 0.0                   # time.monotonic() when it arrived


class TelemetryTracker:
    # Keeps the latest telemetry from ONE drone. CommsNode receives packets on a
    # background thread and calls callback(); the main thread reads latest().
    # The threading.Condition stops the two threads from clashing.
    def __init__(self, drone_id: int, print_live: bool = False) -> None:
        self.drone_id = drone_id                 # only accept packets from this drone
        self._snapshot = TelemetrySnapshot()     # starts empty (no packet yet)
        self._condition = threading.Condition()  # a lock + a way to wake waiting threads
        self._print_live = print_live            # True -> log every packet (--live-telem)

    def callback(self, packet: TelemetryPacket, source_ip: str) -> None:
        # Called by CommsNode (on its network thread) for EVERY telemetry packet.
        # Ignore packets from other drones that may be on the same network.
        if packet.drone_id != self.drone_id:
            return

        # "with" grabs the lock so the main thread can't read half-updated data.
        with self._condition:
            # Replace the stored snapshot with this new packet + arrival time.
            self._snapshot = TelemetrySnapshot(
                packet=packet,
                source_ip=source_ip,
                received_at=time.monotonic(),
            )
            # Wake up anyone sitting in wait_for_first_packet().
            self._condition.notify_all()

            if self._print_live:
                # tag_id is -1 when no AprilTag is seen; show "none" in that case.
                tag_text = "none" if packet.tag_id < 0 else str(packet.tag_id)
                # Print one line per packet: position, heading, nav state, tag info.
                log.info(
                    "TELEM drone=%d pos=(%.2f, %.2f) heading=%.2f state=%s tag=%s dist=%.2f stuck=%s",
                    packet.drone_id,      # which drone
                    packet.ned_x,         # x position (m, North)
                    packet.ned_y,         # y position (m, East)
                    packet.heading_rad,   # which way the nose points (radians)
                    packet.nav_state_name,  # e.g. "FLYING", "ROTATING", "ARRIVED"
                    tag_text,             # AprilTag id or "none"
                    packet.tag_dist_m,    # distance to that tag (m)
                    "yes" if packet.is_stuck else "no",
                )

    def latest(self) -> TelemetrySnapshot:
        # Return a COPY of the current snapshot (taken while holding the lock), so
        # the caller gets a consistent packet/time pair even if a new packet arrives.
        with self._condition:
            return TelemetrySnapshot(
                packet=self._snapshot.packet,
                source_ip=self._snapshot.source_ip,
                received_at=self._snapshot.received_at,
            )

    def wait_for_first_packet(self, timeout_s: float, port: int, heard) -> TelemetrySnapshot:
        # Block until the drone has sent at least one packet, or give up after timeout_s.
        # "heard" is a function returning which drone IDs we HAVE heard from
        # (used only to make the error message more helpful).
        deadline = time.monotonic() + timeout_s   # the moment we give up
        with self._condition:
            # Keep waiting as long as no packet has arrived yet.
            while self._snapshot.packet is None:
                remaining = deadline - time.monotonic()   # time left before giving up
                if remaining <= 0:
                    # Out of time: stop with an error that lists any other drones heard,
                    # e.g. if you typed the wrong --drone-id.
                    raise TimeoutError(
                        f"No telemetry from drone {self.drone_id} on UDP {port} "
                        f"(heard drones: {sorted(heard()) or 'none'})."
                    )
                # Sleep until callback() calls notify_all(), or at most 0.5 s,
                # then loop round and check again.
                self._condition.wait(timeout=min(0.5, remaining))
            # A packet arrived: hand back a copy of it.
            return self.latest()


# ---------------------------------------------------------------------------
# Reading waypoints from the command line or a file
# ---------------------------------------------------------------------------

def parse_waypoint(value: str) -> tuple[float, float]:
    # Turn a string like "1.0, 2.0" into the tuple (1.0, 2.0).
    # Split on the comma and trim spaces from each piece.
    parts = [item.strip() for item in value.split(",")]
    # Must be exactly two numbers: x and y.
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            "Waypoints must be given as x,y (for example 1.0,2.0)."
        )

    try:
        # Convert both pieces from text to numbers.
        x_value = float(parts[0])
        y_value = float(parts[1])
    except ValueError as exc:
        # float() failed, e.g. "abc,2". Give a clear error message instead.
        raise argparse.ArgumentTypeError(
            "Waypoint coordinates must be numeric."
        ) from exc

    return x_value, y_value


def load_waypoints_from_file(path: str) -> list[tuple[float, float]]:
    # Read waypoints from a text file. Each line is "x,y" or "x y".
    # expanduser() turns "~/file.txt" into your real home folder path.
    file_path = Path(path).expanduser()
    if not file_path.exists():
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    waypoints: list[tuple[float, float]] = []   # results go here
    # Read the whole file, split into lines, and number them from 1 (for error messages).
    for line_number, raw_line in enumerate(
        file_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()   # remove leading/trailing spaces
        # Skip empty lines and comment lines starting with "#".
        if not line or line.startswith("#"):
            continue

        if "," in line:
            # "x,y" format: reuse the same parser as --waypoint.
            waypoint = parse_waypoint(line)
        else:
            # "x y" format (separated by spaces/tabs).
            parts = line.split()
            if len(parts) != 2:
                raise ValueError(
                    f"Invalid waypoint format on line {line_number}: {raw_line}"
                )
            try:
                waypoint = (float(parts[0]), float(parts[1]))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid waypoint coordinates on line {line_number}: {raw_line}"
                ) from exc

        waypoints.append(waypoint)   # add this line's (x, y) to the list

    return waypoints


def join_waypoint_values(argv: list[str]) -> list[str]:
    """Turn '--waypoint -1,0' into '--waypoint=-1,0' so argparse accepts it."""
    # Problem: argparse thinks "-1,0" is another option (it starts with "-"),
    # so "--waypoint -1,0" fails. Gluing it as "--waypoint=-1,0" fixes that.
    # out = the rewritten list; it = an iterator so we can grab "the next word".
    out, it = [], iter(argv)
    for arg in it:
        # If this word is "--waypoint", pull the NEXT word and glue them with "=".
        # (next(it, '') returns '' if --waypoint was the last word.)
        # Any other word is copied across unchanged.
        out.append(f"--waypoint={next(it, '')}" if arg == "--waypoint" else arg)
    return out


def parse_args() -> argparse.Namespace:
    # Define every command-line option. Afterwards you read them as args.drone_id,
    # args.x_max, etc. (dashes become underscores).
    parser = argparse.ArgumentParser(
        description="Send a simple takeoff -> waypoint -> land mission from the laptop GCS"
    )
    # Which drone to talk to (must match the ID flashed into its ESP32). Required.
    parser.add_argument(
        "--drone-id",
        type=int,
        required=True,
        help="Flashed ESP32 CONFIG_DRONE_ID.",
    )
    # Flag (no value): if given, coordinates are arena coordinates from setup.yaml
    # instead of "relative to where the drone took off".
    parser.add_argument(
        "--map-frame",
        action="store_true",
        help="Treat waypoints as arena coordinates: send the drone's setup.yaml start "
             "offset. Default: waypoints are relative to the takeoff point.",
    )
    # Where setup.yaml is. Default: the setup.yaml in the same folder as this script.
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("setup.yaml")),
        help="Path to setup.yaml, used only with --map-frame (default: laptop/setup.yaml).",
    )
    # UDP port the laptop listens on for telemetry from the drone.
    parser.add_argument(
        "--telem-port",
        type=int,
        default=5005,
        help="Laptop telemetry UDP port (default: 5005).",
    )
    # UDP port on the drone that we send commands to.
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5006,
        help="ESP32 command UDP port (default: 5006).",
    )
    # A manual waypoint "x,y". action="append" means you can repeat it and each
    # one is added to the list args.waypoints. type=parse_waypoint converts the text.
    parser.add_argument(
        "--waypoint",
        dest="waypoints",
        action="append",
        type=parse_waypoint,
        default=[],
        help="Waypoint as x,y. Repeat this option for multiple waypoints.",
    )
    # Alternatively, load the waypoints from a text file.
    parser.add_argument(
        "--waypoints-file",
        help="Text file containing waypoints, one x,y entry per line (blank lines and # comments are allowed).",
    )
    # How long to wait after takeoff before sending the first waypoint
    # (gives the drone time to climb and settle).
    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=10.0,
        help="Seconds to wait after CMD_START before sending the first waypoint.",
    )
    # Max time allowed to reach EACH waypoint before we give up on it.
    parser.add_argument(
        "--arrival-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for each waypoint to be reached before continuing "
             "(default: 60; a 2 m lane takes ~20 s at 0.1 m/s plus the turn).",
    )
    # --- Lawnmower search area (used when no --waypoint / --waypoints-file) ---
    # The rectangle to sweep, in metres from the takeoff point.
    parser.add_argument("--x-min", type=float, default=0.0, help="Search area min x (m). Default: 0.")
    parser.add_argument("--x-max", type=float, default=1.5, help="Search area max x (m). Default: 1.5.")
    parser.add_argument("--y-min", type=float, default=0.0, help="Search area min y (m). Default: 0.")
    parser.add_argument("--y-max", type=float, default=1.0, help="Search area max y (m). Default: 1.0.")
    # Stay this far inside the rectangle's edges.
    parser.add_argument("--margin", type=float, default=0.2,
                        help="Keep waypoints this far inside the area edges (m). Default: 0.2.")
    # Largest allowed gap between neighbouring lanes.
    parser.add_argument("--lane-spacing", type=float, default=0.5,
                        help="Maximum distance between lanes (m). Default: 0.5.")
    # Whether the long straight lanes run along x or along y.
    parser.add_argument("--lanes-along", choices=["x", "y"], default="x",
                        help="Direction the lanes run. Default: x.")
    # Dry run: print the plan and quit, don't connect to or fly the drone.
    parser.add_argument("--plan-only", action="store_true",
                        help="Print the planned waypoints and exit without connecting.")
    # How close (m) the drone must get to a waypoint for it to count as reached.
    parser.add_argument(
        "--arrival-radius",
        type=float,
        default=0.25,
        help="Horizontal distance threshold (meters) to accept a waypoint as reached (default: 0.25).",
    )
    # How long to wait for the drone's first telemetry before giving up.
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the first telemetry packet (default: 30).",
    )
    # Safety: if we stop hearing from the drone for this long mid-flight, abort and land.
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=3.0,
        help="Land if telemetry is older than this during the mission (default: 3).",
    )
    # What to do at the end: land (default) or just hover.
    parser.add_argument(
        "--finish-action",
        choices=["hold", "land"],
        default="land",
        help="Action after final waypoint: hold current altitude/position or land (default: land).",
    )
    # Safety: make you type MISSION-<id> before the drone takes off.
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Require a confirmation prompt before sending any flight commands.",
    )
    # Print every telemetry packet as it arrives (noisy, useful for debugging).
    parser.add_argument(
        "--live-telem",
        action="store_true",
        help="Print each incoming telemetry update from the drone.",
    )
    # sys.argv[1:] = the words you typed after the script name. Fix up any
    # "--waypoint -1,0" first, then let argparse parse everything.
    return parser.parse_args(join_waypoint_values(sys.argv[1:]))


# ---------------------------------------------------------------------------
# Sending commands to the drone
# ---------------------------------------------------------------------------

def send_command(comms: CommsNode, drone_id: int, command_type: int, name: str,
                 repeat: int = 1) -> None:
    # Send a simple command (START / LAND / HOLD) "repeat" times.
    # "name" is just the text used in log/error messages.
    for _ in range(repeat):
        # comms.send_command returns False if we don't know the drone's IP address
        # yet (we learn it from its telemetry). Treat that as a fatal error.
        if not comms.send_command(drone_id, CommandPacket(command_type)):
            raise RuntimeError(f"Failed to send {name}: drone {drone_id} IP is unknown.")
        # When repeating (e.g. LAND x3, in case a UDP packet is lost),
        # leave 0.1 s between copies.
        if repeat > 1:
            time.sleep(0.1)
    log.info("%s sent to drone %d.", name, drone_id)


def send_goto(comms: CommsNode, drone_id: int, goal_x: float, goal_y: float) -> None:
    # Send one CMD_GOTO telling the drone to fly to (goal_x, goal_y).
    if not comms.send_command(drone_id, CommandPacket(CMD_GOTO, goal_x=goal_x, goal_y=goal_y)):
        raise RuntimeError(f"Failed to send CMD_GOTO: drone {drone_id} IP is unknown.")


def wait_for_arrival(
    tracker: TelemetryTracker,   # where we read the drone's latest position
    resend,                      # function to call to send the CMD_GOTO again
    timeout_s: float,            # give up on this waypoint after this long
    stale_timeout_s: float,      # abort if telemetry is older than this
    goal_x: float,               # where we want the drone to go
    goal_y: float,
    arrival_radius_m: float,     # "close enough" distance
) -> bool:
    # Watch the drone's telemetry until it reaches the goal.
    # Returns True if reached, False if timed out. Raises if telemetry goes stale.
    start = last_sent = time.monotonic()   # when we started waiting / last sent GOTO
    deadline = start + timeout_s           # when we give up
    saw_moving = False           # drone reported ROTATING/FLYING/STUCK

    # Loop until the deadline (each loop takes ~0.2 s because of the sleep at the end).
    while time.monotonic() < deadline:
        now = time.monotonic()           # current time
        snapshot = tracker.latest()      # copy of the newest telemetry
        packet = snapshot.packet         # the packet itself
        age = now - snapshot.received_at # how many seconds old it is
        # If we haven't heard from the drone for too long, the WiFi link may be
        # down: raise an error, which makes main() send a fallback LAND.
        if age > stale_timeout_s:
            raise ConnectionError(f"Telemetry is stale ({age:.1f} s).")

        # Only look at packets that arrived AFTER we sent this GOTO, so we don't
        # judge the new waypoint using an old position.
        if snapshot.received_at > start:
            # Straight-line distance from the drone to the goal (Pythagoras).
            dist_to_goal = math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y)

            # Print where the drone is now and how far it still has to go.
            log.info(
                "Current position: x=%.2f y=%.2f (goal x=%.2f y=%.2f, dist=%.2f m)",
                packet.ned_x,
                packet.ned_y,
                goal_x,
                goal_y,
                dist_to_goal,
            )

            # If the drone says it is turning, flying or stuck, it has clearly
            # received our GOTO, so we stop resending it.
            if packet.nav_state in (NAV_ROTATING, NAV_FLYING, NAV_STUCK):
                saw_moving = True

            # Close enough to the goal? Then this waypoint is done.
            if dist_to_goal <= arrival_radius_m:
                log.info(
                    "Waypoint reached at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
                    packet.ned_x,
                    packet.ned_y,
                    goal_x,
                    goal_y,
                    dist_to_goal,
                )
                return True

            # The drone hasn't started moving and it's been GOTO_RESEND_S seconds
            # since we last sent the GOTO: the packet was probably lost, so resend it.
            if not saw_moving and now - last_sent >= GOTO_RESEND_S:
                log.info("Drone not moving yet (nav=%s) — resending CMD_GOTO", packet.nav_state_name)
                resend()
                last_sent = now   # restart the 2-second resend timer

        # Wait a bit before checking again (5 checks per second).
        time.sleep(0.2)

    # We only get here if the while-loop ran out of time without reaching the goal.
    packet = tracker.latest().packet
    # Log where the drone ended up so you can see how far off it was.
    log.warning(
        "Waypoint arrival timeout; last nav_state=%s at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
        packet.nav_state_name,
        packet.ned_x,
        packet.ned_y,
        goal_x,
        goal_y,
        math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y),
    )
    return False


# ---------------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------------

def main() -> int:
    # Read all the command-line options.
    args = parse_args()

    # Sanity-check the timing options; stop with a message if any are nonsense.
    if (
        args.takeoff_wait < 0.0
        or args.arrival_timeout <= 0.0
        or args.arrival_radius <= 0.0
    ):
        raise SystemExit(
            "Invalid timing values. Use non-negative takeoff wait, positive arrival timeout, and positive arrival radius."
        )

    # Start with any --waypoint values typed on the command line...
    waypoints = list(args.waypoints)
    # ...and add any from --waypoints-file.
    if args.waypoints_file:
        waypoints.extend(load_waypoints_from_file(args.waypoints_file))

    if not waypoints:
        # No manual waypoints: generate the lawnmower sweep, then return home.
        try:
            waypoints = lawnmower(args.x_min, args.x_max, args.y_min, args.y_max,
                                  args.lane_spacing, args.margin, args.lanes_along)
        except ValueError as exc:
            # e.g. margin too big for the area: quit with a readable message.
            raise SystemExit(f"Bad search area: {exc}")
        # Last waypoint: fly back to the takeoff point before landing.
        waypoints.append((0.0, 0.0))

    # Print the full plan, numbered from 1.
    log.info("Mission plan: %d waypoints (last = return to start):", len(waypoints))
    for i, (wx, wy) in enumerate(waypoints, start=1):
        log.info("  %2d: x=%5.2f  y=%5.2f", i, wx, wy)
    # Estimate total distance and flight time (distance / speed; turns not included).
    length = path_length(waypoints)
    log.info("Path length %.1f m -> about %.0f s of flying at %.2f m/s (plus turns).",
             length, length / CRUISE_SPEED_MS, CRUISE_SPEED_MS)

    # --plan-only: we've printed the plan, so stop here without flying.
    if args.plan_only:
        return 0

    # Only use setup.yaml if --map-frame was given; otherwise config_path is None.
    config_path: Optional[str] = args.config if args.map_frame else None
    # With --map-frame, setup.yaml must actually exist.
    if config_path and not Path(config_path).exists():
        raise SystemExit(f"--map-frame needs {config_path}, which was not found.")

    # Create the telemetry tracker for our drone.
    tracker = TelemetryTracker(args.drone_id, print_live=args.live_telem)
    # Create the networking object (not started yet - that's comms.start() below).
    comms = CommsNode(
        listen_port=args.telem_port,   # port we receive telemetry on
        cmd_port=args.cmd_port,        # port we send commands to
        config_path=config_path,       # setup.yaml (or None)
    )
    if config_path is None:
        # Start offset (0, 0) so waypoints are relative to takeoff; this also
        # clears any offset a previous setup.yaml run left on the drone.
        comms.set_nav_tags([], {})   # no nav tags, no start offsets
        log.info("Waypoints are relative to the takeoff point (start offset 0, 0).")
    else:
        log.info("Waypoints are in the arena frame (start offset from %s).", config_path)
    # Tell CommsNode to call tracker.callback for every telemetry packet it receives.
    comms.on_telemetry(tracker.callback)

    # When Ctrl+C is pressed, raise KeyboardInterrupt so the "except" below
    # catches it and the "finally" block can send a fallback LAND.
    def handle_interrupt(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    # Install that handler for Ctrl+C (SIGINT).
    signal.signal(signal.SIGINT, handle_interrupt)

    # Flags used by the safety code in "finally":
    #   start_sent - have we told the drone to take off?
    #   land_sent  - have we already told it to land?
    #   aborted    - did the mission end because of an error or Ctrl+C?
    start_sent = land_sent = aborted = False
    # Open the UDP socket and start the background receive thread.
    comms.start()
    try:
        # Wait until the drone talks to us (this is also how we learn its IP address).
        log.info("Waiting for telemetry from drone %d...", args.drone_id)
        tracker.wait_for_first_packet(args.connect_timeout, args.telem_port, comms.known_drones)
        log.info("Telemetry link established.")

        # With --confirm, make the user type e.g. "MISSION-22" before takeoff.
        if args.confirm:
            confirmation = input(
                f"Type MISSION-{args.drone_id} to start the mission: "
            ).strip()
            # Anything else typed cancels the mission (nothing has been sent yet).
            if confirmation != f"MISSION-{args.drone_id}":
                log.info("Confirmation did not match. Mission cancelled.")
                return 0

        # Arm the motors and take off.
        log.info("Sending CMD_START (arm + takeoff)")
        send_command(comms, args.drone_id, CMD_START, "CMD_START")
        start_sent = True   # from now on, an abort must land the drone
        # Give the drone time to climb to cruise altitude.
        if args.takeoff_wait > 0.0:
            time.sleep(args.takeoff_wait)

        missed = 0   # how many waypoints timed out
        # Fly to each waypoint in turn; index counts 1, 2, 3, ...
        for index, (goal_x, goal_y) in enumerate(waypoints, start=1):
            log.info(
                "Sending waypoint %d/%d to (%.2f, %.2f)",
                index,
                len(waypoints),
                goal_x,
                goal_y,
            )

            # A small function that sends the GOTO for THIS waypoint.
            # The "gx: float = goal_x" default arguments freeze the current goal
            # values, so wait_for_arrival can call resend() with no arguments.
            def resend(gx: float = goal_x, gy: float = goal_y) -> None:
                send_goto(comms, args.drone_id, gx, gy)

            resend()   # send the GOTO for the first time
            # Wait until the drone gets there (True) or we time out (False).
            if not wait_for_arrival(
                tracker,
                resend,
                timeout_s=args.arrival_timeout,
                stale_timeout_s=args.stale_timeout,
                goal_x=goal_x,
                goal_y=goal_y,
                arrival_radius_m=args.arrival_radius,
            ):
                # Timed out: count it as missed, tell the drone to stop and hover,
                # then carry on with the next waypoint.
                missed += 1
                send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")   # stop before next leg
            # Short pause between waypoints so the drone can settle.
            time.sleep(0.5)

        # All waypoints done: land (default) or hover, depending on --finish-action.
        if args.finish_action == "land":
            log.info("Sending CMD_LAND")
            # Send LAND 3 times in case a UDP packet gets lost.
            send_command(comms, args.drone_id, CMD_LAND, "CMD_LAND", repeat=3)
            land_sent = True
        else:
            log.info("Sending CMD_HOLD")
            send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")

        # Exit code 1 if any waypoint was missed, 0 if everything was reached.
        if missed:
            log.warning("%d of %d waypoints not reached.", missed, len(waypoints))
            return 1
        return 0
    except KeyboardInterrupt:
        # Ctrl+C was pressed.
        aborted = True
        log.warning("Mission interrupted.")
        return 130   # 130 is the standard exit code for "stopped by Ctrl+C"
    except Exception as exc:
        # Any other error (lost telemetry, unknown IP, timeout waiting to connect, ...).
        aborted = True
        log.error("Mission aborted: %s", exc)
        return 1
    finally:
        # This block ALWAYS runs, whether the mission succeeded, failed or was interrupted.
        # Safety net: if we aborted after takeoff and haven't sent LAND yet, land now.
        if aborted and start_sent and not land_sent:
            try:
                send_command(comms, args.drone_id, CMD_LAND, "fallback CMD_LAND", repeat=3)
            except Exception as exc:
                log.error("Could not send fallback LAND: %s", exc)
        # Close the network socket and stop the background threads.
        comms.stop()


# This is only true when you run the file directly (python mission2.py), not when
# another file imports it. main()'s return value becomes the program's exit code.
if __name__ == "__main__":
    sys.exit(main())
