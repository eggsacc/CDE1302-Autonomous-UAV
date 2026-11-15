"""
Pure geometry helpers, PX4 local NED convention.

Headings (yaw) are radians, 0 = North, clockwise (towards East) positive,
normalised to [-pi, pi).  No I/O and no state, so everything here is easy to
test by hand.
"""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi) radians."""
    return (a + math.pi) % TWO_PI - math.pi


def angle_diff(a: float, b: float) -> float:
    """Shortest signed rotation from b to a, in [-pi, pi) radians."""
    return wrap_angle(a - b)


def lerp_angle(a: float, b: float, frac: float) -> float:
    """Interpolate from heading a to heading b the short way round.

    frac = 0 gives a, frac = 1 gives b; e.g. 170 deg -> -170 deg passes
    through 180 deg, not through 0.
    """
    return wrap_angle(a + angle_diff(b, a) * frac)


def body_to_ned(forward: float, right: float, x: float, y: float,
                heading: float) -> tuple[float, float]:
    """Horizontal NED position of a point seen in the drone body frame.

    Args:
        forward, right: point in the body frame (m), e.g. Detection.t_body[:2].
            Body frame: x forward, y right, z down (the 'down' part does not
            change the horizontal position, so it is not needed).
        x, y: drone position, PX4 local NED (m).
        heading: drone yaw (rad), 0 = North, clockwise positive.
    Returns:
        (north, east) of the point, PX4 local NED (m):
            north = x + forward*cos(heading) - right*sin(heading)
            east  = y + forward*sin(heading) + right*cos(heading)

    Example: drone at (1, 0) facing East (heading pi/2) sees a tag 0.5 m
    ahead -> (1.0, 0.5).
    """
    c, s = math.cos(heading), math.sin(heading)
    return x + forward * c - right * s, y + forward * s + right * c


def ned_to_body(north: float, east: float, x: float, y: float,
                heading: float) -> tuple[float, float]:
    """Inverse of body_to_ned: (forward, right) of an NED point from the drone."""
    dn, de = north - x, east - y
    c, s = math.cos(heading), math.sin(heading)
    return dn * c + de * s, -dn * s + de * c
