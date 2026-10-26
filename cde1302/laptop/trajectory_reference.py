"""Offline trajectory maths shared by generation, upload, and analysis.

Coordinates: local NED metres; yaw = atan2(East velocity, North velocity),
clockwise from North, radians. Yaw is unwrapped before time interpolation.
"""
from dataclasses import dataclass
from pathlib import Path
import csv
import math

import numpy as np


STATIONARY_SPEED_MPS = 1e-4


@dataclass
class Reference:
    t: np.ndarray
    xyz: np.ndarray
    yaw: np.ndarray | None = None


def read_reference(path, yaw_mode="fixed") -> Reference:
    """Do not silently discard malformed/NaN rows (including the last row)."""
    rows = []
    header_seen = False
    expected = 5 if yaw_mode == "tangent" else 4
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        for line_no, line in enumerate(stream, 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            fields = next(csv.reader([line]))
            names = [c.strip() for c in fields]
            if not rows and not header_seen and names[0] == "t":
                if names != ["t", "x", "y", "z"] + (["yaw_rad"] if expected == 5 else []):
                    raise ValueError(f"{path}:{line_no}: expected " +
                                     ("t,x,y,z,yaw_rad" if expected == 5 else "t,x,y,z"))
                header_seen = True
                continue
            if len(fields) != expected:
                raise ValueError(f"{path}:{line_no}: expected exactly {expected} columns")
            try:
                row = [float(c) for c in fields]
            except ValueError as exc:
                raise ValueError(f"{path}:{line_no}: non-numeric sample") from exc
            if not all(math.isfinite(v) for v in row):
                raise ValueError(f"{path}:{line_no}: all samples must be finite")
            rows.append(row)
    a = np.asarray(rows, dtype=float)
    if len(a) < 2:
        raise ValueError(f"{path}: need at least two samples")
    if np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError(f"{path}: times must be strictly increasing")
    return Reference(a[:, 0], a[:, 1:4],
                     np.unwrap(a[:, 4]) if expected == 5 else None)


def tangent_yaw(t, xyz, speed_epsilon=STATIONARY_SPEED_MPS):
    """Finite-difference tangent of the actual sampled path, not an ideal ellipse.

    A sample is direction-defined only when horizontal speed > epsilon.
    Interpolate unwrapped headings across undefined intervals; hold the first/
    last defined heading outside that range. No automatic initial rotation.
    Raising epsilon is a change in yaw policy and requires explicit TA review.
    """
    if not math.isfinite(speed_epsilon) or speed_epsilon <= 0:
        raise ValueError("stationary speed threshold must be finite and positive")
    # One-sided endpoint differences agree with MATLAB gradient and give zero
    # at explicitly repeated stationary endpoint samples.
    v = np.gradient(xyz, t, axis=0, edge_order=1)
    defined = np.linalg.norm(v[:, :2], axis=1) > speed_epsilon
    if not np.any(defined):
        raise ValueError("horizontal tangent is undefined: entire path is stationary")
    angles = np.unwrap(np.arctan2(v[defined, 1], v[defined, 0]))
    return np.interp(t, t[defined], angles), defined


def sample_reference(ref, dt, preserve_duration=False):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("sample period must be finite and positive")
    duration = ref.t[-1] - ref.t[0]
    if preserve_duration and abs(duration / dt - round(duration / dt)) > 1e-6:
        raise ValueError("duration must be an integer number of upload periods; timing is never shortened")
    tq = ref.t[0] + np.arange(0, duration + 1e-9, dt)
    xyz = np.column_stack([np.interp(tq, ref.t, ref.xyz[:, c]) for c in range(3)])
    yaw = None if ref.yaw is None else np.interp(tq, ref.t, ref.yaw)
    return Reference(tq - tq[0], xyz - xyz[0], yaw)


def rates(ref):
    dt = np.diff(ref.t)
    speed = np.linalg.norm(np.diff(ref.xyz, axis=0), axis=1) / dt
    yaw_rate = None if ref.yaw is None else np.diff(ref.yaw) / dt
    return speed, yaw_rate


def validate_reference(ref, max_speed, max_yaw_rate=None,
                       speed_epsilon=STATIONARY_SPEED_MPS, check_tangent=True):
    if not math.isfinite(max_speed) or max_speed <= 0:
        raise ValueError("--max-speed must be finite and positive")
    speed, yaw_rate = rates(ref)
    if np.max(speed) > max_speed + 1e-6:
        raise ValueError(f"max speed {np.max(speed):.6f} m/s exceeds {max_speed:.6f} m/s")
    if ref.yaw is not None:
        if max_yaw_rate is None or not math.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("tangent mode needs a finite positive --max-yaw-rate (rad/s)")
        if check_tangent:
            expected, defined = tangent_yaw(ref.t, ref.xyz, speed_epsilon)
            error = np.arctan2(np.sin(ref.yaw - expected), np.cos(ref.yaw - expected))
            if np.max(np.abs(error[defined])) > 0.035:
                raise ValueError("yaw column does not follow the sampled horizontal tangent within 0.035 rad")
            # Undefined samples must also obey the stated interpolation/endpoint policy.
            if np.max(np.abs(error[~defined]), initial=0.0) > 0.035:
                raise ValueError("stationary yaw samples violate the documented heading policy")
        if np.max(np.abs(yaw_rate)) > max_yaw_rate + 1e-6:
            raise ValueError(f"max yaw rate {np.max(np.abs(yaw_rate)):.6f} rad/s exceeds "
                             f"{max_yaw_rate:.6f} rad/s; positions/timing have NOT been changed")


def write_tangent_csv(source, destination, speed_epsilon=STATIONARY_SPEED_MPS):
    """Append yaw to each original row; preserve its first four fields verbatim."""
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError("output must be a separate file; original CSV is read-only")
    ref = read_reference(source)
    yaw, defined = tangent_yaw(ref.t, ref.xyz, speed_epsilon)
    out = [f"# stationary_speed_mps={speed_epsilon:.17g}; unwrapped NED yaw in radians\n",
           "t,x,y,z,yaw_rad\n"]
    i = 0
    for line in source.read_text(encoding="utf-8-sig").splitlines():
        if not line.strip() or line.lstrip().startswith("#") or line.split(",")[0].strip() == "t":
            continue
        out.append(line + f",{yaw[i]:.17g}\n")
        i += 1
    destination.write_text("".join(out), encoding="utf-8", newline="")
    return Reference(ref.t, ref.xyz, yaw), defined
