"""
How the camera sits on the drone: camera frame <-> body frame.

laptop/apriltag_host.py hard-codes a 45 deg tilt in Detection.t_body.  That
file is the course's, so the mission keeps the tilt in config.yaml
(tags.camera_pitch_deg) and converts positions here instead.

Body frame: x forward, y right, z down.  Camera frame: x image right, y image
down, z out of the lens.  pitch_deg is how far the lens looks below the
horizon: 0 = straight ahead, 90 = straight down.  The camera tilts about the
drone's right axis, so the top of the image always points forward.
"""

from __future__ import annotations

import math

import numpy as np

import laptop_path  # noqa: F401  (puts laptop/ on sys.path)
from apriltag_host import CAM_POS_M

DEFAULT_PITCH_DEG = 90.0


def r_body_cam(pitch_deg: float) -> np.ndarray:
    """3x3 rotation; columns are the camera's x, y, z axes in the body frame."""
    p = math.radians(pitch_deg)
    return np.array([[0.0, -math.sin(p), math.cos(p)],
                     [1.0, 0.0, 0.0],
                     [0.0, math.cos(p), math.sin(p)]])


def camera_to_body(t, pitch_deg: float) -> tuple[float, float, float]:
    """A camera-frame point as (forward, right, down) from the drone centre (m)."""
    return tuple(float(v) for v in r_body_cam(pitch_deg) @ np.asarray(t) + np.asarray(CAM_POS_M))


def body_to_camera(body, pitch_deg: float) -> tuple[float, float, float]:
    """The inverse of camera_to_body."""
    return tuple(float(v) for v in r_body_cam(pitch_deg).T
                 @ (np.asarray(body) - np.asarray(CAM_POS_M)))
