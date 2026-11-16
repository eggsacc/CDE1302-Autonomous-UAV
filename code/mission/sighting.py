"""
Sighting: one accepted tag detection, already placed in PX4 local NED.

Produced by tag_pipeline.py (real camera) and sim.py (fake camera), consumed
by tag_map.py and written to detections.csv by mission_log.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from drone_link import Pose


@dataclass(frozen=True)
class Sighting:
    """One tag seen in one frame.

    Times are monotonic seconds on the mission clock.
    """
    tag_id: int
    t_capture: float                       # estimated frame capture time
    t_detect: float                        # when the laptop finished detecting it
    x: float                               # tag position, PX4 local NED north (m)
    y: float                               # tag position, PX4 local NED east (m)
    body: tuple[float, float, float]       # tag in body frame: forward, right, down (m)
    drone: Pose                            # drone pose used (interpolated at t_capture)
    cam_t: Optional[tuple[float, float, float]] = None   # camera-frame translation (m)
    cam_R: Optional[tuple[float, ...]] = None            # camera-frame rotation, 9 row-major
    pose_err: Optional[float] = None       # AprilTag pose reprojection error
    decision_margin: Optional[float] = None

    def log_fields(self) -> dict[str, Any]:
        """Columns for MissionLog.detection() (all but the time columns)."""
        row: dict[str, Any] = {
            "tag_id": self.tag_id,
            "body_f": self.body[0], "body_r": self.body[1], "body_d": self.body[2],
            "pose_err": self.pose_err, "decision_margin": self.decision_margin,
            "drone_x": self.drone.x, "drone_y": self.drone.y,
            "drone_heading_rad": self.drone.heading,
            "tag_x": self.x, "tag_y": self.y,
        }
        if self.cam_t is not None:
            row.update(cam_tx=self.cam_t[0], cam_ty=self.cam_t[1], cam_tz=self.cam_t[2])
        if self.cam_R is not None:
            row.update({f"R{i}{j}": self.cam_R[3 * i + j]
                        for i in range(3) for j in range(3)})
        return row
