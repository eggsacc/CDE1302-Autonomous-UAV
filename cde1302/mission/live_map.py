"""
Live 2-D map of the mission: drone position, heading and trail, the planned
lawnmower path, and every AprilTag as soon as it is seen.

    python mission/mission.py --map --sim --auto      # try it with no hardware
    python mission/mission.py --map --dry-run         # real drone, bench test
    python mission/mission.py --map --auto            # real flight

The map window runs the mission: a matplotlib timer calls Mission.tick() at
flight.tick_hz (matplotlib must own the main thread) and redraws at
live_map.redraw_hz from MissionState.snapshot(), so the map never touches the
drone or the tag map directly.  Operator commands still go in the terminal.

Map view: East (y) to the right, North (x) up, metres, PX4 local NED, the
same as `planner.py --plot`.

    grey dashed box   search area        light red box   geofence
    grey dotted line  planned path       orange ring     next waypoint
    blue line + dot   drone trail/pose   blue arrow      heading
    black star        home (takeoff)
    hollow grey square  tag candidate (not yet confirmed)
    red square + label  confirmed tag: id and (x, y)

Camera panel (live_map.show_camera): the newest frame the tag pipeline
decoded, i.e. exactly what the detector saw, with a green outline + id per
accepted detection and a red outline + reason (gate, height, range,
yaw_rate, ...) per rejected one.  The title shows fps, dropped frames and
latency, or NO VIDEO when frames stop.  It reuses the mission's own stream,
so no second viewer is needed (and tag_stream.py must still not run).  In
--sim the image is rendered from the simulated pose (floor tags only).

Closing the window or pressing Ctrl+C in the terminal lands the drone (if it
took off) and ends the mission, exactly like Ctrl+C without the map.  When
the mission ends the window stays open, and the final map is saved as
map.png in the run's log folder (live_map.save_png).
"""

from __future__ import annotations

import logging
import math
import signal
from typing import Optional

from mission import Mission, Phase, geofence
from mission_state import StateSnapshot
from settings import Settings

log = logging.getLogger(__name__)

C_AREA, C_FENCE, C_PLAN, C_NEXT = "0.45", "#d62728", "0.6", "#ff7f0e"
C_DRONE, C_HOME, C_TAG, C_CAND = "#1f77b4", "black", "#d62728", "0.5"
C_SEEN, C_REJECT = "#2ca02c", "#d62728"       # camera panel outlines


class LiveMap:
    """The figure and its artists; update() redraws from one StateSnapshot."""

    def __init__(self, settings: Settings, waypoints: list[tuple[float, float]],
                 title: str = "Mission map") -> None:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch, Rectangle

        self.settings = settings
        self.waypoints = list(waypoints)
        show_cam = settings.live_map.show_camera
        self.fig = plt.figure(figsize=(13, 7) if show_cam else (9, 7))
        if show_cam:
            gs = self.fig.add_gridspec(2, 2, width_ratios=(3, 2), height_ratios=(3, 2))
            self.ax = self.fig.add_subplot(gs[:, 0])
            self.cam_ax = self.fig.add_subplot(gs[0, 1])
            legend_ax = self.fig.add_subplot(gs[1, 1])
        else:
            gs = self.fig.add_gridspec(1, 2, width_ratios=(3, 1))
            self.ax = self.fig.add_subplot(gs[0, 0])
            self.cam_ax = None
            legend_ax = self.fig.add_subplot(gs[0, 1])
        try:
            self.fig.canvas.manager.set_window_title(title)
        except AttributeError:              # headless backend
            pass
        ax = self.ax
        a = settings.area
        # Map view: plot (east, north) = (y, x).
        ax.add_patch(Rectangle((a.y_min, a.x_min), a.y_max - a.y_min, a.x_max - a.x_min,
                               fill=False, ls="--", lw=1.2, ec=C_AREA, label="search area"))
        self._fence = Rectangle((0, 0), 0, 0, fill=False, ls="-", lw=0.8, ec=C_FENCE,
                                alpha=0.35, label="geofence")
        ax.add_patch(self._fence)
        self._set_fence(None)

        if self.waypoints:
            ax.plot([w[1] for w in self.waypoints], [w[0] for w in self.waypoints],
                    ":o", color=C_PLAN, ms=3, lw=1, label="planned path")
            for i, (x, y) in enumerate(self.waypoints, 1):
                ax.annotate(str(i), (y, x), textcoords="offset points", xytext=(4, 4),
                            fontsize=7, color=C_PLAN)
        (self._next,) = ax.plot([], [], "o", ms=14, mfc="none", mec=C_NEXT, mew=2,
                                label="next waypoint")
        (self._trail,) = ax.plot([], [], "-", color=C_DRONE, lw=1.5, label="drone trail")
        (self._drone,) = ax.plot([], [], "o", color=C_DRONE, ms=8, zorder=6)
        self._arrow = ax.quiver([0.0], [0.0], [0.0], [0.0], angles="xy", scale_units="xy",
                                scale=1, color=C_DRONE, width=0.006, zorder=6)
        self._arrow.set_visible(False)
        (self._home,) = ax.plot([], [], "*", color=C_HOME, ms=15, zorder=5, label="home")
        self._cand = ax.scatter([], [], marker="s", s=60, facecolors="none",
                                edgecolors=C_CAND, zorder=4, label="tag candidate")
        self._conf = ax.scatter([], [], marker="s", s=90, color=C_TAG, zorder=7,
                                label="confirmed tag")
        self.tag_labels: dict[int, object] = {}

        ax.set_xlabel("East, y (m)")
        ax.set_ylabel("North, x (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        self._status = self.fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
        self._title = ax.set_title("CONNECTING")
        self._bounds = self._base_bounds(None)
        self._apply_bounds()

        handles, labels = ax.get_legend_handles_labels()
        if self.cam_ax is not None:
            self._init_camera()
            handles += [Patch(fill=False, ec=C_SEEN, lw=2), Patch(fill=False, ec=C_REJECT, lw=2)]
            labels += ["camera: tag accepted", "camera: tag rejected (reason)"]
        legend_ax.axis("off")
        legend_ax.legend(handles, labels, loc="upper left", fontsize=8,
                         ncol=2 if self.cam_ax is not None else 1, frameon=False)
        self.fig.tight_layout(rect=(0, 0.04, 1, 1))

    # -- camera panel -------------------------------------------------------

    def _init_camera(self) -> None:
        import numpy as np

        cam = self.cam_ax
        self._cam_img = cam.imshow(np.zeros((240, 320), np.uint8), cmap="gray", vmin=0,
                                   vmax=255, extent=(0, 320, 240, 0),
                                   interpolation="nearest")
        cam.set_xticks([])
        cam.set_yticks([])
        self._cam_title = cam.set_title("camera: waiting for video", fontsize=9)
        self._cam_artists: list = []
        self._cam_rate: Optional[tuple[float, int]] = None    # (t, frames) for fps
        self._cam_fps = 0.0

    def update_camera(self, view, now: float, stats=None) -> None:
        """Show the newest camera frame with an outline per detection.

        Args:
            view: tag_pipeline.CameraView or None (no frame yet).
            now: mission clock time, to tell a live stream from a stalled one.
            stats: tag_pipeline.PipelineStats, for fps and dropped frames.
        """
        from matplotlib.patches import Polygon

        if self.cam_ax is None:
            return
        for a in self._cam_artists:
            a.remove()
        self._cam_artists = []
        if view is None:
            self._cam_title.set_text("camera: waiting for video")
            self._cam_title.set_color("0.3")
            return

        h, w = view.gray.shape[:2]
        self._cam_img.set_data(view.gray)
        self._cam_img.set_extent((0, w, h, 0))       # AprilTag pixel convention:
        self.cam_ax.set_xlim(0, w)                   # pixel i spans [i, i+1)
        self.cam_ax.set_ylim(h, 0)
        for m in view.marks:
            color = C_SEEN if m.reason is None else C_REJECT
            poly = Polygon(m.corners, closed=True, fill=False, ec=color, lw=2)
            self.cam_ax.add_patch(poly)
            label = str(m.tag_id) if m.reason is None else f"{m.tag_id}: {m.reason}"
            tx = min(c[0] for c in m.corners)
            ty = min(c[1] for c in m.corners)
            text = self.cam_ax.text(tx, max(ty - 3, 8), label, color=color, fontsize=8,
                                    weight="bold", va="bottom",
                                    bbox=dict(boxstyle="round,pad=0.1", fc="white",
                                              ec="none", alpha=0.7))
            self._cam_artists += [poly, text]

        if stats is not None:
            if self._cam_rate is None:
                self._cam_rate = (now, stats.frames)
            elif now - self._cam_rate[0] >= 1.0:
                t0, f0 = self._cam_rate
                self._cam_fps = (stats.frames - f0) / (now - t0)
                self._cam_rate = (now, stats.frames)
        age = now - view.t_capture
        if age > self.settings.live_map.camera_stale_s:
            self._cam_title.set_text(f"camera: NO VIDEO (last frame {age:.1f} s ago)")
            self._cam_title.set_color(C_REJECT)
            return
        parts = [f"camera {self._cam_fps:4.1f} fps"]
        if stats is not None and stats.frames + stats.dropped > 0:
            parts.append(f"dropped {100 * stats.dropped / (stats.frames + stats.dropped):.0f}%")
        if view.age_ms is not None:
            parts.append(f"latency {view.age_ms} ms")
        parts.append(f"{len(view.marks)} tag(s) in view")
        self._cam_title.set_text("   ".join(parts))
        self._cam_title.set_color("black")

    # -- geometry helpers ---------------------------------------------------

    def _set_fence(self, home: Optional[tuple[float, float]]) -> None:
        x0, x1, y0, y1 = geofence(self.settings.area, home,
                                  self.settings.safety.geofence_margin_m)
        self._fence.set_bounds(y0, x0, y1 - y0, x1 - x0)

    def _base_bounds(self, home: Optional[tuple[float, float]]) -> list[float]:
        """[x_min, x_max, y_min, y_max] covering the geofence (area + home + margin)."""
        x0, x1, y0, y1 = geofence(self.settings.area, home,
                                  self.settings.safety.geofence_margin_m)
        return [x0, x1, y0, y1]

    def _grow(self, x: float, y: float) -> None:
        b = self._bounds
        b[0], b[1] = min(b[0], x), max(b[1], x)
        b[2], b[3] = min(b[2], y), max(b[3], y)

    def _apply_bounds(self) -> None:
        m = self.settings.live_map.view_margin_m
        x0, x1, y0, y1 = self._bounds
        self.ax.set_xlim(y0 - m, y1 + m)
        self.ax.set_ylim(x0 - m, x1 + m)

    # -- drawing ------------------------------------------------------------

    def update(self, snap: StateSnapshot) -> None:
        """Redraw everything from one snapshot (cheap: only set_data calls)."""
        before = list(self._bounds)
        if snap.home is not None:
            self._home.set_data([snap.home[1]], [snap.home[0]])
            self._set_fence(snap.home)
            fx0, fx1, fy0, fy1 = self._base_bounds(snap.home)
            self._grow(fx0, fy0)
            self._grow(fx1, fy1)

        if snap.trajectory:
            xs = [p[0] for p in snap.trajectory]
            ys = [p[1] for p in snap.trajectory]
            self._trail.set_data(ys, xs)
            self._grow(min(xs), min(ys))
            self._grow(max(xs), max(ys))

        p = snap.pose
        if p is not None:
            self._drone.set_data([p.y], [p.x])
            L = self.settings.live_map.arrow_m
            # NED forward (cos h, sin h) -> map (east, north) = (sin h, cos h)
            self._arrow.set_offsets([[p.y, p.x]])
            self._arrow.set_UVC([L * math.sin(p.heading)], [L * math.cos(p.heading)])
            self._arrow.set_visible(True)

        exploring = snap.phase in (Phase.EXPLORING.value,)
        if exploring and snap.wp_index < len(snap.waypoints):
            nx, ny = snap.waypoints[snap.wp_index]
            self._next.set_data([ny], [nx])
        elif snap.phase == Phase.RETURNING.value and snap.home is not None:
            self._next.set_data([snap.home[1]], [snap.home[0]])
        else:
            self._next.set_data([], [])

        self._draw_tags(snap)
        if self._bounds != before:
            self._apply_bounds()

        self._title.set_text(self._title_text(snap))
        self._status.set_text(self._status_text(snap))

    def _draw_tags(self, snap: StateSnapshot) -> None:
        cand = [t for t in snap.tags if not t.confirmed]
        conf = [t for t in snap.tags if t.confirmed]
        self._cand.set_offsets([[t.y, t.x] for t in cand] if cand else [[math.nan, math.nan]])
        self._conf.set_offsets([[t.y, t.x] for t in conf] if conf else [[math.nan, math.nan]])
        for t in conf:
            self._grow(t.x, t.y)
            text = f" {t.tag_id}\n ({t.x:.2f}, {t.y:.2f})"
            label = self.tag_labels.get(t.tag_id)
            if label is None:
                self.tag_labels[t.tag_id] = self.ax.annotate(
                    text, (t.y, t.x), textcoords="offset points", xytext=(7, -4),
                    fontsize=8, color=C_TAG, weight="bold", zorder=8,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
            else:
                label.xy = (t.y, t.x)
                label.set_text(text)

    def _title_text(self, snap: StateSnapshot) -> str:
        n = len(snap.confirmed_tags)
        expected = self.settings.tags.expected_count
        found = f"{n}/{expected}" if expected else str(n)
        t = "" if snap.mission_s is None else f"   T+{snap.mission_s:.1f} s"
        dry = "  [DRY RUN]" if snap.dry_run else ""
        return f"{snap.phase}{dry}{t}   tags {found}"

    def _status_text(self, snap: StateSnapshot) -> str:
        p = snap.pose
        pose = (f"x {p.x:+.2f}  y {p.y:+.2f}  hdg {math.degrees(p.heading):+4.0f} deg  "
                f"{p.nav_state_name}" if p else "no telemetry yet")
        wp = f"   wp {min(snap.wp_index + 1, len(snap.waypoints))}/{len(snap.waypoints)}"
        msg = f"   {snap.message}" if snap.message else ""
        return pose + wp + msg

    def draw(self) -> None:
        self.fig.canvas.draw_idle()

    def save(self, path) -> None:
        self.fig.savefig(path, dpi=130, bbox_inches="tight")
        log.info("Map saved to %s", path)


class MapRunner:
    """Drives Mission.tick() from the figure's timer and keeps the map current."""

    def __init__(self, mission: Mission, view: LiveMap, period_s: float) -> None:
        self.mission = mission
        self.view = view
        self.period_s = period_s
        self.redraw_every = max(1, round(1.0 / (mission.settings.live_map.redraw_hz * period_s)))
        self.finished = False
        self._ticks = 0
        self._stop_reason: Optional[tuple[str, int]] = None
        self.timer = None

    def request_stop(self, reason: str, exit_code: int) -> None:
        """Ask for land-and-exit on the next timer tick (safe from a signal handler)."""
        self._stop_reason = (reason, exit_code)

    def on_timer(self) -> None:
        if self.finished:
            return
        alive = True
        try:
            if self._stop_reason is not None:
                reason, code = self._stop_reason
                log.warning("%s", reason)
                self.mission.abort(reason, exit_code=code)
                self.mission.exit_code = code
                alive = False
            else:
                alive = self.mission.tick()
        except Exception as exc:
            log.exception("mission error")
            self.mission.abort(f"error: {exc}", exit_code=1)
            self.mission.exit_code = 1
            alive = False
        self._ticks += 1
        if not alive or self._ticks % self.redraw_every == 0:
            self.redraw()
        if not alive:
            self.finish()

    def redraw(self) -> None:
        """Map + camera panel.  A drawing error is logged, never stops the mission."""
        try:
            self.view.update(self.mission.state.snapshot())
            src = self.mission.source
            self.view.update_camera(src.latest_view(), self.mission.drone.now(), src.stats())
            self.view.draw()
        except Exception:
            log.exception("live map redraw failed")

    def on_close(self, _event=None) -> None:
        """Window closed: same as Ctrl+C (lands if airborne)."""
        if not self.finished:
            self.mission.abort("map window closed", exit_code=130)
            self.mission.exit_code = 130
            self.finish()

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        if self.timer is not None:
            self.timer.stop()
        self.redraw()
        try:
            if self.mission.settings.live_map.save_png:
                self.view.save(self.mission.log.folder / "map.png")
        except Exception:
            log.exception("could not save the final map")
        self.mission.shutdown()
        log.info("Mission finished (exit code %d). Close the map window to exit.",
                 self.mission.exit_code)


def run_with_map(mission: Mission, period_s: float) -> int:
    """Run the mission inside a live map window.  Returns the exit code."""
    import matplotlib.pyplot as plt

    view = LiveMap(mission.settings, mission.waypoints,
                   title=f"Drone {mission.settings.drone_id} - {mission.log.folder.name}")
    runner = MapRunner(mission, view, period_s)
    try:
        mission.start()
    except Exception as exc:
        log.exception("could not start")
        mission.abort(f"start failed: {exc}", exit_code=1)
        mission.exit_code = 1
        mission.shutdown()
        plt.close(view.fig)
        return 1

    # GUI event loops swallow KeyboardInterrupt, so Ctrl+C sets a flag that the
    # next timer tick (<= one period later) acts on.
    previous = signal.signal(signal.SIGINT, lambda *_: runner.request_stop("Ctrl+C", 130))
    view.fig.canvas.mpl_connect("close_event", runner.on_close)
    runner.timer = view.fig.canvas.new_timer(interval=max(1, int(period_s * 1000)))
    runner.timer.add_callback(runner.on_timer)
    runner.timer.start()
    try:
        plt.show()                       # returns when the window is closed
    finally:
        signal.signal(signal.SIGINT, previous)
        runner.on_close()                # no-op if the mission already finished
    return mission.exit_code
