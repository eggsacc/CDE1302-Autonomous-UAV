#!/usr/bin/env python3
"""
Analyse the three mission-demonstration flights for the final report.

Reads   report/data/flights/<run>/   (copies of cde1302/mission/logs/<run>/)
Writes  report/figures/*.png          (figures used by build_report.py)
        report/data/metrics.json      (every number quoted in the report)

Run from the repo root:
    python report/analyse_flights.py

Everything here is computed from the laptop-side mission logs, i.e. from PX4's
own position estimate as relayed over Wi-Fi at ~10 Hz.  It is NOT ground truth:
the only external reference is the set of localisation tags at surveyed spots.
The PX4 .ulg black-box analysis is a separate script (analyse_ulog.py).
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "flights"
FIG = ROOT / "figures"
FIG.mkdir(exist_ok=True)

# Run order, display names and the PX4 horizontal speed cap (MPC_XY_VEL_MAX).
# Only two of the three demonstration flights are reported. The user dropped
# truncated-exploration-1 (unconfirmed cap, "just a slower version" of the last
# flight); its log copy stays in data/flights/ but is not analysed.
# truncated-exploration-2 is called "Run 2" in the report.
RUNS = [
    {"id": "full-exploration", "name": "Run 1: full sweep", "short": "Run 1",
     "cap": 0.4, "cap_confirmed": True},  # MPC_XY_VEL_MAX from the ULog (sess217)
    {"id": "truncated-exploration-2", "name": "Run 2: truncated sweep", "short": "Run 2",
     "cap": 1.5, "cap_confirmed": True},
]

# Validated categorical palette (dataviz skill reference palette, light mode),
# fixed order: run 1, run 2, run 3.  Phases use the same fixed order.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#d9d8d4"
MUTED = "#8a8984"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10,
    "axes.edgecolor": MUTED,
    "axes.labelcolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "legend.frameon": False,
    "figure.dpi": 100,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
    "lines.linewidth": 2.0,
})

ALIGN_RE = re.compile(r"plan turned ([+-]?[\d.]+) deg and moved onto home tag \d+ at "
                      r"\(([-\d.]+), ([-\d.]+)\)")
WP_RE = re.compile(r"(\d+)/(\d+) at \(([-\d.]+), ([-\d.]+)\)")
LOCFIT_RE = re.compile(r"turn ([+-][\d.]+) deg, scale ([\d.]+), shift \(([+-][\d.]+), "
                       r"([+-][\d.]+)\) m, rms ([\d.]+) m over (\d+) fix")
LOO_RE = re.compile(r"global ([\d.]+), time ([\d.]+), hybrid ([\d.]+)")


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load(run_id: str) -> dict:
    d = DATA / run_id
    cfg = yaml.safe_load((d / "config_used.yaml").read_text(encoding="utf-8"))
    ev = pd.read_csv(d / "events.csv")
    tr = pd.read_csv(d / "trajectory.csv")
    tr = tr.dropna(subset=["mission_s"]).drop_duplicates("mission_s").sort_values("mission_s")
    return {
        "cfg": cfg,
        "events": ev,
        "traj": tr.reset_index(drop=True),
        "traj_corr": pd.read_csv(d / "trajectory_corrected.csv"),
        "det": pd.read_csv(d / "detections.csv"),
        "tags": pd.read_csv(d / "tags_final.csv"),
        "tags_corr": pd.read_csv(d / "tags_corrected.csv"),
        "loc": pd.read_csv(d / "localisation.csv"),
    }


def ev_rows(ev: pd.DataFrame, name: str) -> pd.DataFrame:
    return ev[ev.event == name]


def ev_first(ev: pd.DataFrame, name: str, contains: str = "") -> pd.Series:
    rows = ev[(ev.event == name) & ev.details.fillna("").str.contains(contains, regex=False)]
    return rows.iloc[0]


# ---------------------------------------------------------------------------
# Plan reconstruction (same maths as planner.py + mission._align_plan_to_home_tag)
# ---------------------------------------------------------------------------

def lane_positions(lo, hi, spacing):
    n = math.ceil((hi - lo) / spacing)
    step = (hi - lo) / n
    return [lo + i * step for i in range(n + 1)]


def arena_plan(cfg: dict) -> list[tuple[float, float]]:
    a, s = cfg["area"], cfg["sweep"]
    m = a.get("margin", 0.0)
    x0, x1 = a["x_min"] + m, a["x_max"] - m
    wps = []
    for i, y in enumerate(lane_positions(a["y_min"] + m, a["y_max"] - m, s["lane_spacing"])):
        ends = [(x0, y), (x1, y)]
        wps += ends if i % 2 == 0 else ends[::-1]
    return wps


def px4_plan(cfg: dict, ev: pd.DataFrame):
    m = ALIGN_RE.search(ev_first(ev, "ALIGNED").details)
    theta, px, py = math.radians(float(m.group(1))), float(m.group(2)), float(m.group(3))
    c, s = math.cos(theta), math.sin(theta)
    wps = [(px + c * x - s * y, py + s * x + c * y) for x, y in arena_plan(cfg)]
    return wps, math.degrees(theta), (px, py)


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------

def speed_series(tr: pd.DataFrame, dt: float = 0.1, half_window: float = 0.2):
    """Ground speed from the 10 Hz position telemetry: resample to a uniform
    grid, then central difference over +-half_window (s)."""
    t = tr.mission_s.to_numpy()
    tg = np.arange(t[0], t[-1], dt)
    x = np.interp(tg, t, tr.x.to_numpy())
    y = np.interp(tg, t, tr.y.to_numpy())
    k = max(1, int(round(half_window / dt)))
    v = np.full_like(tg, np.nan)
    v[k:-k] = np.hypot(x[2 * k:] - x[:-2 * k], y[2 * k:] - y[:-2 * k]) / (2 * k * dt)
    return tg, v


def segment(tr, t0, t1):
    return tr[(tr.mission_s >= t0) & (tr.mission_s <= t1)]


def lane_metrics(tr, wps, passes, t_end):
    """Cross-track error on the long lanes and overshoot past each lane end.

    passes[i] = mission time waypoint i (0-based) was passed (laptop: within
    arrival_radius_m).  Long lanes are wp 0->1, 2->3, ...
    """
    out = []
    for i in range(0, len(wps) - 1, 2):
        a, b = np.array(wps[i]), np.array(wps[i + 1])
        u = (b - a) / np.linalg.norm(b - a)
        n = np.array([-u[1], u[0]])
        t0 = passes.get(i)
        t1 = passes.get(i + 1)
        if t0 is None or t1 is None:
            continue
        seg = segment(tr, t0, t1)
        p = seg[["x", "y"]].to_numpy() - a
        along = p @ u
        L = float(np.linalg.norm(b - a))
        # ignore the first/last 0.3 m where the drone is still turning in/out
        keep = (along > 0.3) & (along < L - 0.3)
        xt = p[keep] @ n
        # overshoot: from passing the lane end until the next waypoint is passed
        t2 = passes.get(i + 2, t_end)
        seg2 = segment(tr, t1, t2)
        over = float(np.max((seg2[["x", "y"]].to_numpy() - b) @ u)) if len(seg2) else float("nan")
        out.append({
            "lane": i // 2 + 1,
            "length_m": L,
            "duration_s": t1 - t0,
            "xt_rms_m": float(np.sqrt(np.mean(xt ** 2))) if len(xt) else float("nan"),
            "xt_max_m": float(np.max(np.abs(xt))) if len(xt) else float("nan"),
            "xt_mean_m": float(np.mean(xt)) if len(xt) else float("nan"),
            "overshoot_m": max(over, 0.0),
        })
    return out


# ---------------------------------------------------------------------------
# Per-run analysis
# ---------------------------------------------------------------------------

def analyse(run: dict) -> dict:
    D = load(run["id"])
    ev, tr, cfg = D["events"], D["traj"], D["cfg"]
    wps, theta_deg, home_tag_px4 = px4_plan(cfg, ev)

    phase = ev_rows(ev, "PHASE").dropna(subset=["mission_s"])

    def phase_t(target):
        r = phase[phase.details.str.contains(f"-> {target}", regex=False)]
        return float(r.mission_s.iloc[0]) if len(r) else float("nan")

    t_hover = phase_t("HOVERING")
    t_explore = phase_t("EXPLORING")
    t_return = phase_t("RETURNING")
    t_landing = phase_t("LANDING")
    t_done = phase_t("DONE")
    t_sweep_start = float(ev_first(ev, "TRAJ_STARTED", "sweep").mission_s)
    t_sweep_end = float(ev_first(ev, "TRAJ_END", "sweep").mission_s)
    t_home = float(ev_first(ev, "TRAJ_END", "return home").mission_s)
    t_hover_confirmed = float(ev_first(ev, "HOVER_CONFIRMED").mission_s)
    # Mission time as the team defines it: take-off command -> land command
    # (not touchdown, and not the logger's MISSION_COMPLETE, which adds 6 s).
    t_land_cmd = float(ev_first(ev, "LAND_CMD").mission_s)

    passes = {}
    for _, r in ev_rows(ev, "WAYPOINT_PASSED").iterrows():
        m = WP_RE.search(r.details)
        passes[int(m.group(1)) - 1] = float(r.mission_s)

    # Speeds
    tg, v = speed_series(tr)
    sweep = (tg >= t_sweep_start) & (tg <= t_sweep_end)
    vs = v[sweep]
    vs = vs[~np.isnan(vs)]
    sweep_tr = segment(tr, t_sweep_start, t_sweep_end)
    sweep_len = float(np.sum(np.hypot(np.diff(sweep_tr.x), np.diff(sweep_tr.y))))
    plan_len = float(sum(math.dist(p, q) for p, q in zip(wps, wps[1:])))
    air = tr[tr.mission_s >= t_hover]
    total_len = float(np.sum(np.hypot(np.diff(air.x), np.diff(air.y))))

    # Cruise speed: median speed on the middle 60 % of each long lane
    lanes = lane_metrics(tr, wps, passes, t_sweep_end)
    cruise = []
    for i in range(0, len(wps) - 1, 2):
        if i in passes and i + 1 in passes:
            t0, t1 = passes[i], passes[i + 1]
            a = t0 + 0.2 * (t1 - t0)
            b = t1 - 0.2 * (t1 - t0)
            m = (tg >= a) & (tg <= b)
            cruise.extend(v[m][~np.isnan(v[m])])

    # Tags
    tags = D["tags"].merge(D["tags_corr"][["tag_id", "x", "y", "shift_m"]],
                           on="tag_id", suffixes=("", "_corr"))
    tag_rows = []
    for _, t in tags.iterrows():
        det = D["det"][D["det"].tag_id == t.tag_id]
        ts = det.capture_mission_s
        m = (tg >= ts.min()) & (tg <= ts.max())
        vv = v[m][~np.isnan(v[m])]
        tag_rows.append({
            "tag_id": int(t.tag_id),
            "raw_x": float(t.x), "raw_y": float(t.y),
            "corr_x": float(t.x_corr), "corr_y": float(t.y_corr),
            "shift_m": float(t.shift_m),
            "n_sightings": int(t.n_sightings),
            "first_seen_s": float(t.first_seen_mission_s),
            "confirmed_s": float(t.confirmed_mission_s),
            "confirm_delay_s": float(t.confirmed_mission_s - t.first_seen_mission_s),
            "seen_for_s": float(ts.max() - ts.min()),
            "speed_while_seen_mps": float(np.mean(vv)) if len(vv) else float("nan"),
            "decision_margin_median": float(det.decision_margin.median()),
            "range_median_m": float(np.median(np.sqrt(det.body_f ** 2 + det.body_r ** 2
                                                      + det.body_d ** 2))),
        })

    # Localisation tags
    loc = D["loc"].copy()
    loc["raw_err_m"] = np.hypot(loc.px4_x - loc.ref_x, loc.px4_y - loc.ref_y)
    fit_m = LOCFIT_RE.search(ev_first(ev, "LOC_FIT").details)
    loo_m = LOO_RE.search(ev_first(ev, "LOC_DRIFT").details)
    loc_ids_seen = sorted(set(int(i) for i in loc.tag_id))
    loc_ids_cfg = [int(t["id"]) for t in cfg["localisation"]["tags"]]
    det_counts = D["det"].tag_id.value_counts().to_dict()

    # Precision landing
    lt = ev_rows(ev, "LAND_TARGET")
    land_cmd = ev_first(ev, "LAND_CMD").details
    m = re.search(r"\((\d+) cm from the home tag\)|, (\d+) cm from the home tag", land_cmd)
    final_cm = float(m.group(1) or m.group(2)) if m else float("nan")
    corr_attempts = len(ev[(ev.event == "TRAJECTORY")
                           & ev.details.str.contains("land correction", regex=False)])
    first_lt = re.search(r"seen (\d+) cm from", lt.iloc[0].details) if len(lt) else None

    return {
        "run": run,
        "D": D,
        "wps": wps,
        "metrics": {
            "id": run["id"], "name": run["name"],
            "mpc_xy_vel_max": run["cap"], "cap_confirmed": run["cap_confirmed"],
            "lane_spacing_m": cfg["sweep"]["lane_spacing"],
            "n_lanes": len(wps) // 2, "n_waypoints": len(wps),
            "align_deg": theta_deg,
            "home_tag_px4": home_tag_px4,
            "t_hover_s": t_hover, "t_hover_confirmed_s": t_hover_confirmed,
            "t_explore_s": t_explore,
            "t_sweep_start_s": t_sweep_start, "t_sweep_end_s": t_sweep_end,
            "t_return_s": t_return, "t_home_s": t_home,
            "t_landing_s": t_landing, "t_done_s": t_done,
            "t_land_cmd_s": t_land_cmd, "mission_time_s": t_land_cmd,
            "sweep_duration_s": t_sweep_end - t_sweep_start,
            "return_duration_s": t_home - t_return,
            "settle_duration_s": t_landing - t_home,
            "sweep_path_m": sweep_len, "plan_path_m": plan_len,
            "flight_path_m": total_len,
            "speed_mean_mps": sweep_len / (t_sweep_end - t_sweep_start),
            "speed_cruise_median_mps": float(np.median(cruise)) if cruise else float("nan"),
            "speed_p95_mps": float(np.percentile(vs, 95)),
            "speed_max_mps": float(np.max(vs)),
            "heading_min_deg": float(np.degrees(tr.heading_rad.min())),
            "heading_max_deg": float(np.degrees(tr.heading_rad.max())),
            "sweep_heading_min_deg": float(np.degrees(sweep_tr.heading_rad.min())),
            "sweep_heading_max_deg": float(np.degrees(sweep_tr.heading_rad.max())),
            "telemetry_dt_median_s": float(np.median(np.diff(tr.mission_s))),
            "telemetry_dt_max_s": float(np.max(np.diff(tr.mission_s))),
            "lanes": lanes,
            "xt_rms_all_m": float(np.sqrt(np.mean([l["xt_rms_m"] ** 2 for l in lanes]))),
            "xt_max_all_m": float(max(l["xt_max_m"] for l in lanes)),
            "overshoot_max_m": float(max(l["overshoot_m"] for l in lanes)),
            "tags": tag_rows,
            "all_tags_confirmed_s": float(max(r["confirmed_s"] for r in tag_rows)),
            "all_tags_after_sweep_start_s": float(max(r["confirmed_s"] for r in tag_rows)
                                                  - t_sweep_start),
            "detections_total": int(len(D["det"])),
            "detections_by_id": {int(k): int(v) for k, v in det_counts.items()},
            "loc_ids_configured": loc_ids_cfg,
            "loc_ids_fixed": loc_ids_seen,
            "loc_visits": loc[["tag_id", "visit", "n_sightings", "px4_x", "px4_y", "ref_x",
                               "ref_y", "raw_err_m", "residual_m"]].to_dict("records"),
            "loc_raw_err_mean_m": float(loc.raw_err_m.mean()),
            "loc_raw_err_max_m": float(loc.raw_err_m.max()),
            "loc_fit": {
                "turn_deg": float(fit_m.group(1)), "scale": float(fit_m.group(2)),
                "shift_x": float(fit_m.group(3)), "shift_y": float(fit_m.group(4)),
                "rms_m": float(fit_m.group(5)), "n_fixes": int(fit_m.group(6)),
                "loo_global_cm": float(loo_m.group(1)), "loo_time_cm": float(loo_m.group(2)),
                "loo_hybrid_cm": float(loo_m.group(3)),
            },
            "land_targets": int(len(lt)),
            "land_first_offset_cm": float(first_lt.group(1)) if first_lt else float("nan"),
            "land_correction_moves": corr_attempts,
            "land_final_offset_cm": final_cm,
            "land_cmd_reason": land_cmd,
        },
        "speed": (tg, v),
    }


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_timeline(results):
    phases = [("Take-off and hover", "t0", "t_explore_s"),
              ("Lawnmower sweep", "t_explore_s", "t_sweep_end_s"),
              ("Return home", "t_sweep_end_s", "t_home_s"),
              ("Precision-landing hold", "t_home_s", "t_landing_s"),
              ("Landing", "t_landing_s", "t_done_s")]
    fig, ax = plt.subplots(figsize=(6.3, 2.4))
    for row, r in enumerate(results):
        m = dict(r["metrics"], t0=0.0)
        for k, (label, a, b) in enumerate(phases):
            ax.barh(row, m[b] - m[a], left=m[a], height=0.55, color=SERIES[k],
                    edgecolor="white", linewidth=1.5, label=label if row == 0 else None)
        ax.text(m["t_done_s"] + 1, row, f"{m['t_done_s']:.1f} s", va="center",
                color=INK, fontsize=9)
    ax.set_yticks(range(len(results)), [r["run"]["name"] for r in results])
    ax.invert_yaxis()
    ax.set_xlabel("Mission time from take-off command (s)")
    ax.set_xlim(0, max(r["metrics"]["t_done_s"] for r in results) * 1.12)
    ax.grid(axis="y", visible=False)
    ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.32), fontsize=8.5)
    fig.savefig(FIG / "fig_timeline.png")
    plt.close(fig)


def _area_box(ax, cfg, color=MUTED):
    a = cfg["area"]
    ax.add_patch(plt.Rectangle((a["y_min"], a["x_min"]), a["y_max"] - a["y_min"],
                               a["x_max"] - a["x_min"], fill=False, ls="--", lw=1,
                               ec=color, label="Search area"))


def fig_paths(r, idx):
    D, m, cfg = r["D"], r["metrics"], r["D"]["cfg"]
    tr, tc = D["traj"], D["traj_corr"]
    fig, axes = plt.subplots(1, 2, figsize=(6.3, 4.3), sharey=True)
    loc_ref = cfg["localisation"]["tags"]
    for k, ax in enumerate(axes):
        _area_box(ax, cfg)
        if k == 0:
            wy = [0.0] + [w[1] for w in r["wps"]]
            wx = [0.0] + [w[0] for w in r["wps"]]
            ax.plot(wy, wx, ls=":", lw=1.2, color=INK2, label="Planned path (aligned)")
            air = tr[tr.mission_s >= m["t_hover_s"] - 8]
            ax.plot(air.y, air.x, color=SERIES[0], label="Flown path (PX4 estimate)")
            ax.scatter(D["tags"].y, D["tags"].x, s=46, marker="s", color=SERIES[1],
                       edgecolor="white", linewidth=1, zorder=5, label="Search tag (raw)")
            for _, t in D["tags"].iterrows():
                ax.annotate(f"{int(t.tag_id)}", (t.y, t.x), xytext=(6, -3),
                            textcoords="offset points", fontsize=9, color=INK)
            ax.scatter(D["loc"].px4_y, D["loc"].px4_x, s=26, marker="D", color=MUTED,
                       zorder=4, label="Localisation-tag visit (raw)")
            ax.set_title("PX4 local frame (as flown)")
        else:
            ax.plot(tc.y, tc.x, color=SERIES[0], label="Corrected path")
            ax.scatter(D["tags_corr"].y, D["tags_corr"].x, s=46, marker="s", color=SERIES[1],
                       edgecolor="white", linewidth=1, zorder=5, label="Search tag (corrected)")
            for _, t in D["tags_corr"].iterrows():
                ax.annotate(f"{int(t.tag_id)}", (t.y, t.x), xytext=(6, -3),
                            textcoords="offset points", fontsize=9, color=INK)
            ax.scatter([t["y"] for t in loc_ref], [t["x"] for t in loc_ref], s=40,
                       marker="D", facecolor="none", edgecolor=INK, lw=1.2, zorder=4,
                       label="Localisation tag (surveyed)")
            for t in loc_ref:
                ax.annotate(f"L{t['id']}", (t["y"], t["x"]), xytext=(5, 4),
                            textcoords="offset points", fontsize=7.5, color=INK2)
            ax.set_title("Arena frame (drift-corrected)")
        ax.set_aspect("equal")
        ax.set_xlim(-2.2, 1.9)
        ax.set_ylim(-0.5, 4.9)
        ax.set_xlabel("East, y (m)")
    axes[0].set_ylabel("North, x (m)")
    h, l = [], []
    for ax in axes:
        for hh, ll in zip(*ax.get_legend_handles_labels()):
            if ll not in l:
                h.append(hh)
                l.append(ll)
    fig.legend(h, l, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.08), fontsize=8)
    fig.savefig(FIG / f"fig_path_run{idx + 1}.png")
    plt.close(fig)


def fig_speed(results):
    fig, axes = plt.subplots(len(results), 1, figsize=(6.3, 5.2), sharex=True)
    for k, (ax, r) in enumerate(zip(axes, results)):
        tg, v = r["speed"]
        m = r["metrics"]
        t = tg - m["t_sweep_start_s"]
        sel = (t >= -2) & (tg <= m["t_home_s"] + 1)
        ax.plot(t[sel], v[sel], color=SERIES[k], lw=1.6)
        cap = m["mpc_xy_vel_max"]
        ax.axhline(cap, color=INK2, ls="--", lw=1)
        ax.text(0.99, cap, f"MPC_XY_VEL_MAX {cap:.1f} m/s"
                + ("" if m["cap_confirmed"] else " (to confirm)"),
                transform=ax.get_yaxis_transform(), ha="right", va="bottom",
                fontsize=8, color=INK2)
        for tt in (m["t_sweep_end_s"],):
            ax.axvline(tt - m["t_sweep_start_s"], color=MUTED, lw=1, ls=":")
        ax.set_ylim(0, 1.7)
        ax.set_ylabel("Speed (m/s)")
        ax.set_title(r["run"]["name"], loc="left", fontsize=10)
    axes[-1].set_xlabel("Time from sweep start (s); dotted line = end of sweep, "
                        "followed by the return home")
    fig.tight_layout()
    fig.savefig(FIG / "fig_speed.png")
    plt.close(fig)


def fig_sightings(results):
    tag_ids = [t["tag_id"] for t in results[0]["metrics"]["tags"]]
    fig, ax = plt.subplots(figsize=(6.3, 2.8))
    w = 0.26
    for k, r in enumerate(results):
        n = {t["tag_id"]: t["n_sightings"] for t in r["metrics"]["tags"]}
        xs = np.arange(len(tag_ids)) + (k - (len(results) - 1) / 2) * w
        bars = ax.bar(xs, [n[i] for i in tag_ids], width=w - 0.03, color=SERIES[k],
                      label=f"{r['run']['name']} (cap {r['metrics']['mpc_xy_vel_max']:.1f} m/s)")
        for b in bars:
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.6,
                    f"{int(b.get_height())}", ha="center", fontsize=8, color=INK2)
    ax.axhline(results[0]["D"]["cfg"]["tags"]["min_sightings"], color=INK2, ls="--", lw=1)
    ax.text(len(tag_ids) - 0.5, results[0]["D"]["cfg"]["tags"]["min_sightings"] + 0.5,
            "confirmation threshold", ha="right", fontsize=8, color=INK2)
    ax.set_xticks(range(len(tag_ids)), [f"Tag {i}" for i in tag_ids])
    ax.set_ylabel("Accepted sightings")
    ax.set_ylim(0, 48)
    ax.grid(axis="x", visible=False)
    ax.legend(fontsize=8, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.12))
    fig.savefig(FIG / "fig_sightings.png")
    plt.close(fig)


def fig_repeatability(results):
    tag_ids = [t["tag_id"] for t in results[0]["metrics"]["tags"]]
    fig, axes = plt.subplots(1, len(tag_ids), figsize=(6.3, 2.7))
    for ax, tid in zip(axes, tag_ids):
        pts = []
        for k, r in enumerate(results):
            t = next(t for t in r["metrics"]["tags"] if t["tag_id"] == tid)
            ax.scatter(t["raw_y"], t["raw_x"], s=48, facecolor="none", edgecolor=SERIES[k],
                       lw=1.6, label="raw" if k == 0 else None)
            ax.scatter(t["corr_y"], t["corr_x"], s=48, color=SERIES[k], edgecolor="white",
                       lw=1, label=r["run"]["short"])
            ax.plot([t["raw_y"], t["corr_y"]], [t["raw_x"], t["corr_x"]], color=SERIES[k],
                    lw=0.8, alpha=0.6)
            pts += [(t["raw_y"], t["raw_x"]), (t["corr_y"], t["corr_x"])]
        pts = np.array(pts)
        c = pts.mean(axis=0)
        half = max(0.3, np.abs(pts - c).max() + 0.06)
        ax.set_xlim(c[0] - half, c[0] + half)
        ax.set_ylim(c[1] - half, c[1] + half)
        ax.set_aspect("equal")
        ax.set_title(f"Tag {tid}")
        ax.set_xlabel("East, y (m)", fontsize=9)
        ax.tick_params(labelsize=8)
    axes[0].set_ylabel("North, x (m)")
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], marker="o", ls="", color=SERIES[k], label=r["run"]["short"])
               for k, r in enumerate(results)]
    handles += [Line2D([], [], marker="o", ls="", mfc="none", mec=INK2, label="raw (PX4 frame)"),
                Line2D([], [], marker="o", ls="", color=INK2, label="drift-corrected")]
    fig.legend(handles=handles, ncol=5, loc="lower center", bbox_to_anchor=(0.5, -0.1),
               fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG / "fig_repeatability.png")
    plt.close(fig)


def fig_loc(results):
    fig, ax = plt.subplots(figsize=(6.3, 2.8))
    labels, raw, res, col = [], [], [], []
    for k, r in enumerate(results):
        for v in r["metrics"]["loc_visits"]:
            labels.append(f"{r['run']['short']}\nL{int(v['tag_id'])}.{int(v['visit'])}")
            raw.append(v["raw_err_m"] * 100)
            res.append(v["residual_m"] * 100)
            col.append(SERIES[k])
    x = np.arange(len(labels))
    ax.bar(x - 0.2, raw, width=0.38, color=col, alpha=0.45, label="before correction (raw PX4)")
    ax.bar(x + 0.2, res, width=0.38, color=col, label="after global fit (residual)")
    ax.set_xticks(x, labels, fontsize=6.5)
    ax.set_ylabel("Error at localisation tag (cm)")
    ax.grid(axis="x", visible=False)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=INK2, alpha=0.45, label="before correction (raw PX4)"),
                       Patch(color=INK2, label="after global similarity fit (residual)")],
              fontsize=8, loc="upper left")
    fig.savefig(FIG / "fig_loc.png")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Repeatability numbers
# ---------------------------------------------------------------------------

def dispersion(results):
    out = {}
    for tid in [t["tag_id"] for t in results[0]["metrics"]["tags"]]:
        rows = [next(t for t in r["metrics"]["tags"] if t["tag_id"] == tid) for r in results]
        d = {}
        for kind in ("raw", "corr"):
            p = np.array([[t[f"{kind}_x"], t[f"{kind}_y"]] for t in rows])
            c = p.mean(axis=0)
            dist = np.linalg.norm(p - c, axis=1)
            d[kind] = {"mean_x": float(c[0]), "mean_y": float(c[1]),
                       "rms_from_mean_m": float(np.sqrt(np.mean(dist ** 2))),
                       "max_pairwise_m": float(max(np.linalg.norm(a - b) for a in p for b in p))}
        out[tid] = d
    return out


def main():
    results = [analyse(r) for r in RUNS]
    fig_timeline(results)
    for i, r in enumerate(results):
        fig_paths(r, i)
    fig_speed(results)
    fig_sightings(results)
    fig_repeatability(results)
    fig_loc(results)
    metrics = {"runs": [r["metrics"] for r in results],
               "dispersion": {str(k): v for k, v in dispersion(results).items()}}
    (ROOT / "data" / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float),
                                                encoding="utf-8")
    for m in metrics["runs"]:
        print(f"{m['id']}: sweep {m['sweep_duration_s']:.1f} s, mean {m['speed_mean_mps']:.2f} "
              f"m/s, cruise {m['speed_cruise_median_mps']:.2f}, max {m['speed_max_mps']:.2f}, "
              f"xt rms {m['xt_rms_all_m']*100:.1f} cm max {m['xt_max_all_m']*100:.1f} cm, "
              f"overshoot {m['overshoot_max_m']*100:.1f} cm, all tags at "
              f"{m['all_tags_after_sweep_start_s']:.1f} s after sweep start, total "
              f"{m['mission_time_s']:.1f} s (take-off cmd -> land cmd)")
    print(json.dumps(metrics["dispersion"], indent=1))


if __name__ == "__main__":
    main()
