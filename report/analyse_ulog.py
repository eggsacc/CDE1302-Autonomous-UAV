#!/usr/bin/env python3
"""
PX4 on-board log (ULog) analysis of the demonstration flights, for Section 5.

Reads   cde1302/mission/px4_logs/sess<N>/log100.ulg   (pulled off the SD card)
        report/data/flights/<run>/trajectory.csv     (laptop telemetry, to match)
Writes  report/data/ulog_metrics.json                (every ULog number in the report)
        report/figures/fig_ulog_lanes.png            (lane tracking vs Eq. 3)

Run from the repo root:
    python report/analyse_ulog.py

Which log is which flight
-------------------------
The flight controller has no real-time clock (no GPS), so the logs carry no
wall-clock time.  Each log is matched to a run by sliding the PX4 position
(vehicle_local_position x, y) along the laptop's telemetry route
(trajectory.csv, which is the same PX4 estimate relayed over Wi-Fi) and taking
the time offset with the smallest mean distance.  The right pairing agrees to
under 1 cm; every wrong pairing is off by 0.6 m or more.  The arm-to-land time
also matches each run's take-off-command-to-land-command time (about 1 s
shorter, the arming delay).

    sess214, sess215  earlier flights (not the demonstration)
    sess217           Run 1, full-exploration         MPC_XY_VEL_MAX 0.4
    sess218           truncated-exploration-1 (not reported)       0.8
    sess219           Run 2, truncated-exploration-2               1.5

Logging profile: SDLOG_PROFILE = 161, so position/setpoints are at 10 Hz,
trajectory_setpoint and estimator status at 5 Hz, attitude at 20 Hz, and
estimator_states / raw sensor topics at only 1-2 Hz.
estimator_selector_status is not logged; an estimator switch is detected
instead from the primary-IMU id (sensors_status_imu) and the reset counters
of vehicle_local_position, which the selector increments on a switch.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from analyse_flights import SERIES, INK, INK2, MUTED, FIG  # noqa: E402  (also sets rcParams)

LOGS = ROOT.parent / "cde1302" / "mission" / "px4_logs"
FLIGHTS = ROOT / "data" / "flights"

# All three demonstration flights are matched (to prove the pairing); only the
# two reported runs are analysed further.
DEMO = [
    {"sess": "sess217", "id": "full-exploration", "short": "Run 1", "report": True},
    {"sess": "sess218", "id": "truncated-exploration-1", "short": "(not reported)", "report": False},
    {"sess": "sess219", "id": "truncated-exploration-2", "short": "Run 2", "report": True},
]
OTHER_LOGS = ["sess214", "sess215"]

BMI088_GYRO = 6684690   # IMU0, SPI2 (odom_analysis boot-log mapping)
BMI270_GYRO = 3604506   # IMU1, SPI3
LANE_MIN_M = 2.9        # a setpoint leg at least this long is a sweep lane


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class Log:
    def __init__(self, sess: str):
        self.sess = sess
        self.u = ULog(str(LOGS / sess / "log100.ulg"))
        self.t0 = self.u.start_timestamp
        self.P = self.u.initial_parameters
        vs = self.ds("vehicle_status")
        tv = self.t(vs)
        self.t_land = float(tv[np.argmax(vs["nav_state"] == 18)])   # AUTO_LAND
        self.t_disarm = float(tv[-1])
        self.duration = (self.u.last_timestamp - self.t0) / 1e6

    def ds(self, name: str, multi: int = 0) -> dict:
        return self.u.get_dataset(name, multi).data

    def t(self, d: dict) -> np.ndarray:
        return (d["timestamp"].astype(np.int64) - self.t0) / 1e6

    def airborne(self, d: dict, t_from: float = 3.0) -> np.ndarray:
        """Samples from just after take-off (detected at 1.8 s) until AUTO_LAND."""
        t = self.t(d)
        return (t > t_from) & (t < self.t_land)


def match_to_runs(log: Log) -> dict:
    """Best time offset and mean position error of this log against each run."""
    lp = log.ds("vehicle_local_position")
    tu, xu, yu = log.t(lp), lp["x"], lp["y"]
    out = {}
    for run in DEMO:
        tr = pd.read_csv(FLIGHTS / run["id"] / "trajectory.csv").dropna(subset=["mission_s"])
        tm = tr.mission_s.to_numpy()
        best = (np.inf, 0.0)
        for off in np.arange(-10, 10, 0.05):            # ulog time = mission time + off
            tt = tm + off
            m = (tt > tu[0]) & (tt < tu[-1])
            if m.sum() < 100:
                continue
            e = np.hypot(np.interp(tt[m], tu, xu) - tr.x.to_numpy()[m],
                         np.interp(tt[m], tu, yu) - tr.y.to_numpy()[m]).mean()
            if e < best[0]:
                best = (float(e), float(off))
        out[run["id"]] = {"mean_err_m": best[0], "offset_s": best[1]}
    return out


def euler_tilt_deg(q: np.ndarray) -> np.ndarray:
    """Tilt from vertical (deg) from quaternions [w, x, y, z]."""
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


# ---------------------------------------------------------------------------
# Lane tracking: the norm-limited velocity demand (report Eq. 3)
# ---------------------------------------------------------------------------

def lanes(log: Log) -> list[dict]:
    """Per sweep lane: cross-track error and the cross-track velocity demand.

    In waypoint mode the companion sends the next lane end as a position
    setpoint, so each lane is one constant trajectory_setpoint.  Cross-track
    sign: positive = the direction the drone was moving on the leg before the
    lane (i.e. momentum carried into the lane).
    PX4's velocity demand is vehicle_local_position_setpoint (vx, vy), the
    output of the position P-loop after the MPC_XY_VEL_MAX norm limit.
    """
    kp, vmax = log.P["MPC_XY_P"], log.P["MPC_XY_VEL_MAX"]
    d = log.ds("trajectory_setpoint")
    t = log.t(d)
    X = np.c_[d["position[0]"], d["position[1]"]]
    sw = [0] + [i for i in range(1, len(t)) if np.hypot(*(X[i] - X[i - 1])) > 0.05]
    sw.append(len(t) - 1)

    lp = log.ds("vehicle_local_position")
    tl, p, v = log.t(lp), np.c_[lp["x"], lp["y"]], np.c_[lp["vx"], lp["vy"]]
    sp = log.ds("vehicle_local_position_setpoint")
    ts, vs = log.t(sp), np.c_[sp["vx"], sp["vy"]]

    out = []
    for k in range(2, len(sw) - 1):
        a, b, prev = X[sw[k - 1]], X[sw[k]], X[sw[k - 2]]
        L = float(np.hypot(*(b - a)))
        if L < LANE_MIN_M:
            continue
        u = (b - a) / L
        n = np.array([-u[1], u[0]])
        if np.dot(n, a - prev) < 0:                     # point n along the previous leg
            n = -n
        tA, tB = float(t[sw[k]]), float(t[sw[k + 1]])
        m = (tl >= tA) & (tl < tB)
        ec, ea = (p[m] - a) @ n, (b - p[m]) @ u
        # 10 Hz setpoint log vs 5 Hz trajectory_setpoint: the last sample before
        # tB can already belong to the next leg, so stop 0.15 s short.
        ms = (ts >= tA) & (ts < tB - 0.15)
        ecs = np.interp(ts[ms], tl, (p - a) @ n)
        eas = np.interp(ts[ms], tl, (b - p) @ u)
        vc_sp = vs[ms] @ n
        eq3 = -kp * ecs * np.minimum(1.0, vmax / (kp * np.hypot(eas, ecs)))
        j = int(np.argmax(np.abs(ecs)))
        out.append({
            "t_start_s": tA, "t_end_s": tB, "length_m": L,
            "entry_vc_mps": float(v[m][0] @ n),
            "xt_rms_m": float(np.sqrt(np.mean(ec ** 2))),
            "xt_mean_m": float(ec.mean()),
            "xt_peak_m": float(ec[np.argmax(np.abs(ec))]),
            "xt_peak_after_s": float(tl[m][np.argmax(np.abs(ec))] - tA),
            "along_speed_median_mps": float(np.median(v[m] @ u)),
            "at_peak": {"ec_m": float(ecs[j]), "ea_m": float(eas[j]),
                        "vc_demand_mps": float(vc_sp[j]), "eq3_mps": float(eq3[j]),
                        "p_only_mps": float(-kp * ecs[j])},
            "eq3_max_abs_err_mps": float(np.max(np.abs(vc_sp - eq3))),
            "series": {"t": tl[m] - tA, "ec": ec, "ts": ts[ms] - tA, "vc_sp": vc_sp,
                       "eq3": eq3, "p_only": -kp * ecs},
        })
    return out


# ---------------------------------------------------------------------------
# Per-flight metrics
# ---------------------------------------------------------------------------

def analyse(log: Log, run: dict) -> dict:
    P = log.P
    m = {"sess": log.sess, "run": run["id"], "name": run["short"],
         "log_duration_s": log.duration, "arm_to_land_s": log.t_land,
         "params": {k: P.get(k) for k in [
             "MPC_XY_VEL_MAX", "MPC_XY_CRUISE", "MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_ACC_HOR",
             "MPC_TILTMAX_AIR", "MPC_THR_HOVER", "EKF2_MULTI_IMU", "CAL_GYRO0_PRIO",
             "CAL_GYRO1_PRIO", "CAL_ACC0_PRIO", "CAL_ACC1_PRIO", "EKF2_MAG_TYPE",
             "EKF2_OF_CTRL", "EKF2_EV_CTRL", "EKF2_GPS_CTRL", "EKF2_HGT_REF",
             "SENS_FLOW_MAXR", "SDLOG_PROFILE"]},
         "params_changed_in_flight": [c[1] for c in log.u.changed_parameters
                                      if c[0] > log.t0]}

    # Speed (PX4's own velocity estimate, 10 Hz)
    lp = log.ds("vehicle_local_position")
    f = log.airborne(lp)
    sp = np.hypot(lp["vx"], lp["vy"])[f]
    m["speed"] = {"max_mps": float(sp.max()), "p95_mps": float(np.percentile(sp, 95))}
    m["height"] = {"z_mean_m": float(-lp["z"][f].mean()), "z_std_m": float(lp["z"][f].std())}

    # Attitude and actuator margins
    att = log.ds("vehicle_attitude")
    fa = log.airborne(att)
    q = np.c_[att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"]][fa]
    tilt = euler_tilt_deg(q)
    mot = log.ds("actuator_motors")
    fm = log.airborne(mot)
    M = np.c_[[mot[f"control[{i}]"][fm] for i in range(4)]].T
    cas = log.ds("control_allocator_status")
    fc = log.airborne(cas)
    sat = np.c_[[cas[f"actuator_saturation[{i}]"][fc] for i in range(4)]].T
    m["margins"] = {
        "tilt_max_deg": float(tilt.max()), "tilt_p99_deg": float(np.percentile(tilt, 99)),
        "tilt_limit_deg": P["MPC_TILTMAX_AIR"],
        "motor_mean": float(M.mean()), "motor_max": float(M.max()),
        "saturation_frac": float((sat != 0).any(axis=1).mean()),
        "torque_achieved_frac": float(cas["torque_setpoint_achieved"][fc].mean()),
    }

    # Optical flow
    ofv = log.ds("estimator_optical_flow_vel")
    fo = log.airborne(ofv)
    fr = np.hypot(ofv["flow_rate_compensated[0]"], ofv["flow_rate_compensated[1]"])[fo]
    vof = log.ds("vehicle_optical_flow")
    fq = log.airborne(vof, t_from=6.0)                 # after the climb to 0.8 m
    itr = log.ds("estimator_innovation_test_ratios")
    fi = log.airborne(itr)
    m["flow"] = {
        "rate_max_rad_s": float(fr.max()), "rate_limit_rad_s": P["SENS_FLOW_MAXR"],
        "quality_median": float(np.median(vof["quality"][fq])),
        "quality_min": int(vof["quality"][fq].min()),
        "test_ratio_max": float(max(np.nanmax(itr["flow[0]"][fi]), np.nanmax(itr["flow[1]"][fi]))),
    }

    # Heading: EKF2's own uncertainty, IMU yaw rates, estimator switching
    hs = np.degrees(np.sqrt(lp["heading_var"]))
    tl = log.t(lp)
    i_sweep = np.argmax(tl > 3.0)
    i_land = np.argmax(tl >= log.t_land) - 1
    m["heading"] = {
        "sigma_start_deg": float(hs[i_sweep]), "sigma_land_deg": float(hs[i_land]),
        "range_deg": [float(np.degrees(lp["heading"][f].min())),
                      float(np.degrees(lp["heading"][f].max()))],
        "yaw_aligned": bool(log.ds("estimator_status_flags")["cs_yaw_align"].any()),
    }
    rates = {}
    for k in (0, 1):
        im = log.ds("vehicle_imu", k)
        ti = log.t(im)
        wz = np.degrees(im["delta_angle[2]"] / (im["delta_angle_dt"] * 1e-6))
        dev = int(im["gyro_device_id"][0])
        fl = (ti > 3.0) & (ti < log.t_land)
        rates["BMI088" if dev == BMI088_GYRO else "BMI270" if dev == BMI270_GYRO else str(dev)] = {
            "device_id": dev, "yaw_rate_flight_mean_dps": float(wz[fl].mean()),
            "yaw_rate_ground_mean_dps": float(wz[ti < 1.5].mean()),
        }
    m["imu_yaw_rate"] = rates
    si = log.ds("sensors_status_imu")
    fl = log.airborne(lp, t_from=0.0)
    m["switching"] = {
        "primary_gyro_ids": sorted({int(x) for x in si["gyro_device_id_primary"]}),
        "primary_is_bmi088_throughout": bool((si["gyro_device_id_primary"] == BMI088_GYRO).all()),
        "ekf0_gyro_ids": sorted({int(x) for x in log.ds("estimator_status")["gyro_device_id"]}),
        "gyro_inconsistency_mean_dps": float(np.degrees(
            si["gyro_inconsistency_rad_s[1]"][log.airborne(si)].mean())),
        **{f"{c}_reset_changes_in_flight": int(np.count_nonzero(np.diff(lp[f"{c}_reset_counter"][fl])))
           for c in ("xy", "z", "vxy", "heading")},
    }

    m["lanes"] = lanes(log)
    return m


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def fig_lanes(results):
    """(a) cross-track error along every lane of both runs;
    (b) Run 2's worst lane: PX4's logged cross-track velocity demand against
    Eq. 3 and against the unlimited P-loop demand."""
    fig, (a, b) = plt.subplots(1, 2, figsize=(6.6, 2.35),
                               gridspec_kw={"width_ratios": [1, 1]})
    for k, r in enumerate(results):
        for i, ln in enumerate(r["lanes"]):
            s = ln["series"]
            a.plot(s["t"], s["ec"] * 100, color=SERIES[k], lw=1.4,
                   label=f"{r['name']} ({r['params']['MPC_XY_VEL_MAX']:.1f} m/s cap)" if i == 0 else None)
    a.axhline(0, color=MUTED, lw=0.8)
    a.set_xlabel("Time from lane start (s)")
    a.set_ylabel("Cross-track error (cm)")
    a.set_title("(a) Cross-track error, every lane", loc="left", fontsize=10)
    a.legend(fontsize=8, loc="upper right")

    r2 = results[-1]
    ln = max(r2["lanes"], key=lambda l: abs(l["xt_peak_m"]))
    s = ln["series"]
    b.plot(s["ts"], s["p_only"], color=MUTED, lw=1.4, ls=":")
    b.plot(s["ts"], s["vc_sp"], color=SERIES[1], lw=1.6)
    b.plot(s["ts"], s["eq3"], ls="none", marker="o", ms=3.5, mfc="white", mec=INK, mew=0.8)
    # direct labels instead of a legend (the Eq. 3 markers sit on the logged line)
    # Label positions are in data units, placed by eye for this lane's curves.
    jp = int(np.argmin(s["p_only"]))
    b.annotate(r"P-loop alone, $-k_p e_c$", (s["ts"][jp], s["p_only"][jp]),
               xytext=(s["ts"][jp] + 0.45, s["p_only"][jp] - 0.015), fontsize=8, color=INK2,
               va="center", arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8})
    j = int(np.argmin(np.abs(s["ts"] - 0.8)))
    b.annotate("PX4 demand (logged, line)\nEq. (3) prediction (dots)", (s["ts"][j], s["vc_sp"][j]),
               xytext=(1.0, -0.07), fontsize=8, color=INK2, va="center",
               arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8})
    b.axhline(0, color=MUTED, lw=0.8)
    b.set_ylim(min(s["p_only"].min(), s["vc_sp"].min()) - 0.05, 0.04)
    b.set_xlabel("Time from lane start (s)")
    b.set_ylabel("Cross-track velocity (m/s)")
    b.set_title(f"(b) {r2['name']}, worst lane", loc="left", fontsize=10)
    fig.tight_layout(w_pad=1.5)
    fig.savefig(FIG / "fig_ulog_lanes.png")
    plt.close(fig)


# ---------------------------------------------------------------------------

def main():
    ident = []
    for sess in [r["sess"] for r in DEMO] + OTHER_LOGS:
        log = Log(sess)
        match = match_to_runs(log)
        best = min(match, key=lambda k: match[k]["mean_err_m"])
        ident.append({"sess": sess, "log_duration_s": log.duration, "arm_to_land_s": log.t_land,
                      "MPC_XY_VEL_MAX": log.P["MPC_XY_VEL_MAX"],
                      "best_match": best if match[best]["mean_err_m"] < 0.05 else None,
                      "match": match})
        print(f"{sess}: {log.duration:5.1f} s, arm->land {log.t_land:5.1f} s, "
              f"cap {log.P['MPC_XY_VEL_MAX']:.1f} m/s -> "
              + ", ".join(f"{k} {v['mean_err_m']*100:.1f} cm" for k, v in match.items()))
        for run in DEMO:
            if run["sess"] == sess:
                assert ident[-1]["best_match"] == run["id"], f"{sess} does not match {run['id']}"

    results = [analyse(Log(r["sess"]), r) for r in DEMO if r["report"]]
    fig_lanes(results)

    for m in results:
        g, mg, fl, sw = m["heading"], m["margins"], m["flow"], m["switching"]
        print(f"\n{m['name']} ({m['sess']}): cap {m['params']['MPC_XY_VEL_MAX']} m/s, "
              f"speed max {m['speed']['max_mps']:.2f} p95 {m['speed']['p95_mps']:.2f}")
        print(f"  tilt max {mg['tilt_max_deg']:.1f} deg (limit {mg['tilt_limit_deg']:.0f}), motors "
              f"mean {mg['motor_mean']:.2f} max {mg['motor_max']:.2f}, saturated "
              f"{mg['saturation_frac']*100:.1f} % of samples")
        print(f"  flow max {fl['rate_max_rad_s']:.2f} rad/s (limit {fl['rate_limit_rad_s']:.0f}), "
              f"quality median {fl['quality_median']:.0f} min {fl['quality_min']}, "
              f"test ratio max {fl['test_ratio_max']:.2f}")
        print(f"  heading sigma {g['sigma_start_deg']:.1f} -> {g['sigma_land_deg']:.1f} deg, "
              f"yaw aligned {g['yaw_aligned']}; IMU yaw rate in flight "
              + ", ".join(f"{k} {v['yaw_rate_flight_mean_dps']:+.2f} (ground "
                          f"{v['yaw_rate_ground_mean_dps']:+.2f})" for k, v in m["imu_yaw_rate"].items()))
        print(f"  primary BMI088 throughout {sw['primary_is_bmi088_throughout']}, resets in flight "
              + ", ".join(f"{c} {sw[f'{c}_reset_changes_in_flight']}" for c in ("xy", "z", "vxy", "heading")))
        for ln in m["lanes"]:
            pk = ln["at_peak"]
            print(f"  lane {ln['t_start_s']:5.1f}-{ln['t_end_s']:5.1f} s: entry vc "
                  f"{ln['entry_vc_mps']:+.2f} m/s, peak {ln['xt_peak_m']*100:+.1f} cm after "
                  f"{ln['xt_peak_after_s']:.1f} s, rms {ln['xt_rms_m']*100:.1f} cm; at peak demand "
                  f"{pk['vc_demand_mps']:+.3f} vs Eq3 {pk['eq3_mps']:+.3f} vs P {pk['p_only_mps']:+.3f}; "
                  f"|demand - Eq3| max {ln['eq3_max_abs_err_mps']*1000:.0f} mm/s")

    for m in results:
        for ln in m["lanes"]:
            ln.pop("series")
    (ROOT / "data" / "ulog_metrics.json").write_text(
        json.dumps({"identification": ident, "runs": results}, indent=2, default=float),
        encoding="utf-8")


if __name__ == "__main__":
    main()
