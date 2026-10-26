#!/usr/bin/env python3
"""Compare two real PX4 ULogs locally using logged reference and estimated states.

Explicit --*-start-s values are PX4 seconds since boot, not laptop wall time.
Only finite, position-valid data with interpolation gaps <=0.25 s is compared.
No network calls, automatic time alignment, or invented flight measurements.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def angular_error(measured, reference):
    return np.arctan2(np.sin(measured-reference),np.cos(measured-reference))


def interpolate_checked(t, values, query, max_gap=.25):
    order=np.argsort(t); t=np.asarray(t)[order]; values=np.asarray(values)[order]
    t,unique=np.unique(t,return_index=True); values=values[unique]
    if len(t)<2 or np.any(~np.isfinite(t)) or np.any(~np.isfinite(values)):
        raise ValueError("missing/nonfinite state samples")
    idx=np.clip(np.searchsorted(t,query,side="right"),1,len(t)-1)
    valid=(query>=t[0])&(query<=t[-1])&(t[idx]-t[idx-1]<=max_gap)
    if values.ndim==1:
        result=np.interp(query,t,values)
    else:
        result=np.column_stack([np.interp(query,t,values[:,i]) for i in range(values.shape[1])])
    return result,valid


def compare_arrays(ref_t,ref_xyz,ref_yaw,pos_t,xyz,yaw_t,yaw,max_gap=.25):
    measured,pos_ok=interpolate_checked(pos_t,xyz,ref_t,max_gap)
    measured_yaw,yaw_ok=interpolate_checked(yaw_t,np.unwrap(yaw),ref_t,max_gap)
    valid=pos_ok&yaw_ok
    if not np.any(valid): raise ValueError("no overlapping valid reference/estimate samples")
    horizontal=np.linalg.norm(measured[:,:2]-ref_xyz[:,:2],axis=1)
    yaw_error=angular_error(measured_yaw,ref_yaw)
    return dict(t=ref_t,reference_xyz=ref_xyz,reference_yaw=ref_yaw,
                measured_xyz=measured,measured_yaw=measured_yaw,
                horizontal_error=horizontal,yaw_error=yaw_error,valid=valid,
                metrics=dict(valid_samples=int(np.count_nonzero(valid)),
                             excluded_samples=int(np.count_nonzero(~valid)),
                             horizontal_rmse_m=float(np.sqrt(np.mean(horizontal[valid]**2))),
                             horizontal_max_m=float(np.max(horizontal[valid])),
                             yaw_rmse_rad=float(np.sqrt(np.mean(yaw_error[valid]**2))),
                             yaw_max_rad=float(np.max(np.abs(yaw_error[valid])))))


def topic(log,name):
    found=[d.data for d in log.data_list if d.name==name and d.multi_id==0]
    if len(found)!=1: raise ValueError(f"ULog needs exactly one {name} instance 0")
    return found[0]


def reference_fields(data):
    keys=[f"position[{i}]" for i in range(3)]
    if not all(k in data for k in keys):
        keys=["x","y","z"]
    if not all(k in data for k in [*keys,"yaw","timestamp"]):
        raise ValueError("trajectory_setpoint lacks position/yaw/timestamp fields")
    return np.column_stack([data[k] for k in keys]),np.asarray(data["yaw"])


def analyze_ulog(path,start_s,duration):
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError("Install pyulog from laptop/requirements-yaw.txt") from exc
    log=ULog(str(path))
    reference=topic(log,"trajectory_setpoint")
    position=topic(log,"vehicle_local_position")
    attitude=topic(log,"vehicle_attitude")
    ref_xyz,ref_yaw=reference_fields(reference)
    rt=np.asarray(reference["timestamp"],float)/1e6
    selected=(rt>=start_s)&(rt<=start_s+duration)&np.isfinite(ref_xyz).all(axis=1)&np.isfinite(ref_yaw)
    if np.count_nonzero(selected)<2:
        raise ValueError("No finite trajectory_setpoint data in the requested playback interval")
    xyz=np.column_stack([position[k] for k in ("x","y","z")])
    position_valid=np.isfinite(xyz).all(axis=1)
    for flag in ("xy_valid","z_valid"):
        if flag in position: position_valid &= np.asarray(position[flag],bool)
    q=np.column_stack([attitude[f"q[{i}]"] for i in range(4)])
    norms=np.linalg.norm(q,axis=1)
    attitude_valid=np.isfinite(q).all(axis=1)&(norms>.9)&(norms<1.1)
    q=q[attitude_valid]/norms[attitude_valid,None]
    yaw=np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2))
    result=compare_arrays(rt[selected]-start_s,ref_xyz[selected],ref_yaw[selected],
                          np.asarray(position["timestamp"])[position_valid]/1e6-start_s,
                          xyz[position_valid],np.asarray(attitude["timestamp"])[attitude_valid]/1e6-start_s,yaw)
    # Raw diagnostic context, only where present; absence is never reported as OK.
    result["diagnostics"]={}
    for dataset in log.data_list:
        if dataset.name not in {"vehicle_status","estimator_status","estimator_status_flags",
                                "vehicle_local_position"}: continue
        data=dataset.data
        ts=np.asarray(data["timestamp"])/1e6
        mask=(ts>=start_s)&(ts<=start_s+duration)
        fields=[key for key in data if key in {"nav_state","arming_state","xy_valid","z_valid",
                    "filter_fault_flags","innovation_check_flags","gps_check_fail_flags",
                    "xy_reset_counter","z_reset_counter","heading_reset_counter"}
                or key.startswith(("fs_","cs_","reject_","pre_flt_fail"))]
        result["diagnostics"][f"{dataset.name}_{dataset.multi_id}"]={
            "t_s":(ts[mask]-start_s).tolist(),
            **{key:np.asarray(data[key])[mask].tolist() for key in fields}}
    result["messages"]=[dict(t_s=float(msg.timestamp)/1e6-start_s,
                              level=str(msg.log_level),message=str(msg.message))
                         for msg in getattr(log,"logged_messages",[])
                         if start_s<=float(msg.timestamp)/1e6<=start_s+duration]
    result["metrics"]["source"]="Operator-provided PX4 ULog; physical/SITL provenance must be recorded separately"
    result["metrics"]["start_px4_boot_s"]=start_s
    return result


def save_comparison(fixed,tangent,out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out=Path(out); out.mkdir(parents=True,exist_ok=False)
    for name,result in [("fixed",fixed),("tangent",tangent)]:
        valid=result["valid"]
        data=np.column_stack([result["t"],result["reference_xyz"],result["measured_xyz"],
                              result["reference_yaw"],result["measured_yaw"],
                              result["horizontal_error"],result["yaw_error"],valid])
        # Excluded interpolation intervals must not look like measurements.
        data[~valid,4:7]=np.nan
        data[~valid,8:11]=np.nan
        np.savetxt(out/f"{name}_tracking.csv",data,delimiter=",",comments="",
                   header="t,ref_n,ref_e,ref_d,est_n,est_e,est_d,ref_yaw,est_yaw,horizontal_error_m,yaw_error_rad,valid")
        (out/f"{name}_diagnostics.json").write_text(json.dumps(dict(
            diagnostics=result.get("diagnostics",{}),messages=result.get("messages",[])),indent=2),encoding="utf-8")
        fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True,layout="constrained")
        for i,label in enumerate(["North (m)","East (m)","Down (m)"]):
            axes[i].plot(result["t"],result["reference_xyz"][:,i],label="Requested (ULog)")
            measured=result["measured_xyz"][:,i].copy(); measured[~valid]=np.nan
            axes[i].plot(result["t"],measured,label="Estimated (ULog)"); axes[i].set_ylabel(label)
            axes[i].grid(alpha=.25)
        est=result["measured_yaw"].copy(); est[~valid]=np.nan
        axes[3].plot(result["t"],np.unwrap(result["reference_yaw"]),label="Requested (ULog)")
        # Shift unwrapped branches by 2pi for display, never for error metrics.
        ref_start=result["reference_yaw"][np.flatnonzero(valid)[0]]
        est-=2*np.pi*round((est[np.flatnonzero(valid)[0]]-ref_start)/(2*np.pi))
        axes[3].plot(result["t"],est,label="Estimated (ULog)")
        axes[3].set_ylabel("Yaw (rad)"); axes[3].set_xlabel("PX4 time since chosen playback start (s)")
        axes[0].legend(); axes[3].legend(); axes[3].grid(alpha=.25)
        fig.suptitle(f"{name.title()} yaw: operator-provided PX4 ULog"); fig.savefig(out/f"{name}_tracking.png",dpi=160); plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout="constrained")
    for name,result in [("Fixed",fixed),("Tangent",tangent)]:
        v=result["valid"]; p=result["measured_xyz"].copy(); p[~v]=np.nan
        # Local NED origins may differ per flight: subtract each logged initial reference.
        origin=result["reference_xyz"][0]
        axes[0].plot(p[:,1]-origin[1],p[:,0]-origin[0],label=f"{name} estimate")
        e=result["horizontal_error"].copy(); e[~v]=np.nan
        axes[1].plot(result["t"],e,label=name)
        e=np.degrees(result["yaw_error"]); e[~v]=np.nan
        axes[2].plot(result["t"],e,label=name)
    axes[0].set(xlabel="East offset (m)",ylabel="North offset (m)",title="Estimated path"); axes[0].axis("equal")
    axes[1].set(xlabel="Time (s)",ylabel="Horizontal error (m)")
    axes[2].set(xlabel="Time (s)",ylabel="Yaw error (deg, shortest arc)")
    for ax in axes: ax.grid(alpha=.25); ax.legend()
    fig.savefig(out/"comparison.png",dpi=160); plt.close(fig)
    (out/"metrics.json").write_text(json.dumps(dict(fixed=fixed["metrics"],tangent=tangent["metrics"]),indent=2),encoding="utf-8")


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fixed-ulog",required=True)
    ap.add_argument("--tangent-ulog",required=True)
    ap.add_argument("--fixed-start-s",type=float,required=True)
    ap.add_argument("--tangent-start-s",type=float,required=True)
    ap.add_argument("--duration",type=float,default=34.)
    ap.add_argument("--output",required=True)
    args=ap.parse_args()
    if not np.isfinite([args.fixed_start_s,args.tangent_start_s,args.duration]).all() or args.duration<=0:
        raise SystemExit("Times must be finite and duration positive")
    fixed=analyze_ulog(args.fixed_ulog,args.fixed_start_s,args.duration)
    tangent=analyze_ulog(args.tangent_ulog,args.tangent_start_s,args.duration)
    save_comparison(fixed,tangent,args.output)
    print(f"Local ULog comparison written to {args.output}; no logs uploaded")


if __name__=="__main__": main()
