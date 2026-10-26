#!/usr/bin/env python3
"""Generate an offline yaw reference without regenerating position or timing."""
import argparse
from pathlib import Path
import numpy as np
from trajectory_reference import STATIONARY_SPEED_MPS, rates, write_tangent_csv


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="trajectory/ellipse_min_snap_traj.csv")
    ap.add_argument("--output", default="trajectory/ellipse_min_snap_traj_tangent_yaw.csv")
    ap.add_argument("--stationary-speed", type=float, default=STATIONARY_SPEED_MPS,
                    help="m/s; raising this changes the yaw experiment and needs TA review")
    ap.add_argument("--plot", help="Optional local PNG audit of the reference, never flight measurements")
    args = ap.parse_args()
    ref, defined = write_tangent_csv(args.source, args.output, args.stationary_speed)
    speed, yr = rates(ref)
    print(f"Wrote {Path(args.output)}: {len(ref.t)} original position/time samples unchanged")
    print(f"Initial yaw {np.degrees(ref.yaw[0]):.3f} deg CW from North; "
          f"{np.count_nonzero(~defined)} direction-undefined samples")
    print(f"Speed peak {max(speed):.6f} m/s; yaw rate [{min(yr):.6f}, {max(yr):.6f}] rad/s")
    print("Reference generation is not flight clearance. The uploader and firmware enforce rate limits.")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout="constrained")
        axes[0].plot(ref.xyz[:,1],ref.xyz[:,0],color="#2166ac")
        axes[0].plot(ref.xyz[0,1],ref.xyz[0,0],"ro",label="Start / end")
        axes[0].set(xlabel="East (m)",ylabel="North (m)",title="Original position reference")
        axes[0].axis("equal"); axes[0].legend()
        axes[1].plot(ref.t,np.degrees(ref.yaw),label="Strict tangent")
        axes[1].axhline(0,color="gray",ls="--",label="Fixed +North reference")
        axes[1].set(xlabel="Time (s)",ylabel="Unwrapped yaw (deg)",title="Requested yaw only")
        axes[1].legend()
        axes[2].plot(ref.t[:-1],yr,color="#b2182b")
        axes[2].axhline(1,color="black",ls="--",label="Firmware ceiling: 1 rad/s")
        axes[2].set(xlabel="Time (s)",ylabel="Yaw rate (rad/s)",title="Strict tangent: flight BLOCKED")
        axes[2].legend()
        for ax in axes: ax.grid(alpha=.25)
        fig.suptitle("OFFLINE REFERENCE AUDIT - no physical flight data")
        output=Path(args.plot); output.parent.mkdir(parents=True,exist_ok=True)
        fig.savefig(output,dpi=160); plt.close(fig)
        print(f"Saved reference-only plot to {output}")


if __name__ == "__main__":
    main()
