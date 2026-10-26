"""Synthetic arrays test analysis maths. They are not physical flight evidence."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_yaw_experiments import compare_arrays, reference_fields, interpolate_checked, analyze_ulog, save_comparison


def test_errors_use_same_ned_frame_and_shortest_yaw_arc():
    t=np.array([0.,.1,.2])
    ref=np.zeros((3,3)); actual=np.tile([.3,.4,2.],(3,1))
    result=compare_arrays(t,ref,np.radians([179,179,179]),t,actual,t,np.radians([-179,-179,-179]))
    assert result["horizontal_error"]==pytest.approx([.5]*3)
    assert result["yaw_error"]==pytest.approx(np.radians([2]*3))
    assert result["metrics"]["horizontal_rmse_m"]==pytest.approx(.5)


def test_gaps_and_extrapolation_are_excluded():
    _,valid=interpolate_checked(np.array([0.,.1,1.]),np.array([1.,2.,3.]),np.array([-.1,.05,.5,1.1]))
    assert valid.tolist()==[False,True,False,False]
    with pytest.raises(ValueError,match="overlapping"):
        compare_arrays(np.array([3.,4.]),np.zeros((2,3)),np.zeros(2),
                       np.array([0.,1.]),np.zeros((2,3)),np.array([0.,1.]),np.zeros(2))


def test_setpoint_schema_requires_actual_reference():
    modern={"timestamp":[1,2],"position[0]":[0,1],"position[1]":[2,3],"position[2]":[-1,-1],"yaw":[0,.1]}
    p,yaw=reference_fields(modern)
    assert p.tolist()==[[0,2,-1],[1,3,-1]]
    with pytest.raises(ValueError,match="lacks"):
        reference_fields({"x":[0],"y":[0],"z":[0]})


def test_ulog_adapter_and_plot_export_with_labelled_synthetic_fixture(tmp_path,monkeypatch):
    import pyulog
    timestamps=np.arange(0,101_000,10_000,dtype=np.uint64)
    n=len(timestamps); z=np.zeros(n)
    def dataset(name,**data):
        return SimpleNamespace(name=name,multi_id=0,data=dict(timestamp=timestamps,**data))
    samples=[dataset("trajectory_setpoint",**{"position[0]":z,"position[1]":z,"position[2]":z,"yaw":z}),
             dataset("vehicle_local_position",x=z,y=z,z=z,xy_valid=np.ones(n),z_valid=np.ones(n)),
             dataset("vehicle_attitude",**{"q[0]":np.ones(n),"q[1]":z,"q[2]":z,"q[3]":z}),
             dataset("vehicle_status",nav_state=np.full(n,14),arming_state=np.full(n,2))]
    monkeypatch.setattr(pyulog,"ULog",lambda path:SimpleNamespace(data_list=samples,logged_messages=[]))
    result=analyze_ulog("SYNTHETIC_UNIT_TEST.ulg",0.,.1)
    assert result["metrics"]["horizontal_rmse_m"]==0
    assert "vehicle_status_0" in result["diagnostics"]
    out=tmp_path/"synthetic_analysis"
    save_comparison(result,result,out)
    for name in ["comparison.png","metrics.json","fixed_tracking.csv","tangent_tracking.png"]:
        assert (out/name).is_file() and (out/name).stat().st_size>20
