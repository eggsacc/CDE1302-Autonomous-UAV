"""Offline acceptance tests; never open drone sockets or send flight commands."""
import ctypes
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import zlib

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parent
sys.path.insert(0,str(HERE))
from protocol import *
from send_trajectory import load_trajectory
from trajectory_reference import (Reference, read_reference, write_tangent_csv,
                                  tangent_yaw, sample_reference, validate_reference)
from trajectory_runner import prepare, run, YawLink
from types import SimpleNamespace


def arguments(path, **kw):
    defaults = dict(drone_id=2, trajectory=str(path), yaw_mode="tangent", dt=0.05,
                    max_speed=0.30, max_yaw_rate=0.6, stationary_speed=1e-4,
                    dry_run=True, check_firmware=False, takeoff=False,
                    takeoff_wait=12.0, stale_timeout=3.0, cmd_port=5006,
                    esp_ip=None, log_dir="logs", config="", confirm=True,
                    telem_port=5005, finish="land")
    return SimpleNamespace(**(defaults | kw))


def simple_csv(tmp_path):
    source=tmp_path/"simple.csv"
    source.write_text("t,x,y,z\n0,0,0,0\n0.05,0,0.005,0\n0.1,0,0.01,0\n")
    dest=tmp_path/"yaw.csv"
    write_tangent_csv(source,dest)
    return dest


def test_original_position_and_time_fields_identical():
    def rows(path):
        return [line for line in path.read_text().splitlines()
                if line and not line.startswith("#")][1:]
    fixed=ROOT/"trajectory/ellipse_min_snap_traj.csv"
    yaw=ROOT/"trajectory/ellipse_min_snap_traj_tangent_yaw.csv"
    assert rows(fixed)==[line.rsplit(",",1)[0] for line in rows(yaw)]
    a=load_trajectory(str(fixed),0.05)
    b=sample_reference(read_reference(yaw,"tangent"),0.05,True)
    assert np.array_equal(a,b.xyz)
    assert b.t[-1]==34.0 and len(b.t)==681


def test_strict_ellipse_refuses_practical_limit():
    path=ROOT/"trajectory/ellipse_min_snap_traj_tangent_yaw.csv"
    ref=read_reference(path,"tangent")
    expected,defined=tangent_yaw(ref.t,ref.xyz)
    assert np.allclose(ref.yaw,expected,atol=1e-13)
    assert np.count_nonzero(~defined)==6
    assert np.degrees(ref.yaw[0])==pytest.approx(80.04127,abs=1e-4)
    with pytest.raises(ValueError,match="54.377"):
        prepare(arguments(path,max_yaw_rate=1.0))


@pytest.mark.parametrize("bad", ["nan", "inf", "-inf", "oops", ""])
def test_bad_data_not_silently_removed(tmp_path,bad):
    p=tmp_path/"bad.csv"
    p.write_text(f"t,x,y,z,yaw_rad\n0,0,0,0,0\n0.05,0.01,0,0,{bad}\n0.1,0.02,0,0,0\n")
    with pytest.raises(ValueError): read_reference(p,"tangent")


def test_rejects_missing_yaw_nonmonotonic_and_wrong_schema(tmp_path):
    for text in ["t,x,y,z\n0,0,0,0\n1,1,0,0\n",
                 "t,x,y,z,yaw_rad\n0,0,0,0,0\n0,0,0,0,0\n",
                 "t,x,y,z,heading\n0,0,0,0,0\n1,0,0,0,0\n"]:
        p=tmp_path/"bad.csv"; p.write_text(text)
        with pytest.raises(ValueError): read_reference(p,"tangent")


def test_wrapping_interpolates_through_pi_not_zero():
    ref=Reference(np.array([0.,1.]),np.zeros((2,3)),np.unwrap(np.radians([179,-179])))
    q=sample_reference(ref,0.5,True)
    assert q.yaw[1]==pytest.approx(math.pi)
    assert np.diff(q.yaw)==pytest.approx(np.radians([1,1]))


def test_stationary_endpoints_and_interior_are_deterministic():
    t=np.arange(7,dtype=float)
    p=np.array([[0,0,0],[0,0,0],[0,1,0],[0,1,0],[0,1,0],[0,2,0],[0,2,0]])
    yaw,defined=tangent_yaw(t,p)
    assert not defined[0] and not defined[3] and not defined[-1]
    assert yaw==pytest.approx(np.full(7,math.pi/2))
    with pytest.raises(ValueError,match="entire path is stationary"):
        tangent_yaw(t,np.zeros((7,3)))


def test_speed_rate_and_duration_bounds():
    ref=Reference(np.array([0.,0.1]),np.array([[0.,0.,0.],[1.,0.,0.]]))
    with pytest.raises(ValueError,match="max speed"): validate_reference(ref,0.3)
    ref.xyz[:]=0; ref.yaw=np.array([0.,1.])
    with pytest.raises(ValueError,match="max yaw rate"):
        validate_reference(ref,0.3,0.6,check_tangent=False)
    with pytest.raises(ValueError,match="integer number"):
        sample_reference(ref,0.06,True)


def test_dry_runs_open_no_sockets(tmp_path,monkeypatch,capsys):
    def boom(*a,**kw): raise AssertionError("dry-run opened a socket")
    monkeypatch.setattr("socket.socket",boom)
    run(arguments(simple_csv(tmp_path)))
    run(arguments(ROOT/"trajectory/ellipse_min_snap_traj.csv",yaw_mode="fixed"))
    assert capsys.readouterr().out.count("DRY RUN PASS")==2


def test_packet_shapes_and_float32_roundtrip(tmp_path):
    ref,dt,ident,packets,start,begin=prepare(arguments(simple_csv(tmp_path)))
    assert len(begin)==20 and len(start)==8
    assert start[1]==CMD_YAW_START and all(p[1]==CMD_YAW_DATA for p in packets)
    points=np.column_stack([ref.xyz,ref.yaw]); payload=yaw_payload(points)
    assert np.array_equal(np.frombuffer(payload,dtype="<f4").reshape(-1,4),points.astype("f4"))
    big=yaw_payload([(0,0,0,0)]*2400)
    assert max(map(len,build_yaw_packets(2,12,big)))==971
    assert build_traj_packets(1,[(0,0,0)]*2)[0][1]!=packets[0][1]


@pytest.mark.parametrize("field,value",[(1,2),(2,3),(4,456),(3,CMD_YAW_START)])
def test_wrong_version_target_transfer_or_operation_rejected(field,value):
    values=[PKT_TRAJ_YAW,1,2,CMD_YAW_CAPS,123,YAW_CAPABLE,0,2400,50,1.0,YAW_ALIGNMENT_RAD,0.0]
    values[field]=value
    with pytest.raises(ValueError):
        require_yaw_reply(struct.pack(YAW_REPLY_FMT,*values),2,123,CMD_YAW_CAPS)


def test_old_firmware_timeout_cannot_fall_back(monkeypatch):
    class Silent:
        def connect(self,*a): pass
        def settimeout(self,*a): pass
        def send(self,p): assert p[1]==CMD_YAW_CAPS
        def recv(self,*a):
            import socket
            raise socket.timeout()
        def close(self): pass
    monkeypatch.setattr("socket.socket",lambda *a: Silent())
    with YawLink("192.168.1.202",5006,2,123) as link:
        with pytest.raises(RuntimeError,match="no compatible yaw"):
            link.caps()


def test_legacy_telemetry_does_not_invent_px4_state():
    data=struct.pack("<BBfffBbf32sBH",1,2,0.,0.,0.,0,-1,0.,bytes(32),0,65535)
    old=parse_telemetry(data)
    assert old.px4_link_ok is None and old.px4_armed is None
    new=parse_telemetry(data+struct.pack("<HHBBB",10,20,1,6,0))
    assert new.px4_link_ok and new.px4_armed==1 and new.px4_custom_main_mode==6


@pytest.fixture(scope="module")
def core(tmp_path_factory):
    compiler=os.environ.get("YAW_CC")
    zig=os.environ.get("ZIG")
    command=[compiler] if compiler else ([zig,"cc"] if zig else None)
    if command is None:
        found=shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
        if not found: pytest.skip("host C compiler unavailable; portable firmware tests not run")
        command=[found]
    directory=tmp_path_factory.mktemp("yaw_core")
    library=directory/("yaw_core.dll" if os.name=="nt" else "yaw_core.so")
    cmd=command+["-std=c11","-O2","-Wall","-Wextra","-Werror","-shared",
                 "-I",str(ROOT/"main"),str(ROOT/"main/traj_yaw.c"),
                 str(Path(__file__).with_name("yaw_core_wrapper.c")),"-o",str(library)]
    if os.name!="nt": cmd.extend(["-fPIC","-lm"])
    built=subprocess.run(cmd,capture_output=True,text=True,timeout=240)
    assert built.returncode==0,(built.stdout+built.stderr)[-4000:]
    dll=ctypes.CDLL(str(library))
    dll.yaw_packet.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                             ctypes.c_float,ctypes.c_void_p]
    dll.yaw_point.argtypes=[ctypes.c_int,ctypes.c_int]; dll.yaw_point.restype=ctypes.c_float
    dll.yaw_interpolate.argtypes=[ctypes.c_float]*3; dll.yaw_interpolate.restype=ctypes.c_float
    return dll


def call(core,p,flying=False,heading=0.,busy=False):
    reply=ctypes.create_string_buffer(YAW_REPLY_SIZE)
    if not core.yaw_packet(p,len(p),busy,flying,heading,reply): return None
    return parse_yaw_reply(reply.raw)


def upload(core,points=None,ident=123,dt=50,rate=.6,crc=None):
    core.yaw_reset()
    if points is None: points=[(0,0,0,3.13),(-.005,0,0,3.15)]
    payload=yaw_payload(points)
    begin=build_yaw_begin(2,ident,len(points),dt,rate,zlib.crc32(payload) if crc is None else crc)
    assert call(core,begin).status==YAW_UPLOADING
    packets=build_yaw_packets(2,ident,payload)
    return points,packets,begin


def test_real_c_core_roundtrip_start_wrap_done_no_replay(core):
    points,packets,begin=upload(core)
    for p in packets: assert call(core,p).status==YAW_READY
    for i,row in enumerate(points):
        for j,value in enumerate(row): assert core.yaw_point(i,j)==pytest.approx(value)
    start=yaw_header(CMD_YAW_START,2,123)
    assert call(core,start).status==YAW_NOT_FLYING
    assert call(core,start,True,0).status==YAW_MISALIGNED
    assert call(core,start,True,3.13).status==YAW_PLAYING
    assert call(core,start,True,3.13).status==YAW_PLAYING
    assert abs(core.yaw_interpolate(3.13,3.15,.5))==pytest.approx(3.14,abs=1e-5)
    assert call(core,begin).status==YAW_BUSY
    core.yaw_done()
    assert call(core,start,True,3.13).status==YAW_DONE
    assert call(core,begin).status==YAW_BAD_ID


def test_c_incomplete_wrong_id_version_target_and_legacy(core):
    points,packets,begin=upload(core)
    start=yaw_header(CMD_YAW_START,2,123)
    assert call(core,start,True,3.13).status==YAW_INCOMPLETE
    assert call(core,yaw_header(CMD_YAW_START,2,124),True,3.13).status==YAW_BAD_ID
    bad=bytearray(begin); bad[2]=2
    assert call(core,bytes(bad)).status==YAW_BAD_VERSION
    bad[2]=1; bad[3]=3
    assert call(core,bytes(bad)) is None
    assert call(core,build_traj_start(123,50),True,3.13) is None
    assert call(core,packets[0],busy=True).status==YAW_BUSY


@pytest.mark.parametrize("mutation",["short","long","offset","count","nan","inf","crc","duplicate"])
def test_c_malformed_corrupt_uploads_cannot_start(core,mutation):
    _,packets,_=upload(core,crc=0 if mutation=="crc" else None)
    p=bytearray(packets[0])
    if mutation=="short": p=p[:-1]
    elif mutation=="long": p+=b"x"
    elif mutation=="offset": struct.pack_into("<H",p,8,65535)
    elif mutation=="count": p[10]=61
    elif mutation=="nan": struct.pack_into("<f",p,11,float("nan"))
    elif mutation=="inf": struct.pack_into("<f",p,15,float("inf"))
    elif mutation=="duplicate":
        assert call(core,bytes(p)).status==YAW_READY
        struct.pack_into("<f",p,27,-.004)
    reply=call(core,bytes(p))
    assert reply.status in {YAW_BAD_PACKET,YAW_CORRUPT}
    assert call(core,yaw_header(CMD_YAW_START,2,123),True,3.13).status!=YAW_PLAYING


def test_c_reorder_duplicates_speed_yaw_rate_and_cancel(core):
    points=[(i*.005,0,0,0) for i in range(125)]
    _,packets,_=upload(core,points)
    for p in reversed(packets): call(core,p)
    assert call(core,packets[0]).status==YAW_READY
    assert call(core,yaw_header(CMD_YAW_STATUS,2,123)).received==125
    core.yaw_cancel()
    assert call(core,yaw_header(CMD_YAW_START,2,123),True,0).status==YAW_CANCELLED
    for points in [[(0,0,0,0),(1,0,0,0)],[(0,0,0,0),(0,0,0,1)],
                   [(1,0,0,0),(1,0,0,0)]]:
        _,packets,_=upload(core,points)
        assert call(core,packets[0]).status==YAW_LIMIT
        assert call(core,yaw_header(CMD_YAW_START,2,123),True,0).status==YAW_LIMIT


def test_c_manifest_and_envelope_bounds(core):
    core.yaw_reset()
    valid=build_yaw_begin(2,123,2,50,.6,0)
    for off,fmt,val in [(8,"<H",2401),(10,"<H",49),(12,"<f",float("nan")),(12,"<f",1.1)]:
        bad=bytearray(valid); struct.pack_into(fmt,bad,off,val)
        assert call(core,bytes(bad)).status==YAW_LIMIT
    assert call(core,valid+b"x").status==YAW_BAD_PACKET
    assert call(core,yaw_header(CMD_YAW_CAPS,2,123)).status==YAW_CAPABLE


def test_live_orchestration_against_real_c_core_uses_only_fake_transport(core,tmp_path,monkeypatch):
    """Python -> actual C protocol -> ACKs; all drone/mission I/O is fake."""
    import comms
    import trajectory_runner
    core.yaw_reset()
    calls=[]
    packet=TelemetryPacket(2,0.,0.,math.pi/2,0,-1,0.,[False]*32,False,65535,
                           0,0,0,0,0)
    class FakeComms:
        def __init__(self,**kw): pass
        def on_telemetry(self,cb): self.cb=cb
        def start(self): self.cb(packet,"192.168.1.202")
        def stop(self): pass
        def send_command(self,drone,cmd):
            assert drone==2
            calls.append(cmd.cmd_type)
            if cmd.cmd_type==CMD_START:
                packet.px4_armed=1; packet.px4_custom_main_mode=6
                core.yaw_cancel()  # mission_task's reset at takeoff
            self.cb(packet,"192.168.1.202")
            return True
    class NativeCoreLink:
        def __init__(self,ip,port,drone,ident): self.ident=ident
        def __enter__(self): return self
        def __exit__(self,*a): pass
        def request(self,p,accepted,received_at_least=0):
            calls.append(p[1])
            reply=call(core,p,bool(packet.px4_armed),packet.heading_rad)
            assert reply is not None and reply.status in accepted
            assert reply.received>=received_at_least
            if p[1]==CMD_YAW_START: core.yaw_done()
            return reply
        def caps(self): return self.request(yaw_header(CMD_YAW_CAPS,2,self.ident),{YAW_CAPABLE})
    monkeypatch.setattr(comms,"CommsNode",FakeComms)
    monkeypatch.setattr(trajectory_runner,"YawLink",NativeCoreLink)
    monkeypatch.setattr("builtins.input",lambda prompt:"TANGENT-2")
    args=arguments(simple_csv(tmp_path),dry_run=False,takeoff=True,takeoff_wait=.0001,
                   log_dir=str(tmp_path/"local_records"))
    run(args)
    assert calls.index(CMD_YAW_CAPS)<calls.index(CMD_START)<calls.index(CMD_YAW_BEGIN)<calls.index(CMD_YAW_START)
    assert calls.count(CMD_LAND)==3
    assert list((tmp_path/"local_records").rglob("reference_sent.csv"))


def test_bad_firmware_preflight_sends_no_arm_takeoff_or_land(tmp_path,monkeypatch):
    import comms
    import trajectory_runner
    calls=[]
    packet=TelemetryPacket(2,0.,0.,math.pi/2,0,-1,0.,[False]*32,False,65535,
                           0,0,0,0,0)
    class FakeComms:
        def __init__(self,**kw): pass
        def on_telemetry(self,cb): self.cb=cb
        def start(self): self.cb(packet,"192.168.1.202")
        def stop(self): pass
        def send_command(self,*a): calls.append(a); return True
    class Incompatible:
        def __init__(self,*a): pass
        def __enter__(self): return self
        def __exit__(self,*a): pass
        def caps(self): raise ValueError("wrong firmware version")
    monkeypatch.setattr(comms,"CommsNode",FakeComms)
    monkeypatch.setattr(trajectory_runner,"YawLink",Incompatible)
    with pytest.raises(ValueError,match="wrong firmware"):
        run(arguments(simple_csv(tmp_path),dry_run=False,takeoff=True,
                      log_dir=str(tmp_path/"local_records")))
    assert calls==[]


def test_drone2_config_is_separate_and_preserves_other_settings(tmp_path):
    sys.path.insert(0,str(ROOT/"tools"))
    from prepare_yaw_drone2 import prepare as prepare_config, verify_header
    original='CONFIG_IDF_TARGET="esp32s3"\nCONFIG_DRONE_ID=22\nCONFIG_HOST_IPV4_ADDR="192.168.1.122"\nCONFIG_DRONE_STATIC_IPV4_ADDR="192.168.1.222"\nCONFIG_EXAMPLE_UNRELATED=123\n'
    source=tmp_path/"sdkconfig"; source.write_text(original)
    output=prepare_config(tmp_path)
    assert source.read_text()==original
    assert "CONFIG_DRONE_ID=2\n" in output.read_text()
    assert 'CONFIG_HOST_IPV4_ADDR="192.168.1.102"' in output.read_text()
    assert 'CONFIG_EXAMPLE_UNRELATED=123' in output.read_text()
    header=tmp_path/"sdkconfig.h"
    header.write_text('#define CONFIG_IDF_TARGET "esp32s3"\n#define CONFIG_DRONE_ID 2\n#define CONFIG_HOST_IPV4_ADDR "192.168.1.102"\n#define CONFIG_DRONE_STATIC_IPV4_ADDR "192.168.1.202"\n')
    verify_header(header)
    header.write_text(header.read_text().replace("CONFIG_DRONE_ID 2","CONFIG_DRONE_ID 22"))
    with pytest.raises(ValueError,match="CONFIG_DRONE_ID"): verify_header(header)
