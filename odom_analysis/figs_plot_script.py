import sys; sys.path.insert(0,'/tmp')
import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
B0,B1,INK,INK2,MUTED,GRID,SURF,CRIT='#2a78d6','#eb6834','#0b0b0b','#52514e','#8a8984','#e6e5e1','#fcfcfb','#d03b3b'
plt.rcParams.update({'figure.facecolor':SURF,'axes.facecolor':SURF,'axes.edgecolor':MUTED,'axes.labelcolor':INK2,'xtick.color':INK2,'ytick.color':INK2,
 'axes.grid':True,'grid.color':GRID,'grid.linewidth':0.8,'axes.spines.top':False,'axes.spines.right':False,'font.size':10,'axes.titlesize':11,
 'axes.titleweight':'bold','axes.titlecolor':INK,'lines.linewidth':2,'legend.frameon':False,'legend.fontsize':9,'savefig.dpi':160,'savefig.bbox':'tight'})
OUT='/home/claude/figs/'
FL={'A':dict(f='/tmp/logs/A.ulg',sw=45.44,name='Flight A (eaeb93f7)'),'B':dict(f='/tmp/logs/B.ulg',sw=117.06,name='Flight B (0c0bde59)')}
def yaw(a):
    q0,q1,q2,q3=[a[f'q[{i}]'] for i in range(4)]
    return np.degrees(np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2)))
def unwrap(y): return np.degrees(np.unwrap(np.radians(y)))
D={}
for k,v in FL.items():
    u=ULog(v['f']); t0=u.start_timestamp; d={}
    for x in u.data_list:
        x.data['t']=(x.data['timestamp'].astype(np.int64)-t0)/1e6; d[(x.name,x.multi_id)]=x.data
    ev={}
    for m in u.logged_messages:
        tm=(m.timestamp-t0)/1e6
        if 'Kill engaged' in m.message: ev['kill']=tm
        if 'Landing at current' in m.message: ev['land']=tm
        if 'Takeoff' in m.message: ev['takeoff']=tm
    v.update(d=d,ev=ev)
def vline(ax,t,label,color,ls,ha='left',lim=None,inside=False):
    if lim and not (lim[0]<t<lim[1]): return
    ax.axvline(t,color=color,ls=ls,lw=1.3,zorder=1)
    if label and inside: ax.text(t,0.96,(' '+label) if ha=='left' else (label+' '),transform=ax.get_xaxis_transform(),color=color,fontsize=8.5,va='top',ha=ha,bbox=dict(fc=SURF,ec='none',pad=1))
    elif label: ax.text(t,1.0 if color==CRIT else 1.07,(' '+label) if ha=='left' else (label+' '),transform=ax.get_xaxis_transform(),color=color,fontsize=8.5,va='bottom',ha=ha,clip_on=False)
def markers(ax,v,labels=True,lim=None,inside=False):
    sw=v['sw']; kl=v['ev'].get('kill')
    vline(ax,sw,'EKF switch' if labels else None,CRIT,'--',ha='left',lim=lim,inside=inside)
    if kl is not None:
        vline(ax,kl,'kill' if labels else None,INK2,':',ha=('right' if kl<sw else 'left'),lim=lim,inside=inside)
        if kl>sw and labels and (lim is None or lim[0]<kl<lim[1]): pass

# Fig 1: heading per instance
fig,axs=plt.subplots(1,2,figsize=(12,4.2),sharey=False)
for ax,(k,v) in zip(axs,FL.items()):
    d=v['d']; a0,a1,va=d[('estimator_attitude',0)],d[('estimator_attitude',1)],d[('vehicle_attitude',0)]
    ax.plot(a0['t'],unwrap(yaw(a0)),color=B0,label='EKF0 (BMI088)')
    ax.plot(a1['t'],unwrap(yaw(a1)),color=B1,label='EKF1 (BMI270)')
    ax.plot(va['t'],unwrap(yaw(va)),color=INK,ls=(0,(4,2)),lw=1.3,label='Published (vehicle_attitude)')
    markers(ax,v); ax.set_title(v['name'],loc='left',pad=16); ax.set_xlabel('time since arm [s]'); ax.set_ylabel('yaw [deg] (unwrapped)')
    ax.set_xlim(0,va['t'].max())
axs[0].legend(loc='lower left')
fig.suptitle('Fig. 1 — Heading estimate per EKF instance: EKF1 drifts with no heading reference; published yaw jumps at switch',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.04)
fig.savefig(OUT+'fig1_heading_per_ekf.png'); plt.close(fig)

# Fig 2: per-IMU yaw rate + inconsistency vs threshold
fig,axs=plt.subplots(2,2,figsize=(12,6.4),sharex='col')
for c,(k,v) in enumerate(FL.items()):
    d=v['d']; ax=axs[0,c]
    for i,col,lab in [(0,B0,'IMU0 BMI088'),(1,B1,'IMU1 BMI270')]:
        vi=d[('vehicle_imu',i)]; r=np.degrees(vi['delta_angle[2]']/(vi['delta_angle_dt']*1e-6))
        rm=np.convolve(r,np.ones(10)/10,mode='same')
        ax.plot(vi['t'],r,color=col,lw=0.8,alpha=0.35)
        ax.plot(vi['t'][5:-5],rm[5:-5],color=col,label=lab+' (5 s mean)')
    ax.axhline(0,color=MUTED,lw=0.8); ax.set_ylim(-12,12); ax.set_ylabel('yaw rate [deg/s]')
    markers(ax,v); ax.set_title(v['name'],loc='left',pad=16)
    ax=axs[1,c]; si=d[('sensors_status_imu',0)]
    gi=np.degrees(np.abs(si['gyro_inconsistency_rad_s[1]']))
    ax.plot(si['t'],gi,color=INK2,lw=1.2,label='|gyro inconsistency| (sensors_status_imu)')
    ax.axhline(7,color=CRIT,lw=1.3,ls='-'); ax.text(si['t'].max()*0.02,7.2,'fault threshold EKF2_SEL_IMU_RAT = 7 deg/s',ha='left',va='bottom',color=CRIT,fontsize=8.5)
    ax.set_ylim(0,9); ax.set_ylabel('deg/s'); ax.set_xlabel('time since arm [s]'); markers(ax,v,labels=False)
axs[0,0].legend(loc='lower left'); axs[1,0].legend(loc='upper left',bbox_to_anchor=(0,0.78))
fig.suptitle('Fig. 2 — Per-IMU yaw rate: BMI270 reads ≈ −2 deg/s relative to BMI088 in flight, but stays far below the IMU-fault threshold',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.02)
fig.savefig(OUT+'fig2_imu_yaw_rate.png'); plt.close(fig)

# Fig 3: XY tracks
fig,axs=plt.subplots(1,2,figsize=(12,5.6))
for ax,(k,v) in zip(axs,FL.items()):
    d=v['d']; e0,e1,lp=d[('estimator_local_position',0)],d[('estimator_local_position',1)],d[('vehicle_local_position',0)]
    ax.plot(e0['y'],e0['x'],color=B0,label='EKF0 track'); ax.plot(e1['y'],e1['x'],color=B1,label='EKF1 track')
    i=np.argmin(abs(lp['t']-v['sw']))
    ax.plot(lp['y'][:i],lp['x'][:i],color=INK,ls=(0,(4,2)),lw=1.3,label='Published odometry')
    ax.plot(lp['y'][i:],lp['x'][i:],color=INK,ls=(0,(4,2)),lw=1.3)
    ax.annotate('',xy=(lp['y'][i],lp['x'][i]),xytext=(lp['y'][i-1],lp['x'][i-1]),arrowprops=dict(arrowstyle='->',color=CRIT,lw=2))
    jump=np.hypot(lp['x'][i]-lp['x'][i-1],lp['y'][i]-lp['y'][i-1])
    mx,my=(lp['x'][i]+lp['x'][i-1])/2,(lp['y'][i]+lp['y'][i-1])/2
    ax.text(my+(0.35 if k=='B' else 0),mx+(0 if k=='B' else 0.35),f'jump {jump:.1f} m at t = {v["sw"]:.1f} s',color=CRIT,fontsize=9,va='center',ha='left' if k=='B' else 'center',bbox=dict(fc=SURF,ec='none',pad=1.5))
    ax.plot(0,0,'o',color=INK,ms=8,mfc=SURF,mew=2); ax.text(0.12,-0.12,'start',fontsize=8.5,color=INK2,va='top')
    ax.set_aspect('equal','datalim'); ax.set_xlabel('East y [m]'); ax.set_ylabel('North x [m]'); ax.set_title(v['name'],loc='left')
axs[0].legend(loc='best')
fig.suptitle('Fig. 3 — Top-down position: the two instances integrate the same flow through different headings',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.0)
fig.savefig(OUT+'fig3_xy_tracks.png'); plt.close(fig)

# Fig 4: divergence
fig,axs=plt.subplots(2,1,figsize=(10,6),sharex=True)
for k,v in FL.items():
    d=v['d']; a0,a1=d[('estimator_attitude',0)],d[('estimator_attitude',1)]; e0,e1=d[('estimator_local_position',0)],d[('estimator_local_position',1)]
    tt=a0['t']; dy=unwrap(yaw(a1))-np.interp(tt,a0['t'],unwrap(yaw(a0))) if False else np.interp(tt,a1['t'],unwrap(yaw(a1)))-unwrap(yaw(a0))
    m=tt<v['sw']; ls='-' if k=='A' else (0,(5,2))
    axs[0].plot(tt[m],dy[m],color=INK2 if k=='B' else INK,ls=ls,label=v['name'])
    te=e0['t']; dxy=np.hypot(np.interp(te,e1['t'],e1['x'])-e0['x'],np.interp(te,e1['t'],e1['y'])-e0['y']); m=te<v['sw']
    axs[1].plot(te[m],dxy[m],color=INK2 if k=='B' else INK,ls=ls,label=v['name'])
    axs[1].plot(te[m][-1],dxy[m][-1],'o',color=CRIT,ms=8); axs[1].text(te[m][-1],dxy[m][-1],f'  {dxy[m][-1]:.1f} m at switch',color=CRIT,fontsize=9,va='center')
    axs[0].plot(tt[tt<v['sw']][-1],dy[tt<v['sw']][-1],'o',color=CRIT,ms=8); axs[0].text(tt[tt<v['sw']][-1],dy[tt<v['sw']][-1],f'  {dy[tt<v["sw"]][-1]:.0f}° at switch',color=CRIT,fontsize=9,va='center')
axs[0].set_ylabel('yaw(EKF1) − yaw(EKF0) [deg]'); axs[1].set_ylabel('|xy(EKF1) − xy(EKF0)| [m]'); axs[1].set_xlabel('time since arm [s]')
axs[0].legend(loc='lower left'); axs[0].set_title('Heading divergence',loc='left'); axs[1].set_title('Position divergence',loc='left')
fig.suptitle('Fig. 4 — Divergence between EKF instances grows steadily until the switch exposes it',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.01)
fig.tight_layout(); fig.savefig(OUT+'fig4_divergence.png'); plt.close(fig)

# Fig 5: switch zoom
for k,v in FL.items():
    d=v['d']; sw=v['sw']; lo,hi=(sw-2.5,sw+3.5)
    fig,axs=plt.subplots(5,1,figsize=(10,11),sharex=True)
    ss=d[('estimator_selector_status',0)]; m=(ss['t']>lo)&(ss['t']<hi)
    ax=axs[0]; ax.step(ss['t'][m],ss['combined_test_ratio[0]'][m],where='post',color=B0,label='EKF0'); ax.step(ss['t'][m],ss['combined_test_ratio[1]'][m],where='post',color=B1,label='EKF1')
    ax.set_ylabel('combined\ntest ratio'); ax.legend(loc='upper left'); ax.set_title('a) Selector input: innovation test ratio per instance',loc='left')
    ax=axs[1]; ax.step(ss['t'][m],ss['relative_test_ratio[1]'][m],where='post',color=B1,label='EKF1 relative score')
    ax.axhline(-0.5,color=CRIT,lw=1.2); ax.text(lo+0.05,-0.55,'≈ switch threshold (−0.5)',color=CRIT,fontsize=8.5,va='top')
    ax.set_ylim(-1.1,1.15); ax.set_ylabel('relative\ntest ratio'); ax.set_title('b) Selector score of EKF1 vs primary (+1 = worse, clamped ±1); resets to 0 after switch',loc='left')
    lp=d[('vehicle_local_position',0)]; tr=d[('trajectory_setpoint',0)]; ml=(lp['t']>lo)&(lp['t']<hi); mt=(tr['t']>lo)&(tr['t']<hi)
    ax=axs[2]; ax.plot(lp['t'][ml],lp['x'][ml],color=INK,label='x (North)'); ax.plot(lp['t'][ml],lp['y'][ml],color=INK2,ls=(0,(4,2)),label='y (East)')
    ax.plot(tr['t'][mt],tr['position[0]'][mt],'o',color=INK,ms=4,mfc=SURF,label='x setpoint'); ax.plot(tr['t'][mt],tr['position[1]'][mt],'s',color=INK2,ms=4,mfc=SURF,label='y setpoint')
    ax.set_ylabel('position [m]'); ax.legend(loc='center left',bbox_to_anchor=(1.0,0.5)); ax.set_title('c) Published odometry (vehicle_local_position) and position setpoint',loc='left')
    va=d[('vehicle_attitude',0)]; sp=d[('vehicle_attitude_setpoint',0)]; ma=(va['t']>lo)&(va['t']<hi); msp=(sp['t']>lo)&(sp['t']<hi)
    ax=axs[3]; ax.plot(va['t'][ma],yaw(va)[ma],color=INK,label='yaw estimate'); ax.plot(sp['t'][msp],np.degrees(sp['yaw_body'][msp]),color=INK2,ls=(0,(4,2)),lw=1.3,label='yaw setpoint')
    ax.set_ylabel('yaw [deg]'); ax.legend(loc='center left'); ax.set_title('d) Published heading and yaw setpoint',loc='left')
    av=d[('vehicle_angular_velocity',0)]; mv=(av['t']>lo)&(av['t']<hi)
    ax=axs[4]; ax.plot(av['t'][mv],np.degrees(av['xyz[2]'][mv]),color=INK,lw=1.2,label='measured yaw rate (BMI088)')
    vs=d[('vehicle_status',0)]; 
    for i in range(1,len(vs['t'])):
        if int(vs['nav_state'][i])!=int(vs['nav_state'][i-1]) and lo<vs['t'][i]<hi:
            for a in axs: a.axvline(vs['t'][i],color=MUTED,ls='-.',lw=1.1)
            axs[1].text(vs['t'][i],0.9,' AUTO_LAND' if int(vs['nav_state'][i])==18 else f' nav {int(vs["nav_state"][i])}',transform=axs[1].get_xaxis_transform(),color=INK2,fontsize=8.5,va='top')
    ax.set_ylabel('yaw rate [deg/s]'); ax.set_xlabel('time since arm [s]'); ax.legend(loc='lower right'); ax.set_title('e) Vehicle reaction: body yaw rate',loc='left')
    for i,a in enumerate(axs): markers(a,v,labels=(i==4),lim=(lo,hi),inside=True) ; a.set_xlim(lo,hi)
    fig.suptitle(f'Fig. 5{k.lower()} — {v["name"]}: the EKF instance switch and its effect (±3 s)',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.0)
    fig.tight_layout(); fig.savefig(OUT+f'fig5{k.lower()}_switch_zoom_{k}.png'); plt.close(fig)

# Fig 6: FIFO overflow timing vs switch
fig,axs=plt.subplots(2,1,figsize=(10,3.6),sharex=False)
for ax,(k,v) in zip(axs,FL.items()):
    d=v['d']
    for i,col,lab in [(0,B0,'BMI088 (IMU0) accel error count'),(1,B1,'BMI270 (IMU1) accel error count')]:
        s=d[('vehicle_imu_status',i)]; o=np.argsort(s['t']); ax.step(s['t'][o],s['accel_error_count'][o].astype(int),where='post',color=col,label=lab)
    markers(ax,v); ax.set_title(v['name'],loc='left',pad=14); ax.set_ylabel('count'); ax.set_ylim(-0.3,3.2); ax.set_xlim(0,d[('vehicle_imu_status',0)]['t'].max())
axs[1].set_xlabel('time since arm [s]'); axs[0].legend(loc='center left')
fig.suptitle('Fig. 6 — BMI088 accel FIFO overflows occur ≥ 28 s before the switch, on the healthy IMU: not the trigger',x=0.01,ha='left',fontsize=11.5,color=INK,y=1.04)
fig.tight_layout(); fig.savefig(OUT+'fig6_fifo_overflow_timing.png'); plt.close(fig)
print('ok')
