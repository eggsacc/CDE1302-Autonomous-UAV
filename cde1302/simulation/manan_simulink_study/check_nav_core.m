function report=check_nav_core()
%CHECK_NAV_CORE Hand-checkable tests of the sensor/estimator core.
% These drive nav_core with synthetic truth; they are NOT vehicle simulations.
checked={};
root=fileparts(mfilename('fullpath')); addpath(root);

% 1. Ideal sensors reproduce truth exactly, including while turning.
cfg=scenario_config(struct('idealSensors',true,'anchors','none'));
truth=@(t) syntheticTruth(t,[0.3;-0.1],0.4,0.8);   % constant world velocity, turning
out=runCore(cfg,truth,20);
assert(max(abs(out.dpos(:)))<1e-9 && max(abs(out.dpsi))<1e-9);
checked{end+1}='ideal sensors: estimate equals truth while translating and turning';

% 2. Camera geometry: footprint size from the team intrinsics.
q=[1 0 0 0]; p=[0;0;-0.8];
poly=nav_core('footprint',p,q,cfg);
width=max(poly(2,:))-min(poly(2,:)); len=max(poly(1,:))-min(poly(1,:));
assert(abs(width-2*0.8*160/163.5)<1e-9 && abs(len-2*0.8*120/153.2)<1e-9);
assert(abs(mean(poly(1,:))-0.08)<1e-9);
checked{end+1}=sprintf(['footprint at 0.8 m is %.3f m across x %.3f m along, ' ...
    'centred 0.08 m ahead (fx 163.5, fy 153.2, 320x240)'],width,len);
assert(nav_core('visible',p,q,[0.1;0.1;0],0,0.112,cfg));
assert(~nav_core('visible',p,q,[0;0.75;0],0,0.112,cfg));   % edge clips the tag
assert(~nav_core('visible',p,q,[0;2;0],0,0.112,cfg));
checked{end+1}='tags under the camera are visible; tags past the image edge are not';

% 3. Latency and frame pose: a sighting arrives 0.13 s after capture.
cfgA=scenario_config(struct('idealSensors',true,'anchors',[29 0.6 0.6]));
cfgA.nav.cam.pDetect=1;
hover=@(t) struct('pos',[0;0;-0.8],'vel',[0;0;0],'quat',[1 0 0 0],'rates',[0;0;0]);
out=runCore(cfgA,hover,1);
anchorSlot=numel(cfgA.targets.id)+1;
arrivals=out.t(any(isfinite(out.sight(:,anchorSlot,1)),2));
tc=out.sight(any(isfinite(out.sight(:,anchorSlot,1)),2),anchorSlot,1);
assert(~isempty(arrivals) && all(arrivals-tc>=0.13-1e-9) && all(arrivals-tc<0.13+0.02+1e-9));
assert(max(abs(out.sight(isfinite(out.sight(:,anchorSlot,3)),anchorSlot,3)))<1e-9);
checked{end+1}='sightings arrive 0.13-0.15 s after capture and use the capture pose';

% 4. Yaw-rate gate: frames taken while turning at 40 deg/s detect nothing.
spin=@(t) syntheticTruth(t,[0;0],deg2rad(40),0.8);
out=runCore(cfgA,spin,3);
tcap=out.sight(:,:,1); tcap=tcap(isfinite(tcap));
% The t=0 frame has no rate history yet; every later frame must be rejected.
assert(all(tcap==0) && out.diag(end,11)==out.diag(end,12)-1);
checked{end+1}='team 30 deg/s yaw gate rejects every frame while turning at 40 deg/s';

% 5. Method C removes drift that Method A keeps (flow bias, hover over anchor).
biasA=cfgA; biasA.nav.err.flowBias=[0.05;0]; biasA.nav.method='A';
biasC=biasA; biasC.nav.method='C';
outA=runCore(biasA,hover,20); outC=runCore(biasC,hover,20);
errA=norm(outA.dpos(end,1:2)); errC=norm(outC.dpos(end,1:2));
assert(errA>0.9 && errC<0.1*errA);
checked{end+1}=sprintf('0.05 m/s flow bias, 20 s hover: Method A error %.2f m, Method C %.3f m',errA,errC);

% 6. Delayed corrections are not double counted: no overshoot after a step error.
stepC=biasC; stepC.nav.err.flowBias=[0;0]; stepC.nav.err.initialHeading=0;
outS=runCoreWithOffset(stepC,hover,6,[0.30;0]);
e=outS.dpos(:,1);
assert(abs(e(end))<0.01 && min(e)>-0.02);
checked{end+1}='0.30 m initial error converges without overshoot (frames in flight are shifted)';

% 7. Outlier gating: a gross sighting is rejected by the chi-square gate.
outC2=biasC; outC2.nav.err.flowBias=[0;0];
outC2.nav.cam.outlierProb=0.02;   % idealSensors had switched outliers off
outC2.nav.noise.outlier(:)=1; outC2.nav.noise.outlier(30,anchorSlot)=0; % frame 29 only
outC2.nav.noise.outlierVec(30,anchorSlot)=0.5;
o=runCore(outC2,hover,5);
st=o.sight(:,anchorSlot,2); st=st(isfinite(st));
assert(any(st==2) && nnz(st==2)==1 && max(abs(o.dpos(:)))<0.01);
checked{end+1}='a single 0.5 m outlier is rejected (status 2) and does not move the estimate';

% 8. Anchor orientation corrects the initial heading error (align-to-home-tag).
head=biasC; head.nav.err.flowBias=[0;0]; head.nav.err.initialHeading=deg2rad(3);
head.nav.initialHeadingStdDeg=2; % filter's prior; the ideal config would claim 0.5 deg
oh=runCore(head,hover,5);
assert(abs(rad2deg(oh.dpsi(1)))>2.9 && abs(rad2deg(oh.dpsi(end)))<0.3);
checked{end+1}=sprintf('3 deg heading error over the home tag falls to %.2f deg in 5 s',rad2deg(oh.dpsi(end)));

% 9. Determinism and pairing.
c1=scenario_config(struct('seed',4,'anchors','none'));
c2=scenario_config(struct('seed',4,'anchors','home_corners_centre6','yawMode','tangent','method','C'));
assert(isequal(c1.targets.ned,c2.targets.ned) && isequal(c1.nav.err,c2.nav.err));
assert(isequal(c1.nav.noise.flow,c2.nav.noise.flow) && ...
    isequal(c1.nav.noise.detect(:,1:3),c2.nav.noise.detect(:,1:3)));
o1=runCore(scenario_config(struct('seed',4)),truth,5);
o2=runCore(scenario_config(struct('seed',4)),truth,5);
assert(isequaln(o1.dpos,o2.dpos));
checked{end+1}='same seed: identical targets, sensor errors and noise across layouts, yaw and method';

report=struct('passed',true,'count',numel(checked),'checks',{checked}, ...
    'scope','Sensor/estimator logic with synthetic truth; not Simulink plant performance');
fprintf('Navigation core checks: %d passed (logic only).\n',report.count);
end

function s=syntheticTruth(t,v,yawRate,height)
yaw=yawRate*t;
s.pos=[v*t;-height]; s.vel=[v;0];
s.quat=[cos(yaw/2) 0 0 sin(yaw/2)]; s.rates=[0;0;yawRate];
end

function out=runCore(cfg,truthFcn,T)
out=runCoreWithOffset(cfg,truthFcn,T,[0;0]);
end

function out=runCoreWithOffset(cfg,truthFcn,T,offset)
dt=cfg.nav.dt; ts=(0:dt:T).'; n=numel(ts);
x=nav_core('initial',cfg); L=nav_core('layout',cfg);
out.t=ts; out.dpos=nan(n,3); out.dpsi=nan(n,1); out.diag=nan(n,L.diagLen);
out.sight=nan(n,max(1,L.nS),L.F);
for k=1:n
    s=truthFcn(ts(k));
    [dp,~,qM,dg,sg,x]=nav_core('step',ts(k),s.pos,s.vel,s.quat,s.rates,x,cfg);
    if k==1 && any(offset)
        % Known initial offset, and a prior that admits it (else it is gated).
        x(1:2)=x(1:2)+offset; dp(1:2)=dp(1:2)+offset;
        P=reshape(x(4:12),3,3); P(1:2,1:2)=norm(offset)^2*eye(2); x(4:12)=P(:);
    end
    out.dpos(k,:)=dp.'; M=reshape(qM,4,4); out.dpsi(k)=2*atan2(M(4,1),M(1,1));
    out.diag(k,:)=dg.'; out.sight(k,:,:)=reshape(reshape(sg,L.F,[]).',1,[],L.F);
end
end
