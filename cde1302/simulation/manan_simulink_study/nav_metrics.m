function M = nav_metrics(res)
%NAV_METRICS Numbers that answer the Phase 2/3 questions for one run.
% Truth is used here ONLY to score the run after it has finished.
%   estimation  |estimate - truth| horizontally, heading error
%   tracking    truth vs command (where the drone really went) and
%               estimate vs command (what the controller thought)
%   targets     reported target errors (in flight; Method B if present)
%   landing     true horizontal distance from home at the end
%   anchors     share of camera frames with a landmark in view / used
cfg=res.config; t=res.time; p=res.position; r=res.reference; d=res.diagnostics;
home=cfg.frames.homeNed(:).'; h=home(3)-p(:,3);
air=h>cfg.nav.cam.minHeightM;
est=res.estimatedPosition;
eEst=vecnorm(est(:,1:2)-p(:,1:2),2,2);
psiTrue=unwrap(res.euler(:,3));
psiHat=interp1(res.nav.time,res.nav.psiHat,t,'previous','extrap');
eYaw=rad2deg(atan2(sin(psiHat-psiTrue),cos(psiHat-psiTrue)));
sweep=d(:,1)==3;
M.scenario=cfg.scenario.description;
M.method=cfg.scenario.method; M.yawMode=cfg.mission.yawMode; M.seed=cfg.scenario.seed;
M.anchorLayout=cfg.scenario.anchorLayout; M.anchorCount=numel(cfg.anchors.id);
% Completion from the native mission log (the 100 Hz truth grid can miss the
% final DONE tick; that wrongly marked 2 of 12 Phase 3 runs incomplete).
dn=d; if isfield(res,'diagnosticsNative'), dn=res.diagnosticsNative; end
M.mission.completed=dn(end,1)==8 && dn(end,3)==1;
M.mission.failed=any(dn(:,4)~=0);
M.mission.durationS=t(end)-t(1);
M.mission.maxTiltDeg=rad2deg(max(acos(max(-1,min(1,cos(res.euler(:,1)).*cos(res.euler(:,2)))))));
M.mission.maxSpeedMps=max(vecnorm(res.velocity(:,1:2),2,2));
M.estimation.rmsM=wrms(t,eEst,air);
M.estimation.maxM=maxOf(eEst,air);
M.estimation.endM=eEst(end);
M.heading.rmsDeg=wrms(t,eYaw,air);
M.heading.maxAbsDeg=maxOf(abs(eYaw),air);
M.heading.endDeg=eYaw(end);
% Yaw behaviour (Phase 3): does the drone follow its yaw command, and how
% much does it actually turn? Commanded yaw is unwrapped by the mission.
yawCmd=r(:,10); dtv=max(diff(t),eps);
eYawCmd=rad2deg(atan2(sin(psiTrue-yawCmd),cos(psiTrue-yawCmd)));
rate=[0;rad2deg(diff(psiTrue))./dtv];
M.yaw.mode=cfg.mission.yawMode;
M.yaw.rmsTrueVsCommandDeg=wrms(t,eYawCmd,air);
M.yaw.maxAbsTrueVsCommandDeg=maxOf(abs(eYawCmd),air);
M.yaw.maxTrueRateDegps=maxOf(abs(rate),air);
M.yaw.totalRotationDeg=rad2deg(sum(abs(diff(psiTrue))));
M.yaw.netRotationDeg=rad2deg(psiTrue(end)-psiTrue(1));
M.yaw.timeTurningS=sum(dtv(abs(rate(2:end))>5 & air(2:end)));
M.yaw.limitDegps=rad2deg(cfg.mission.maxYawRateRadps);
M.yaw.cameraGateDegps=cfg.nav.cam.maxYawRateDegps;
eTrue=vecnorm(p(:,1:2)-r(:,1:2),2,2);
eView=vecnorm(est(:,1:2)-r(:,1:2),2,2);
M.tracking.rmsTrueVsCommandSweepM=wrms(t,eTrue,sweep);
M.tracking.rmsEstimateVsCommandSweepM=wrms(t,eView,sweep);
M.tracking.maxTrueVsCommandSweepM=maxOf(eTrue,sweep);
route=[home(1:2);cfg.mission.routeNed(:,1:2);home(1:2)];
dRoute=inf(numel(t),1);
for k=1:size(route,1)-1, dRoute=min(dRoute,segDist(p(:,1:2),route(k,:),route(k+1,:))); end
M.tracking.rmsTrueDistanceToPlannedRouteSweepM=wrms(t,dRoute,sweep);
M.tracking.maxTrueDistanceToPlannedRouteSweepM=maxOf(dRoute,sweep);
ri=find(d(:,1)==5,1,'last');
M.landing.returnErrorBeforeDescentM=NaN;
if ~isempty(ri), M.landing.returnErrorBeforeDescentM=norm(p(ri,1:2)-home(1:2)); end
% Touchdown = first sample in DESCEND or later with true height <= 2 cm.
% After that the lecturer model has no floor and the drone floats with no
% horizontal flow feedback, so later motion is a model artefact.
td=find(d(:,1)>=6 & h<=0.02,1,'first');
if isempty(td), td=numel(t); M.landing.touchdownFound=false; else, M.landing.touchdownFound=true; end
M.landing.touchdownTimeS=t(td);
M.landing.trueHorizontalErrorM=norm(p(td,1:2)-home(1:2));
M.landing.estimatedHorizontalErrorM=norm(est(td,1:2)-home(1:2));
M.landing.endTrueHorizontalErrorM=norm(p(end,1:2)-home(1:2));
M.landing.slideAfterTouchdownM=norm(p(end,1:2)-p(td,1:2));
M.landing.interpretation=['Scored at touchdown (true height <= 2 cm). No contact or PX4 LAND; ' ...
    'motion after touchdown is a no-floor artefact. The drone lands where its ESTIMATE says home is.'];
M.targets.inFlight=targetSummary(res.tags);
if isfield(res,'methodB')
    M.targets.methodB=targetSummary(res.methodB.targets);
    M.methodB.correction=res.methodB.description;
    M.methodB.fixes=numel(res.methodB.fixes);
    tB=res.methodB.trajectoryTime; neB=res.methodB.trajectoryNE;
    pAtB=interp1(t,p(:,1:2),tB,'linear','extrap');
    airB=interp1(t,double(air),tB,'previous','extrap')>0.5;
    eB=vecnorm(neB-pAtB,2,2);
    M.methodB.correctedTrajectoryRmsM=wrms(tB,eB,airB);
    M.methodB.interpretation='Post-flight only: corrects the report, not where the drone flew or landed.';
end
nd=res.nav.data(end,:);
framesTotal=nd(12);
M.anchors.framesTotal=framesTotal;
M.anchors.fractionFramesAnchorVisible=ratio(nd(13),framesTotal);
M.anchors.fractionFramesAnchorUsed=ratio(nd(14),framesTotal);
M.anchors.sightingsTotal=nd(15);
M.anchors.sightingsUsed=nd(9);
M.anchors.sightingsRejected=nd(10);
M.camera.fractionFramesYawGated=ratio(nd(11),framesTotal);
M.camera.targetSightingsTotal=nd(16);
M.note=['Simulation with assumed sensor errors (scenario_config); lecturer RPT ' ...
    'controller, not PX4; study EKF, not EKF2. Not validated against flight logs.'];
end

function s=targetSummary(T)
s.count=numel(T);
s.confirmed=nnz([T.confirmed]);
s.seen=nnz([T.nSightings]>0);
err=[T.error]; ok=[T.confirmed];
s.errorsM=err;
s.meanErrorConfirmedM=mean(err(ok)); if ~any(ok), s.meanErrorConfirmedM=NaN; end
s.maxErrorConfirmedM=max([err(ok) NaN]);
if any(ok), s.maxErrorConfirmedM=max(err(ok)); end
end

function v=wrms(t,x,mask)
mask=mask(:); x=x(:); use=mask(1:end-1)&mask(2:end); dt=diff(t(:));
if ~any(use), v=NaN; return; end
mid=(x(1:end-1).^2+x(2:end).^2)/2;
v=sqrt(sum(mid(use).*dt(use))/sum(dt(use)));
end
function v=maxOf(x,mask)
if any(mask), v=max(x(mask)); else, v=NaN; end
end
function r=ratio(a,b)
if b>0, r=a/b; else, r=NaN; end
end
function dd=segDist(xy,a,b)
ab=b-a; den=sum(ab.^2);
if den<=eps, dd=vecnorm(xy-a,2,2); return; end
u=max(0,min(1,((xy-a)*ab.')/den)); dd=vecnorm(xy-(a+u.*ab),2,2);
end
