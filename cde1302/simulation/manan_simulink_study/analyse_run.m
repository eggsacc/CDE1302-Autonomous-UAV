function [metrics,checks,figures] = analyse_run(result)
%ANALYSE_RUN Time-weighted errors for the lecturer ideal-feedback baseline.
t=result.time(:); p=result.position; v=result.velocity;
e=result.euler; r=result.reference; d=result.diagnostics;
c=result.config; n=numel(t); tol=1e-8;
assert(n>=2 && all(isfinite(t)) && all(diff(t)>0),'Need increasing finite time.');
assert(isequal(size(p),[n 3]) && isequal(size(v),[n 3]) && ...
    isequal(size(e),[n 3]) && isequal(size(r),[n 11]) && ...
    isequal(size(d),[n 7]),'Signals must align with truth timestamps.');
home=c.frames.homeNed(:).'; ha=c.frames.homeArena(:).';
a=c.frames.arenaToLocalYaw; R=[cos(a) -sin(a);sin(a) cos(a)];
pa=(p(:,1:2)-home(1:2))*R+ha;
ra=(r(:,1:2)-home(1:2))*R+ha;
route=[home(1:2);c.mission.routeNed(:,1:2);home(1:2)];
routeA=[ha;c.mission.routeArena;ha];
err=p-r(:,1:3); exy=vecnorm(err(:,1:2),2,2); e3=vecnorm(err,2,2);
h=home(3)-p(:,3); hr=home(3)-r(:,3);
eyaw=atan2(sin(e(:,3)-r(:,10)),cos(e(:,3)-r(:,10)));
speed=vecnorm(v(:,1:2),2,2); speed3=vecnorm(v,2,2);
near=inf(n,1); active=nan(n,1);
for k=1:size(route,1)-1
    near=min(near,segmentDistance(p(:,1:2),route(k,:),route(k+1,:)));
    if k<size(route,1)-1, idx=d(:,1)==3 & d(:,2)==k;
    else, idx=d(:,1)==4; end
    active(idx)=segmentDistance(p(idx,1:2),route(k,:),route(k+1,:));
end
margin=boundaryMargin(pa,c.geometry.outerBounds);
refMargin=boundaryMargin(ra,c.geometry.outerBounds);
penetration=max(p(:,3)-home(3),0);
names={'TAKEOFF','HOVER','SWEEP','RETURN','HOME_SETTLE','DESCEND','ENDPOINT_HOLD','DONE','FAILED'};
metrics.geometryLabel=char(c.geometry.label);
metrics.controller='Lecturer RPT simulation; not PX4 firmware or EKF2';
metrics.feedback='Ideal feedback equals simulated truth; no sensor estimator';
metrics.interpretation='Setpoint error differs from route distance. No contact, motor shutdown or touchdown model.';
metrics.durationS=t(end)-t(1); metrics.sampleCount=n;
metrics.maxOutputStepS=max(diff(t));
metrics.rmsSetpointErrorXYM=sqrt(wmean(t,exy.^2,true(n,1)));
metrics.maxSetpointErrorXYM=max(exy);
metrics.rmsSetpointError3DM=sqrt(wmean(t,e3.^2,true(n,1)));
metrics.maxSetpointError3DM=max(e3);
metrics.meanSetpointErrorNedM=trapz(t,err,1)/metrics.durationS;
metrics.rmsNearestRouteDistanceM=sqrt(wmean(t,near.^2,true(n,1)));
metrics.maxNearestRouteDistanceM=max(near);
metrics.rmsYawErrorDeg=rad2deg(sqrt(wmean(t,eyaw.^2,true(n,1))));
metrics.maxAbsYawErrorDeg=rad2deg(max(abs(eyaw)));
metrics.maxTiltDeg=rad2deg(max(acos(max(-1,min(1,cos(e(:,1)).*cos(e(:,2)))))));
metrics.minimumHorizontalNoseProjection=min(abs(cos(e(:,2))));
metrics.yawInterpretation='Euler yaw; inspect tilt and near-vertical nose projection before attributing heading excursions to independent yaw drift.';
metrics.rmsHeightTrackingErrorM=sqrt(wmean(t,(h-hr).^2,true(n,1)));
metrics.minimumHeightM=min(h); metrics.maximumHeightM=max(h);
metrics.meanSpeedXYMps=wmean(t,speed,true(n,1));
metrics.maxSpeedXYMps=max(speed); metrics.maxSpeed3DMps=max(speed3);
metrics.horizontalDistanceFlownM=trapz(t,speed);
metrics.maxGroundPlanePenetrationM=max(penetration);
metrics.minTrueCentreBoundaryMarginM=min(margin);
metrics.minCommandCentreBoundaryMarginM=min(refMargin);
metrics.initialPositionErrorFromHomeM=norm(p(1,:)-home);
metrics.initialSpeedMps=norm(v(1,:));
metrics.initialYawErrorDeg=rad2deg(atan2(sin(e(1,3)-c.mission.yawRad),cos(e(1,3)-c.mission.yawRad)));
for k=1:numel(names)
    idx=d(:,1)==k; dt=diff(t);
    s.sampleCount=nnz(idx); s.durationS=sum(dt(idx(1:end-1)));
    s.rmsSetpointErrorXYM=sqrt(wmean(t,exy.^2,idx));
    s.maxSetpointErrorXYM=maskedMax(exy,idx);
    s.rmsSetpointError3DM=sqrt(wmean(t,e3.^2,idx));
    s.rmsActiveLegDistanceM=sqrt(wmean(t,active.^2,idx & isfinite(active)));
    s.maxActiveLegDistanceM=maskedMax(active,idx & isfinite(active));
    s.rmsHeightTrackingErrorM=sqrt(wmean(t,(h-hr).^2,idx));
    s.rmsYawErrorDeg=rad2deg(sqrt(wmean(t,eyaw.^2,idx)));
    s.meanSpeedXYMps=wmean(t,speed,idx);
    metrics.stages.(names{k})=s;
end
ri=find(d(:,1)==5,1,'last');
metrics.returnHold.available=~isempty(ri);
metrics.returnHold.lastTimeS=NaN; metrics.returnHold.horizontalErrorM=NaN;
metrics.returnHold.horizontalSpeedMps=NaN;
metrics.returnHold.settleSatisfied=false;
metrics.returnHold.settleTimedOut=false;
metrics.returnHold.exitReason='Return hold not observed';
if ~isempty(ri)
    metrics.returnHold.lastTimeS=t(ri);
    metrics.returnHold.horizontalErrorM=norm(p(ri,1:2)-home(1:2));
    metrics.returnHold.horizontalSpeedMps=speed(ri);
    descentStart=find(d(:,1)==6,1,'first');
    metrics.returnHold.exitReason='No descent transition observed';
    if ~isempty(descentStart)
        stableSince=d(ri,7);
        metrics.returnHold.settleSatisfied=isfinite(stableSince) && ...
            t(descentStart)-stableSince>=c.mission.returnHoldS-tol;
        metrics.returnHold.settleTimedOut=~metrics.returnHold.settleSatisfied && ...
            t(descentStart)-d(ri,6)>=c.mission.returnMaxSettleS-c.mission.tickS-tol;
        if metrics.returnHold.settleSatisfied
            metrics.returnHold.exitReason='Stable return gate';
        elseif metrics.returnHold.settleTimedOut
            metrics.returnHold.exitReason='Settle timeout; descent proceeded without stable return';
        end
    end
end
metrics.endpoint.timeS=t(end);
metrics.endpoint.horizontalErrorFromHomeM=norm(p(end,1:2)-home(1:2));
metrics.endpoint.positionErrorFromHome3DM=norm(p(end,:)-home);
metrics.endpoint.heightM=h(end); metrics.endpoint.verticalSpeedDownMps=v(end,3);
metrics.endpoint.horizontalSpeedMps=speed(end); metrics.endpoint.speed3DMps=speed3(end);
metrics.endpoint.insideConfiguredStartZone=NaN;
if all(isfinite(c.geometry.startBounds),'all')
    metrics.endpoint.insideConfiguredStartZone=boundaryMargin(pa(end,:),c.geometry.startBounds)>=-tol;
end
metrics.endpoint.interpretation='Descent endpoint only; not physical touchdown or PX4 autonomous landing';
checks.numericalToleranceM=tol;
checks.integrity.finiteStateAndReference=all(isfinite([p v e r]),'all');
checks.integrity.finiteRequiredDiagnostics=all(isfinite(d(:,1:6)),'all');
checks.integrity.strictlyIncreasingTime=true;
checks.integrity.initialPositionMatchesHome=metrics.initialPositionErrorFromHomeM<=tol;
checks.reference.zeroVelocityAccelerationFeedforward=max(abs(r(:,4:9)),[],'all')<=tol;
checks.reference.fixedYaw=max(abs(atan2(sin(r(:,10)-c.mission.yawRad),cos(r(:,10)-c.mission.yawRad))))<=tol;
checks.reference.zeroYawRateFeedforward=max(abs(r(:,11)))<=tol;
checks.reference.heightWithinConfiguredRange=min(hr)>=-tol && max(hr)<=c.mission.heightM+tol;
checks.geometry.boundaryCheckApplicable=c.geometry.boundaryComplianceApplicable;
checks.geometry.commandCentreInsideOuterBoundary=NaN;
checks.geometry.trueCentreInsideOuterBoundary=NaN;
if c.geometry.boundaryComplianceApplicable
    checks.geometry.commandCentreInsideOuterBoundary=min(refMargin)>=-tol;
    checks.geometry.trueCentreInsideOuterBoundary=min(margin)>=-tol;
end
checks.geometry.endpointInsideConfiguredStartZone=metrics.endpoint.insideConfiguredStartZone;
checks.geometry.noGroundPlanePenetration=max(penetration)<=tol;
checks.mission.completed=logical(d(end,3)) && d(end,1)==8;
checks.mission.failed=any(d(:,4)~=0) || any(d(:,1)==9);
checks.mission.stoppedBeforeCompletion=~checks.mission.completed;
checks.mission.finalStageCode=d(end,1); checks.mission.returnHoldObserved=~isempty(ri);
checks.mission.returnSettleSatisfied=metrics.returnHold.settleSatisfied;
checks.mission.returnSettleTimedOut=metrics.returnHold.settleTimedOut;
checks.mission.endpointWithinConfiguredPositionGate=norm(p(end,:)-home)<=c.mission.gateRadiusM;
checks.scope.trackingPassThresholdSpecified=false;
checks.scope.groundContactModelImplemented=false;
checks.scope.officialGeometryVerified=false;
checks.scope.physicalFlightOrPx4Validated=false;
checks.scope.note='Scope fields describe limitations, not failed software tests. Boundary checks concern centre position, not rotor clearance.';
out=char(result.outputDir); if ~isfolder(out), mkdir(out); end
writeJson(fullfile(out,'metrics.json'),metrics); writeJson(fullfile(out,'checks.json'),checks);
values=[t p v e r p d err exy e3 near active margin penetration];
columns={'time_s','true_n_m','true_e_m','true_d_m','true_vn_mps','true_ve_mps','true_vd_mps', ...
    'true_roll_rad','true_pitch_rad','true_yaw_rad','ref_n_m','ref_e_m','ref_d_m', ...
    'ref_vn_mps','ref_ve_mps','ref_vd_mps','ref_an_mps2','ref_ae_mps2','ref_ad_mps2', ...
    'ref_yaw_rad','ref_yawrate_radps','ideal_feedback_n_m','ideal_feedback_e_m','ideal_feedback_d_m', ...
    'stage','goal_index','mission_complete','mission_failed','gate_error_m','stage_entry_s','stable_since_s', ...
    'error_n_m','error_e_m','error_d_m','error_xy_m','error_3d_m','nearest_route_distance_m', ...
    'active_leg_distance_m','true_centre_boundary_margin_m','ground_plane_penetration_m'};
writetable(array2table(values,'VariableNames',columns),fullfile(out,'trajectory.csv'));
figures=gobjects(3,1);
figures(1)=figure('Visible','off','Color','w','Position',[100 100 1150 800]);
ax=axes(figures(1)); hold(ax,'on');
drawBounds(ax,c.geometry.outerBounds,[.15 .15 .15],'-','Outer boundary');
drawBounds(ax,c.geometry.searchBounds,[.55 .25 .65],'--','Configured search');
drawBounds(ax,c.geometry.startBounds,[.15 .55 .25],'-','Configured start/endpoint zone');
plot(ax,routeA(:,1),routeA(:,2),'--','Color',[.55 .55 .55],'DisplayName','Planned centreline');
plot(ax,ra(:,1),ra(:,2),':','Color',[.85 .4 .05],'LineWidth',1.3,'DisplayName','Position commands (joined for display)');
plot(ax,pa(:,1),pa(:,2),'b-','LineWidth',1.5,'DisplayName','True position = ideal feedback');
plot(ax,pa(1,1),pa(1,2),'ko','MarkerFaceColor','k','DisplayName','Initial position');
plot(ax,pa(end,1),pa(end,2),'rs','MarkerFaceColor','r','DisplayName','Final endpoint');
axis(ax,'equal'); grid(ax,'on');
xlabel(ax,'Arena x (m); configured direction, not geographic North'); ylabel(ax,'Arena y (m)');
title(ax,{char(c.geometry.label),'Lecturer RPT, not PX4; endpoint is not validated touchdown'},'Interpreter','none');
legend(ax,'Location','eastoutside','Interpreter','none');
lightFigure(figures(1));
exportgraphics(figures(1),fullfile(out,'arena_route.png'),'Resolution',160);
figures(2)=figure('Visible','off','Color','w','Position',[100 100 1150 850]);
lay=tiledlayout(figures(2),3,1,'TileSpacing','compact');
ax=nexttile(lay); plot(ax,t,h,'b-',t,hr,'r--','LineWidth',1.1); hold(ax,'on');
yline(ax,0,':k','Ground plane; no contact physics'); ylabel(ax,'Height above home (m)');
legend(ax,'True height','Commanded height','Location','best'); grid(ax,'on');
ax=nexttile(lay); plot(ax,t,rad2deg(e(:,3)),'b-',t,rad2deg(r(:,10)),'r--','LineWidth',1.1);
ylabel(ax,'Yaw (deg)'); legend(ax,'True yaw','Commanded yaw','Location','best'); grid(ax,'on');
ax=nexttile(lay); plot(ax,t,speed,'b-','LineWidth',1.1); hold(ax,'on');
plot(ax,t,speed3,'--','Color',[.4 .4 .4],'LineWidth',1.1);
ylabel(ax,'True speed (m/s)'); xlabel(ax,'Simulation time (s)');
legend(ax,'Horizontal','3D','Location','best'); grid(ax,'on');
title(lay,{char(c.geometry.label),'Lecturer RPT; position commands, zero velocity/acceleration feedforward'},'Interpreter','none');
lightFigure(figures(2));
exportgraphics(figures(2),fullfile(out,'state_history.png'),'Resolution',160);
figures(3)=figure('Visible','off','Color','w','Position',[100 100 1150 950]);
lay=tiledlayout(figures(3),4,1,'TileSpacing','compact');
ax=nexttile(lay); plot(ax,t,exy,'b-',t,e3,'r--','LineWidth',1.1);
ylabel(ax,'Setpoint error (m)'); legend(ax,'Horizontal','3D','Location','best'); grid(ax,'on');
ax=nexttile(lay); plot(ax,t,near,':','Color',[.4 .4 .4],'LineWidth',1.1); hold(ax,'on');
plot(ax,t,active,'b-','LineWidth',1.1); ylabel(ax,'Route distance (m)');
legend(ax,'Nearest complete route','Active horizontal leg','Location','best'); grid(ax,'on');
ax=nexttile(lay); plot(ax,t,err,'LineWidth',1); ylabel(ax,'True - command (m)');
legend(ax,'Local N','Local E','Local D','Location','best'); grid(ax,'on');
ax=nexttile(lay); stairs(ax,t,d(:,1),'k-','LineWidth',1.1);
yticks(ax,1:9); yticklabels(ax,names); ylim(ax,[.5 9.5]); grid(ax,'on');
set(ax,'TickLabelInterpreter','none'); % Keep stage names like ENDPOINT_HOLD literal.
ylabel(ax,'Stage'); xlabel(ax,'Simulation time (s)');
title(lay,{char(c.geometry.label),'Lecturer RPT; errors use simulated truth, not measured flight truth'},'Interpreter','none');
lightFigure(figures(3));
exportgraphics(figures(3),fullfile(out,'tracking_errors.png'),'Resolution',160);
if nargout<3, close(figures); end
end

function lightFigure(fig)
% Explicit export colours are independent of the user's desktop theme.
set(fig,'Color','w');
ax=findall(fig,'Type','axes');
set(ax,'Color','w','XColor',[.12 .12 .12],'YColor',[.12 .12 .12], ...
    'GridColor',[.65 .65 .65],'FontSize',10);
txt=findall(fig,'Type','text'); set(txt,'Color',[.1 .1 .1]);
% Tiled-layout titles are separate objects and otherwise export light grey.
lays=findall(fig,'Type','tiledlayout');
for L=lays(:).'
    if ~isempty(L.Title), L.Title.Color=[.1 .1 .1]; end
    if ~isempty(L.Subtitle), L.Subtitle.Color=[.1 .1 .1]; end
end
leg=findall(fig,'Type','legend');
set(leg,'Color','w','TextColor',[.1 .1 .1],'EdgeColor',[.5 .5 .5]);
end

function value=wmean(t,x,mask)
use=mask(1:end-1) & mask(2:end); dt=diff(t);
if ~any(use) || any(~isfinite(x(mask))), value=NaN; return; end
mid=(x(1:end-1)+x(2:end))/2;
value=sum(mid(use).*dt(use))/sum(dt(use));
end
function value=maskedMax(x,mask)
if any(mask), value=max(x(mask)); else, value=NaN; end
end
function margin=boundaryMargin(xy,b)
margin=min([xy(:,1)-b(1,1),b(1,2)-xy(:,1),xy(:,2)-b(2,1),b(2,2)-xy(:,2)],[],2);
end
function d=segmentDistance(xy,a,b)
ab=b-a; denominator=sum(ab.^2);
if denominator<=eps, d=vecnorm(xy-a,2,2); return; end
u=max(0,min(1,((xy-a)*ab.')/denominator));
d=vecnorm(xy-(a+u.*ab),2,2);
end
function drawBounds(ax,b,colour,style,name)
if any(~isfinite(b),'all'), return; end
plot(ax,b(1,[1 2 2 1 1]),b(2,[1 1 2 2 1]),'Color',colour, ...
    'LineStyle',style,'LineWidth',1.4,'DisplayName',name);
end
function writeJson(file,value)
fid=fopen(file,'w','n','UTF-8'); assert(fid>=0,'Cannot write %s.',file);
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
