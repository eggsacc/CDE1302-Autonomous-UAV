function cfg = study_config(preset,frameOverrides,missionOverrides)
%STUDY_CONFIG Reproducible settings; demo geometry is not the official map.
%   cfg = study_config(preset)                      Phase 1 defaults.
%   cfg = study_config(preset,frameOverrides)       explicit arena/local frame.
%   cfg = study_config(preset,frameOverrides,missionOverrides) changes flight
%   parameters (height, lane spacing, speed, yaw mode...) BEFORE the route
%   and turn plan are built. Phase 1 behaviour is unchanged when omitted.
if nargin<1, preset='demo'; end
if nargin<2 || isempty(frameOverrides), frameOverrides=struct(); end
if nargin<3 || isempty(missionOverrides), missionOverrides=struct(); end
preset = char(preset);
cfg.root = fileparts(mfilename('fullpath'));
cfg.preset = preset;
cfg.provenance.teamCommit = 'e7a231eff2f9239ff0d6948a058cab19b4b5757c';
cfg.provenance.controller = 'Lecturer RPT simulation; not PX4 firmware or EKF2';
cfg.geometry.officialResolved = false;
cfg.frames.homeNed = [0; 0; 0];
cfg.frames.arenaToLocalYaw = 0;
cfg.mission.heightM = 0.5;
cfg.mission.yawRad = 0;
cfg.mission.gateRadiusM = 0.15;
cfg.mission.tickS = 0.05;
cfg.mission.maxTimeS = 360;
cfg.mission.nominalSpeedMps = 0.15; % Metadata, NOT achieved speed control.
cfg.mission.guidanceMode='bounded_position';
cfg.mission.maxReferenceSpeedMps=0.15;
cfg.mission.maxVerticalReferenceSpeedMps=0.15;
cfg.mission.takeoffMinS = 2;
cfg.mission.hoverHoldS = 2;
cfg.mission.hoverRadiusM = 0.10;
cfg.mission.altitudeToleranceM = 0.05;
cfg.mission.returnRadiusM = 0.05;
cfg.mission.returnHoldS = 1;
cfg.mission.returnMaxSettleS = 10;
cfg.mission.descentRateMps = 0.10; % Assumed simulation-only landing reference.
cfg.mission.endpointAltitudeToleranceM = 0.02;
cfg.mission.endpointRadiusM = 0.05;
cfg.mission.endpointHoldS = 2;
% Yaw (Phase 3). 'fixed' holds yawRad; 'tangent' points the nose along the
% planned leg. 0.6 rad/s is the offline default of Manan's yaw_test_lab5
% uploader, not a measured airframe limit. Zero yaw-rate feedforward is kept.
cfg.mission.yawMode = 'fixed';
cfg.mission.maxYawRateRadps = 0.6;
% 'none' reproduces Phase 1. 'matched' stops the position command at the start
% of every leg for |turn|/maxYawRate + turnSettleS in BOTH yaw modes, so fixed
% and tangent runs receive identical position commands.
cfg.mission.turnPauseMode = 'none';
cfg.mission.turnSettleS = 0.5;
switch preset
    case 'demo'
        cfg.geometry.label = 'ASSUMED DEMO GEOMETRY; OFFICIAL OFFSETS UNVERIFIED';
        cfg.geometry.outerBounds = [0 5.4; 0 4.5];
        cfg.geometry.searchBounds = [1.2 4.8; 0.45 4.05];
        cfg.geometry.startBounds = [0.15 1.05; 0.15 1.05];
        cfg.geometry.boundaryComplianceApplicable = true;
        cfg.frames.homeArena = [0.6; 0.6];
        cfg.mission.laneSpacingM = 0.6; % Not a calibrated camera swath.
    case 'team_head_placeholder'
        cfg.geometry.label = 'PINNED TEAM PLACEHOLDER; OFFICIAL ARENA UNRESOLVED';
        cfg.geometry.outerBounds = nan(2);
        cfg.geometry.startBounds = nan(2);
        cfg.geometry.searchBounds = [0 4.8; -1.35 1.0];
        cfg.geometry.boundaryComplianceApplicable = false;
        cfg.frames.homeArena = [0; 0];
        cfg.mission.laneSpacingM = 2.5;
    case 'assessment'
        error('MananStudy:OfficialGeometryUnresolved', ...
            'Assessment refused: authoritative arena geometry/frame is unresolved.');
    otherwise
        error('MananStudy:UnknownPreset', 'Unknown study preset: %s', preset);
end
% Apply explicit frame configuration before transforming the actual route.
allowed={'homeNed','homeArena','arenaToLocalYaw'};
fields=fieldnames(frameOverrides);
for k=1:numel(fields)
    assert(ismember(fields{k},allowed),'MananStudy:FrameField','Unknown frame field.');
    cfg.frames.(fields{k})=frameOverrides.(fields{k});
end
allowedMission={'heightM','laneSpacingM','maxReferenceSpeedMps', ...
    'maxVerticalReferenceSpeedMps','nominalSpeedMps','yawMode','yawRad', ...
    'maxYawRateRadps','turnPauseMode','turnSettleS','maxTimeS', ...
    'gateRadiusM','descentRateMps','guidanceMode'};
missionFields=fieldnames(missionOverrides);
for k=1:numel(missionFields)
    assert(ismember(missionFields{k},allowedMission),'MananStudy:MissionField', ...
        'Unknown mission override: %s',missionFields{k});
    cfg.mission.(missionFields{k})=missionOverrides.(missionFields{k});
end
cfg.mission.yawMode=char(cfg.mission.yawMode);
cfg.mission.turnPauseMode=char(cfg.mission.turnPauseMode);
assert(ismember(cfg.mission.yawMode,{'fixed','tangent'}),'MananStudy:YawMode', ...
    'yawMode must be fixed or tangent.');
assert(ismember(cfg.mission.turnPauseMode,{'none','matched'}),'MananStudy:TurnPause', ...
    'turnPauseMode must be none or matched.');
validateattributes(cfg.mission.maxYawRateRadps,{'double'},{'scalar','positive','finite'});
validateattributes(cfg.mission.heightM,{'double'},{'scalar','positive','finite'});
validateattributes(cfg.mission.laneSpacingM,{'double'},{'scalar','positive','finite'});
validateattributes(cfg.frames.homeNed,{'double'},{'vector','numel',3,'finite'});
validateattributes(cfg.frames.homeArena,{'double'},{'vector','numel',2,'finite'});
validateattributes(cfg.frames.arenaToLocalYaw,{'double'},{'scalar','finite'});
cfg.frames.homeNed=cfg.frames.homeNed(:);
cfg.frames.homeArena=cfg.frames.homeArena(:);
% Alternating lanes as mission/planner.py. Home return is a separate stage.
b = cfg.geometry.searchBounds;
nGaps = ceil((b(2,2)-b(2,1))/cfg.mission.laneSpacingM);
lanes = linspace(b(2,1),b(2,2),max(1,nGaps)+1);
cfg.mission.routeArena = zeros(2*numel(lanes),2);
for k=1:numel(lanes)
    ends = [b(1,1),lanes(k);b(1,2),lanes(k)];
    if mod(k,2)==0, ends=flipud(ends); end
    cfg.mission.routeArena(2*k-1:2*k,:) = ends;
end
cfg.mission.routeArena = round(cfg.mission.routeArena,3);
c=cos(cfg.frames.arenaToLocalYaw); s=sin(cfg.frames.arenaToLocalYaw);
R=[c -s;s c];
xy=(R*(cfg.mission.routeArena.'-cfg.frames.homeArena)).';
xy=xy+cfg.frames.homeNed(1:2).';
cfg.mission.routeNed=[xy,repmat(cfg.frames.homeNed(3)- ...
    cfg.mission.heightM,size(xy,1),1)];
cfg.mission.turnPlan=turn_plan(cfg);
if ~isempty(missionFields) && ~isfield(missionOverrides,'maxTimeS')
    % Overridden flights get a cap well above their planned duration.
    cfg.mission.maxTimeS=max(cfg.mission.maxTimeS, ...
        ceil(1.5*cfg.mission.turnPlan.plannedDurationS+60));
end
cfg.mission.routeDescription='Sweep goals only; home return is separate.';
cfg.mission.governorDescription=['Study adaptation: bound the moving position ' ...
    'reference, zero v/a feedforward, retain corner goals and arrival gate.'];
cfg.mission.landingDescription=['Reference descent/hold only; no ground ' ...
    'contact, motor disarm or PX4 LAND implementation.'];
end

function plan=turn_plan(cfg)
%TURN_PLAN Leg headings, shortest unwrapped turns and matched pauses.
% Leg k (k <= N) flies to sweep corner k; leg N+1 returns home. Heading is
% atan2(East, North) of the leg in local NED, i.e. the strict tangent.
m=cfg.mission; h=cfg.frames.homeNed(:).';
pts=[h(1:2); m.routeNed(:,1:2); h(1:2)];
n=size(pts,1)-1;
plan.legHeadingRad=zeros(n,1); plan.headingUnwrapped=zeros(n,1);
plan.deltaRad=zeros(n,1); plan.pauseS=zeros(n,1); plan.legLengthM=zeros(n,1);
previous=m.yawRad;
for k=1:n
    d=pts(k+1,:)-pts(k,:); plan.legLengthM(k)=norm(d);
    if plan.legLengthM(k)<1e-9
        heading=previous;
    else
        heading=atan2(d(2),d(1));
    end
    delta=atan2(sin(heading-previous),cos(heading-previous)); % shortest turn
    plan.legHeadingRad(k)=heading;
    plan.deltaRad(k)=delta;
    plan.headingUnwrapped(k)=previous+delta;
    previous=plan.headingUnwrapped(k);
    if strcmp(m.turnPauseMode,'matched') && abs(delta)>1e-9
        plan.pauseS(k)=abs(delta)/m.maxYawRateRadps+m.turnSettleS;
    end
end
plan.totalPauseS=sum(plan.pauseS);
plan.totalTurnRad=sum(abs(plan.deltaRad));
plan.maxCornerTurnDeg=rad2deg(max(abs(plan.deltaRad)));
plan.cornerTurnTimeAtLimitS=abs(plan.deltaRad)/m.maxYawRateRadps;
plan.strictTangentConflict=sprintf(['Strict tangent yaw needs an instantaneous ' ...
    'heading change at %d leg junctions (largest %.1f deg), i.e. an unbounded ' ...
    'yaw rate at the commanded speed. At the %.2f rad/s (%.1f deg/s) limit ' ...
    'these turns take %.1f s in total. Path and limits are NOT altered; with ' ...
    'turnPauseMode=matched the position command stops for each turn in both ' ...
    'yaw modes.'],nnz(abs(plan.deltaRad)>1e-9),plan.maxCornerTurnDeg, ...
    m.maxYawRateRadps,rad2deg(m.maxYawRateRadps),sum(plan.cornerTurnTimeAtLimitS));
height=m.heightM;
plan.plannedDurationS=height/m.maxVerticalReferenceSpeedMps+m.takeoffMinS+ ...
    m.hoverHoldS+5+sum(plan.legLengthM)/m.maxReferenceSpeedMps+ ...
    plan.totalPauseS+m.returnHoldS+height/m.descentRateMps+m.endpointHoldS+10;
end
