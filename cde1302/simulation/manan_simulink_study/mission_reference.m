function varargout = mission_reference(varargin)
%MISSION_REFERENCE Discrete position mission and a pure unit-test interface.
% ref=[p3;zeros6;yaw;0]; diagnostic=[stage;goal;done;failed;gate;entry;stable].
% stage:1takeoff 2hover 3sweep 4return 5settle 6descent 7endpoint 8done 9failed.
% The position input p is whatever the navigation system believes: simulated
% truth in the ideal Phase 1 baseline, the estimate in Phase 2 runs.
% Yaw: 'fixed' commands cfg.mission.yawRad; 'tangent' slews (at most
% maxYawRateRadps) to the planned leg heading. turnPauseMode='matched' holds
% the position command for the same planned turn time in both yaw modes.
if nargin>0 && (ischar(varargin{1}) || isstring(varargin{1}))
    switch char(varargin{1})
        case 'initial', varargout{1}=initialState(varargin{2});
        case 'step', [varargout{1:nargout}]=stepMission(varargin{2:end});
        otherwise, error('MananStudy:MissionDispatch','Unknown mission dispatch.');
    end
    return;
end
setup(varargin{1});
end

function setup(block)
block.NumDialogPrms=1; block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1; block.NumOutputPorts=2;
block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=3; block.InputPort(1).DatatypeID=0;
block.InputPort(1).Complexity='Real'; block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=11; block.OutputPort(2).Dimensions=7;
for k=1:2
    block.OutputPort(k).DatatypeID=0; block.OutputPort(k).Complexity='Real';
end
cfg=block.DialogPrm(1).Data; block.SampleTimes=[cfg.mission.tickS 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@postSetup);
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end

function postSetup(block)
block.NumDworks=1; block.Dwork(1).Name='missionState';
block.Dwork(1).Dimensions=16; block.Dwork(1).DatatypeID=0;
block.Dwork(1).Complexity='Real'; block.Dwork(1).UsedAsDiscState=true;
end

function initialize(block)
block.Dwork(1).Data=initialState(block.DialogPrm(1).Data);
end

function outputs(block)
[ref,diag]=stepMission(block.CurrentTime,block.InputPort(1).Data, ...
    block.Dwork(1).Data,block.DialogPrm(1).Data);
block.OutputPort(1).Data=ref; block.OutputPort(2).Data=diag;
end

function update(block)
[~,~,next]=stepMission(block.CurrentTime,block.InputPort(1).Data, ...
    block.Dwork(1).Data,block.DialogPrm(1).Data);
block.Dwork(1).Data=next;
end

function state=initialState(cfg)
validateattributes(cfg.mission.routeNed,{'double'},{'2d','ncols',3,'nonempty','finite'});
% 1 stage, 2 goal, 3 done, 4 failed, 5 stage entry time, 6 stable since,
% 7 last update time, 8 descent start altitude, 9 pending corner handoff,
% 10:12 commanded position, 13 commanded yaw (unwrapped), 14 pause until,
% 15 target leg heading (unwrapped), 16 leg whose turn has been scheduled.
assert(ismember(cfg.mission.guidanceMode,{'bounded_position','waypoints'}), ...
    'MananStudy:GuidanceMode','Unknown guidance mode.');
y0=cfg.mission.yawRad;
state=[1;1;0;0;0;nan;-inf;cfg.frames.homeNed(3)-cfg.mission.heightM;0; ...
    cfg.frames.homeNed(:);y0;-inf;y0;0];
end

function [ref,diag,next]=stepMission(t,p,state,cfg)
p=double(p(:)); next=state(:); m=cfg.mission; h=cfg.frames.homeNed(:);
cruise=[h(1:2);h(3)-m.heightM];
yawMode=optionalField(m,'yawMode','fixed');
pauseMode=optionalField(m,'turnPauseMode','none');
validP=numel(p)==3 && all(isfinite(p));
advance=~isfinite(t) || t>next(7)+1e-10;
if advance && next(1)~=8 && next(1)~=9
    if ~validP || ~isfinite(t) || t>=m.maxTimeS
        next=enterStage(next,9,t); next(4)=1;
    else
        switch next(1)
            case 1
                atHover=norm(p(1:2)-h(1:2))<=m.hoverRadiusM && ...
                    abs(p(3)-cruise(3))<=m.altitudeToleranceM;
                if atHover && t>=m.takeoffMinS, next=enterStage(next,2,t); end
            case 2
                stable=norm(p(1:2)-h(1:2))<=m.hoverRadiusM && ...
                    abs(p(3)-cruise(3))<=m.altitudeToleranceM;
                next=updateStable(next,stable,t);
                if dwellReached(next,t,m.hoverHoldS)
                    next=enterStage(next,3,t); next(2)=1;
                end
            case 3
                if next(9)==1
                    next(9)=0;
                    if next(2)<size(m.routeNed,1), next(2)=next(2)+1;
                    else, next=enterStage(next,4,t); end
                else
                    goal=m.routeNed(next(2),:).';
                    commandReached=true;
                    if strcmp(m.guidanceMode,'bounded_position')
                        commandReached=norm(state(10:12)-goal)<=1e-9;
                    end
                    if norm(p(1:2)-goal(1:2))<=max(0.05,m.gateRadiusM) && commandReached
                        next(9)=1;
                    end
                end
            case 4
                if norm(p(1:2)-h(1:2))<=max(0.05,m.gateRadiusM)
                    next=enterStage(next,5,t);
                end
            case 5
                next=updateStable(next,norm(p(1:2)-h(1:2))<=m.returnRadiusM,t);
                if dwellReached(next,t,m.returnHoldS) || t-next(5)>=m.returnMaxSettleS
                    next=enterStage(next,6,t); next(8)=cruise(3);
                end
            case 6
                if t-next(5)>=abs(h(3)-next(8))/m.descentRateMps
                    next=enterStage(next,7,t);
                end
            case 7
                stable=norm(p(1:2)-h(1:2))<=m.endpointRadiusM && ...
                    abs(p(3)-h(3))<=m.endpointAltitudeToleranceM;
                next=updateStable(next,stable,t);
                if dwellReached(next,t,m.endpointHoldS)
                    next=enterStage(next,8,t); next(3)=1;
                end
        end
    end
    next(7)=t;
end
goal=cruise;
if next(1)==3
    goal=m.routeNed(next(2),:).';
elseif next(1)==6
    % Only the commanded descent stops at the floor; plant truth is free.
    goal(3)=min(h(3),next(8)+m.descentRateMps*(t-next(5)));
elseif next(1)==7 || next(1)==8
    goal=h;
end
% A new horizontal leg schedules its turn once: leg k flies to corner k,
% leg N+1 returns home. The plan is fixed geometry, identical in both modes.
legKey=0;
if next(1)==3, legKey=next(2); elseif next(1)==4, legKey=size(m.routeNed,1)+1; end
if advance && isfinite(t) && legKey>0 && legKey~=state(16) && isfield(m,'turnPlan')
    next(16)=legKey;
    next(15)=m.turnPlan.headingUnwrapped(legKey);
    if strcmp(pauseMode,'matched'), next(14)=t+m.turnPlan.pauseS(legKey); end
end
paused=isfinite(t) && t<next(14)-1e-10;
command=goal;
if strcmp(m.guidanceMode,'bounded_position')
    command=state(10:12);
    if advance && isfinite(t) && validP
        dt=0;
        if isfinite(state(7)), dt=max(0,t-state(7)); end
        delta=goal(1:2)-command(1:2); distance=norm(delta);
        maxStep=m.maxReferenceSpeedMps*dt;
        if paused, maxStep=0; end
        if distance>0
            command(1:2)=command(1:2)+delta*min(1,maxStep/distance);
        end
        verticalStep=m.maxVerticalReferenceSpeedMps*dt;
        command(3)=command(3)+max(-verticalStep,min(verticalStep,goal(3)-command(3)));
    end
end
next(10:12)=command;
yawCommand=m.yawRad;
if strcmp(yawMode,'tangent')
    yawCommand=state(13);
    if advance && isfinite(t)
        dt=0;
        if isfinite(state(7)), dt=max(0,t-state(7)); end
        yawStep=m.maxYawRateRadps*dt;
        yawCommand=yawCommand+max(-yawStep,min(yawStep,next(15)-yawCommand));
    end
end
next(13)=yawCommand;
ref=[command;zeros(6,1);yawCommand;0];
gateError=nan; if validP, gateError=norm(p(1:2)-goal(1:2)); end
diag=[next(1:4);gateError;next(5:6)];
end

function value=optionalField(s,name,default)
if isfield(s,name), value=char(s.(name)); else, value=default; end
end
function state=enterStage(state,stage,t)
state(1)=stage; state(5)=t; state(6)=nan;
end
function state=updateStable(state,stable,t)
if ~stable, state(6)=nan; elseif isnan(state(6)), state(6)=t; end
end
function yes=dwellReached(state,t,duration)
yes=~isnan(state(6)) && t-state(6)>=duration-1e-10;
end
