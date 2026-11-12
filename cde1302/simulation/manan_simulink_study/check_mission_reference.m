function report=check_mission_reference()
%CHECK_MISSION_REFERENCE Causal mission checks, not vehicle performance tests.
cfg=study_config('demo'); checked={};
s=mission_reference('initial',cfg); assert(s(1)==1 && s(3)==0);
checked{end+1}='initial state is incomplete takeoff';
team=study_config('team_head_placeholder');
assert(isequal(team.mission.routeArena,[0 -1.35;4.8 -1.35;4.8 1;0 1]));
checked{end+1}='pinned team rectangle alternates lanes in order';
try
    study_config('assessment');
    error('MananStudy:TestMissingRefusal','Unresolved assessment accepted.');
catch err
    assert(strcmp(err.identifier,'MananStudy:OfficialGeometryUnresolved'));
end
checked{end+1}='unresolved assessment geometry is refused';
gateCfg=cfg; gateCfg.mission.guidanceMode='waypoints';
s(1)=3; s(2)=1; s(7)=0; goal=cfg.mission.routeNed(1,:).';
[~,d,sOutside]=mission_reference('step',0.05,goal+[0.151;0;0],s,gateCfg);
assert(d(2)==1 && sOutside(9)==0);
[~,d,sInside]=mission_reference('step',0.05,goal+[0.149;0;0],s,gateCfg);
assert(d(2)==1 && sInside(9)==1);
checked{end+1}='0.151 m waits and 0.149 m accepts the 0.15 m gate';
[~,~,repeated]=mission_reference('step',0.05,cfg.mission.routeNed(2,:).',sInside,gateCfg);
assert(isequaln(repeated,sInside));
[~,d]=mission_reference('step',0.10,goal,sInside,gateCfg); assert(d(2)==2);
checked{end+1}='same-time evaluations do not advance; next tick hands off';
cfgYaw=cfg; cfgYaw.mission.yawRad=0.7;
r=mission_reference('step',0.05,goal,s,cfgYaw);
assert(all(r(4:9)==0) && r(10)==0.7 && r(11)==0);
checked{end+1}='fixed yaw with zero v/a/rate feedforward';
shifted=gateCfg; shifted.frames.homeNed=[2;-3;0.4];
shifted.mission.routeNed=cfg.mission.routeNed+[2 -3 0.4];
s=mission_reference('initial',shifted); s(1)=4;
r=mission_reference('step',1,[4;4;-0.1],s,shifted);
assert(norm(r(1:3)-[2;-3;-0.1])<1e-12);
checked{end+1}='return preserves displaced home and NED altitude';
% This ideal follower verifies stage logic only; it is not the UAV simulation.
s=mission_reference('initial',cfg); p=cfg.frames.homeNed; stages=[];
previousCommand=p; previousTime=0; visitedGoals=false(size(cfg.mission.routeNed,1),1);
for t=0:cfg.mission.tickS:cfg.mission.maxTimeS
    [r,d,s]=mission_reference('step',t,p,s,cfg);
    dt=t-previousTime;
    assert(norm(r(1:2)-previousCommand(1:2))<=cfg.mission.maxReferenceSpeedMps*dt+1e-10);
    assert(abs(r(3)-previousCommand(3))<=cfg.mission.maxVerticalReferenceSpeedMps*dt+1e-10);
    for g=1:numel(visitedGoals)
        if norm(r(1:3)-cfg.mission.routeNed(g,:).')<1e-9, visitedGoals(g)=true; end
    end
    stages(end+1)=d(1); p=r(1:3); %#ok<AGROW>
    previousCommand=r(1:3); previousTime=t;
    if d(3) || d(4), break; end
end
assert(d(3)==1 && d(4)==0 && all(ismember(1:8,stages)));
assert(all(visitedGoals));
checked{end+1}='all stages complete with a declared ideal follower';
checked{end+1}='horizontal and vertical reference slew bounds at every tick';
checked{end+1}='bounded reference reaches every planned corner';
s=mission_reference('initial',cfg);
[~,d]=mission_reference('step',cfg.mission.maxTimeS,cfg.frames.homeNed,s,cfg);
assert(d(3)==0 && d(4)==1 && d(1)==9);
checked{end+1}='timeout is failure';
% A 90-degree rotation has an independently hand-checkable first goal.
rotated=study_config('demo',struct('homeNed',[2;-3;.4], ...
    'arenaToLocalYaw',pi/2));
assert(norm(rotated.mission.routeNed(1,:)-[2.15 -2.4 -.1])<1e-12);
R=[0 -1;1 0];
recovered=R.'*(rotated.mission.routeNed(:,1:2).'-[2;-3])+rotated.frames.homeArena;
assert(max(abs(recovered.'-rotated.mission.routeArena),[],'all')<1e-12);
checked{end+1}='production route transform with 90-degree rotation and displaced origin';
% ---- Phase 3 yaw modes (logic only) ----
fixedCfg=study_config('demo',[],struct('heightM',0.8,'laneSpacingM',1.0, ...
    'maxReferenceSpeedMps',0.3,'turnPauseMode','matched','yawMode','fixed'));
tangentCfg=study_config('demo',[],struct('heightM',0.8,'laneSpacingM',1.0, ...
    'maxReferenceSpeedMps',0.3,'turnPauseMode','matched','yawMode','tangent'));
plan=tangentCfg.mission.turnPlan;
assert(all(abs(plan.deltaRad)<=pi+1e-12));
% Leg 1 is the diagonal from home, leg 2 runs along lane 1, leg 3 is the first
% lane-end cross-over, which must be a 90-degree turn either way.
assert(abs(abs(plan.deltaRad(3))-pi/2)<1e-12);
checked{end+1}='turn plan uses shortest turns (|turn| <= 180 deg; lane ends are 90 deg)';
[cmdF,yawF,stF]=idealRun(fixedCfg); [cmdT,yawT,stT]=idealRun(tangentCfg);
assert(isequal(size(cmdF),size(cmdT)) && max(abs(cmdF-cmdT),[],'all')<1e-12);
assert(isequal(stF,stT));
checked{end+1}='matched pauses give identical position commands and stages in both yaw modes';
assert(all(yawF==fixedCfg.mission.yawRad));
checked{end+1}='fixed mode keeps the commanded yaw constant';
dt=tangentCfg.mission.tickS;
assert(max(abs(diff(yawT)))<=tangentCfg.mission.maxYawRateRadps*dt+1e-12);
checked{end+1}='tangent yaw command never exceeds the yaw-rate limit per tick';
% While the command moves along leg k, tangent yaw equals that leg's heading.
moving=[false; vecnorm(diff(cmdT(:,1:2)),2,2)>1e-9];
legHeading=nan(size(yawT));
legHeading(stT(:,1)==3)=plan.headingUnwrapped(stT(stT(:,1)==3,2));
legHeading(stT(:,1)==4)=plan.headingUnwrapped(end);
check=moving & isfinite(legHeading);
assert(any(check) && max(abs(yawT(check)-legHeading(check)))<1e-9);
checked{end+1}='tangent: the nose is on the leg heading whenever the command moves';
plain=study_config('demo',[],struct('yawMode','tangent','turnPauseMode','none'));
assert(plain.mission.turnPlan.totalPauseS==0 && ...
    contains(plain.mission.turnPlan.strictTangentConflict,'unbounded'));
checked{end+1}='strict-tangent corner conflict is reported, no pause without matched mode';
report=struct('passed',true,'count',numel(checked),'checks',{checked}, ...
    'scope','Mission logic only; not Simulink plant performance');
fprintf('Mission reference checks: %d passed (logic only).\n',report.count);
end

function [command,yaw,stage]=idealRun(cfg)
% Declared ideal follower: position equals the command. Logic test only.
s=mission_reference('initial',cfg); p=cfg.frames.homeNed;
n=floor(cfg.mission.maxTimeS/cfg.mission.tickS)+1;
command=nan(n,3); yaw=nan(n,1); stage=nan(n,2); k=0;
for t=0:cfg.mission.tickS:cfg.mission.maxTimeS
    [r,d,s]=mission_reference('step',t,p,s,cfg); k=k+1;
    command(k,:)=r(1:3).'; yaw(k)=r(10); stage(k,:)=d(1:2).'; p=r(1:3);
    if d(3) || d(4), break; end
end
assert(d(3)==1 && d(4)==0,'Ideal follower did not complete the mission.');
command=command(1:k,:); yaw=yaw(1:k); stage=stage(1:k,:);
end
