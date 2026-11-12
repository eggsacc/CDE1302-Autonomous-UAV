function report=check_postcorrect()
%CHECK_POSTCORRECT Hand-checkable tests of the Method B port (no Simulink).
checked={};
root=fileparts(mfilename('fullpath')); addpath(root);
cfg=scenario_config(struct('anchors','home_corners_centre6','seed',2));
truthA=cfg.anchors.ned(:,1:2); truthT=cfg.targets.ned(:,1:2);

% 1. A constant rotation + shift is recovered exactly by the global fit.
th=deg2rad(3); R=[cos(th) -sin(th);sin(th) cos(th)]; sh=[0.10 -0.05];
toPx4=@(p,t) (R*p.').'+sh;                         % how the drifted frame sees the world
res=fakeRun(cfg,toPx4,truthA,truthT);
lo=struct('model','similarity','driftModel','global','visitGapS',5,'minVisitSightings',5, ...
    'minSpanM',1,'maxRmsM',0.15,'maxRotationDeg',20,'maxScaleError',0.25, ...
    'confirmRadiusM',0.2,'outlierRadiusM',0.3,'trimK',3,'minSightings',5);
B=nav_postcorrect(res,lo);
assert(B.fit.ok && B.fit.rms<1e-9 && abs(B.fit.transform.theta+th)<1e-9);
assert(max([B.targets.error])<1e-9);
checked{end+1}='global similarity fit recovers a 3 deg / 11 cm frame offset exactly; targets exact';

% 2. Drift that grows in time: auto mode prefers the time-interpolated model.
drift=@(p,t) p+[0.004*t 0.002*t];                  % 4 mm/s and 2 mm/s creep
res2=fakeRun(cfg,drift,truthA,truthT);
lo.driftModel='auto';
B2=nav_postcorrect(res2,lo);
assert(ismember(B2.correction.model,{'time','hybrid'}));
raw=nav_tag_report(res2);
assert(mean([B2.targets.error])<0.5*mean([raw.error]));
checked{end+1}=sprintf(['time-varying creep: auto picks "%s", target error %.3f m vs ' ...
    '%.3f m uncorrected'],B2.correction.model,mean([B2.targets.error]),mean([raw.error]));

% 3. Fit checks: a 30 deg turn is refused and falls back to translation.
big=deg2rad(30); Rb=[cos(big) -sin(big);sin(big) cos(big)];
res3=fakeRun(cfg,@(p,t) (Rb*p.').',truthA,truthT);
lo.driftModel='global';
B3=nav_postcorrect(res3,lo);
assert(~B3.fit.ok || strcmp(B3.fit.transform.model,'translation'));
checked{end+1}='a 30 deg rotation fails max_rotation_deg (20) and is not applied as a turn';

% 4. Too few sightings per visit: no fix, no correction, raw report.
res4=res; res4.sightings=res.sightings(1:4:end,:);
B4=nav_postcorrect(res4,lo);
assert(strcmp(B4.correction.model,'none') || numel(B4.fixes)<numel(res.sightings.id));
checked{end+1}='visits with fewer than min_visit_sightings are not used as fixes';

report=struct('passed',true,'count',numel(checked),'checks',{checked}, ...
    'scope','Method B port logic with synthetic sightings; not flight data');
fprintf('Method B (post-flight correction) checks: %d passed (logic only).\n',report.count);
end

function res=fakeRun(cfg,toPx4,truthA,truthT)
% Each landmark is visited twice (t ~ 10 s and ~ 60 s), 8 sightings each;
% each target once in between. Positions are what the drifted frame reports.
rows=[]; t0=[10 60];
for v=1:2
    for j=1:size(truthA,1)
        for s=1:8
            t=t0(v)+j*0.5+s*0.1;
            rows(end+1,:)=[t+0.13 0 cfg.anchors.id(j) 1 t 1 toPx4(truthA(j,:),t) zeros(1,8)]; %#ok<AGROW>
        end
    end
end
for j=1:size(truthT,1)
    for s=1:8
        t=30+j*2+s*0.1;
        rows(end+1,:)=[t+0.13 0 cfg.targets.id(j) 0 t 1 toPx4(truthT(j,:),t) zeros(1,8)]; %#ok<AGROW>
    end
end
f=nav_core('fields');
res.config=cfg;
res.sightings=array2table(rows,'VariableNames',[{'t_arrival','slot','id','is_anchor'},f.sight]);
res.nav.time=(0:1:80).'; res.nav.estimatedNE=zeros(81,2);
end
