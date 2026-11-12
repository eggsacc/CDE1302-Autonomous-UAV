function out = run_phase2_checks()
%RUN_PHASE2_CHECKS One-command Phase 2 verification for the MATLAB desktop.
%   out = run_phase2_checks()  runs, in order:
%     1. the three logic test suites (seconds)
%     2. a perfect-sensor run (wiring check, ~2 min)
%     3. realistic Method A (+B) and Method C runs, seed 1, 5 landmarks (~4 min)
%   then prints PASS/CHECK verdicts and a paste-friendly summary, saves the
%   runs to results/phase2_checks/runs.mat and comparison.png.
% "CHECK" means "paste this to Claude", not necessarily a bug.
root=fileparts(mfilename('fullpath')); addpath(root);
outDir=fullfile(root,'results','phase2_checks'); if ~isfolder(outDir), mkdir(outDir); end
verdicts={};
started=tic;
fprintf('\n===== 1. Logic tests (no Simulink) =====\n');
check_nav_core; check_postcorrect; check_mission_reference;

fprintf('\n===== 2. Perfect sensors: wiring check (about 2 min) =====\n');
ideal=run_nav_scenario(struct('idealSensors',true,'anchors','home_corners5','method','A'));
m=ideal.metrics;
verdicts(end+1,:)=v('ideal: mission completed',m.mission.completed==1,m.mission.completed);
verdicts(end+1,:)=v('ideal: airborne estimate error max < 0.01 m',m.estimation.maxM<0.01,m.estimation.maxM);
verdicts(end+1,:)=v('ideal: heading error max < 0.01 deg',m.heading.maxAbsDeg<0.01,m.heading.maxAbsDeg);
verdicts(end+1,:)=v('ideal: touchdown found',m.landing.touchdownFound,m.landing.touchdownFound);
verdicts(end+1,:)=v('ideal: true landing error at touchdown < 0.05 m',m.landing.trueHorizontalErrorM<0.05,m.landing.trueHorizontalErrorM);
verdicts(end+1,:)=v('ideal: all 3 targets confirmed',m.targets.inFlight.confirmed==3,m.targets.inFlight.confirmed);
verdicts(end+1,:)=v('ideal: target mean error < 0.01 m',m.targets.inFlight.meanErrorConfirmedM<0.01,m.targets.inFlight.meanErrorConfirmedM);

fprintf('\n===== 3. Realistic sensors: Method A (+B) and Method C (about 4 min) =====\n');
resA=run_nav_scenario(struct('anchors','home_corners5','method','A'));
resC=run_nav_scenario(struct('anchors','home_corners5','method','C'));
a=resA.metrics; c=resC.metrics;
verdicts(end+1,:)=v('A and C missions completed',a.mission.completed && c.mission.completed, ...
    [a.mission.completed c.mission.completed]);
verdicts(end+1,:)=v('A drifts: airborne estimate RMS > 0.03 m',a.estimation.rmsM>0.03,a.estimation.rmsM);
verdicts(end+1,:)=v('C estimate RMS lower than A',c.estimation.rmsM<a.estimation.rmsM,[a.estimation.rmsM c.estimation.rmsM]);
verdicts(end+1,:)=v('C heading RMS lower than A',c.heading.rmsDeg<a.heading.rmsDeg,[a.heading.rmsDeg c.heading.rmsDeg]);
verdicts(end+1,:)=v('B target error lower than A',a.targets.methodB.meanErrorConfirmedM<a.targets.inFlight.meanErrorConfirmedM, ...
    [a.targets.inFlight.meanErrorConfirmedM a.targets.methodB.meanErrorConfirmedM]);
verdicts(end+1,:)=v('C landing (touchdown) better than A',c.landing.trueHorizontalErrorM<a.landing.trueHorizontalErrorM, ...
    [a.landing.trueHorizontalErrorM c.landing.trueHorizontalErrorM]);
verdicts(end+1,:)=v('C rejects < 10 % of landmark sightings',c.anchors.sightingsRejected<0.1*max(1,c.anchors.sightingsTotal), ...
    [c.anchors.sightingsRejected c.anchors.sightingsTotal]);
verdicts(end+1,:)=v('C max tilt < 15 deg (correction jerk)',c.mission.maxTiltDeg<15,c.mission.maxTiltDeg);

fprintf('\n===== VERDICTS (paste this block) =====\n');
for k=1:size(verdicts,1)
    fprintf('%-6s %-52s %s\n',verdicts{k,2},verdicts{k,1},verdicts{k,3});
end
fprintf('\n===== SUMMARY (paste this block too) =====\n');
print_nav_summary(ideal,resA,resC);
fprintf('total wall clock %.1f min\n',toc(started)/60);
out=struct('ideal',ideal,'resA',resA,'resC',resC,'verdicts',{verdicts});
save(fullfile(outDir,'runs.mat'),'ideal','resA','resC','verdicts','-v7.3');
try
    fig=plot_nav_comparison({ideal,resA,resC},{'ideal A','A (+B)','C'}, ...
        fullfile(outDir,'comparison.png'));
    fprintf('saved %s\n',fullfile(outDir,'comparison.png'));
    figure(fig);
catch err
    fprintf('plot skipped: %s\n',err.message);
end
end

function row=v(name,ok,value)
if ok, s='PASS'; else, s='CHECK'; end
row={name,s,mat2str(round(double(value),4))};
end
