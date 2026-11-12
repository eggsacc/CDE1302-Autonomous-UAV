function out = run_phase3_yaw(varargin)
%RUN_PHASE3_YAW Paired fixed vs tangent yaw study (desktop; ~35 min default).
%   out = run_phase3_yaw()  runs, for seeds 1-3 and Methods A and C, the same
%   mission with fixed and with tangent yaw (12 runs), plus one sensitivity
%   pair (2 runs): Method A with a 5 cm optical-flow sensor offset that PX4
%   is assumed NOT to compensate. Prints a paste-friendly summary and saves
%   results/phase3_yaw/summary.csv, seed1_runs.mat and yaw_seed1.png.
%
% Pairing: same seed => same targets, sensor errors, noise, path, speed and
% (matched) turn pauses. Only the commanded heading differs. No yaw-dependent
% error is injected; differences come from the plant, the team's 30 deg/s
% camera gate, camera geometry and the assumed gyro/flow error models.
p=inputParser;
p.addParameter('seeds',1:3);
p.addParameter('methods',{'A','C'});
p.addParameter('layout','home_corners5');
p.addParameter('leverArmCase',true);
p.addParameter('leverArmM',[0.05;0]);
p.parse(varargin{:}); o=p.Results;
root=fileparts(mfilename('fullpath')); addpath(root);
outDir=fullfile(root,'results','phase3_yaw'); if ~isfolder(outDir), mkdir(outDir); end
yaws={'fixed','tangent'}; rows=[]; seed1={}; seed1Labels={}; started=tic;
nTotal=numel(o.seeds)*numel(o.methods)*2+2*o.leverArmCase; k=0;
for seed=o.seeds
    for m=o.methods
        for y=yaws
            k=k+1;
            fprintf('[%d/%d %.0f min] seed %d method %s %s yaw\n',k,nTotal,toc(started)/60,seed,m{1},y{1});
            res=run_nav_scenario(struct('seed',seed,'method',m{1},'yawMode',y{1}, ...
                'anchors',o.layout),'quiet',true);
            rows=[rows;row(res,'baseline')]; %#ok<AGROW>
            if seed==o.seeds(1)
                seed1{end+1}=res; seed1Labels{end+1}=sprintf('%s %s',y{1},m{1}); %#ok<AGROW>
            end
        end
    end
end
if o.leverArmCase
    for y=yaws
        k=k+1;
        fprintf('[%d/%d %.0f min] sensitivity: uncompensated %.0f cm flow offset, method A, %s yaw\n', ...
            k,nTotal,toc(started)/60,100*norm(o.leverArmM),y{1});
        res=run_nav_scenario(struct('seed',o.seeds(1),'method','A','yawMode',y{1}, ...
            'anchors',o.layout,'sensor',struct('flowLeverErrM',o.leverArmM)),'quiet',true);
        rows=[rows;row(res,sprintf('flow_offset_%gcm',100*norm(o.leverArmM)))]; %#ok<AGROW>
    end
end
T=struct2table(rows);
writetable(T,fullfile(outDir,'summary.csv'));
save(fullfile(outDir,'seed1_runs.mat'),'seed1','seed1Labels','-v7.3');
report(T,seed1{1}.config);
fprintf('total wall clock %.1f min | saved %s\n',toc(started)/60,outDir);
out=struct('table',T,'seed1',{seed1},'labels',{seed1Labels});
try
    plotYaw(seed1,seed1Labels,fullfile(outDir,'yaw_seed1.png'));
    plot_nav_comparison(seed1,seed1Labels,fullfile(outDir,'comparison_seed1.png'));
catch err
    fprintf('plots skipped: %s\n',err.message);
end
end

function r=row(res,caseName)
m=res.metrics; c=res.config;
r.scenario_case=string(caseName); r.seed=c.scenario.seed; r.method=string(c.scenario.method);
r.yaw=string(c.mission.yawMode); r.completed=m.mission.completed;
r.duration_s=m.mission.durationS;
r.est_rms_m=m.estimation.rmsM; r.est_max_m=m.estimation.maxM;
r.heading_rms_deg=m.heading.rmsDeg; r.heading_max_deg=m.heading.maxAbsDeg;
r.route_dev_rms_m=m.tracking.rmsTrueDistanceToPlannedRouteSweepM;
r.track_true_vs_cmd_rms_m=m.tracking.rmsTrueVsCommandSweepM;
r.landing_err_m=m.landing.trueHorizontalErrorM;
r.targets_confirmed=m.targets.inFlight.confirmed;
r.target_mean_err_m=m.targets.inFlight.meanErrorConfirmedM;
r.target_sightings=m.camera.targetSightingsTotal;
r.landmark_frames_used=m.anchors.fractionFramesAnchorUsed;
r.yaw_gated_frac=m.camera.fractionFramesYawGated;
r.yaw_track_rms_deg=m.yaw.rmsTrueVsCommandDeg;
r.yaw_track_max_deg=m.yaw.maxAbsTrueVsCommandDeg;
r.max_yaw_rate_degps=m.yaw.maxTrueRateDegps;
r.total_rotation_deg=m.yaw.totalRotationDeg;
r.turning_s=m.yaw.timeTurningS;
r.max_tilt_deg=m.mission.maxTiltDeg;
r.methodB_target_err_m=NaN;
if isfield(m.targets,'methodB'), r.methodB_target_err_m=m.targets.methodB.meanErrorConfirmedM; end
end

function report(T,cfg)
plan=cfg.mission.turnPlan;
fprintf('\n===== PHASE 3 SUMMARY (paste this block) =====\n');
fprintf('Path conflict: %s\n',plan.strictTangentConflict);
cols={'est_rms_m','heading_rms_deg','route_dev_rms_m','landing_err_m','target_mean_err_m', ...
    'methodB_target_err_m','targets_confirmed','target_sightings','landmark_frames_used', ...
    'yaw_gated_frac','yaw_track_rms_deg','max_yaw_rate_degps','total_rotation_deg','max_tilt_deg'};
base=T(T.scenario_case=="baseline",:);
fprintf('\n-- Means over seeds (std) --\n');
for m=unique(base.method).'
    for y=["fixed","tangent"]
        s=base(base.method==m & base.yaw==y,:);
        if isempty(s), continue; end
        fprintf('%s %-7s n=%d completed=%d\n',m,y,height(s),nnz(s.completed));
        for c=cols
            v=s.(c{1}); fprintf('    %-24s %9.4f (%.4f)\n',c{1},mean(v,'omitnan'),std(v,'omitnan'));
        end
    end
end
fprintf('\n-- Paired difference, tangent minus fixed (same seed): mean [min max] --\n');
for m=unique(base.method).'
    f=sortrows(base(base.method==m & base.yaw=="fixed",:),'seed');
    g=sortrows(base(base.method==m & base.yaw=="tangent",:),'seed');
    [~,ia,ib]=intersect(f.seed,g.seed);
    fprintf('method %s (%d pairs)\n',m,numel(ia));
    for c=cols
        d=g.(c{1})(ib)-f.(c{1})(ia);
        fprintf('    %-24s %+9.4f [%+.4f %+.4f]\n',c{1},mean(d,'omitnan'),min(d),max(d));
    end
end
lever=T(T.scenario_case~="baseline",:);
if ~isempty(lever)
    fprintf('\n-- Sensitivity: %s (method A, seed %d) --\n',lever.scenario_case(1),lever.seed(1));
    ref=base(base.method=="A" & base.seed==lever.seed(1),:);
    for y=["fixed","tangent"]
        a=lever(lever.yaw==y,:); b=ref(ref.yaw==y,:);
        if isempty(a) || isempty(b), continue; end
        fprintf('  %-7s est RMS %.4f -> %.4f m | target err %.4f -> %.4f m | landing %.4f -> %.4f m\n', ...
            y,b.est_rms_m,a.est_rms_m,b.target_mean_err_m,a.target_mean_err_m,b.landing_err_m,a.landing_err_m);
    end
end
fprintf(['\n(Simulation with ASSUMED sensor errors; lecturer RPT controller, not PX4; ' ...
    'study EKF, not EKF2; not flight evidence.)\n']);
end

function plotYaw(runs,labels,file)
fig=figure('Color','w','Position',[80 80 1300 800],'Name','Fixed vs tangent yaw, seed 1');
tl=tiledlayout(fig,3,1,'TileSpacing','compact');
ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
for i=1:numel(runs)
    r=runs{i};
    plot(ax,r.time,rad2deg(unwrap(r.euler(:,3))),'LineWidth',1.1,'DisplayName',[labels{i} ' true']);
end
ylabel(ax,'True yaw (deg)'); legend(ax,'Location','eastoutside','Interpreter','none');
title(ax,'Heading over the mission (tangent turns at each leg; fixed stays put)');
ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
for i=1:numel(runs)
    r=runs{i}; psi=unwrap(r.euler(:,3));
    plot(ax,r.time,[0;rad2deg(diff(psi))./diff(r.time)],'LineWidth',1,'DisplayName',labels{i});
end
yline(ax,[30 -30],'r--','30 deg/s camera gate','HandleVisibility','off');
ylabel(ax,'Yaw rate (deg/s)'); legend(ax,'Location','eastoutside','Interpreter','none');
ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
for i=1:numel(runs)
    r=runs{i};
    plot(ax,r.time,vecnorm(r.estimatedPosition(:,1:2)-r.position(:,1:2),2,2), ...
        'LineWidth',1.1,'DisplayName',labels{i});
end
ylabel(ax,'|estimate - truth| (m)'); xlabel(ax,'Simulation time (s)');
legend(ax,'Location','eastoutside','Interpreter','none');
title(tl,{'Fixed vs tangent yaw, seed 1 (paired: same path, timing, sensors)', ...
    'Simulation with ASSUMED sensor errors; not flight evidence'});
exportgraphics(fig,file,'Resolution',150);
end
