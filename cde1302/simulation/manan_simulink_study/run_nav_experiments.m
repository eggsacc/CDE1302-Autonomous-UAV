function T = run_nav_experiments(varargin)
%RUN_NAV_EXPERIMENTS Repeatable landmark-layout and yaw experiments (batch-safe).
%   T = run_nav_experiments() runs the default grid and writes
%   results/nav_experiments/summary.csv and summary.mat (no plots; make them
%   in the desktop with plot_nav_experiments).
%
% Design (paired): for each seed and yaw mode
%   * ONE Method A flight with every candidate landmark on the floor. In A the
%     landmarks never affect the flight, so Method B for each layout is the
%     post-flight correction using only that layout's landmark sightings.
%   * One Method C flight per non-empty layout (landmarks feed the controller).
% Same seed => same targets, sensor errors and noise in every row.
p=inputParser;
p.addParameter('seeds',1:3);
p.addParameter('yawModes',{'fixed','tangent'});
p.addParameter('layouts',{'none','home','corners4','corners_centre5','home_corners_centre6'});
p.addParameter('flightPreset','team_loc_tags');
p.addParameter('outDir','');
p.addParameter('keepRuns',false);
p.parse(varargin{:}); o=p.Results;
root=fileparts(mfilename('fullpath')); addpath(root);
outDir=o.outDir; if isempty(outDir), outDir=fullfile(root,'results','nav_experiments'); end
if ~isfolder(outDir), mkdir(outDir); end
rows={}; started=tic; count=0;
nTotal=numel(o.seeds)*numel(o.yawModes)*(1+nnz(~strcmp(o.layouts,'none')));
for seed=o.seeds
    for y=o.yawModes
        base=struct('flightPreset',o.flightPreset,'seed',seed,'yawMode',y{1});
        cfg0=scenario_config(base);
        union=unionLayouts(o.layouts,cfg0);
        optsA=base; optsA.method='A'; optsA.anchors=union;
        count=count+1;
        fprintf('[%d/%d %.0f min] seed %d %s A (+B for %d layouts)\n',count,nTotal, ...
            toc(started)/60,seed,y{1},numel(o.layouts));
        resA=run_nav_scenario(optsA,'quiet',true);
        if o.keepRuns, saveRun(outDir,sprintf('seed%d_%s_A_union',seed,y{1}),resA); end
        for L=o.layouts
            ids=layoutIds(L{1},cfg0);
            rows(end+1,:)=rowA(resA,L{1},numel(ids),seed,y{1},ids); %#ok<AGROW>
            rows(end+1,:)=rowB(resA,L{1},ids,seed,y{1}); %#ok<AGROW>
        end
        for L=o.layouts
            if strcmp(L{1},'none'), continue; end
            optsC=base; optsC.method='C'; optsC.anchors=L{1};
            count=count+1;
            fprintf('[%d/%d %.0f min] seed %d %s C %s\n',count,nTotal,toc(started)/60,seed,y{1},L{1});
            resC=run_nav_scenario(optsC,'quiet',true);
            if o.keepRuns, saveRun(outDir,sprintf('seed%d_%s_C_%s',seed,y{1},L{1}),resC); end
            rows(end+1,:)=rowC(resC,L{1},seed,y{1}); %#ok<AGROW>
        end
        T=cell2table(rows,'VariableNames',columns());
        writetable(T,fullfile(outDir,'summary.csv'));   % checkpoint after each pair
    end
end
T=cell2table(rows,'VariableNames',columns());
meta=struct('seeds',o.seeds,'yawModes',{o.yawModes},'layouts',{o.layouts}, ...
    'flightPreset',o.flightPreset,'wallClockMin',toc(started)/60, ...
    'matlab',version,'finished',char(datetime('now','TimeZone','Asia/Singapore')), ...
    'note',['Simulation with assumed sensor errors; lecturer RPT controller (not PX4); ' ...
    'study EKF (not EKF2); demo arena geometry assumed. Not validated against flight logs.']);
save(fullfile(outDir,'summary.mat'),'T','meta');
writetable(T,fullfile(outDir,'summary.csv'));
fprintf('NAV_EXPERIMENTS_DONE rows=%d minutes=%.1f out=%s\n',height(T),meta.wallClockMin,outDir);
end

function c=columns()
c={'seed','yaw','layout','method','n_anchors','completed','duration_s', ...
    'est_rms_m','est_end_m','heading_rms_deg','heading_end_deg', ...
    'track_true_vs_cmd_rms_m','route_dev_rms_m','route_dev_max_m', ...
    'landing_err_m','targets_confirmed','targets_total','target_mean_err_m', ...
    'target_max_err_m','anchor_frames_detected_frac','anchor_sightings', ...
    'anchor_rejected','yaw_gated_frac','max_tilt_deg','correction'};
end

function A=unionLayouts(layouts,cfg)
A=zeros(0,3);
for L=layouts
    X=anchor_layout(L{1},cfg);
    for k=1:size(X,1)
        if ~any(A(:,1)==X(k,1)), A(end+1,:)=X(k,:); %#ok<AGROW>
        else
            assert(isequal(A(A(:,1)==X(k,1),2:3),X(k,2:3)), ...
                'MananStudy:LayoutIds','Landmark id %d has two positions.',X(k,1));
        end
    end
end
end

function ids=layoutIds(name,cfg)
X=anchor_layout(name,cfg); ids=X(:,1);
end

function f=frameFraction(res,ids)
S=res.sightings; s=S(S.is_anchor==1 & ismember(S.id,ids),:);
f=numel(unique(round(s.t_capture*100)))/max(1,res.metrics.anchors.framesTotal);
end

function r=common(res,layout,nA,seed,yaw,method)
m=res.metrics;
r={seed,yaw,layout,method,nA,m.mission.completed,m.mission.durationS, ...
    m.estimation.rmsM,m.estimation.endM,m.heading.rmsDeg,m.heading.endDeg, ...
    m.tracking.rmsTrueVsCommandSweepM,m.tracking.rmsTrueDistanceToPlannedRouteSweepM, ...
    m.tracking.maxTrueDistanceToPlannedRouteSweepM,m.landing.trueHorizontalErrorM};
end

function r=rowA(res,layout,nA,seed,yaw,ids)
m=res.metrics; t=m.targets.inFlight;
r=[common(res,layout,nA,seed,yaw,'A'),{t.confirmed,t.count,t.meanErrorConfirmedM, ...
    t.maxErrorConfirmedM,frameFraction(res,ids),NaN,NaN,m.camera.fractionFramesYawGated, ...
    m.mission.maxTiltDeg,'none'}];
end

function r=rowB(resA,layout,ids,seed,yaw)
res=resA;
keepA=ismember(res.config.anchors.id,ids);
res.config.anchors.id=res.config.anchors.id(keepA);
res.config.anchors.ned=res.config.anchors.ned(keepA,:);
res.config.anchors.arena=res.config.anchors.arena(keepA,:);
S=res.sightings; res.sightings=S(S.is_anchor==0 | ismember(S.id,ids),:);
B=nav_postcorrect(res);
m=resA.metrics;
tg=struct('confirmed',nnz([B.targets.confirmed]),'count',numel(B.targets));
err=[B.targets.error]; ok=[B.targets.confirmed];
meanErr=NaN; maxErr=NaN;
if any(ok), meanErr=mean(err(ok)); maxErr=max(err(ok)); end
t=resA.time; p=resA.position; air=(resA.config.frames.homeNed(3)-p(:,3))>resA.config.nav.cam.minHeightM;
pB=interp1(t,p(:,1:2),B.trajectoryTime,'linear','extrap');
airB=interp1(t,double(air),B.trajectoryTime,'previous','extrap')>0.5;
eB=vecnorm(B.trajectoryNE-pB,2,2);
r={seed,yaw,layout,'B',numel(ids),m.mission.completed,m.mission.durationS, ...
    rmsMask(B.trajectoryTime,eB,airB),eB(end),NaN,NaN, ...
    m.tracking.rmsTrueVsCommandSweepM,m.tracking.rmsTrueDistanceToPlannedRouteSweepM, ...
    m.tracking.maxTrueDistanceToPlannedRouteSweepM,m.landing.trueHorizontalErrorM, ...
    tg.confirmed,tg.count,meanErr,maxErr,frameFraction(resA,ids),NaN,NaN, ...
    m.camera.fractionFramesYawGated,m.mission.maxTiltDeg,B.correction.model};
end

function r=rowC(res,layout,seed,yaw)
m=res.metrics; t=m.targets.inFlight;
r=[common(res,layout,numel(res.config.anchors.id),seed,yaw,'C'),{t.confirmed,t.count, ...
    t.meanErrorConfirmedM,t.maxErrorConfirmedM,m.anchors.fractionFramesAnchorUsed, ...
    m.anchors.sightingsUsed,m.anchors.sightingsRejected,m.camera.fractionFramesYawGated, ...
    m.mission.maxTiltDeg,'online'}];
end

function v=rmsMask(t,x,mask)
use=mask(1:end-1)&mask(2:end); dt=diff(t(:));
if ~any(use), v=NaN; return; end
mid=(x(1:end-1).^2+x(2:end).^2)/2; v=sqrt(sum(mid(use).*dt(use))/sum(dt(use)));
end

function saveRun(outDir,name,res) %#ok<INUSD>
save(fullfile(outDir,[name '.mat']),'res','-v7.3');
end
