function res = run_nav_scenario(opts,varargin)
%RUN_NAV_SCENARIO Simulate one Phase 2/3 scenario through the lecturer model.
%   res = run_nav_scenario(opts)  opts as scenario_config (or a cfg from it).
%   res = run_nav_scenario(opts,'save',true,'name','myrun') also writes
%         results/nav/<name>/result.mat and summary.json (no plots: batch-safe).
%
% Method 'A' runs also carry Method B (post-flight correction of the same
% flight) in res.methodB. Method 'C' feeds the online estimate to the
% controller. Nothing here is PX4 or EKF2; see scenario_config for every
% assumed sensor number.
p=inputParser; p.addParameter('save',false); p.addParameter('name','');
p.addParameter('quiet',false); p.parse(varargin{:}); o=p.Results;
root=fileparts(mfilename('fullpath'));
oldPath=path; pathCleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(root);
if isfield(opts,'nav'), cfg=opts; else, cfg=scenario_config(opts); end
oldGeneration=Simulink.fileGenControl('getConfig');
generationCleanup=onCleanup(@() Simulink.fileGenControl( ...
    'setConfig','config',oldGeneration)); %#ok<NASGU>
Simulink.fileGenControl('set','CacheFolder',fullfile(root,'generated','cache'), ...
    'CodeGenFolder',fullfile(root,'generated','codegen'),'createDir',true);
if bdIsLoaded('MananStudy_Nav'), close_system('MananStudy_Nav',0); end
wall=tic;
mdl=build_study_model(cfg);
modelCleanup=onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
simIn=Simulink.SimulationInput(mdl);
simIn=simIn.setModelParameter('StopTime',num2str(cfg.mission.maxTimeS,17), ...
    'ReturnWorkspaceOutputs','on');
out=sim(simIn);
res=extract(out,cfg);
res.execution.wallClockS=toc(wall);
res.execution.matlab=version;
res.execution.timestamp=char(datetime('now','TimeZone','Asia/Singapore'));
res.tags=nav_tag_report(res);
if strcmp(cfg.scenario.method,'A')
    res.methodB=nav_postcorrect(res);
end
res.metrics=nav_metrics(res);
if ~o.quiet
    fprintf('NAV_RUN %s | %.0f s simulated in %.0f s wall | completed=%d\n', ...
        cfg.scenario.description,res.time(end),res.execution.wallClockS, ...
        res.metrics.mission.completed);
end
if o.save
    name=o.name;
    if isempty(name)
        name=sprintf('%s_%s_seed%d_%s',cfg.mission.yawMode,cfg.scenario.method, ...
            cfg.scenario.seed,cfg.scenario.anchorLayout);
    end
    outDir=fullfile(root,'results','nav',name); if ~isfolder(outDir), mkdir(outDir); end
    res.outputDir=outDir;
    save(fullfile(outDir,'result.mat'),'res','-v7.3');
    fid=fopen(fullfile(outDir,'summary.json'),'w','n','UTF-8');
    fprintf(fid,'%s\n',jsonencode(struct('scenario',cfg.scenario, ...
        'execution',res.execution,'metrics',res.metrics),'PrettyPrint',true));
    fclose(fid);
end
end

function res=extract(out,cfg)
L=out.logsout;
[t,pos]=unpack(L,'truth_position');
res.config=cfg;
res.time=t; res.position=pos;
res.velocity=align(L,'truth_velocity',t,'linear');
res.euler=align(L,'truth_euler',t,'linear');
res.quaternion=align(L,'truth_quaternion',t,'linear');
res.feedbackPosition=align(L,'feedback_position',t,'linear');
res.feedbackVelocity=align(L,'feedback_velocity',t,'linear');
res.reference=align(L,'reference',t,'previous');
res.diagnostics=align(L,'mission_diagnostics',t,'previous');
[res.referenceNativeTime,res.referenceNative]=unpack(L,'reference');
% Native 20 Hz mission status: the 100 Hz truth grid can end a few ms before
% the final DONE tick, so completion is judged from this log, not the grid.
[res.diagnosticsNativeTime,res.diagnosticsNative]=unpack(L,'mission_diagnostics');
[tn,nd]=unpack(L,'nav_diagnostics');
names=nav_core('fields');
res.nav.time=tn; res.nav.data=nd; res.nav.fields=names.diag;
res.nav.estimatedNE=nd(:,1:2); res.nav.psiHat=nd(:,3);
res.estimatedPosition=[interp1(tn,nd(:,1:2),t,'previous','extrap'),res.feedbackPosition(:,3)];
res.estimateAvailable=true;
[ts,sd]=unpack(L,'nav_sightings');
res.sightings=sightingTable(ts,sd,cfg,names.sight);
end

function T=sightingTable(ts,sd,cfg,fields)
F=numel(fields); nS=cfg.nav.nSlots;
rows=cell(0,1);
if nS>0
    D=reshape(sd.',F,max(1,nS),[]);            % F x slot x time
    for j=1:nS
        v=squeeze(D(:,j,:)).';                   % time x F
        keep=isfinite(v(:,1));
        if any(keep)
            k=nnz(keep);
            rows{end+1,1}=[ts(keep),repmat([j cfg.nav.slotId(j) cfg.nav.slotIsAnchor(j)],k,1),v(keep,:)]; %#ok<AGROW>
        end
    end
end
M=vertcat(rows{:});
if isempty(M), M=zeros(0,4+F); end
T=array2table(M,'VariableNames',[{'t_arrival','slot','id','is_anchor'},fields]);
T=sortrows(T,'t_arrival');
end

function [t,data]=unpack(L,name)
ts=L.getElement(name).Values;
t=double(ts.Time(:)); d=double(ts.Data);
if size(d,1)==numel(t)
    data=reshape(d,numel(t),[]);
elseif size(d,ndims(d))==numel(t)
    data=reshape(permute(d,[ndims(d) 1:ndims(d)-1]),numel(t),[]);
else
    error('MananStudy:LogShape','Unexpected logged dimensions for %s.',name);
end
[t,keep]=unique(t,'last'); data=data(keep,:);
end

function x=align(L,name,query,method)
[t,d]=unpack(L,name);
if isequal(t,query), x=d; return; end
x=interp1(t,d,query,method,'extrap');
end
