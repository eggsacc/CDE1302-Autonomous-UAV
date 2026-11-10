function result = run_baseline(preset)
%RUN_BASELINE Run fixed-yaw position navigation with ideal state feedback.
%   result = run_baseline('demo') uses explicitly assumed arena placement.
%   The model is the lecturer RPT model, not PX4 or EKF2.

if nargin < 1, preset = 'demo'; end
root = fileparts(mfilename('fullpath'));
oldPath = path;
pathCleanup = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(root);
if isstruct(preset)
    cfg=preset; preset=cfg.preset;
else
    cfg=study_config(preset);
end
check_environment(root);
verify_sources();
missionChecks=check_mission_reference();
write_text(fullfile(root,'evidence','mission_logic_checks.json'), ...
    jsonencode(missionChecks,'PrettyPrint',true));
runName=['fixed_yaw_' char(preset) '_' cfg.mission.guidanceMode];
outputDir=fullfile(root,'results',runName);
if ~isfolder(outputDir), mkdir(outputDir); end
oldGeneration = Simulink.fileGenControl('getConfig');
generationCleanup = onCleanup(@() Simulink.fileGenControl( ...
    'setConfig','config',oldGeneration)); %#ok<NASGU>
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(root,'generated','cache'), ...
    'CodeGenFolder',fullfile(root,'generated','codegen'),'createDir',true);
mdl = build_study_model(cfg);
modelCleanup = onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
sampleTimes=Simulink.BlockDiagram.getSampleTimes(mdl);
sampleText='Compiled sample times (period, offset in seconds):';
for k=1:numel(sampleTimes)
    sampleText=sprintf('%s\n%s %s',sampleText,mat2str(sampleTimes(k).Value), ...
        char(sampleTimes(k).Description));
end
write_text(fullfile(outputDir,'sample_times.txt'),sampleText);
simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setModelParameter('StopTime',num2str(cfg.mission.maxTimeS,17), ...
    'ReturnWorkspaceOutputs','on');
out = sim(simIn);
% Raw output is saved before any postprocessing assertion can fail.
save(fullfile(outputDir,'raw_simulation.mat'),'out','-v7.3');
% Shared with postprocess_baseline(preset), which rebuilds everything below
% from raw_simulation.mat without re-simulating.
result = postprocess_baseline(out,cfg,outputDir);
fprintf('BASELINE_EXECUTED preset=%s samples=%d endTime=%.3f output=%s\n', ...
    char(preset),numel(result.time),result.time(end),outputDir);
disp(result.metrics);
disp(result.checks);
end

function write_text(file,text)
fid = fopen(file,'w');
assert(fid~=-1,'MananStudy:Write','Cannot write %s.',file);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',text);
end
