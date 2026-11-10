function evidence = run_lecturer_example()
%RUN_LECTURER_EXAMPLE Execute the preserved lecturer circle script unchanged.
% Use a fresh MATLAB session: the original script uses the base workspace,
% opens its model and closes figures. This wrapper restores path, directory,
% file-generation settings and default figure visibility afterwards.

root = fileparts(mfilename('fullpath'));
check_environment(root);
verify_sources();
fprintf('ORIGINAL_STEP initialize_model_api\n');
assert(~bdIsLoaded('PosControl_Sim_RPT'),'MananStudy:OriginalOpen', ...
    'Use a fresh session or close PosControl_Sim_RPT without saving first.');
vendorRoot = fullfile(root,'vendor','lecturer');
outputDir = fullfile(root,'results','original_circle');
if ~isfolder(outputDir), mkdir(outputDir); end
oldDir = pwd; oldPath = path;
oldVisibility = get(groot,'DefaultFigureVisible');
fprintf('ORIGINAL_STEP configure_runtime_folders\n');
oldGeneration = Simulink.fileGenControl('getConfig');
cleanup = onCleanup(@() restore(oldDir,oldPath,oldVisibility,oldGeneration)); %#ok<NASGU>
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(root,'generated','original_cache'), ...
    'CodeGenFolder',fullfile(root,'generated','original_codegen'),'createDir',true);
set(groot,'DefaultFigureVisible','off');
cd(vendorRoot);
% The vendored empty helper path supports the original addpath funct line.
addpath(fullfile(vendorRoot,'position_control'), ...
    fullfile(vendorRoot,'position_control','icon'));
scriptPath=fullfile(vendorRoot,'circle_traj_demo.m');
fprintf('ORIGINAL_STEP execute_circle\n');
batchMode=batchStartupOptionUsed;
if batchMode
    % Batch does not need a model editor window. Only this UI call changes;
    % the reference equations, model, parameters and sim call are preserved.
    code=fileread(scriptPath);
    code=strrep(code,'open_system(model);','load_system(model);');
    evalin('base',code);
else
    escaped=strrep(scriptPath,'''','''''');
    evalin('base',sprintf('run(''%s'');',escaped));
end
fprintf('ORIGINAL_STEP simulation_returned\n');
positionLog = evalin('base','PosE');
eulerLog = evalin('base','AngEuler');
reference = evalin('base','ref');
yawRate = evalin('base','Yawrate');
t = positionLog.time;
p = positionLog.signals.values;
desired = interp1(reference.time,reference.signals.values(:,1:3),t,'linear');
errorNorm = vecnorm(p-desired,2,2);
assert(all(isfinite(p),'all') && abs(t(end)-15)<1e-8, ...
    'MananStudy:OriginalFailed','Original run is nonfinite or incomplete.');
evidence.executed = true;
evidence.matlab = version;
evidence.lecturerCommit = '9d80ddab01b20b01a19cb849b5af5424ca0ae07a';
evidence.originalScriptUnchanged = true;
evidence.batchUiAdaptation = batchMode;
evidence.flightGearEnabled = true;
evidence.durationS = t(end);
evidence.samples = numel(t);
evidence.positionFinite = all(isfinite(p),'all');
evidence.rmse3dM = sqrt(trapz(t,errorNorm.^2)/(t(end)-t(1)));
after = t>=5;
evidence.after5sRmse3dM = sqrt(trapz(t(after),errorNorm(after).^2)/(t(end)-t(find(after,1))));
evidence.max3dErrorM = max(errorNorm);
evidence.final3dErrorM = errorNorm(end);
evidence.finalPositionNedM = p(end,:);
evidence.minimumReferenceYawRateRadS = min(yawRate);
evidence.limitations = {'Not PX4','Initial reference is 2 m from initial vehicle', ...
    'Original yaw wrapping defect retained','No takeoff, return or physical landing'};
figs = findall(groot,'Type','figure');
for k=1:numel(figs)
    exportgraphics(figs(k),fullfile(outputDir,sprintf('original_figure_%d.png',k)), ...
        'Resolution',150);
end
save(fullfile(outputDir,'original_circle.mat'),'positionLog','eulerLog','reference','evidence');
fid = fopen(fullfile(root,'evidence','original_circle.json'),'w');
assert(fid~=-1,'Cannot write original run evidence.');
fprintf(fid,'%s\n',jsonencode(evidence,'PrettyPrint',true)); fclose(fid);
fprintf('ORIGINAL_REPRODUCED duration=%.3f samples=%d finalError=%.6f\n', ...
    t(end),numel(t),evidence.final3dErrorM);
disp(evidence);
end

function restore(oldDir,oldPath,visibility,generation)
if bdIsLoaded('PosControl_Sim_RPT'), close_system('PosControl_Sim_RPT',0); end
cd(oldDir); path(oldPath);
set(groot,'DefaultFigureVisible',visibility);
Simulink.fileGenControl('setConfig','config',generation);
end
