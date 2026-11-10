function result = postprocess_baseline(source,cfg,outputDir)
%POSTPROCESS_BASELINE Build the structured result from a saved simulation.
%   result = postprocess_baseline('demo') reloads
%   results/fixed_yaw_demo_<guidanceMode>/raw_simulation.mat for the preset
%   and regenerates run.mat, trajectory.csv, metrics/checks JSON, the three
%   PNG plots and evidence/<run>.json. It does NOT simulate anything; the
%   execution record keeps the raw file's save time as the simulation time
%   and adds a separate post-processing time.
%
%   result = postprocess_baseline(out,cfg,outputDir) is the shared path used
%   by run_baseline immediately after sim().

root = fileparts(mfilename('fullpath'));
oldPath = path;
pathCleanup = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(root);
if ischar(source) || isstring(source)
    cfg = study_config(char(source));
    runName = ['fixed_yaw_' cfg.preset '_' cfg.mission.guidanceMode];
    outputDir = fullfile(root,'results',runName);
    rawFile = fullfile(outputDir,'raw_simulation.mat');
    assert(isfile(rawFile),'MananStudy:NoRaw','No saved simulation at %s.',rawFile);
    loaded = load(rawFile,'out');
    out = loaded.out;
    rawInfo = dir(rawFile);
    simulationTime = char(datetime(rawInfo.datenum,'ConvertFrom','datenum'));
else
    out = source;
    runName = ['fixed_yaw_' cfg.preset '_' cfg.mission.guidanceMode];
    simulationTime = char(datetime('now','TimeZone','Asia/Singapore'));
end
if ~isfolder(outputDir), mkdir(outputDir); end
logs = out.logsout;
[time,position] = unpack(logs,'truth_position');
[tv,velocity] = unpack(logs,'truth_velocity');
[te,euler] = unpack(logs,'truth_euler');
[tq,quaternion] = unpack(logs,'truth_quaternion');
[tb,bodyRates] = unpack(logs,'truth_body_rates');
[tr,reference] = unpack(logs,'reference');
[td,diagnostics] = unpack(logs,'mission_diagnostics');
assert(numel(time)>1 && all(diff(time)>0),'MananStudy:Time', ...
    'Need multiple strictly increasing truth timestamps; inspect raw_simulation.mat.');
result.time = time;
result.position = position;
result.velocity = align(tv,velocity,time,'linear');
result.euler = align(te,euler,time,'linear'); % Lecturer output is radians.
result.quaternion = align(tq,quaternion,time,'linear');
result.bodyRates = align(tb,bodyRates,time,'linear');
result.reference = align(tr,reference,time,'previous');
result.diagnostics = align(td,diagnostics,time,'previous');
% Native 20 Hz command samples, kept so slew is judged at command ticks and
% not on the 1 ms zero-order-held truth grid.
[result.referenceNativeTime,keep] = unique(tr,'last');
result.referenceNative = reference(keep,:);
result.feedbackPosition = result.position;
result.feedbackVelocity = result.velocity;
result.estimatedPosition = [];
result.estimateAvailable = false;
result.config = cfg;
result.outputDir = outputDir;
result.provenance.lecturerCommit = '9d80ddab01b20b01a19cb849b5af5424ca0ae07a';
result.provenance.teamCommit = 'e7a231eff2f9239ff0d6948a058cab19b4b5757c';
result.provenance.model = 'Lecturer RPT with unchanged vehicle/controller parameters';
result.provenance.feedback = 'Ideal plant position, velocity, attitude and body rates';
result.provenance.guidanceMode = cfg.mission.guidanceMode;
result.provenance.landing = 'Descent/endpoint tracking only; no contact, disarm or PX4 land mode';
result.execution.matlab = version;
result.execution.simulationTimestamp = simulationTime;
result.execution.postprocessTimestamp = char(datetime('now','TimeZone','Asia/Singapore'));
result.execution.timestamp = simulationTime; % Backward-compatible field.
[result.metrics,result.checks] = analyse_run(result);
save(fullfile(outputDir,'run.mat'),'result','-v7');
summary = struct('execution',result.execution,'provenance',result.provenance, ...
    'geometry',cfg.geometry,'metrics',result.metrics,'checks',result.checks);
write_text(fullfile(root,'evidence',[runName '.json']), ...
    jsonencode(summary,'PrettyPrint',true));
fprintf('POSTPROCESS_DONE run=%s samples=%d endTime=%.3f output=%s\n', ...
    runName,numel(time),time(end),outputDir);
end

function [time,data] = unpack(logs,name)
ts = logs.getElement(name).Values;
time = double(ts.Time(:));
d = double(ts.Data);
if size(d,1)==numel(time)
    data = reshape(d,numel(time),[]);
elseif size(d,ndims(d))==numel(time)
    order = [ndims(d) 1:ndims(d)-1];
    data = reshape(permute(d,order),numel(time),[]);
else
    error('MananStudy:LogShape','Unexpected logged dimensions for %s.',name);
end
end

function data = align(t,x,query,method)
[t,keep] = unique(t,'last');
x = x(keep,:);
if isequal(t,query), data=x; return; end
assert(t(1)<=query(1)+1e-9 && t(end)>=query(end)-1e-9, ...
    'MananStudy:LogCoverage','Logged signal does not cover truth time.');
data = interp1(t,x,query,method,'extrap');
end

function write_text(file,text)
fid = fopen(file,'w');
assert(fid~=-1,'MananStudy:Write','Cannot write %s.',file);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',text);
end
