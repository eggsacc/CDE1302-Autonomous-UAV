function mdl = build_study_model(cfg)
%BUILD_STUDY_MODEL Instrument an in-memory copy of the lecturer RPT model.
% The six-degree-of-freedom plant, RPT/attitude/rate controllers and their
% gains are retained. Reference generation, initialization scope, logging and
% external visualization change. Without cfg.nav the controller receives
% ideal truth feedback (Phase 1, model MananStudy_RPT). With cfg.nav.enabled
% (Phase 2/3, model MananStudy_Nav) a navigation estimator is inserted and
% the controller and mission receive ESTIMATED position, velocity and heading.

root = fileparts(mfilename('fullpath'));
vendor = fullfile(root,'vendor','lecturer','position_control');
navOn = isfield(cfg,'nav') && isfield(cfg.nav,'enabled') && cfg.nav.enabled;
mdl = 'MananStudy_RPT';
if navOn, mdl = 'MananStudy_Nav'; end
assert(~bdIsLoaded(mdl),'MananStudy:ModelOpen', ...
    'Close the existing %s model before rebuilding it.',mdl);
new_system(mdl,'FromFile',fullfile(vendor,'PosControl_Sim_RPT.slx'));
try
    set_param(mdl,'InitFcn','','StopTime',num2str(cfg.mission.maxTimeS,17), ...
        'SignalLogging','on','SignalLoggingName','logsout', ...
        'ReturnWorkspaceOutputs','on');
    % Parameters live in the copy's workspace, not the user's base workspace.
    ws = get_param(mdl,'ModelWorkspace');
    oldPath = path;
    restorePath = onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(vendor,fullfile(vendor,'icon'),root);
    evalin(ws,'Init_control;');
    assignin(ws,'ModelInit_PosE',reshape(cfg.frames.homeNed,1,3));
    assignin(ws,'ModelInit_VelB',[0 0 0]);
    assignin(ws,'ModelInit_AngEuler',[0 0 cfg.mission.yawRad]);
    assignin(ws,'ModelInit_RateB',[0 0 0]);
    assignin(ws,'studyConfig',cfg);
    % The mission block needs only the mission and frames; passing the full
    % configuration (with pre-drawn sensor noise) on every call is slow.
    assignin(ws,'studyMissionConfig',struct('mission',cfg.mission,'frames',cfg.frames));
    % No FlightGear UDP output or wall-clock pacing in the study copy.
    set_param([mdl '/To FlightGear'],'Commented','on');
    icon = strrep(fullfile(vendor,'icon','pixhawk.png'),'\','/');
    set_param([mdl '/Control System'],'MaskDisplay',sprintf('image(''%s'');',icon));

    refBlock = [mdl '/reference from trajectory generation'];
    oldPosition = get_param(refBlock,'Position');
    oldPorts = get_param(refBlock,'PortHandles');
    refLine = get_param(oldPorts.Outport(1),'Line');
    destinations = get_param(refLine,'DstPortHandle');
    delete_line(refLine);
    delete_block(refBlock);
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
        refBlock,'FunctionName','mission_reference','Parameters','studyMissionConfig', ...
        'Position',oldPosition);
    refPorts = get_param(refBlock,'PortHandles');
    dynamicsPorts = get_param([mdl '/Quadcopter Dynamics'],'PortHandles');
    add_line(mdl,dynamicsPorts.Outport(1),refPorts.Inport(1),'autorouting','on');
    for k = 1:numel(destinations)
        add_line(mdl,refPorts.Outport(1),destinations(k),'autorouting','on');
    end
    if navOn
        insertEstimator(mdl,dynamicsPorts);
    end
    % Resolve the S-function's second diagnostic output before instrumenting.
    set_param(mdl,'SimulationCommand','update');
    refPorts = get_param(refBlock,'PortHandles');
    % Stop at actual completion/failure instead of diluting mission metrics
    % with a long extra hold until the maximum-time cap.
    % Select flags without multiplying unused NaN diagnostics by zero.
    add_block('simulink/Signal Routing/Demux',[mdl '/Mission status fields'], ...
        'Outputs','7','Position',[80 690 85 810]);
    add_block('simulink/Logic and Bit Operations/Logical Operator', ...
        [mdl '/Mission finished'],'Operator','OR','Inputs','2', ...
        'Position',[150 710 185 750]);
    add_block('simulink/Sinks/Stop Simulation',[mdl '/Stop mission'], ...
        'Position',[220 710 260 750]);
    fieldPorts=get_param([mdl '/Mission status fields'],'PortHandles');
    finishPorts=get_param([mdl '/Mission finished'],'PortHandles');
    stopPorts=get_param([mdl '/Stop mission'],'PortHandles');
    add_line(mdl,refPorts.Outport(2),fieldPorts.Inport(1),'autorouting','on');
    add_line(mdl,fieldPorts.Outport(3),finishPorts.Inport(1),'autorouting','on');
    add_line(mdl,fieldPorts.Outport(4),finishPorts.Inport(2),'autorouting','on');
    add_line(mdl,finishPorts.Outport(1),stopPorts.Inport(1),'autorouting','on');
    % Unused status fields end in terminators (silences "not connected").
    for k=[1 2 5 6 7]
        term=sprintf('%s/Status field %d sink',mdl,k);
        add_block('simulink/Sinks/Terminator',term,'Position',[110 650+25*k 125 665+25*k]);
        tp=get_param(term,'PortHandles');
        add_line(mdl,fieldPorts.Outport(k),tp.Inport(1),'autorouting','on');
    end
    names = {'truth_position','truth_velocity','truth_euler', ...
        'truth_body_rates','truth_motor_rpm','truth_quaternion'};
    % 1 ms truth is logged at 100 Hz in estimator runs to keep files small.
    decimation = 1; if navOn, decimation = 10; end
    for k = 1:numel(names)
        mark(dynamicsPorts.Outport(k),names{k},decimation);
    end
    mark(refPorts.Outport(1),'reference',1);
    mark(refPorts.Outport(2),'mission_diagnostics',1);
    if navOn
        mark(portOf(mdl,'Feedback position',1),'feedback_position',decimation);
        mark(portOf(mdl,'Feedback velocity',1),'feedback_velocity',decimation);
        mark(portOf(mdl,'Navigation estimator',4),'nav_diagnostics',1);
        mark(portOf(mdl,'Navigation estimator',5),'nav_sightings',1);
    end
    % Mission diagnostics govern stopping and are never controller feedback.
    % Saving the generated copy makes the wiring inspectable; it is ignored
    % by Git and reproducible from this builder and the pinned vendor model.
    generated = fullfile(root,'generated');
    if ~isfolder(generated), mkdir(generated); end
    save_system(mdl,fullfile(generated,[mdl '.slx']));
catch err
    if bdIsLoaded(mdl), close_system(mdl,0); end
    rethrow(err);
end
end

function insertEstimator(mdl,dyn)
% Truth -> simulated sensors -> estimator -> estimate error. The controller
% and mission then receive truth + error, i.e. the estimate, never truth.
nav=[mdl '/Navigation estimator'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',nav, ...
    'FunctionName','nav_estimator','Parameters','studyConfig', ...
    'Position',[-420 820 -300 960]);
np=get_param(nav,'PortHandles');
add_line(mdl,dyn.Outport(1),np.Inport(1),'autorouting','on');   % position
add_line(mdl,dyn.Outport(2),np.Inport(2),'autorouting','on');   % velocity
add_line(mdl,dyn.Outport(6),np.Inport(3),'autorouting','on');   % quaternion
add_line(mdl,dyn.Outport(4),np.Inport(4),'autorouting','on');   % body rates
add_block('simulink/Math Operations/Sum',[mdl '/Feedback position'], ...
    'Inputs','++','Position',[-220 820 -190 850]);
add_block('simulink/Math Operations/Sum',[mdl '/Feedback velocity'], ...
    'Inputs','++','Position',[-220 870 -190 900]);
pp=get_param([mdl '/Feedback position'],'PortHandles');
vp=get_param([mdl '/Feedback velocity'],'PortHandles');
add_line(mdl,dyn.Outport(1),pp.Inport(1),'autorouting','on');
add_line(mdl,np.Outport(1),pp.Inport(2),'autorouting','on');
add_line(mdl,dyn.Outport(2),vp.Inport(1),'autorouting','on');
add_line(mdl,np.Outport(2),vp.Inport(2),'autorouting','on');
% Heading error: q_feedback = M(dpsi) * q_true (world-z pre-rotation).
add_block('simulink/Math Operations/Reshape',[mdl '/Heading error matrix'], ...
    'OutputDimensionality','Customize','OutputDimensions','[4 4]', ...
    'Position',[-220 920 -190 950]);
add_block('simulink/Math Operations/Reshape',[mdl '/True quaternion column'], ...
    'OutputDimensionality','Customize','OutputDimensions','[4 1]', ...
    'Position',[-220 970 -190 1000]);
add_block('simulink/Math Operations/Product',[mdl '/Feedback quaternion product'], ...
    'Multiplication','Matrix(*)','Inputs','**','Position',[-150 930 -120 990]);
add_block('simulink/Math Operations/Reshape',[mdl '/Feedback quaternion'], ...
    'OutputDimensionality','1-D array','Position',[-90 945 -60 975]);
mp=get_param([mdl '/Heading error matrix'],'PortHandles');
qp=get_param([mdl '/True quaternion column'],'PortHandles');
xp=get_param([mdl '/Feedback quaternion product'],'PortHandles');
fp=get_param([mdl '/Feedback quaternion'],'PortHandles');
add_line(mdl,np.Outport(3),mp.Inport(1),'autorouting','on');
add_line(mdl,dyn.Outport(6),qp.Inport(1),'autorouting','on');
add_line(mdl,mp.Outport(1),xp.Inport(1),'autorouting','on');
add_line(mdl,qp.Outport(1),xp.Inport(2),'autorouting','on');
add_line(mdl,xp.Outport(1),fp.Inport(1),'autorouting','on');
% Re-route every navigation consumer from truth to the estimate.
reroute(mdl,'Quadcopter Dynamics/1','Demux2/1',pp.Outport(1));
reroute(mdl,'Quadcopter Dynamics/1','reference from trajectory generation/1',pp.Outport(1));
reroute(mdl,'Quadcopter Dynamics/2','Demux3/1',vp.Outport(1));
reroute(mdl,'Quadcopter Dynamics/6','Control System/16',fp.Outport(1));
% Terminate the unused diagnostic/sighting outputs (logged, not consumed).
for k=4:5
    t=sprintf('%s/Nav output %d sink',mdl,k);
    add_block('simulink/Sinks/Terminator',t,'Position',[-250 960+30*k -230 980+30*k]);
    tp=get_param(t,'PortHandles');
    add_line(mdl,np.Outport(k),tp.Inport(1),'autorouting','on');
end
end

function reroute(mdl,src,dst,newSource)
delete_line(mdl,src,dst);
slash=find(dst=='/',1,'last');
ph=get_param([mdl '/' dst(1:slash-1)],'PortHandles');
port=str2double(dst(slash+1:end));
add_line(mdl,newSource,ph.Inport(port),'autorouting','on');
end

function h=portOf(mdl,block,k)
ph=get_param([mdl '/' block],'PortHandles'); h=ph.Outport(k);
end

function mark(port,name,decimation)
set_param(port,'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName',name);
if decimation>1
    set_param(port,'DataLoggingDecimateData','on', ...
        'DataLoggingDecimation',num2str(decimation));
end
end
