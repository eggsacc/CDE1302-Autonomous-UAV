function mdl = build_study_model(cfg)
%BUILD_STUDY_MODEL Instrument an in-memory copy of the lecturer RPT model.
% The six-degree-of-freedom plant, RPT/attitude/rate controllers and their
% truth-feedback connections are retained. Only reference generation,
% initialization scope, logging and external visualization change.

root = fileparts(mfilename('fullpath'));
vendor = fullfile(root,'vendor','lecturer','position_control');
mdl = 'MananStudy_RPT';
assert(~bdIsLoaded(mdl),'MananStudy:ModelOpen', ...
    'Close the existing MananStudy_RPT model before rebuilding it.');
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
        refBlock,'FunctionName','mission_reference','Parameters','studyConfig', ...
        'Position',oldPosition);
    refPorts = get_param(refBlock,'PortHandles');
    dynamicsPorts = get_param([mdl '/Quadcopter Dynamics'],'PortHandles');
    add_line(mdl,dynamicsPorts.Outport(1),refPorts.Inport(1),'autorouting','on');
    for k = 1:numel(destinations)
        add_line(mdl,refPorts.Outport(1),destinations(k),'autorouting','on');
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
    names = {'truth_position','truth_velocity','truth_euler', ...
        'truth_body_rates','truth_motor_rpm','truth_quaternion'};
    for k = 1:numel(names)
        mark(dynamicsPorts.Outport(k),names{k});
    end
    mark(refPorts.Outport(1),'reference');
    mark(refPorts.Outport(2),'mission_diagnostics');
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

function mark(port,name)
set_param(port,'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName',name);
end
