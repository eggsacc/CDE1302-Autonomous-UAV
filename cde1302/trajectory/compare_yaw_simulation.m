function results = compare_yaw_simulation(output_dir)
% SIMULATION ONLY. Same CSV positions/timing for fixed/tangent experiments.
% Does not save or modify the original Simulink model or controller gains.
if nargin < 1, output_dir = fullfile('logs','simulation_yaw'); end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'position_control'), fullfile(here,'position_control','icon'));
data = readmatrix(fullfile(here,'ellipse_min_snap_traj.csv'));
t = data(:,1); p = data(:,2:4);
[yaw, ~, velocity] = tangent_yaw_reference(t,p,1e-4);
accel = zeros(size(velocity));
for c=1:3, accel(:,c) = gradient(velocity(:,c),t); end
fprintf('SIMULATION ONLY: strict tangent rate %.3f rad/s; hardware limit is NOT waived.\n', ...
    max(abs(diff(yaw)./diff(t))));
if ~exist(output_dir,'dir'), mkdir(output_dir); end
model = 'PosControl_Sim_RPT';
load_system(model);
original_init = get_param(model,'InitFcn');
original_dirty = get_param(model,'Dirty');
cleanup = onCleanup(@() restoreModel(model,original_init,original_dirty)); %#ok<NASGU>
% The model InitFcn loads its default zero initial yaw. Set only the simulated
% initial nose angle to the first reference, avoiding an artificial yaw step.
set_param(model,'InitFcn', [original_init newline ...
    'ModelInit_AngEuler(3)=ref.signals.values(1,10);']);
results = struct();
for mode = ["fixed", "tangent"]
    if mode == "fixed", psi = zeros(size(yaw)); else, psi = yaw; end
    ref.time = t;
    ref.signals.values = [p velocity accel psi gradient(psi,t)];
    ref.signals.dimensions = 11;
    % The existing model reads From Workspace data and runs Init_control in
    % the base workspace. Save/restore the sole input variable explicitly.
    had_ref = evalin('base','exist(''ref'',''var'')');
    saved_ref = [];
    if had_ref, saved_ref = evalin('base','ref'); end
    input_cleanup = onCleanup(@() restoreRef(had_ref,saved_ref)); %#ok<NASGU>
    assignin('base','ref',ref);
    fprintf('Starting %s simulation (120 s wall-clock timeout).\n',mode);
    simout = sim(model,'StopTime',num2str(t(end)), 'ReturnWorkspaceOutputs','on', 'Timeout',120);
    results.(mode).simulation_output = simout;
    results.(mode).reference = ref;
    pos = simout.get('PosE');
    assert(~isempty(pos), 'Model did not export PosE; inspect model signal logging.');
    results.(mode).last_time_s = pos.time(end);
    results.(mode).completed = pos.time(end) >= t(end) - 0.01;
    if pos.time(end) < t(end) - 0.01
        save(fullfile(output_dir,mode + "_incomplete.mat"),'ref','simout');
        warning('YawExperiment:IncompleteSimulation', ...
            '%s simulation stopped at %.6f s of %.1f s. No tracking pass is claimed.', ...
            mode,pos.time(end),t(end));
        clear input_cleanup;
        continue;
    end
    if ~isempty(pos)
        f = figure('Visible','off','Color','w');
        plot3(p(:,1),p(:,2),p(:,3),'--'); hold on;
        plot3(pos.signals.values(:,1),pos.signals.values(:,2),pos.signals.values(:,3));
        grid on; axis equal; legend('Reference','SIMULATED estimate');
        xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
        title(mode + " yaw: SIMULATION ONLY");
        exportgraphics(f,fullfile(output_dir,mode + "_position.png")); close(f);
        estimated = interp1(pos.time,pos.signals.values,t);
        f = figure('Visible','off','Color','w');
        plot(t,hypot(estimated(:,1)-p(:,1),estimated(:,2)-p(:,2)));
        xlabel('Time (s)'); ylabel('Horizontal tracking error (m)'); grid on;
        title(mode + " yaw: SIMULATION ONLY");
        exportgraphics(f,fullfile(output_dir,mode + "_tracking_error.png")); close(f);
    end
    % Only plot measured yaw when the model actually exports that signal.
    names = simout.who;
    results.(mode).exported_signals = names;
    f = figure('Visible','off','Color','w');
    plot(t,psi); grid on; xlabel('Time (s)'); ylabel('Yaw reference (rad, unwrapped)');
    angles = simout.get('AngEuler');
    if ~isempty(angles)
        hold on; plot(angles.time,unwrap(angles.signals.values(:,3)));
        legend('Reference','SIMULATED estimate');
        estimated_yaw = interp1(angles.time,unwrap(angles.signals.values(:,3)),t);
        yaw_error = atan2(sin(estimated_yaw-psi),cos(estimated_yaw-psi));
        results.(mode).yaw_error_rad = yaw_error;
    end
    title(mode + " yaw reference: SIMULATION ONLY");
    exportgraphics(f,fullfile(output_dir,mode + "_yaw_reference.png")); close(f);
    save(fullfile(output_dir,mode + "_simulation.mat"),'ref','simout');
    if had_ref, assignin('base','ref',saved_ref); else, evalin('base','clear ref'); end
    clear input_cleanup;
end
save(fullfile(output_dir,'comparison.mat'),'results');
summary = table(["fixed";"tangent"], ...
    [results.fixed.completed;results.tangent.completed], ...
    [results.fixed.last_time_s;results.tangent.last_time_s], ...
    'VariableNames',{'mode','completed','last_time_s'});
writetable(summary,fullfile(output_dir,'simulation_status.csv'));
disp(summary);
end

function restoreRef(had_ref,value)
if had_ref, assignin('base','ref',value); else, evalin('base','clear ref'); end
end
function restoreModel(model,init,dirty)
set_param(model,'InitFcn',init,'Dirty',dirty);
end
