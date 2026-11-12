function S = summarise_nav_experiments(T)
%SUMMARISE_NAV_EXPERIMENTS Paste-friendly text summary of run_nav_experiments.
%   summarise_nav_experiments()        reads results/nav_experiments/summary.csv
%   summarise_nav_experiments(T)       uses the table returned by the runner
% Part 1: landmark layouts and methods (mean +- std over seeds), per yaw mode.
% Part 2: paired yaw effect, tangent minus fixed, same seed/layout/method.
if nargin<1 || isempty(T)
    root=fileparts(mfilename('fullpath'));
    T=readtable(fullfile(root,'results','nav_experiments','summary.csv'),'TextType','string');
end
T.yaw=string(T.yaw); T.layout=string(T.layout); T.method=string(T.method);
cols={'target_mean_err_m','est_rms_m','landing_err_m','route_dev_rms_m', ...
    'heading_rms_deg','anchor_frames_detected_frac','targets_confirmed', ...
    'yaw_gated_frac','max_tilt_deg'};
short={'tgtErr','estRMS','landErr','routeDev','hdgRMS','lmFrames','tgtFound','yawGated','tilt'};
fprintf('\nRows: %d | seeds %s | completed %d/%d\n',height(T), ...
    mat2str(unique(T.seed).'),nnz(T.completed),height(T));
G=groupsummary(T,{'yaw','layout','method'},{'mean','std'},cols);
fprintf('\n=== Part 1: layouts x methods (mean +- std over seeds; metres unless noted) ===\n');
fprintf('%-8s %-22s %-3s','yaw','layout','M');
fprintf(' %15s',short{:}); fprintf('\n');
for i=1:height(G)
    fprintf('%-8s %-22s %-3s',G.yaw(i),G.layout(i),G.method(i));
    for c=1:numel(cols)
        mu=G.("mean_"+cols{c})(i); sd=G.("std_"+cols{c})(i);
        fprintf(' %7.3f+-%-6.3f',mu,sd);
    end
    fprintf('\n');
end
fprintf('\n=== Part 2: paired yaw effect (tangent - fixed), mean [min max] over seeds ===\n');
keys=unique(T(:,{'layout','method'}),'rows');
P=table();
for i=1:height(keys)
    f=T(T.yaw=="fixed" & T.layout==keys.layout(i) & T.method==keys.method(i),:);
    g=T(T.yaw=="tangent" & T.layout==keys.layout(i) & T.method==keys.method(i),:);
    [~,ia,ib]=intersect(f.seed,g.seed);
    if isempty(ia), continue; end
    fprintf('%-22s %-3s',keys.layout(i),keys.method(i));
    for c=[1 2 3 5 8]
        d=g.(cols{c})(ib)-f.(cols{c})(ia);
        fprintf(' %s %+.3f [%+.3f %+.3f]',short{c},mean(d,'omitnan'),min(d),max(d));
    end
    fprintf('\n');
    P=[P;table(keys.layout(i),keys.method(i),numel(ia),'VariableNames',{'layout','method','pairs'})]; %#ok<AGROW>
end
fprintf(['\nlmFrames = fraction of camera frames with a landmark sighting; ' ...
    'yawGated = fraction of frames rejected by the 30 deg/s gate.\n' ...
    'Simulation with ASSUMED sensor errors; not flight evidence.\n']);
S=struct('groups',G,'pairs',P);
end
