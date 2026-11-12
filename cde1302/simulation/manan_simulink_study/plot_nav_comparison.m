function fig = plot_nav_comparison(runs,labels,outFile)
%PLOT_NAV_COMPARISON Compare several run_nav_scenario results (desktop only).
%   plot_nav_comparison(runs,labels)          runs: cell array of results
%   plot_nav_comparison(runs,labels,'x.png')  also exports a PNG
% Top: horizontal estimation error and heading error against time.
% Bottom: summary bars. Method B appears as the post-flight target error of
% the A flight it came from. Simulation with assumed sensor errors only.
if nargin<2 || isempty(labels)
    labels=cellfun(@(r) sprintf('%s %s',r.config.mission.yawMode,r.config.scenario.method), ...
        runs,'UniformOutput',false);
end
fig=figure('Color','w','Position',[80 80 1300 850],'Name','Run comparison (simulation)');
tl=tiledlayout(fig,2,2,'TileSpacing','compact');
ax1=nexttile(tl); hold(ax1,'on'); grid(ax1,'on');
ax2=nexttile(tl); hold(ax2,'on'); grid(ax2,'on');
colours=lines(numel(runs));
for i=1:numel(runs)
    r=runs{i}; e=vecnorm(r.estimatedPosition(:,1:2)-r.position(:,1:2),2,2);
    plot(ax1,r.time,e,'Color',colours(i,:),'LineWidth',1.1,'DisplayName',labels{i});
    psiTrue=unwrap(r.euler(:,3));
    psiHat=interp1(r.nav.time,r.nav.psiHat,r.time,'previous','extrap');
    plot(ax2,r.time,rad2deg(atan2(sin(psiHat-psiTrue),cos(psiHat-psiTrue))), ...
        'Color',colours(i,:),'LineWidth',1.1,'DisplayName',labels{i});
end
ylabel(ax1,'|estimate - truth| horizontal (m)'); xlabel(ax1,'Simulation time (s)');
ylabel(ax2,'Heading estimate error (deg)'); xlabel(ax2,'Simulation time (s)');
legend(ax1,'Location','northwest','Interpreter','none');
title(ax1,'Position estimation error'); title(ax2,'Heading estimation error');
names={'Est. RMS (m)','Landing error (m)','Target mean err, in flight (m)','Target mean err, Method B (m)'};
V=nan(numel(runs),4);
for i=1:numel(runs)
    m=runs{i}.metrics;
    V(i,1:3)=[m.estimation.rmsM m.landing.trueHorizontalErrorM m.targets.inFlight.meanErrorConfirmedM];
    if isfield(m.targets,'methodB'), V(i,4)=m.targets.methodB.meanErrorConfirmedM; end
end
ax3=nexttile(tl,[1 2]);
b=bar(ax3,categorical(names,names),V.'); grid(ax3,'on');
for i=1:numel(b), b(i).FaceColor=colours(i,:); b(i).DisplayName=labels{i}; end
ylabel(ax3,'metres'); legend(ax3,'Location','northeastoutside','Interpreter','none');
title(ax3,'Summary (NaN bars: not applicable, e.g. no Method B for a C flight)');
title(tl,{'Simulation with ASSUMED sensor errors; lecturer RPT controller (not PX4); study EKF (not EKF2)', ...
    'Not validated against flight logs'},'FontSize',10);
if nargin>=3 && ~isempty(outFile), exportgraphics(fig,outFile,'Resolution',150); end
end
