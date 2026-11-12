function figs = plot_nav_experiments(summaryFile,outDir)
%PLOT_NAV_EXPERIMENTS Desktop plots of run_nav_experiments output.
%   plot_nav_experiments()  reads results/nav_experiments/summary.csv and
%   writes landmarks.png and yaw.png next to it. Bars are means over seeds;
%   black dots are the individual seeds, so the spread stays visible.
root=fileparts(mfilename('fullpath'));
if nargin<1 || isempty(summaryFile)
    summaryFile=fullfile(root,'results','nav_experiments','summary.csv');
end
if nargin<2 || isempty(outDir), outDir=fileparts(summaryFile); end
T=readtable(summaryFile,'TextType','string');
layouts=unique(T.layout,'stable'); methods=["A","B","C"];
metrics={'target_mean_err_m','Target mean error (m)'; ...
    'est_rms_m','Position estimate RMS (m) [B: corrected track]'; ...
    'landing_err_m','True landing error (m)'; ...
    'anchor_frames_detected_frac','Frames with a landmark sighting (fraction)'};
figs(1)=figure('Color','w','Position',[60 60 1400 900],'Name','Landmark layouts');
tl=tiledlayout(figs(1),2,2,'TileSpacing','compact');
for k=1:size(metrics,1)
    ax=nexttile(tl); groupedSeeds(ax,T(T.yaw=="fixed",:),layouts,methods,metrics{k,1});
    ylabel(ax,metrics{k,2}); title(ax,[metrics{k,2} ', fixed yaw'],'FontWeight','normal');
end
title(tl,{'Landmark layouts (fixed yaw). Bars: mean over seeds; dots: individual seeds.', ...
    'Simulation with ASSUMED sensor errors; lecturer RPT (not PX4); study EKF (not EKF2)'});
exportgraphics(figs(1),fullfile(outDir,'landmarks.png'),'Resolution',150);
yawMetrics={'target_mean_err_m','Target mean error (m)'; ...
    'est_rms_m','Position estimate RMS (m)'; 'heading_rms_deg','Heading estimate RMS (deg)'; ...
    'yaw_gated_frac','Camera frames rejected by the 30 deg/s gate (fraction)'; ...
    'targets_confirmed','Targets confirmed (of 3)'; 'track_true_vs_cmd_rms_m','True vs command RMS, sweep (m)'};
figs(2)=figure('Color','w','Position',[60 60 1400 900],'Name','Fixed vs tangent yaw');
tl=tiledlayout(figs(2),2,3,'TileSpacing','compact');
for k=1:size(yawMetrics,1)
    ax=nexttile(tl); yawPairs(ax,T,layouts,yawMetrics{k,1});
    title(ax,yawMetrics{k,2},'FontWeight','normal');
end
title(tl,{'Fixed (blue) vs tangent (orange) yaw; same seeds, path and command timing', ...
    'Rows: method/layout. Simulation with ASSUMED sensor errors; not flight evidence'});
exportgraphics(figs(2),fullfile(outDir,'yaw.png'),'Resolution',150);
end

function groupedSeeds(ax,T,layouts,methods,col)
hold(ax,'on'); grid(ax,'on');
M=nan(numel(layouts),numel(methods)); colours=lines(3);
for i=1:numel(layouts)
    for j=1:numel(methods)
        v=T.(col)(T.layout==layouts(i) & T.method==methods(j));
        if ~isempty(v), M(i,j)=mean(v,'omitnan'); end
    end
end
b=bar(ax,M,'grouped');
for j=1:numel(b)
    b(j).FaceColor=colours(j,:); b(j).DisplayName=char(methods(j));
    x=b(j).XEndPoints;
    for i=1:numel(layouts)
        v=T.(col)(T.layout==layouts(i) & T.method==methods(j));
        plot(ax,repmat(x(i),numel(v),1),v,'k.','MarkerSize',10,'HandleVisibility','off');
    end
end
xticks(ax,1:numel(layouts)); xticklabels(ax,cellstr(layouts)); xtickangle(ax,20);
set(ax,'TickLabelInterpreter','none'); legend(ax,'Location','best');
end

function yawPairs(ax,T,layouts,col)
hold(ax,'on'); grid(ax,'on');
keys=strings(0,1); F=[]; G=[];
for L=layouts.'
    for m=["A","B","C"]
        f=T.(col)(T.layout==L & T.method==m & T.yaw=="fixed");
        g=T.(col)(T.layout==L & T.method==m & T.yaw=="tangent");
        if isempty(f) || isempty(g), continue; end
        keys(end+1)=m+" "+L; F(end+1)=mean(f,'omitnan'); G(end+1)=mean(g,'omitnan'); %#ok<AGROW>
    end
end
b=barh(ax,[F;G].','grouped');
b(1).FaceColor=[.2 .45 .8]; b(2).FaceColor=[.9 .5 .1];
b(1).DisplayName='fixed'; b(2).DisplayName='tangent';
yticks(ax,1:numel(keys)); yticklabels(ax,cellstr(keys)); set(ax,'TickLabelInterpreter','none');
legend(ax,'Location','best');
end
