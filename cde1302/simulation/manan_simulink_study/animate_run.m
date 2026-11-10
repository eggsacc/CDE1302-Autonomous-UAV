function animate_run(source,speedup)
%ANIMATE_RUN Replay a saved baseline run as a moving top-down and 3D view.
%   animate_run('demo')      replays results/fixed_yaw_demo_<mode>/run.mat
%   animate_run(result)      replays a result struct already in memory
%   animate_run('demo',20)   replays at 20x real time (default 10x)
%
%   This is a replay of logged simulation data, not a live simulation. The
%   blue aircraft is simulated truth (ideal feedback). The orange marker is
%   the position command the controller was actually given at that time.
%   Close the figure to stop early.

if nargin<2 || isempty(speedup), speedup=10; end
root=fileparts(mfilename('fullpath'));
if ischar(source) || isstring(source)
    addpath(root);
    cfg=study_config(char(source));
    runFile=fullfile(root,'results',['fixed_yaw_' cfg.preset '_' ...
        cfg.mission.guidanceMode],'run.mat');
    assert(isfile(runFile),'MananStudy:NoRun', ...
        'No run.mat at %s. Run run_baseline or postprocess_baseline first.',runFile);
    loaded=load(runFile,'result'); result=loaded.result;
else
    result=source;
end
c=result.config; t=result.time(:); p=result.position; r=result.reference;
d=result.diagnostics; e=result.euler;
home=c.frames.homeNed(:).'; ha=c.frames.homeArena(:).';
a=c.frames.arenaToLocalYaw; R=[cos(a) -sin(a);sin(a) cos(a)];
toArena=@(xy) (xy-home(1:2))*R+ha;
pa=toArena(p(:,1:2)); ra=toArena(r(:,1:2));
h=home(3)-p(:,3); hr=home(3)-r(:,3);
routeA=[ha;c.mission.routeArena;ha];
names={'TAKEOFF','HOVER','SWEEP','RETURN','HOME SETTLE','DESCEND','ENDPOINT HOLD','DONE','FAILED'};

fig=figure('Name','Replay of saved simulation (not live)','Color','w', ...
    'Position',[80 80 1400 650],'NumberTitle','off');
lay=tiledlayout(fig,1,2,'TileSpacing','compact');
% Top-down arena view
ax1=nexttile(lay); hold(ax1,'on'); axis(ax1,'equal'); grid(ax1,'on');
drawBounds(ax1,c.geometry.outerBounds,[.15 .15 .15],'-');
drawBounds(ax1,c.geometry.searchBounds,[.55 .25 .65],'--');
drawBounds(ax1,c.geometry.startBounds,[.15 .55 .25],'-');
plot(ax1,routeA(:,1),routeA(:,2),'--','Color',[.7 .7 .7]);
trail=plot(ax1,nan,nan,'b-','LineWidth',1.2);
cmd=plot(ax1,nan,nan,'o','Color',[.85 .4 .05],'MarkerSize',7,'LineWidth',1.5);
body=plot(ax1,nan,nan,'k-','LineWidth',2);         % arm cross, rotated by yaw
nose=plot(ax1,nan,nan,'r-','LineWidth',2.5);       % nose direction (fixed yaw)
xlabel(ax1,'Arena x (m)'); ylabel(ax1,'Arena y (m)');
if all(isfinite(c.geometry.outerBounds),'all')
    xlim(ax1,c.geometry.outerBounds(1,:)+[-.3 .3]);
    ylim(ax1,c.geometry.outerBounds(2,:)+[-.3 .3]);
end
legend(ax1,{'Outer boundary','Search area','Start zone','Planned route', ...
    'True path','Position command','Airframe','Nose'},'Location','southoutside', ...
    'NumColumns',4);
% 3D view
ax2=nexttile(lay); hold(ax2,'on'); grid(ax2,'on'); view(ax2,-35,28);
plot3(ax2,routeA(:,1),routeA(:,2),c.mission.heightM*ones(size(routeA,1),1), ...
    '--','Color',[.7 .7 .7]);
trail3=plot3(ax2,nan,nan,nan,'b-','LineWidth',1.2);
cmd3=plot3(ax2,nan,nan,nan,'o','Color',[.85 .4 .05],'MarkerSize',7,'LineWidth',1.5);
body3=plot3(ax2,nan,nan,nan,'k-','LineWidth',2);
drop=plot3(ax2,nan,nan,nan,':','Color',[.4 .4 .4]);
if all(isfinite(c.geometry.outerBounds),'all')
    b=c.geometry.outerBounds;
    patch(ax2,b(1,[1 2 2 1]),b(2,[1 1 2 2]),[0 0 0 0],[.93 .93 .93],'EdgeColor',[.3 .3 .3]);
    xlim(ax2,b(1,:)); ylim(ax2,b(2,:));
end
zlim(ax2,[-0.1 max(0.8,max(h)+0.1)]); daspect(ax2,[1 1 1]);
xlabel(ax2,'x (m)'); ylabel(ax2,'y (m)'); zlabel(ax2,'Height above home (m)');
ttl=title(lay,'','Interpreter','none');
subtitle(lay,sprintf('%s | Lecturer RPT model, ideal feedback; replay of logged data at %gx', ...
    char(c.geometry.label),speedup),'Interpreter','none');

arm=0.12; % Display size only; not the real vehicle's span.
frameDt=0.04; % wall seconds per frame
stepT=frameDt*speedup;
nextT=t(1); k=1; n=numel(t); tic0=tic;
while k<=n && isvalid(fig)
    k=find(t>=nextT,1); if isempty(k), break; end
    yaw=e(k,3)+a; cy=cos(yaw); sy=sin(yaw);
    cx=pa(k,1); cyy=pa(k,2);
    armsX=[cx-arm*cy, cx+arm*cy, nan, cx+arm*sy, cx-arm*sy];
    armsY=[cyy-arm*sy, cyy+arm*sy, nan, cyy-arm*cy, cyy+arm*cy];
    set(trail,'XData',pa(1:k,1),'YData',pa(1:k,2));
    set(cmd,'XData',ra(k,1),'YData',ra(k,2));
    set(body,'XData',armsX,'YData',armsY);
    set(nose,'XData',[cx cx+1.6*arm*cy],'YData',[cyy cyy+1.6*arm*sy]);
    set(trail3,'XData',pa(1:k,1),'YData',pa(1:k,2),'ZData',h(1:k));
    set(cmd3,'XData',ra(k,1),'YData',ra(k,2),'ZData',hr(k));
    set(body3,'XData',armsX,'YData',armsY,'ZData',h(k)*ones(1,5));
    set(drop,'XData',[cx cx],'YData',[cyy cyy],'ZData',[0 h(k)]);
    st=d(k,1); stName='?'; if st>=1 && st<=9, stName=names{st}; end
    set(ttl,'String',sprintf('t = %6.1f s   stage: %s   height %.2f m   command-to-true gap %.3f m', ...
        t(k),stName,h(k),norm(p(k,:)-r(k,1:3))));
    drawnow limitrate;
    nextT=nextT+stepT;
    % Pace to wall clock so the replay speed is honest.
    target=(nextT-t(1))/speedup; lag=target-toc(tic0);
    if lag>0, pause(min(lag,frameDt)); end
end
if isvalid(fig), set(ttl,'String',[get(ttl,'String') '   (replay finished)']); end
end

function drawBounds(ax,b,colour,style)
if any(~isfinite(b),'all'), return; end
plot(ax,b(1,[1 2 2 1 1]),b(2,[1 1 2 2 1]),'Color',colour,'LineStyle',style,'LineWidth',1.4);
end
