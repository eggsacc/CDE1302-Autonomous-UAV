function app = uav_study_gui()
%UAV_STUDY_GUI Interactive AprilTag-landmark and yaw study on the lecturer model.
%   uav_study_gui   opens the app (MATLAB desktop; figures do not render in
%   -batch mode on this machine).
%
%   Left: top-down arena (ASSUMED demo geometry) with the start box, search
%   area, planned lawnmower route, the three random target tags (simulation
%   truth, unknown to the drone) and your known landmark tags.
%   Right: set up the flight, place landmarks (preset, click on the map, or
%   type in the table), choose fixed or tangent yaw and the methods, Run.
%   Each run is a full Simulink simulation of the lecturer quadrotor with the
%   study estimator in the loop (about 1-3 minutes each).
%
%   Methods: A = optical-flow dead reckoning only. B = the team's post-flight
%   correction applied to the A flight (report only). C = online landmark
%   corrections fed to the controller.
root=fileparts(mfilename('fullpath')); addpath(root);
S.runs={}; S.labels={}; S.clickMode=false; S.cfg=[]; S.replay=[];
fig=uifigure('Name','CDE1302 landmark and yaw study (simulation)', ...
    'Position',[60 60 1400 820]);
g=uigridlayout(fig,[1 2]); g.ColumnWidth={'2x','1x'};
% ---------------- map ----------------
mapPanel=uipanel(g,'Title','Arena (assumed demo geometry; official offsets unverified)');
mg=uigridlayout(mapPanel,[2 1]); mg.RowHeight={'1x',70};
ax=uiaxes(mg); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'Arena x (m)'); ylabel(ax,'Arena y (m)');
ax.ButtonDownFcn=@onMapClick;
rg=uigridlayout(mg,[2 4]); rg.ColumnWidth={110,'1x',80,80}; rg.RowHeight={22,22};
uilabel(rg,'Text','Replay run:');
runDrop=uidropdown(rg,'Items',{'(no runs yet)'},'ValueChangedFcn',@(~,~) setReplayRun());
playBtn=uibutton(rg,'Text','Play','ButtonPushedFcn',@(~,~) play());
uibutton(rg,'Text','Stop','ButtonPushedFcn',@(~,~) stopPlay());
uilabel(rg,'Text','Time (s):');
timeSlider=uislider(rg,'Limits',[0 1],'ValueChangingFcn',@(~,ev) drawReplay(ev.Value), ...
    'ValueChangedFcn',@(src,~) drawReplay(src.Value));
timeSlider.Layout.Row=2; timeSlider.Layout.Column=[2 4];
% ---------------- controls ----------------
tabs=uitabgroup(g);
setupTab=uitab(tabs,'Title','Setup'); resultTab=uitab(tabs,'Title','Results');
sg=uigridlayout(setupTab,[18 2]);
sg.RowHeight=[repmat({24},1,10),{150},repmat({24},1,5),{44},{'1x'}];
sg.ColumnWidth={150,'1x'}; sg.Scrollable='on';
uilabel(sg,'Text','Flight preset');
presetDrop=uidropdown(sg,'Items',{'team_loc_tags','phase1_demo'}, ...
    'ItemsData',{'team_loc_tags','phase1_demo'},'ValueChangedFcn',@onPreset);
uilabel(sg,'Text','Height (m)'); heightEdit=uieditfield(sg,'numeric','Value',0.8, ...
    'Limits',[0.3 1.5],'ValueChangedFcn',@(~,~) refresh());
uilabel(sg,'Text','Lane spacing (m)'); laneEdit=uieditfield(sg,'numeric','Value',1.0, ...
    'Limits',[0.3 2],'ValueChangedFcn',@(~,~) refresh());
uilabel(sg,'Text','Speed (m/s)'); speedEdit=uieditfield(sg,'numeric','Value',0.30, ...
    'Limits',[0.05 0.5],'ValueChangedFcn',@(~,~) refresh());
uilabel(sg,'Text','Yaw mode');
yawDrop=uidropdown(sg,'Items',{'fixed','tangent','both (paired)'}, ...
    'ItemsData',{'fixed','tangent','both'},'Value','fixed');
uilabel(sg,'Text','Methods to simulate');
methodDrop=uidropdown(sg,'Items',{'A + B (one flight)','C (online)','A + B and C'}, ...
    'ItemsData',{'A','C','AC'},'Value','AC');
uilabel(sg,'Text','Random seed'); seedEdit=uispinner(sg,'Value',1,'Limits',[1 9999], ...
    'Step',1,'RoundFractionalValues','on','ValueChangedFcn',@(~,~) refresh());
uilabel(sg,'Text','Target tags'); nTargetEdit=uispinner(sg,'Value',3,'Limits',[1 8], ...
    'Step',1,'RoundFractionalValues','on','ValueChangedFcn',@(~,~) refresh());
uilabel(sg,'Text','Landmark preset');
layoutDrop=uidropdown(sg,'Items',[anchor_layout() {'custom'}],'Value','home_corners5', ...
    'ValueChangedFcn',@onLayout);
clickBtn=uibutton(sg,'state','Text','Click map to add landmark', ...
    'ValueChangedFcn',@(src,~) setClick(src.Value));
place(clickBtn,10,[1 2]);
anchorTable=uitable(sg,'ColumnName',{'ID','x (m)','y (m)'},'ColumnEditable',[true true true], ...
    'CellEditCallback',@(~,~) onTableEdit(),'Data',zeros(0,3));
place(anchorTable,11,[1 2]);
removeBtn=uibutton(sg,'Text','Remove selected landmark','ButtonPushedFcn',@(~,~) removeSelected());
place(removeBtn,12,1);
clearBtn=uibutton(sg,'Text','Clear landmarks','ButtonPushedFcn',@(~,~) setAnchors(zeros(0,3),'custom'));
place(clearBtn,12,2);
showTruth=uicheckbox(sg,'Text','Show true target positions (unknown to the drone)', ...
    'Value',true,'ValueChangedFcn',@(~,~) refresh());
place(showTruth,13,[1 2]);
showFoot=uicheckbox(sg,'Text','Show camera footprint during replay','Value',true);
place(showFoot,14,[1 2]);
runBtn=uibutton(sg,'Text','Run simulation','FontWeight','bold', ...
    'BackgroundColor',[0.85 0.93 0.85],'ButtonPushedFcn',@(~,~) runAll());
place(runBtn,15,1);
stopBtn=uibutton(sg,'Text','Stop run','Enable','off', ...
    'BackgroundColor',[0.95 0.85 0.85],'ButtonPushedFcn',@(~,~) stopRun());
place(stopBtn,15,2);
place(uibutton(sg,'Text','Clear results','ButtonPushedFcn',@(~,~) clearResults()),16,1);
place(uibutton(sg,'Text','Comparison plots','ButtonPushedFcn',@(~,~) comparisonPlots()),16,2);
statusLabel=uilabel(sg,'Text','Ready. Place landmarks, then Run.','WordWrap','on');
place(statusLabel,17,[1 2]);
infoText=uitextarea(sg,'Editable','off','Value',{''});
place(infoText,18,[1 2]);
rg2=uigridlayout(resultTab,[2 1]); rg2.RowHeight={'1x',120};
resultTable=uitable(rg2,'ColumnName',{'Run','Est. RMS (m)','Heading RMS (deg)', ...
    'True vs cmd RMS (m)','Landing err (m)','Targets confirmed','Target mean err (m)', ...
    'B target mean err (m)','Anchor in view %','Anchor used %','Yaw-gated frames %'});
noteText=uitextarea(rg2,'Editable','off','Value',{['All numbers come from simulation ' ...
    'with ASSUMED sensor errors (scenario_config.m), the lecturer RPT controller ' ...
    '(not PX4) and a study EKF (not EKF2). They are not validated against flight logs.']});
app.figure=fig;
onLayout(layoutDrop,[]);

% ================= callbacks =================
    function opts=currentOpts(yawMode,method)
        opts=struct('flightPreset',presetDrop.Value,'heightM',heightEdit.Value, ...
            'laneSpacingM',laneEdit.Value,'speedMps',speedEdit.Value, ...
            'yawMode',yawMode,'method',method,'seed',seedEdit.Value, ...
            'nTargets',nTargetEdit.Value,'anchors',anchorTable.Data);
    end

    function onPreset(src,~)
        if strcmp(src.Value,'team_loc_tags'), v=[0.8 1.0 0.30]; else, v=[0.5 0.6 0.15]; end
        heightEdit.Value=v(1); laneEdit.Value=v(2); speedEdit.Value=v(3); refresh();
    end

    function onLayout(src,~)
        if strcmp(src.Value,'custom'), refresh(); return; end
        base=scenario_config(currentOpts('fixed','A'));
        setAnchors(anchor_layout(src.Value,base),src.Value);
    end

    function setAnchors(A,name)
        anchorTable.Data=A; layoutDrop.Value=name; refresh();
    end

    function onTableEdit()
        layoutDrop.Value='custom'; refresh();
    end

    function setClick(on)
        S.clickMode=on;
        if on, statusLabel.Text='Click on the map to add a floor landmark. Toggle the button off when done.';
        else, statusLabel.Text='Ready.'; end
    end

    function onMapClick(~,~)
        if ~S.clickMode, return; end
        pt=ax.CurrentPoint(1,1:2); pt=round(pt,2);
        A=anchorTable.Data; used=[A(:,1);(0:21).'];
        id=22; while any(used==id), id=id+1; end
        if id>29
            uialert(fig,'Ids above 29 do not exist in tag16h5; the simulation still runs.', ...
                'Landmark id','Icon','warning');
        end
        anchorTable.Data=[A;id pt]; layoutDrop.Value='custom'; refresh();
    end

    function removeSelected()
        sel=anchorTable.Selection;
        if isempty(sel), statusLabel.Text='Select a row in the landmark table first.'; return; end
        rows=unique(sel(:,1)); A=anchorTable.Data; A(rows,:)=[];
        anchorTable.Data=A; layoutDrop.Value='custom'; refresh();
    end

    function refresh()
        try
            S.cfg=scenario_config(currentOpts(yawDrop.Value,'A'));
        catch err
            statusLabel.Text=['Configuration error: ' err.message]; return;
        end
        drawArena();
        p=S.cfg.mission.turnPlan; c=S.cfg;
        fp=nav_core('footprint',[0;0;-c.mission.heightM],[1 0 0 0],c);
        infoText.Value={sprintf('Route: %d corners, %.1f m, planned %.0f s (incl. %.1f s of matched turn pauses).', ...
            size(c.mission.routeNed,1),sum(p.legLengthM),p.plannedDurationS,p.totalPauseS), ...
            sprintf('Camera footprint at %.2f m: %.2f m across x %.2f m along track (team intrinsics).', ...
            c.mission.heightM,max(fp(2,:))-min(fp(2,:)),max(fp(1,:))-min(fp(1,:))), ...
            p.strictTangentConflict, ...
            sprintf('Landmarks: %d. Targets: %d (ids %s).',numel(c.anchors.id), ...
            numel(c.targets.id),mat2str(c.targets.id.'))};
    end

    function drawArena()
        cla(ax); c=S.cfg; hold(ax,'on');
        drawBox(c.geometry.outerBounds,[.2 .2 .2],'-',1.5);
        drawBox(c.geometry.searchBounds,[.55 .25 .65],'--',1.2);
        drawBox(c.geometry.startBounds,[.15 .55 .25],'-',1.5);
        r=[c.frames.homeArena.';c.mission.routeArena;c.frames.homeArena.'];
        plot(ax,r(:,1),r(:,2),':','Color',[.5 .5 .5],'LineWidth',1,'HitTest','off');
        if showTruth.Value
            for k=1:numel(c.targets.id)
                tagSquare(c.targets.arena(k,:),c.targets.yawRad(k),c.nav.targetSizeM,[.85 .4 .05]);
                text(ax,c.targets.arena(k,1)+0.08,c.targets.arena(k,2)+0.08, ...
                    sprintf('T%d',c.targets.id(k)),'Color',[.75 .3 0],'HitTest','off','FontSize',9);
            end
        end
        for k=1:numel(c.anchors.id)
            tagSquare(c.anchors.arena(k,:),0,c.nav.anchorSizeM,[.1 .55 .2]);
            text(ax,c.anchors.arena(k,1)+0.08,c.anchors.arena(k,2)-0.1, ...
                sprintf('L%d',c.anchors.id(k)),'Color',[.05 .45 .15],'HitTest','off', ...
                'FontWeight','bold','FontSize',9);
        end
        b=c.geometry.outerBounds; xlim(ax,b(1,:)+[-.2 .2]); ylim(ax,b(2,:)+[-.2 .2]);
        title(ax,'Orange T = unknown targets (truth)  |  Green L = known landmarks','FontWeight','normal');
        S.replay=[];
    end

    function drawBox(b,colour,style,w)
        if any(~isfinite(b),'all'), return; end
        plot(ax,b(1,[1 2 2 1 1]),b(2,[1 1 2 2 1]),'Color',colour,'LineStyle',style, ...
            'LineWidth',w,'HitTest','off');
    end

    function tagSquare(xy,yaw,s,colour)
        h=s/2; P=[-h h h -h;-h -h h h]; R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
        Q=R*P+xy(:);
        fill(ax,Q(1,:),Q(2,:),colour,'FaceAlpha',.6,'EdgeColor',colour*.6,'HitTest','off');
    end

    function runAll()
        if strcmp(yawDrop.Value,'both'), yaws={'fixed','tangent'}; else, yaws={yawDrop.Value}; end
        methods=num2cell(methodDrop.Value);
        n=numel(yaws)*numel(methods); k=0; S.cancel=false;
        runBtn.Enable='off'; stopBtn.Enable='on';
        cleanup=onCleanup(@() finishRuns());
        for y=yaws
            for m=methods
                if S.cancel, break; end
                k=k+1;
                opts=currentOpts(y{1},m{1});
                S.cfg=scenario_config(opts); drawArena(); startLive();
                statusLabel.Text=sprintf(['Run %d of %d: %s yaw, method %s. Building ' ...
                    'and simulating (about 1-2 min); the map updates live.'],k,n,y{1},m{1});
                drawnow;
                try
                    nav_live('set',@liveUpdate,0.2);
                    res=run_nav_scenario(S.cfg,'quiet',true);
                    nav_live('clear');
                catch err
                    nav_live('clear');
                    uialert(fig,getReport(err,'basic','hyperlinks','off'),'Simulation failed');
                    return;
                end
                addRun(res);
                if ~res.metrics.mission.completed && S.cancel
                    statusLabel.Text='Run stopped early; its partial result is listed.';
                end
            end
            if S.cancel, break; end
        end
        statusLabel.Text=sprintf(['Finished %d run(s). Results tab has the numbers; ' ...
            'Play under the map replays the selected run. In the Command Window: ' ...
            'print_nav_summary(guiRuns)'],k);
    end

    function finishRuns()
        nav_live('clear');
        if isvalid(fig), runBtn.Enable='on'; stopBtn.Enable='off'; end
    end

    function stopRun()
        S.cancel=true;
        try
            if bdIsLoaded('MananStudy_Nav')
                set_param('MananStudy_Nav','SimulationCommand','stop');
            end
        catch
            statusLabel.Text='Will stop after the current run (Simulink refused an immediate stop).';
            return;
        end
        statusLabel.Text='Stopping after the current simulation step...';
    end

    function startLive()
        c=S.cfg;
        S.live.trueXY=zeros(0,2); S.live.estXY=zeros(0,2);
        S.live.trueLine=plot(ax,nan,nan,'b-','LineWidth',1.2,'HitTest','off');
        S.live.estLine=plot(ax,nan,nan,'--','Color',[.85 .1 .1],'LineWidth',1,'HitTest','off');
        S.live.foot=fill(ax,nan(1,4),nan(1,4),[.3 .6 1],'FaceAlpha',.15, ...
            'EdgeColor',[.2 .4 .9],'HitTest','off');
        S.live.body=plot(ax,nan,nan,'k-','LineWidth',2,'HitTest','off');
        S.live.nose=plot(ax,nan,nan,'r-','LineWidth',3,'HitTest','off');
        S.live.est=plot(ax,nan,nan,'o','Color',[.85 .1 .1],'MarkerSize',9,'LineWidth',2,'HitTest','off');
        S.live.text=text(ax,0.02,0.98,'starting...','Units','normalized', ...
            'VerticalAlignment','top','BackgroundColor',[1 1 1 .85],'HitTest','off');
        a=c.frames.arenaToLocalYaw;
        S.live.Rm=[cos(a) -sin(a);sin(a) cos(a)];
        title(ax,{'LIVE: blue = true path, red dashed = estimated path, blue patch = camera view', ...
            c.scenario.description},'FontWeight','normal','Interpreter','none');
    end

    function liveUpdate(t,pos,quat,dg)
        if ~isvalid(fig) || ~isfield(S,'live'), return; end
        c=S.cfg; toA=@(ne) (ne(:).'-c.frames.homeNed(1:2).')*S.live.Rm+c.frames.homeArena.';
        tr=toA(pos(1:2)); es=toA(dg(1:2));
        S.live.trueXY(end+1,:)=tr; S.live.estXY(end+1,:)=es;
        set(S.live.trueLine,'XData',S.live.trueXY(:,1),'YData',S.live.trueXY(:,2));
        set(S.live.estLine,'XData',S.live.estXY(:,1),'YData',S.live.estXY(:,2));
        q=quat(:)/norm(quat);
        yaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2))+c.frames.arenaToLocalYaw;
        a=0.15;
        set(S.live.body,'XData',[tr(1)+[-a a]*cos(yaw),nan,tr(1)+[a -a]*sin(yaw)], ...
            'YData',[tr(2)+[-a a]*sin(yaw),nan,tr(2)+[-a a]*cos(yaw)]);
        set(S.live.nose,'XData',tr(1)+[0 1.8*a*cos(yaw)],'YData',tr(2)+[0 1.8*a*sin(yaw)]);
        set(S.live.est,'XData',es(1),'YData',es(2));
        fp=nav_core('footprint',pos(:),q.',c);
        if all(isfinite(fp),'all')
            corners=(fp.'-c.frames.homeNed(1:2).')*S.live.Rm+c.frames.homeArena.'; % fp is 2x4 [N;E]
            set(S.live.foot,'XData',corners(:,1).','YData',corners(:,2).');
        else
            set(S.live.foot,'XData',nan(1,4),'YData',nan(1,4));
        end
        S.live.text.String=sprintf(['t = %.1f s  |  height %.2f m  |  estimate error %.3f m  |  ' ...
            'heading error %.2f deg  |  landmarks in view %d  |  landmark fixes used %d'], ...
            t,c.frames.homeNed(3)-pos(3),norm(dg(1:2)-pos(1:2)), ...
            rad2deg(angle(exp(1i*(dg(3)-yaw+c.frames.arenaToLocalYaw)))),dg(7),dg(9));
        drawnow limitrate;
    end

    function addRun(res)
        c=res.config;
        label=sprintf('#%d %s %s seed%d %dL',numel(S.runs)+1,c.mission.yawMode, ...
            c.scenario.method,c.scenario.seed,numel(c.anchors.id));
        S.runs{end+1}=res; S.labels{end+1}=label;
        if strcmp(c.scenario.method,'A'), S.labels{end}=[label ' (+B)']; end
        % Hand every run back to the Command Window: print_nav_summary(guiRuns)
        assignin('base','guiRuns',S.runs); assignin('base','guiRunLabels',S.labels);
        fig.UserData=struct('runs',{S.runs},'labels',{S.labels});
        updateTable(); runDrop.Items=S.labels; runDrop.ItemsData=1:numel(S.runs);
        runDrop.Value=numel(S.runs); setReplayRun();
    end

    function updateTable()
        rows=cell(numel(S.runs),11);
        for i=1:numel(S.runs)
            m=S.runs{i}.metrics; tB=NaN;
            if isfield(m.targets,'methodB'), tB=m.targets.methodB.meanErrorConfirmedM; end
            rows(i,:)={S.labels{i},r3(m.estimation.rmsM),r3(m.heading.rmsDeg), ...
                r3(m.tracking.rmsTrueVsCommandSweepM),r3(m.landing.trueHorizontalErrorM), ...
                sprintf('%d/%d',m.targets.inFlight.confirmed,m.targets.inFlight.count), ...
                r3(m.targets.inFlight.meanErrorConfirmedM),r3(tB), ...
                r3(100*m.anchors.fractionFramesAnchorVisible),r3(100*m.anchors.fractionFramesAnchorUsed), ...
                r3(100*m.camera.fractionFramesYawGated)};
        end
        resultTable.Data=rows;
    end

    function v=r3(x), v=round(x,3); end

    function clearResults()
        S.runs={}; S.labels={}; resultTable.Data={};
        runDrop.ItemsData={}; runDrop.Items={'(no runs yet)'}; refresh();
    end

    function setReplayRun()
        if isempty(S.runs), return; end
        res=S.runs{runDrop.Value}; S.cfg=res.config;
        drawArena(); c=res.config;
        toA=@(ne) ne-c.frames.homeNed(1:2).'+c.frames.homeArena.'; % zero arena rotation
        if c.frames.arenaToLocalYaw~=0
            Rm=[cos(c.frames.arenaToLocalYaw) -sin(c.frames.arenaToLocalYaw); ...
                sin(c.frames.arenaToLocalYaw) cos(c.frames.arenaToLocalYaw)];
            toA=@(ne) (ne-c.frames.homeNed(1:2).')*Rm+c.frames.homeArena.';
        end
        tr=toA(res.position(:,1:2)); es=toA(res.estimatedPosition(:,1:2));
        plot(ax,tr(:,1),tr(:,2),'b-','LineWidth',1.2,'HitTest','off');
        plot(ax,es(:,1),es(:,2),'--','Color',[.85 .1 .1],'LineWidth',1,'HitTest','off');
        for i=1:numel(res.tags)
            if all(isfinite(res.tags(i).estimate))
                e=toA(res.tags(i).estimate);
                plot(ax,e(1),e(2),'kx','MarkerSize',10,'LineWidth',2,'HitTest','off');
                text(ax,e(1)-0.1,e(2)-0.12,sprintf('%d',res.tags(i).id),'HitTest','off','FontSize',8);
            end
        end
        if isfield(res,'methodB')
            for i=1:numel(res.methodB.targets)
                if all(isfinite(res.methodB.targets(i).estimate))
                    e=toA(res.methodB.targets(i).estimate);
                    plot(ax,e(1),e(2),'m+','MarkerSize',10,'LineWidth',2,'HitTest','off');
                end
            end
        end
        title(ax,{['Blue = true path, red dashed = estimated path; x = reported target' ...
            ', magenta + = Method B corrected'],res.config.scenario.description}, ...
            'FontWeight','normal','Interpreter','none');
        S.replay.res=res; S.replay.toA=toA;
        S.replay.dyn=gobjects(0);
        timeSlider.Limits=[res.time(1) res.time(end)]; timeSlider.Value=res.time(1);
        drawReplay(res.time(1));
    end

    function drawReplay(tq)
        if isempty(S.replay), return; end
        delete(S.replay.dyn(isgraphics(S.replay.dyn))); S.replay.dyn=gobjects(0);
        res=S.replay.res; c=res.config; [~,k]=min(abs(res.time-tq));
        tr=S.replay.toA(res.position(k,1:2)); es=S.replay.toA(res.estimatedPosition(k,1:2));
        yaw=res.euler(k,3)+c.frames.arenaToLocalYaw; a=0.15;
        h=[plot(ax,tr(1)+[-a a]*cos(yaw),tr(2)+[-a a]*sin(yaw),'k-','LineWidth',2,'HitTest','off'), ...
            plot(ax,tr(1)+[a -a]*sin(yaw),tr(2)+[-a a]*cos(yaw),'k-','LineWidth',2,'HitTest','off'), ...
            plot(ax,tr(1)+[0 1.8*a*cos(yaw)],tr(2)+[0 1.8*a*sin(yaw)],'r-','LineWidth',3,'HitTest','off'), ...
            plot(ax,es(1),es(2),'o','Color',[.85 .1 .1],'MarkerSize',9,'LineWidth',2,'HitTest','off')];
        if showFoot.Value
            fp=nav_core('footprint',res.position(k,:).',res.quaternion(k,:),c);
            if all(isfinite(fp),'all')
                fa=S.replay.toA(fp.');
                h(end+1)=fill(ax,fa(:,1),fa(:,2),[.3 .6 1],'FaceAlpha',.15, ...
                    'EdgeColor',[.2 .4 .9],'HitTest','off');
            end
        end
        stage={'TAKEOFF','HOVER','SWEEP','RETURN','HOME SETTLE','DESCEND','ENDPOINT','DONE','FAILED'};
        st=res.diagnostics(k,1); ttl='';
        if st>=1 && st<=9, ttl=stage{st}; end
        nk=find(res.nav.time<=res.time(k),1,'last');
        h(end+1)=text(ax,0.05,0.97,sprintf('t = %.1f s  %s  |  est. error %.3f m  |  landmarks in view %d', ...
            res.time(k),ttl,norm(res.estimatedPosition(k,1:2)-res.position(k,1:2)), ...
            res.nav.data(nk,7)),'Units','normalized','VerticalAlignment','top', ...
            'BackgroundColor',[1 1 1 .8],'HitTest','off');
        S.replay.dyn=h;
    end

    function play()
        if isempty(S.replay), return; end
        S.playing=true; playBtn.Enable='off';
        t0=timeSlider.Value; tEnd=timeSlider.Limits(2); speed=15; tic0=tic;
        while S.playing && isvalid(fig)
            tq=t0+speed*toc(tic0);
            if tq>tEnd, tq=tEnd; S.playing=false; end
            timeSlider.Value=tq; drawReplay(tq); drawnow limitrate; pause(0.03);
        end
        if isvalid(fig), playBtn.Enable='on'; end
    end

    function stopPlay(), S.playing=false; end

    function place(h,row,col)
        h.Layout.Row=row; h.Layout.Column=col;
    end

    function comparisonPlots()
        if isempty(S.runs), statusLabel.Text='Run at least one simulation first.'; return; end
        plot_nav_comparison(S.runs,S.labels);
    end
end
