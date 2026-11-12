function B = nav_postcorrect(res,lo)
%NAV_POSTCORRECT Method B: the team's post-flight drift correction, in MATLAB.
%   B = nav_postcorrect(res) uses the landmark sightings recorded during a
%   Method A flight (same flight, nothing fed back) and corrects the reported
%   target positions and, for analysis, the estimated trajectory.
%
% Port of cde1302/mission/localisation.py and tag_map.robust_estimate on
% origin/feature/loc-tags (commit 690637e, 11 Nov 2026):
%   visits (gap > visit_gap_s splits) with >= min_visit_sightings become
%   fixes at their dense-seeded median; a global similarity / rigid /
%   translation Umeyama fit with span, rms, rotation and scale checks and one
%   leave-one-out trim; 'time' and 'hybrid' drift interpolated between visit
%   times; 'auto' picks the lowest leave-one-out error (>= 3 fixes).
% Differences: none intended. Any mismatch is a porting bug, not a finding.
if nargin<2 || isempty(lo), lo=defaults(); end
cfg=res.config; S=res.sightings;
B.parameters=lo;
anchors=S(S.is_anchor==1,:);
fixes=findFixes(anchors,cfg,lo);
B.fixes=fixes;
B.fit=solveFixes(fixes,lo);
B.correction=driftCorrection(B.fit,fixes,lo);
B.description=describe(B.correction);
% Corrected targets: every sighting at its own time, then the robust median.
targets=S(S.is_anchor==0,:);
B.targets=correctTargets(B.correction,targets,cfg,lo);
% Corrected trajectory (analysis only; a post-flight fix cannot move the drone).
tn=res.nav.time; ne=res.nav.estimatedNE;
B.trajectoryNE=applyMany(B.correction,tn,ne);
B.trajectoryTime=tn;
end

function lo=defaults()
% feature/loc-tags config.yaml `localisation:` and `tags:` values.
lo.model='similarity'; lo.driftModel='auto';
lo.visitGapS=5.0; lo.minVisitSightings=5; lo.minSpanM=1.0;
lo.maxRmsM=0.15; lo.maxRotationDeg=20.0; lo.maxScaleError=0.25;
lo.confirmRadiusM=0.20; lo.outlierRadiusM=0.30; lo.trimK=3.0;
lo.minSightings=5;
end

% ------------------------------------------------------------ fixes
function fixes=findFixes(A,cfg,lo)
fixes=struct('id',{},'visit',{},'tStart',{},'tEnd',{},'n',{},'px4',{},'ref',{},'spread',{});
ids=unique(A.id).';
for id=ids
    rows=sortrows(A(A.id==id,:),'t_capture');
    ref=cfg.anchors.ned(cfg.anchors.id==id,1:2);
    t=rows.t_capture; pts=[rows.est_n rows.est_e];
    starts=[1;find(diff(t)>lo.visitGapS)+1]; stops=[starts(2:end)-1;numel(t)];
    visit=0;
    for g=1:numel(starts)
        idx=starts(g):stops(g);
        if numel(idx)<lo.minVisitSightings, continue; end
        [est,inl]=robustEstimate(pts(idx,:),lo.confirmRadiusM,lo.outlierRadiusM);
        visit=visit+1;
        spread=median(vecnorm(pts(idx(inl),:)-est,2,2));
        fixes(end+1)=struct('id',id,'visit',visit,'tStart',t(idx(1)),'tEnd',t(idx(end)), ...
            'n',numel(idx),'px4',est,'ref',ref,'spread',spread); %#ok<AGROW>
    end
end
if ~isempty(fixes)
    [~,order]=sort([fixes.tStart]); fixes=fixes(order);
end
end

function [est,inliers,support]=robustEstimate(pts,confirmR,outlierR)
% tag_map.robust_estimate: dense-seeded median with one refinement pass.
d=sqrt((pts(:,1)-pts(:,1).').^2+(pts(:,2)-pts(:,2).').^2);
[~,seed]=max(sum(d<=confirmR,2));
inliers=d(seed,:).'<=outlierR;
est=median(pts(inliers,:),1);
dd=vecnorm(pts-est,2,2);
if any(dd<=outlierR)
    inliers=dd<=outlierR; est=median(pts(inliers,:),1); dd=vecnorm(pts-est,2,2);
end
support=nnz(dd<=confirmR);
end

% ------------------------------------------------------------ global fit
function T=fitTransform(src,dst,model)
T=struct('model',model,'scale',1,'theta',0,'t',[0 0]);
if strcmp(model,'translation')
    T.t=mean(dst-src,1); return;
end
ms=mean(src,1); md=mean(dst,1); a=src-ms; b=dst-md;
dotp=sum(a.*b,'all'); crossp=sum(a(:,1).*b(:,2)-a(:,2).*b(:,1));
T.theta=atan2(crossp,dotp);
if strcmp(model,'similarity')
    T.scale=hypot(dotp,crossp)/sum(a.^2,'all');
end
T.t=md-(matrix(T)*ms.').';
end

function M=matrix(T)
c=cos(T.theta); s=sin(T.theta); M=T.scale*[c -s;s c];
end

function q=applyT(T,p)
q=(matrix(T)*p.').'+T.t;
end

function s=span(pts)
if size(pts,1)<2, s=0; return; end
s=max(sqrt((pts(:,1)-pts(:,1).').^2+(pts(:,2)-pts(:,2).').^2),[],'all');
end

function problem=checkFit(T,rms,lo)
problem='';
if rms>lo.maxRmsM, problem=sprintf('rms %.3f m > %.2f',rms,lo.maxRmsM);
elseif abs(rad2deg(T.theta))>lo.maxRotationDeg, problem=sprintf('turn %.1f deg',rad2deg(T.theta));
elseif abs(T.scale-1)>lo.maxScaleError, problem=sprintf('scale %.3f',T.scale);
end
end

function [worst,T]=trimOne(src,dst,model,lo)
worst=0; T=[]; best=inf; bestOff=0; bestT=[];
for i=1:size(src,1)
    keep=true(size(src,1),1); keep(i)=false;
    if ~strcmp(model,'translation') && span(dst(keep,:))<lo.minSpanM, continue; end
    Ti=fitTransform(src(keep,:),dst(keep,:),model);
    r=vecnorm(applyT(Ti,src)-dst,2,2);
    rmsRest=sqrt(mean(r(keep).^2));
    if rmsRest<best, best=rmsRest; worst=i; bestT=Ti; bestOff=r(i); end
end
if worst==0, return; end
if bestOff>max(lo.trimK*max(best,0.01),lo.maxRmsM), T=bestT; else, worst=0; end
end

function F=solveFixes(fixes,lo)
F=struct('ok',false,'transform',[],'used',[],'residuals',[],'rms',nan,'notes','');
if isempty(fixes), F.notes='no localisation tag was seen often enough'; return; end
src=vertcat(fixes.px4); dst=vertcat(fixes.ref);
models={'similarity','rigid','translation'};
start=find(strcmp(models,lo.model));
notes={};
for m=models(start:end)
    model=m{1};
    need=1+~strcmp(model,'translation');
    if ~strcmp(model,'translation') && span(dst)<lo.minSpanM
        notes{end+1}=sprintf('%s: known positions span %.2f m < %.2f',model,span(dst),lo.minSpanM); %#ok<AGROW>
        continue;
    end
    if size(src,1)<need, continue; end
    used=true(size(src,1),1);
    T=fitTransform(src,dst,model);
    r=vecnorm(applyT(T,src)-dst,2,2);
    if size(src,1)>need+1 && max(r)>lo.maxRmsM
        [w,Tt]=trimOne(src,dst,model,lo);
        if w>0
            used(w)=false; T=Tt; r=vecnorm(applyT(T,src)-dst,2,2);
            notes{end+1}=sprintf('trimmed tag %d visit %d (%.2f m off)', ...
                fixes(w).id,fixes(w).visit,r(w)); %#ok<AGROW>
        end
    end
    rms=sqrt(mean(r(used).^2));
    problem=checkFit(T,rms,lo);
    if isempty(problem)
        F=struct('ok',true,'transform',T,'used',used,'residuals',r,'rms',rms, ...
            'notes',strjoin(notes,'; '));
        return;
    end
    notes{end+1}=sprintf('%s: %s',model,problem); %#ok<AGROW>
end
F.notes=strjoin(notes,'; ');
end

% ------------------------------------------------------------ drift models
function C=noCorrection()
C=struct('model','none','base',fitTransformIdentity(),'knotT',[],'knotD',zeros(0,2),'looCm',[]);
end
function T=fitTransformIdentity()
T=struct('model','identity','scale',1,'theta',0,'t',[0 0]);
end

function C=build(model,fixes,lo)
C=[];
if isempty(fixes), return; end
if strcmp(model,'time')
    C=noCorrection(); C.model='time'; [C.knotT,C.knotD]=knots(fixes,C.base); return;
end
F=solveFixes(fixes,lo);
if ~F.ok, return; end
C=noCorrection(); C.base=F.transform; C.model=model;
if strcmp(model,'hybrid'), [C.knotT,C.knotD]=knots(fixes,C.base); end
end

function [kt,kd]=knots(fixes,base)
mid=([fixes.tStart]+[fixes.tEnd])/2; [mid,order]=sort(mid); fixes=fixes(order);
kt=zeros(numel(fixes),1); kd=zeros(numel(fixes),2);
for i=1:numel(fixes)
    t=mid(i); if i>1 && t<=kt(i-1), t=kt(i-1)+1e-6; end
    kt(i)=t; kd(i,:)=fixes(i).ref-applyT(base,fixes(i).px4);
end
end

function xy=applyC(C,t,xy)
xy=applyT(C.base,xy);
if ~isempty(C.knotT)
    if numel(C.knotT)==1
        xy=xy+C.knotD;
    else
        tc=min(max(t,C.knotT(1)),C.knotT(end));   % np.interp clamps
        xy=xy+[interp1(C.knotT,C.knotD(:,1),tc),interp1(C.knotT,C.knotD(:,2),tc)];
    end
end
end

function xy=applyMany(C,t,xy)
for k=1:size(xy,1), xy(k,:)=applyC(C,t(k),xy(k,:)); end
end

function e=looCm(model,fixes,lo)
errs=[];
for i=1:numel(fixes)
    keep=true(1,numel(fixes)); keep(i)=false;
    C=build(model,fixes(keep),lo);
    if isempty(C), continue; end
    f=fixes(i); p=applyC(C,(f.tStart+f.tEnd)/2,f.px4);
    errs(end+1)=norm(p-f.ref); %#ok<AGROW>
end
if numel(errs)<max(2,numel(fixes)-1), e=nan; else, e=100*mean(errs); end
end

function C=driftCorrection(~,fixes,lo)
C=noCorrection();
if isempty(fixes), return; end
loo=struct('model',{},'cm',{});
if strcmp(lo.driftModel,'auto') && numel(fixes)>=3
    for m={'global','time','hybrid'}
        e=looCm(m{1},fixes,lo);
        if isfinite(e), loo(end+1)=struct('model',m{1},'cm',e); end %#ok<AGROW>
    end
end
if ~strcmp(lo.driftModel,'auto'), order={lo.driftModel,'global'};
elseif ~isempty(loo), [~,i]=min([loo.cm]); order={loo(i).model,'global'};
else, order={'global','time'};
end
for m=order
    c=build(m{1},fixes,lo);
    if ~isempty(c), C=c; C.looCm=loo; return; end
end
C.looCm=loo;
end

function s=describe(C)
what=struct('global','one transform for the whole flight', ...
    'time','drift interpolated in time between landmark visits', ...
    'hybrid','one transform, then the drift left interpolated in time', ...
    'none','no correction (raw positions reported)');
s=sprintf('drift correction: %s (%s)',C.model,what.(C.model));
if ~isempty(C.looCm)
    s=[s '; leave-one-out cm: ' strjoin(arrayfun(@(l) sprintf('%s %.1f',l.model,l.cm), ...
        C.looCm,'UniformOutput',false),', ')];
end
end

% ------------------------------------------------------------ targets
function out=correctTargets(C,T,cfg,lo)
ids=cfg.targets.id(:);
out=struct('id',num2cell(ids),'nSightings',0,'confirmed',false, ...
    'estimate',nan(1,2),'truth',nan(1,2),'error',nan);
for i=1:numel(ids)
    rows=T(T.id==ids(i),:);
    out(i).truth=cfg.targets.ned(i,1:2);
    out(i).nSightings=height(rows);
    if isempty(rows), continue; end
    pts=applyMany(C,rows.t_capture,[rows.est_n rows.est_e]);
    [est,~,support]=robustEstimate(pts,lo.confirmRadiusM,lo.outlierRadiusM);
    out(i).confirmed=support>=lo.minSightings;
    out(i).estimate=est; out(i).error=norm(est-out(i).truth);
end
end
