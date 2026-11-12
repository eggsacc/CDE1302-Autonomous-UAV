function R = nav_tag_report(res)
%NAV_TAG_REPORT Reported target positions as the drone made them in flight.
%   Per target id, the robust median of all sightings in the estimator's
%   frame (team tag_map: dense-seeded median, confirmed with >= 5 sightings
%   inside 0.20 m, outliers beyond 0.30 m ignored). Method A: raw drift.
%   Method C: positions made with the online-corrected pose. Landmarks are
%   never reported as targets.
cfg=res.config; S=res.sightings;
lo=struct('confirmRadiusM',0.20,'outlierRadiusM',0.30,'minSightings',5);
ids=cfg.targets.id(:);
R=struct('id',num2cell(ids),'nSightings',0,'confirmed',false,'estimate',nan(1,2), ...
    'truth',nan(1,2),'error',nan,'firstSeenS',nan);
for i=1:numel(ids)
    rows=S(S.is_anchor==0 & S.id==ids(i),:);
    R(i).truth=cfg.targets.ned(i,1:2);
    R(i).nSightings=height(rows);
    if isempty(rows), continue; end
    pts=[rows.est_n rows.est_e];
    d=sqrt((pts(:,1)-pts(:,1).').^2+(pts(:,2)-pts(:,2).').^2);
    [~,seed]=max(sum(d<=lo.confirmRadiusM,2));
    inl=d(seed,:).'<=lo.outlierRadiusM; est=median(pts(inl,:),1);
    dd=vecnorm(pts-est,2,2);
    if any(dd<=lo.outlierRadiusM)
        inl=dd<=lo.outlierRadiusM; est=median(pts(inl,:),1); dd=vecnorm(pts-est,2,2);
    end
    R(i).confirmed=nnz(dd<=lo.confirmRadiusM)>=lo.minSightings;
    R(i).estimate=est; R(i).error=norm(est-R(i).truth);
    R(i).firstSeenS=min(rows.t_capture);
end
end
