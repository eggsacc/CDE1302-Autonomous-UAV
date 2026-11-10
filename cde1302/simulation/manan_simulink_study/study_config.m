function cfg = study_config(preset,frameOverrides)
%STUDY_CONFIG Reproducible settings; demo geometry is not the official map.
if nargin<1, preset='demo'; end
if nargin<2, frameOverrides=struct(); end
preset = char(preset);
cfg.root = fileparts(mfilename('fullpath'));
cfg.preset = preset;
cfg.provenance.teamCommit = 'e7a231eff2f9239ff0d6948a058cab19b4b5757c';
cfg.provenance.controller = 'Lecturer RPT simulation; not PX4 firmware or EKF2';
cfg.geometry.officialResolved = false;
cfg.frames.homeNed = [0; 0; 0];
cfg.frames.arenaToLocalYaw = 0;
cfg.mission.heightM = 0.5;
cfg.mission.yawRad = 0;
cfg.mission.gateRadiusM = 0.15;
cfg.mission.tickS = 0.05;
cfg.mission.maxTimeS = 360;
cfg.mission.nominalSpeedMps = 0.15; % Metadata, NOT achieved speed control.
cfg.mission.guidanceMode='bounded_position';
cfg.mission.maxReferenceSpeedMps=0.15;
cfg.mission.maxVerticalReferenceSpeedMps=0.15;
cfg.mission.takeoffMinS = 2;
cfg.mission.hoverHoldS = 2;
cfg.mission.hoverRadiusM = 0.10;
cfg.mission.altitudeToleranceM = 0.05;
cfg.mission.returnRadiusM = 0.05;
cfg.mission.returnHoldS = 1;
cfg.mission.returnMaxSettleS = 10;
cfg.mission.descentRateMps = 0.10; % Assumed simulation-only landing reference.
cfg.mission.endpointAltitudeToleranceM = 0.02;
cfg.mission.endpointRadiusM = 0.05;
cfg.mission.endpointHoldS = 2;
switch preset
    case 'demo'
        cfg.geometry.label = 'ASSUMED DEMO GEOMETRY; OFFICIAL OFFSETS UNVERIFIED';
        cfg.geometry.outerBounds = [0 5.4; 0 4.5];
        cfg.geometry.searchBounds = [1.2 4.8; 0.45 4.05];
        cfg.geometry.startBounds = [0.15 1.05; 0.15 1.05];
        cfg.geometry.boundaryComplianceApplicable = true;
        cfg.frames.homeArena = [0.6; 0.6];
        cfg.mission.laneSpacingM = 0.6; % Not a calibrated camera swath.
    case 'team_head_placeholder'
        cfg.geometry.label = 'PINNED TEAM PLACEHOLDER; OFFICIAL ARENA UNRESOLVED';
        cfg.geometry.outerBounds = nan(2);
        cfg.geometry.startBounds = nan(2);
        cfg.geometry.searchBounds = [0 4.8; -1.35 1.0];
        cfg.geometry.boundaryComplianceApplicable = false;
        cfg.frames.homeArena = [0; 0];
        cfg.mission.laneSpacingM = 2.5;
    case 'assessment'
        error('MananStudy:OfficialGeometryUnresolved', ...
            'Assessment refused: authoritative arena geometry/frame is unresolved.');
    otherwise
        error('MananStudy:UnknownPreset', 'Unknown study preset: %s', preset);
end
% Apply explicit frame configuration before transforming the actual route.
allowed={'homeNed','homeArena','arenaToLocalYaw'};
fields=fieldnames(frameOverrides);
for k=1:numel(fields)
    assert(ismember(fields{k},allowed),'MananStudy:FrameField','Unknown frame field.');
    cfg.frames.(fields{k})=frameOverrides.(fields{k});
end
validateattributes(cfg.frames.homeNed,{'double'},{'vector','numel',3,'finite'});
validateattributes(cfg.frames.homeArena,{'double'},{'vector','numel',2,'finite'});
validateattributes(cfg.frames.arenaToLocalYaw,{'double'},{'scalar','finite'});
cfg.frames.homeNed=cfg.frames.homeNed(:);
cfg.frames.homeArena=cfg.frames.homeArena(:);
% Alternating lanes as mission/planner.py. Home return is a separate stage.
b = cfg.geometry.searchBounds;
nGaps = ceil((b(2,2)-b(2,1))/cfg.mission.laneSpacingM);
lanes = linspace(b(2,1),b(2,2),max(1,nGaps)+1);
cfg.mission.routeArena = zeros(2*numel(lanes),2);
for k=1:numel(lanes)
    ends = [b(1,1),lanes(k);b(1,2),lanes(k)];
    if mod(k,2)==0, ends=flipud(ends); end
    cfg.mission.routeArena(2*k-1:2*k,:) = ends;
end
cfg.mission.routeArena = round(cfg.mission.routeArena,3);
c=cos(cfg.frames.arenaToLocalYaw); s=sin(cfg.frames.arenaToLocalYaw);
R=[c -s;s c];
xy=(R*(cfg.mission.routeArena.'-cfg.frames.homeArena)).';
xy=xy+cfg.frames.homeNed(1:2).';
cfg.mission.routeNed=[xy,repmat(cfg.frames.homeNed(3)- ...
    cfg.mission.heightM,size(xy,1),1)];
cfg.mission.routeDescription='Sweep goals only; home return is separate.';
cfg.mission.governorDescription=['Study adaptation: bound the moving position ' ...
    'reference, zero v/a feedforward, retain corner goals and arrival gate.'];
cfg.mission.landingDescription=['Reference descent/hold only; no ground ' ...
    'contact, motor disarm or PX4 LAND implementation.'];
end
