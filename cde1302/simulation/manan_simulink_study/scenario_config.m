function cfg = scenario_config(opts)
%SCENARIO_CONFIG Phase 2/3 localisation scenario on top of study_config.
%   cfg = scenario_config(opts) with optional fields:
%     flightPreset  'team_loc_tags' (default: 0.8 m, 1.0 m lanes, 0.30 m/s)
%                   | 'phase1_demo' (0.5 m, 0.6 m lanes, 0.15 m/s)
%     heightM, laneSpacingM, speedMps   override the preset
%     yawMode       'fixed' (default) | 'tangent'
%     method        'A' (dead reckoning, no anchor use; B is post-processed
%                   from this flight) | 'C' (online anchor corrections)
%     seed          integer (default 1). Targets AND sensor-error draws use
%                   it, so the same seed pairs fixed/tangent and A/C runs.
%     nTargets      default 3 unknown search tags
%     anchors       N x 3 [id x y] in ARENA metres, or a layout name for
%                   anchor_layout ('none','home','corners4',...)
%     sensor        struct of sensor overrides (see defaults below)
%     idealSensors  true = every sensor error zero (regression check)
%
% Every number below is an ASSUMPTION unless its comment names a team source.
% Nothing here is calibrated against the team's flight logs (none supplied).

if nargin<1 || isempty(opts), opts=struct(); end
preset=getOpt(opts,'flightPreset','team_loc_tags');
switch preset
    case 'team_loc_tags'   % feature/loc-tags config.yaml (expected_down_m,
        flight=struct('heightM',0.8,'laneSpacingM',1.0,'speedMps',0.30); % sweep, flight)
    case 'phase1_demo'
        flight=struct('heightM',0.5,'laneSpacingM',0.6,'speedMps',0.15);
    otherwise
        error('MananStudy:FlightPreset','Unknown flight preset %s.',preset);
end
flight.heightM=getOpt(opts,'heightM',flight.heightM);
flight.laneSpacingM=getOpt(opts,'laneSpacingM',flight.laneSpacingM);
flight.speedMps=getOpt(opts,'speedMps',flight.speedMps);
missionOverrides=struct('heightM',flight.heightM,'laneSpacingM',flight.laneSpacingM, ...
    'maxReferenceSpeedMps',flight.speedMps,'nominalSpeedMps',flight.speedMps, ...
    'yawMode',getOpt(opts,'yawMode','fixed'),'turnPauseMode','matched');
cfg=study_config('demo',[],missionOverrides);
cfg.scenario.flightPreset=preset;
cfg.scenario.seed=getOpt(opts,'seed',1);
cfg.scenario.method=upper(char(getOpt(opts,'method','A')));
assert(ismember(cfg.scenario.method,{'A','C'}),'MananStudy:Method', ...
    'Simulated method must be A or C (B is post-processing of an A flight).');

% ---------------- Sensor and estimator model ----------------
n.enabled=true;
n.method=cfg.scenario.method;
n.dt=0.02;                     % 50 Hz estimator/flow update (assumption)
% Optical flow (MTF-02P). Error magnitudes are assumptions; the team's own
% simulator uses drift_scale 0.05 and creep 0.0005 m/s as EXAMPLES.
n.flowScaleStd=0.03;           % per-axis scale-factor error (1 sigma)
n.flowBiasStdMps=0.001;        % constant velocity bias (1 sigma)
n.flowNoiseStdMps=0.03;        % white noise per 50 Hz sample
n.flowRotResidual=0.05;        % uncompensated fraction of h*(roll/pitch rate)
n.flowLeverErrM=[0;0];         % residual flow-sensor offset error (PX4 SENS_FLOW_POS assumed right)
n.flowMinHeightM=0.02;         % flow unused below this true height. The lecturer model has
                               % no floor, so below it the drone "floats" at -1 cm with no
                               % horizontal feedback; landing is scored at touchdown instead.
% Gyro (BMI088/BMI270) yaw channel; no compass is used by the team.
n.gyroBiasStdDegps=0.01;       % constant yaw bias (1 sigma)
n.gyroScaleStd=0.005;          % scale-factor error (1 sigma)
n.gyroNoiseStdDegps=0.05;      % white noise per sample
n.initialHeadingStdDeg=2.0;    % drone put down off-square (team saw ~3.5 deg once)
n.velocitySmoothingS=0.2;      % velocity error seen by the controller is low-passed
% Camera: ESP32-S3, team values from feature/loc-tags config.yaml / camera_mount.py.
n.cam.width=320; n.cam.height=240;
n.cam.fx=163.5; n.cam.fy=153.2;          % course defaults in config.yaml
n.cam.cx=160; n.cam.cy=120;
n.cam.offsetB=[0.08;0;0];                % tags.camera_forward_m (marked MEASURE)
n.cam.Rbc=[0 -1 0;1 0 0;0 0 1];          % r_body_cam(90): image right = body right, image top = forward
n.cam.fps=10;                            % firmware default (camera.fps 0 -> 10)
n.cam.latencyS=0.13;                     % team sim latency_s
n.cam.maxRangeM=1.5;                     % tags.max_detect_range_m
n.cam.maxYawRateDegps=30;                % tags.max_yaw_rate_deg_s (frame rejected)
n.cam.edgeMarginPx=2;                    % whole tag must be inside the image (assumption)
n.cam.minTagPx=8;                        % smallest decodable tag side (assumption)
n.cam.pDetect=0.9;                       % per-frame detection probability when visible (assumption)
n.cam.noiseStdM=0.03;                    % team sim sighting_noise_m (body frame)
n.cam.outlierProb=0.02;                  % gross-error sightings (assumption)
n.cam.outlierMaxM=0.5;
n.cam.tagYawNoiseDeg=2.0;                % anchor orientation noise (assumption)
n.cam.tiltCompensation=false;            % team projection is yaw-only (level flight)
n.cam.minHeightM=0.2;                    % frames below this are not counted/used
% Method C filter (study EKF, NOT PX4 EKF2).
n.ekf.qPos0=0.01;              % m/sqrt(s) position random walk
n.ekf.qPosPerSpeed=0.25;       % per m/s of speed: covers ~3 % scale error over a minute
n.ekf.resetAfterRejects=5;     % consecutive anchor rejections before widening P
n.ekf.resetPosM=0.5;           % position sigma added on such a reset
n.ekf.qPsiDegRtS=0.02;         % heading random walk
n.ekf.qPsiPerRate=0.01;        % extra per rad turned (gyro scale)
n.ekf.rPosM=0.04;              % anchor position measurement 1 sigma
n.ekf.rYawDeg=2.0;             % anchor orientation measurement 1 sigma
n.ekf.gatePos=9.21;            % chi-square 2 dof, 99 %
n.ekf.gateYaw=6.63;            % chi-square 1 dof, 99 %
n.ekf.useAnchorYaw=true;       % team uses tag orientation live (align_to_home_tag)
n.ekf.p0PosM=0.02;
n.queueFrames=4;
% Tags
n.targetSizeM=0.112;           % tags.tag_size_m (black square)
n.anchorSizeM=0.127;           % localisation.tag_size_m
sensor=getOpt(opts,'sensor',struct());
names=fieldnames(sensor);
for k=1:numel(names)
    if isstruct(sensor.(names{k})) && isfield(n,names{k})
        sub=fieldnames(sensor.(names{k}));
        for j=1:numel(sub), n.(names{k}).(sub{j})=sensor.(names{k}).(sub{j}); end
    else
        assert(isfield(n,names{k}),'MananStudy:SensorField','Unknown sensor field %s.',names{k});
        n.(names{k})=sensor.(names{k});
    end
end
if getOpt(opts,'idealSensors',false)
    n.flowScaleStd=0; n.flowBiasStdMps=0; n.flowNoiseStdMps=0; n.flowRotResidual=0;
    n.flowLeverErrM=[0;0]; n.gyroBiasStdDegps=0; n.gyroScaleStd=0;
    n.gyroNoiseStdDegps=0; n.initialHeadingStdDeg=0;
    n.cam.noiseStdM=0; n.cam.outlierProb=0; n.cam.tagYawNoiseDeg=0;
end
cfg.nav=n;

% ---------------- Tags ----------------
nTargets=getOpt(opts,'nTargets',3);
cfg.targets=random_targets(cfg,nTargets,cfg.scenario.seed);
anchors=getOpt(opts,'anchors','none');
if ischar(anchors) || isstring(anchors)
    cfg.scenario.anchorLayout=char(anchors);
    anchors=anchor_layout(char(anchors),cfg);
else
    cfg.scenario.anchorLayout='custom';
end
cfg.anchors=make_anchors(cfg,anchors);
cfg.nav.nSlots=numel(cfg.targets.id)+numel(cfg.anchors.id);
cfg.nav.slotIsAnchor=[false(numel(cfg.targets.id),1);true(numel(cfg.anchors.id),1)];
cfg.nav.slotId=[cfg.targets.id(:);cfg.anchors.id(:)];
cfg.nav.slotNed=[cfg.targets.ned;cfg.anchors.ned];
cfg.nav.slotYaw=[cfg.targets.yawRad(:);cfg.anchors.yawRad(:)];
cfg.nav.slotSize=[repmat(n.targetSizeM,numel(cfg.targets.id),1); ...
    repmat(n.anchorSizeM,numel(cfg.anchors.id),1)];

% ---------------- Random draws (seeded, layout independent) ----------------
cfg.nav.err=draw_errors(cfg.nav,cfg.scenario.seed);
cfg.nav.noise=draw_noise(cfg,cfg.scenario.seed);
cfg.scenario.description=sprintf(['%s flight (h %.2f m, lanes %.2f m, %.2f m/s), ' ...
    '%s yaw, method %s, seed %d, %d targets, %d anchors (%s)'],preset, ...
    flight.heightM,flight.laneSpacingM,flight.speedMps,cfg.mission.yawMode, ...
    cfg.scenario.method,cfg.scenario.seed,numel(cfg.targets.id), ...
    numel(cfg.anchors.id),cfg.scenario.anchorLayout);
end

function value=getOpt(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), value=s.(name); else, value=default; end
end

function t=random_targets(cfg,n,seed)
% Unknown search tags on the floor, uniformly inside the search area with a
% margin, at least 0.5 m apart. Independent of anchors so layouts pair.
rs=RandStream('mt19937ar','Seed',seed);
b=cfg.geometry.searchBounds; margin=0.15;
xy=zeros(0,2);
tries=0;
while size(xy,1)<n
    tries=tries+1; assert(tries<10000,'MananStudy:Targets','Cannot place targets.');
    p=[b(1,1)+margin+rand(rs)*(diff(b(1,:))-2*margin), ...
        b(2,1)+margin+rand(rs)*(diff(b(2,:))-2*margin)];
    if isempty(xy) || min(vecnorm(xy-p,2,2))>=0.5, xy(end+1,:)=p; end %#ok<AGROW>
end
pool=0:21;                                  % tag16h5 ids below the anchors' 22-29
t.id=pool(randperm(rs,numel(pool),n)).';
t.arena=xy;
t.ned=[arena_to_ned(cfg,xy),repmat(cfg.frames.homeNed(3),n,1)];
t.yawRad=2*pi*rand(rs,n,1);                 % taped at any angle
end

function a=make_anchors(cfg,A)
if isempty(A), A=zeros(0,3); end
assert(size(A,2)>=3,'MananStudy:Anchors','Anchors are N x 3 [id x y].');
a.id=A(:,1); a.arena=A(:,2:3);
assert(numel(unique(a.id))==numel(a.id),'MananStudy:AnchorIds','Anchor ids must be unique.');
a.ned=[arena_to_ned(cfg,a.arena),repmat(cfg.frames.homeNed(3),size(A,1),1)];
a.yawRad=zeros(size(A,1),1)+cfg.frames.arenaToLocalYaw; % taped square to the tiles
end

function ned=arena_to_ned(cfg,xy)
if isempty(xy), ned=zeros(0,2); return; end
c=cos(cfg.frames.arenaToLocalYaw); s=sin(cfg.frames.arenaToLocalYaw);
ned=([c -s;s c]*(xy.'-cfg.frames.homeArena)).'+cfg.frames.homeNed(1:2).';
end

function e=draw_errors(n,seed)
rs=RandStream('mt19937ar','Seed',seed+1000);
e.flowScale=n.flowScaleStd*randn(rs,2,1);
e.flowBias=n.flowBiasStdMps*randn(rs,2,1);
e.gyroBias=deg2rad(n.gyroBiasStdDegps)*randn(rs);
e.gyroScale=n.gyroScaleStd*randn(rs);
e.initialHeading=deg2rad(n.initialHeadingStdDeg)*randn(rs);
end

function z=draw_noise(cfg,seed)
% Pre-drawn so the S-function is a pure function of time. Slot j always uses
% column j: targets first, then anchors in table order, so adding an anchor
% does not change any target's noise.
n=cfg.nav;
nSteps=ceil(cfg.mission.maxTimeS/n.dt)+2;
nFrames=ceil(cfg.mission.maxTimeS*n.cam.fps)+2;
maxSlots=max(n.nSlots,1);
rs=RandStream('mt19937ar','Seed',seed+2000);
z.flow=n.flowNoiseStdMps*randn(rs,nSteps,2);
z.gyro=deg2rad(n.gyroNoiseStdDegps)*randn(rs,nSteps,1);
rsT=RandStream('mt19937ar','Seed',seed+3000);
base=40; % fixed column budget so slot j's draws never depend on slot count
assert(maxSlots<=base,'MananStudy:TooManyTags','At most %d tags in total.',base);
z.detect=rand(rsT,nFrames,base);
z.sight=n.cam.noiseStdM*randn(rsT,nFrames,base,2);
z.outlier=rand(rsT,nFrames,base);
z.outlierVec=n.cam.outlierMaxM*sqrt(rand(rsT,nFrames,base)).*exp(1i*2*pi*rand(rsT,nFrames,base));
z.tagYaw=deg2rad(n.cam.tagYawNoiseDeg)*randn(rsT,nFrames,base);
z.detect=z.detect(:,1:maxSlots); z.sight=z.sight(:,1:maxSlots,:);
z.outlier=z.outlier(:,1:maxSlots); z.outlierVec=z.outlierVec(:,1:maxSlots);
z.tagYaw=z.tagYaw(:,1:maxSlots);
end
