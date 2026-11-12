function varargout = nav_core(varargin)
%NAV_CORE Sensor models and navigation estimator as pure functions.
%
%   state = nav_core('initial',cfg)
%   [dpos,dvel,qMat,diag,sight,next] = nav_core('step',t,pos,vel,quat,rates,state,cfg)
%   [visible,uv] = nav_core('visible',pos,quat,tagNed,tagYaw,tagSize,cfg)
%   poly = nav_core('footprint',pos,quat,cfg)       % 2 x 4 floor corners (NED)
%   names = nav_core('fields')                      % sighting/diag field names
%
% What it models (all assumptions are in scenario_config):
%   * Optical-flow dead reckoning: body-frame velocity with scale error, bias,
%     white noise and roll/pitch rotation residual, rotated by the ESTIMATED
%     heading and integrated. No absolute position source.
%   * Gyro heading: true yaw increment x (1+scale) + bias + noise. No compass.
%   * Downward camera (team intrinsics/mount): a tag is detectable when all
%     four corners are inside the image, within range, large enough, the
%     measured yaw rate is under the team's 30 deg/s gate and a seeded
%     detection draw succeeds. Sightings carry body-frame noise, rare gross
%     outliers, and reach the estimator latencyS after capture.
%   * Method 'A': dead reckoning only. Sightings are recorded (Method B is
%     their post-flight use). Method 'C': a 3-state EKF [pN pE psi] uses
%     landmark sightings online with chi-square gating. Delayed sightings use
%     the pose stored at capture; corrections are also applied to frames
%     still in flight so a correction is never counted twice.
%
% The estimator never reads truth. Truth enters only through the simulated
% sensors and is compared with the estimate to produce the error outputs that
% are added to the controller feedback (error-state injection):
%   dpos = [pHat - p; 0], dvel = smoothed [vHat - v; 0],
%   qMat = left-multiplication matrix of a rotation by (psiHat - psi) about
%          world z, so q_feedback = qMat * q_true carries the heading error.
% This is a study EKF, NOT PX4 EKF2.
switch varargin{1}
    case 'initial', varargout{1}=initialState(varargin{2});
    case 'step', [varargout{1:6}]=step(varargin{2:end});
    case 'visible'
        [pos,q,c,yaw,s,cfg]=varargin{2:7};
        [varargout{1},varargout{2}]=tagVisibility(pos(:),q2r(q(:)),c(:),yaw,s,cfg.nav);
    case 'footprint', varargout{1}=footprint(varargin{2:4});
    case 'fields', varargout{1}=fieldNames();
    case 'layout', varargout{1}=layout(varargin{2});
    otherwise, error('MananStudy:NavDispatch','Unknown nav_core command.');
end
end

% ---------------------------------------------------------------- layout
function L=layout(cfg)
n=cfg.nav;
L.core=30; L.hdr=10; L.G=5; L.nS=n.nSlots; L.Q=n.queueFrames;
L.frame=L.hdr+L.nS*L.G;
L.total=L.core+L.Q*L.frame;
L.F=12;                         % sighting output fields per slot
L.sightLen=max(1,L.nS)*L.F;
L.diagLen=16;
end

function f=fieldNames()
f.sight={'t_capture','status','est_n','est_e','rel_x','rel_y','phat_n_capture', ...
    'phat_e_capture','psihat_capture','yaw_meas','ptrue_n_capture','ptrue_e_capture'};
f.diag={'phat_n','phat_e','psihat','sigma_n','sigma_e','sigma_psi', ...
    'anchors_visible_now','targets_visible_now','anchor_used_total', ...
    'anchor_rejected_total','frames_yaw_gated_total','frames_total', ...
    'frames_anchor_visible_total','frames_anchor_used_total', ...
    'anchor_sightings_total','target_sightings_total'};
end

% ---------------------------------------------------------------- state
function x=initialState(cfg)
L=layout(cfg); x=zeros(L.total,1);
x(25)=-1;     % last camera frame index
x(28)=0;      % not initialised; first step reads the initial pose
end

function [dpos,dvel,qMat,diagOut,sight,x]=step(t,pos,vel,quat,rates,x,cfg)
n=cfg.nav; L=layout(cfg); e=n.err; z=n.noise;
pos=double(pos(:)); vel=double(vel(:)); q=double(quat(:)); q=q/norm(q);
w=double(rates(:)); x=double(x(:));
Rbw=q2r(q);
psiRaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
h=cfg.frames.homeNed(3)-pos(3);
sightM=nan(max(1,L.nS),L.F);
if x(28)==0
    % PX4 starts its frame where the drone sits; the heading is off by the
    % placement error because nothing measures absolute heading.
    psiTrue=psiRaw;
    x(1:2)=pos(1:2); x(3)=psiTrue+e.initialHeading;
    p0=n.ekf.p0PosM; s0=max(deg2rad(n.initialHeadingStdDeg),deg2rad(0.5));
    x(4:12)=reshape(diag([p0^2 p0^2 s0^2]),9,1);
    x(13)=psiTrue; x(14)=t; x(28)=1;
    dt=0; gyroInc=0; vMeas=[0;0];
else
    psiTrue=x(13)+wrapPi(psiRaw-x(13));
    dt=max(0,t-x(14));
    k=min(round(t/n.dt)+1,size(z.flow,1));
    gyroInc=(psiTrue-x(13))*(1+e.gyroScale)+(e.gyroBias+z.gyro(k))*dt;
    vb=Rbw.'*vel;
    rot=n.flowRotResidual*h*[w(2);-w(1)];
    lever=[-w(3)*n.flowLeverErrM(2); w(3)*n.flowLeverErrM(1)];
    vMeas=(1+e.flowScale).*vb(1:2)+e.flowBias+z.flow(k,:).'+rot+lever;
    if h<n.flowMinHeightM, vMeas=[0;0]; end   % at/under the floor: no flow fusion
    % Heading first: vMeas is measured in the CURRENT body frame.
    x(3)=x(3)+gyroInc;
    c=cos(x(3)); s=sin(x(3));
    x(1:2)=x(1:2)+[c -s;s c]*vMeas*dt;
    F=[1 0 (-s*vMeas(1)-c*vMeas(2))*dt; 0 1 (c*vMeas(1)-s*vMeas(2))*dt; 0 0 1];
    qp=(n.ekf.qPos0+n.ekf.qPosPerSpeed*norm(vMeas))^2*dt;
    qpsi=deg2rad(n.ekf.qPsiDegRtS)^2*dt+(n.ekf.qPsiPerRate*abs(gyroInc))^2;
    P=reshape(x(4:12),3,3); P=F*P*F.'+diag([qp qp qpsi]); x(4:12)=P(:);
    x(13)=psiTrue; x(14)=t;
end
yawRateMeas=0; if dt>0, yawRateMeas=gyroInc/dt; end
x(29)=yawRateMeas;
% Velocity error the controller sees: smoothed (the real EKF fuses the IMU).
c=cos(x(3)); s=sin(x(3));
dvRaw=[c -s;s c]*vMeas-vel(1:2);
a=dt/(n.velocitySmoothingS+dt); if dt==0, a=0; end
x(15:16)=x(15:16)+a*(dvRaw-x(15:16));

% ---- camera frame capture ----
frameIdx=round(t*n.cam.fps);
if abs(t*n.cam.fps-frameIdx)<1e-6 && frameIdx>x(25)
    x(25)=frameIdx; jF=min(frameIdx+1,size(z.detect,1));
    airborne=h>=n.cam.minHeightM;
    gated=abs(yawRateMeas)>deg2rad(n.cam.maxYawRateDegps);
    slots=zeros(max(1,L.nS),L.G); nAv=0; nTv=0;
    for j=1:L.nS
        vis=tagVisibility(pos,Rbw,n.slotNed(j,:).',n.slotYaw(j),n.slotSize(j),n);
        if vis
            if n.slotIsAnchor(j), nAv=nAv+1; else, nTv=nTv+1; end
        end
        if airborne && vis && ~gated && z.detect(jF,j)<n.cam.pDetect
            rel=Rbw.'*(n.slotNed(j,:).'-pos);
            if n.cam.tiltCompensation
                cz=cos(-psiTrue); sz=sin(-psiTrue);
                rel=[cz -sz 0;sz cz 0;0 0 1]*(Rbw*rel); % level, still body-yawed
            end
            relXY=rel(1:2)+reshape(z.sight(jF,j,:),2,1);
            isOut=z.outlier(jF,j)<n.cam.outlierProb;
            if isOut, relXY=relXY+[real(z.outlierVec(jF,j));imag(z.outlierVec(jF,j))]; end
            yawMeas=nan;
            if n.slotIsAnchor(j), yawMeas=psiTrue+z.tagYaw(jF,j); end
            slots(j,:)=[1 relXY.' yawMeas isOut];
        end
    end
    if airborne
        x(17)=x(17)+1;
        if gated, x(18)=x(18)+1; end
        if nAv>0, x(23)=x(23)+1; end
    end
    x(26)=nAv; x(27)=nTv;
    % Store the frame with the pose the drone had at capture (frame pose).
    slotBase=L.core; free=0;
    for b=1:L.Q
        if x(slotBase+(b-1)*L.frame+1)==0, free=b; break; end
    end
    if free==0, free=oldestFrame(x,L); end
    o=slotBase+(free-1)*L.frame;
    x(o+(1:L.hdr))=[1;t;t+n.cam.latencyS;x(1);x(2);x(3);pos(1);pos(2);psiTrue;frameIdx];
    x(o+L.hdr+(1:L.nS*L.G))=reshape(slots(1:L.nS,:).',[],1);
end

% ---- sightings arriving now (after latency) ----
while true
    b=dueFrame(x,L,t);
    if b==0, break; end
    o=L.core+(b-1)*L.frame;
    hdr=x(o+(1:L.hdr));
    slots=reshape(x(o+L.hdr+(1:L.nS*L.G)),L.G,[]).';
    corr=zeros(3,1); usedAnchor=false;
    for j=1:L.nS
        if slots(j,1)~=1, continue; end
        pC=hdr(4:5)+corr(1:2); psiC=hdr(6)+corr(3);
        cc=cos(psiC); ss=sin(psiC); rel=slots(j,2:3).';
        est=pC+[cc -ss;ss cc]*rel;
        status=1;
        if n.slotIsAnchor(j)
            x(21)=x(21)+1;
            if strcmp(n.method,'C')
                P=reshape(x(4:12),3,3);
                if x(30)>=n.ekf.resetAfterRejects
                    % Repeated rejections mean the filter, not the tags, is
                    % wrong: widen the position uncertainty (as EKF2 resets do).
                    P(1:2,1:2)=P(1:2,1:2)+n.ekf.resetPosM^2*eye(2); x(30)=0;
                end
                H=[eye(2),[-ss -cc;cc -ss]*rel];
                y=n.slotNed(j,1:2).'-est;
                S=H*P*H.'+n.ekf.rPosM^2*eye(2);
                if y.'*(S\y)<=n.ekf.gatePos
                    x(30)=0;
                    K=P*H.'/S; dx=K*y;
                    x(1:2)=x(1:2)+dx(1:2); x(3)=x(3)+dx(3); corr=corr+dx;
                    P=(eye(3)-K*H)*P; P=(P+P.')/2;
                    if n.ekf.useAnchorYaw && isfinite(slots(j,4))
                        yy=wrapPi(slots(j,4)-(psiC+dx(3)));
                        Sy=P(3,3)+deg2rad(n.ekf.rYawDeg)^2;
                        if yy^2/Sy<=n.ekf.gateYaw
                            Ky=P(:,3)/Sy; dy=Ky*yy;
                            x(1:2)=x(1:2)+dy(1:2); x(3)=x(3)+dy(3); corr=corr+dy;
                            P=P-Ky*P(3,:); P=(P+P.')/2;
                        end
                    end
                    x(4:12)=P(:);
                else
                    status=2; x(20)=x(20)+1; x(30)=x(30)+1;
                    x(4:12)=P(:);
                end
            end
            if status==1, x(19)=x(19)+1; usedAnchor=true; end
        else
            x(22)=x(22)+1;
        end
        sightM(j,:)=[hdr(2) status est.' rel.' hdr(4) hdr(5) hdr(6) slots(j,4) hdr(7) hdr(8)];
    end
    if usedAnchor, x(24)=x(24)+1; end
    x(o+1)=0;                                  % frame consumed
    if any(corr)                               % shift frames still in flight
        for bb=1:L.Q
            oo=L.core+(bb-1)*L.frame;
            if x(oo+1)==1
                x(oo+4:oo+5)=x(oo+4:oo+5)+corr(1:2); x(oo+6)=x(oo+6)+corr(3);
            end
        end
    end
end

% ---- outputs ----
dpsi=x(3)-psiTrue;
dpos=[x(1:2)-pos(1:2);0];
dvel=[x(15:16);0];
qd=[cos(dpsi/2);0;0;sin(dpsi/2)];
qMat=reshape(leftMatrix(qd),16,1);
P=reshape(x(4:12),3,3);
diagOut=[x(1:3);sqrt(max(diag(P),0));x(26);x(27);x(19);x(20);x(18);x(17); ...
    x(23);x(24);x(21);x(22)];
sight=reshape(sightM.',[],1);
end

function b=dueFrame(x,L,t)
b=0; best=inf;
for k=1:L.Q
    o=L.core+(k-1)*L.frame;
    if x(o+1)==1 && x(o+3)<=t+1e-9 && x(o+3)<best, best=x(o+3); b=k; end
end
end

function b=oldestFrame(x,L)
b=1; best=inf;
for k=1:L.Q
    o=L.core+(k-1)*L.frame;
    if x(o+2)<best, best=x(o+2); b=k; end
end
end

% ---------------------------------------------------------------- camera
function [vis,uv]=tagVisibility(pos,Rbw,c,yaw,s,n)
cam=pos+Rbw*n.cam.offsetB; Rcw=Rbw*n.cam.Rbc;
cy=cos(yaw); sy=sin(yaw); hs=s/2;
offs=[-hs hs hs -hs;-hs -hs hs hs];
corners=[c(1:2)+[cy -sy;sy cy]*offs; repmat(c(3),1,4)];
Pc=Rcw.'*(corners-cam);
uv=nan(2,4); vis=false;
if any(Pc(3,:)<=0.05), return; end
uv=[n.cam.cx+n.cam.fx*Pc(1,:)./Pc(3,:); n.cam.cy+n.cam.fy*Pc(2,:)./Pc(3,:)];
m=n.cam.edgeMarginPx;
inside=all(uv(1,:)>=m & uv(1,:)<=n.cam.width-m & uv(2,:)>=m & uv(2,:)<=n.cam.height-m);
if ~inside, return; end
if norm(c-cam)>n.cam.maxRangeM, return; end
side=min(vecnorm(uv-uv(:,[2 3 4 1]),2,1));
vis=side>=n.cam.minTagPx;
end

function poly=footprint(pos,quat,cfg)
n=cfg.nav; Rbw=q2r(quat(:)/norm(quat)); pos=pos(:);
cam=pos+Rbw*n.cam.offsetB; Rcw=Rbw*n.cam.Rbc;
uv=[0 n.cam.width n.cam.width 0;0 0 n.cam.height n.cam.height];
poly=nan(2,4); floorD=cfg.frames.homeNed(3);
for k=1:4
    d=Rcw*[(uv(1,k)-n.cam.cx)/n.cam.fx;(uv(2,k)-n.cam.cy)/n.cam.fy;1];
    if d(3)>1e-6
        lam=(floorD-cam(3))/d(3);
        if lam>0, poly(:,k)=cam(1:2)+lam*d(1:2); end
    end
end
end

% ---------------------------------------------------------------- maths
function R=q2r(q)
w=q(1); a=q(2); b=q(3); c=q(4);
R=[1-2*(b^2+c^2) 2*(a*b-w*c) 2*(a*c+w*b);
   2*(a*b+w*c) 1-2*(a^2+c^2) 2*(b*c-w*a);
   2*(a*c-w*b) 2*(b*c+w*a) 1-2*(a^2+b^2)];
end
function M=leftMatrix(q)
w=q(1); a=q(2); b=q(3); c=q(4);
M=[w -a -b -c; a w -c b; b c w -a; c -b a w];
end
function a=wrapPi(a)
a=atan2(sin(a),cos(a));
end
