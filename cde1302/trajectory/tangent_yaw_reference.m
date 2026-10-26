function [yaw, defined, velocity] = tangent_yaw_reference(t, position, speed_epsilon)
% Strict tangent of the actual sampled NED path, rad CW from North.
% Below epsilon: interpolate unwrapped headings between defined samples;
% hold the nearest defined heading at the stationary beginning/end.
% This does not rotate a physical drone or change position/timing.
if nargin < 3, speed_epsilon = 1e-4; end
assert(isfinite(speed_epsilon) && speed_epsilon > 0);
t = t(:);
assert(numel(t) >= 2 && all(isfinite(t)) && all(diff(t) > 0));
assert(size(position,1) == numel(t) && size(position,2) == 3 ...
    && all(isfinite(position), 'all'));
velocity = zeros(size(position));
for c = 1:3
    velocity(:,c) = gradient(position(:,c), t);
end
defined = hypot(velocity(:,1), velocity(:,2)) > speed_epsilon;
assert(any(defined), 'Horizontal tangent is undefined for an entirely stationary path.');
angles = unwrap(atan2(velocity(defined,2), velocity(defined,1)));
if nnz(defined) == 1
    yaw = repmat(angles, size(t));
else
    yaw = interp1(t(defined), angles, t, 'linear', 'extrap');
    first = find(defined,1,'first'); last = find(defined,1,'last');
    yaw(t < t(first)) = angles(1);
    yaw(t > t(last)) = angles(end);
end
end
