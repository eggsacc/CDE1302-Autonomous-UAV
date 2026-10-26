function [yaw, defined] = write_tangent_yaw_csv(source, destination, speed_epsilon)
% Append yaw to an existing Lab 5 CSV, preserving t,x,y,z text verbatim.
% Run from cde1302: addpath('trajectory'); write_tangent_yaw_csv(...).
if nargin < 3, speed_epsilon = 1e-4; end
assert(~strcmpi(char(java.io.File(source).getCanonicalPath()), ...
                char(java.io.File(destination).getCanonicalPath())), ...
    'Destination must differ from the original CSV.');
lines = readlines(source);
data_lines = lines(strlength(strtrim(lines)) > 0 & ...
    ~startsWith(strtrim(lines), '#') & ~startsWith(strtrim(lines), 't,'));
data = zeros(numel(data_lines),4);
for i = 1:numel(data_lines)
    row = str2double(split(data_lines(i), ','));
    assert(numel(row) == 4 && all(isfinite(row)), 'Malformed source CSV row.');
    data(i,:) = row.';
end
[yaw, defined] = tangent_yaw_reference(data(:,1), data(:,2:4), speed_epsilon);
output = [sprintf('# stationary_speed_mps=%.17g; unwrapped NED yaw in radians',speed_epsilon); ...
          "t,x,y,z,yaw_rad"; data_lines + "," + compose('%.17g',yaw)];
writelines(output, destination);
rate = diff(yaw)./diff(data(:,1));
fprintf('Wrote %s; all %d t,x,y,z rows unchanged.\n', destination, numel(yaw));
fprintf('Initial yaw %.3f deg; yaw rate [%.6f, %.6f] rad/s.\n', rad2deg(yaw(1)), min(rate), max(rate));
fprintf('Generation is offline only. The uploader and firmware must enforce the approved rate limit.\n');
end
