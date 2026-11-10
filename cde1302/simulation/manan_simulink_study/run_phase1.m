function result = run_phase1(preset)
%RUN_PHASE1 Reproduce the original, then run the fixed-yaw mission baseline.
if nargin<1, preset='demo'; end
root=fileparts(mfilename('fullpath'));
addpath(root);
check_analysis();
run_lecturer_example();
result=run_baseline(preset);
end
