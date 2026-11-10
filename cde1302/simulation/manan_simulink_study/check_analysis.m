function report=check_analysis()
%CHECK_ANALYSIS Hand-checkable synthetic fixture; not vehicle/flight evidence.
c=study_config('demo');
c.geometry.label='SYNTHETIC ANALYSIS FIXTURE; NOT A FLIGHT OR VEHICLE RUN';
c.mission.routeNed=[1 0 -.5;1 1 -.5];
c.mission.routeArena=c.mission.routeNed(:,1:2)+c.frames.homeArena.';
r.config=c; r.outputDir=fullfile(c.root,'results','analysis_fixture');
r.time=[0;1;2];
r.position=[0 .1 -.5;.5 .1 -.5;1 .1 -.5];
r.velocity=repmat([.5 0 0],3,1);
r.euler=repmat([0 0 2*pi],3,1);
r.reference=[[0; .5; 1],zeros(3,1),-.5*ones(3,1),zeros(3,8)];
r.diagnostics=repmat([3 1 0 0 .1 0 NaN],3,1);
[m,k]=analyse_run(r);
assert(abs(m.rmsSetpointErrorXYM-.1)<1e-12);
assert(abs(m.rmsSetpointError3DM-.1)<1e-12);
assert(abs(m.stages.SWEEP.rmsActiveLegDistanceM-.1)<1e-12);
assert(abs(m.horizontalDistanceFlownM-1)<1e-12);
assert(m.rmsYawErrorDeg<1e-10);
assert(~m.returnHold.available && isnan(m.returnHold.horizontalErrorM));
assert(~k.mission.completed && k.mission.stoppedBeforeCompletion);
report.passed=true;
report.checks={'0.1 m independent tracking offset','0.1 m active-leg distance', ...
    '1 m integrated horizontal travel','2 pi yaw equals zero heading', ...
    'missing return is unavailable','stop time does not imply completion'};
report.count=numel(report.checks);
report.scope='Synthetic arithmetic fixture, not a Simulink or flight run';
fid=fopen(fullfile(c.root,'evidence','analysis_checks.json'),'w');
assert(fid>=0,'Cannot write analysis-check evidence.');
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
fprintf('Analysis checks: %d passed (synthetic fixture only).\n',report.count);
end
