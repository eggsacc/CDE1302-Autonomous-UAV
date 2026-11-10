function report = check_environment(studyRoot)
%CHECK_ENVIRONMENT Check prerequisites; success is not a simulation result.
if nargin<1 || isempty(studyRoot), studyRoot=fileparts(mfilename('fullpath')); end
report.checked_at_utc=char(datetime('now','TimeZone','UTC', ...
    'Format','yyyy-MM-dd''T''HH:mm:ss''Z'''));
report.matlab_version=version;
report.matlab_release=version('-release');
report.computer=computer;
report.matlab_root=matlabroot;
report.jvm_available=usejava('jvm');
report.installed_products=ver;
report.license_test_limitation='Licence existence does not prove a model compiles or reserve a network seat.';
products={'Simulink','Aerospace Blockset','Aerospace Toolbox', ...
    'Robotics System Toolbox','Optimization Toolbox'};
features={'Simulink','Aerospace_Blockset','Aerospace_Toolbox', ...
    'Robotics_System_Toolbox','Optimization_Toolbox'};
essential=[true true true false false];
checks=repmat(struct('product','','feature','','essential',false, ...
    'installed',false,'license_exists',false,'error',''),1,numel(products));
installedNames={report.installed_products.Name}; failures={};
fprintf('MATLAB %s (%s), %s\n',report.matlab_release,report.matlab_version,report.computer);
for k=1:numel(products)
    checks(k).product=products{k}; checks(k).feature=features{k};
    checks(k).essential=essential(k);
    checks(k).installed=any(strcmp(installedNames,products{k}));
    try, checks(k).license_exists=logical(license('test',features{k}));
    catch err, checks(k).error=err.message; end
    fprintf('  %-25s installed=%d licence_test=%d\n',products{k}, ...
        checks(k).installed,checks(k).license_exists);
    if essential(k) && ~(checks(k).installed && checks(k).license_exists)
        failures{end+1}=products{k}; %#ok<AGROW>
    end
end
report.products=checks;
required={'sim','load_system','new_system','set_param','Simulink.SimulationInput', ...
    'quatmultiply','quatinv','quatnormalize','quat2rotm','rotm2quat','rotm2eul'};
names=[required {'quadprog','eul2rotm','uifigure'}];
providers=repmat(struct('name','','essential',false,'selected_provider','', ...
    'all_providers',{{}}),1,numel(names));
for k=1:numel(names)
    providers(k).name=names{k}; providers(k).essential=k<=numel(required);
    providers(k).selected_provider=which(names{k});
    found=which(names{k},'-all'); if ischar(found), found=cellstr(found); end
    providers(k).all_providers=found;
    if providers(k).essential && isempty(providers(k).selected_provider)
        failures{end+1}=['Missing function: ' names{k}]; %#ok<AGROW>
    end
end
if ~report.jvm_available, failures{end+1}='Java VM for source hashing'; end
report.functions=providers;
report.optional_toolbox_note='Optimization is optional minimum snap; actual execution verifies rotation-function licensing.';
report.prerequisites_passed=isempty(failures);
report.failures=failures;
report.simulation_executed_by_this_check=false;
evidenceDir=fullfile(studyRoot,'evidence');
if ~isfolder(evidenceDir), mkdir(evidenceDir); end
fid=fopen(fullfile(evidenceDir,'environment.json'),'wt','n','UTF-8');
assert(fid>=0,'Cannot write environment evidence.');
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
if ~report.prerequisites_passed
    error('MananStudy:MissingDependency','Prerequisite failure: %s',strjoin(failures,'; '));
end
end
