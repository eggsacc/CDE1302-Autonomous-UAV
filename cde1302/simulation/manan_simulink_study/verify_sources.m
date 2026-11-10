function report = verify_sources(studyRoot)
%VERIFY_SOURCES Check exact immutable lecturer bytes without repairing them.
if nargin<1 || isempty(studyRoot), studyRoot=fileparts(mfilename('fullpath')); end
assert(usejava('jvm'),'MananStudy:HashJava','SHA-256 check requires Java.');
vendor=fullfile(studyRoot,'vendor','lecturer');
paths={'circle_traj_demo.m','position_control/PosControl_Sim_RPT.slx', ...
    'position_control/Init_control.m','position_control/icon/Init.m', ...
    'position_control/icon/pixhawk.png'};
hashes={ ...
    '2862d7f5b931198ca0e4fd7a210645b7c6f3b887094832bec6df951192406e36', ...
    '5fbd13baa84f6f687af2ff9cdde3b3dc2edf25b5472452c40e0024e7e63bfd57', ...
    '41b08cfae0390c48ea72db9ea288cd061fc211f1bc219b04b2ff648230476eff', ...
    'ac7cb685af4f62eefad212460a00dcd4291b946122a8d15d1fa2204d0735ab3f', ...
    '428d130d6deabd96917d62576bf502b1a00da03beb43d0b9d1da33b760c96052'};
expectedBytes=[3395 658745 3274 2235 115476];
files=repmat(struct('relative_path','','expected_sha256','', ...
    'actual_sha256','','expected_bytes',0,'actual_bytes',0,'verified',false),1,numel(paths));
for k=1:numel(paths)
    files(k).relative_path=paths{k}; files(k).expected_sha256=hashes{k};
    files(k).expected_bytes=expectedBytes(k);
    file=fullfile(vendor,paths{k});
    if isfile(file)
        [files(k).actual_sha256,files(k).actual_bytes]=hash_file(file);
        files(k).verified=strcmp(files(k).actual_sha256,hashes{k}) && ...
            files(k).actual_bytes==expectedBytes(k);
    end
end
report.source_repository='https://github.com/linfengfrank/CDE1302_UAV';
report.source_commit='9d80ddab01b20b01a19cb849b5af5424ca0ae07a';
report.files=files; report.all_verified=all([files.verified]);
fprintf('Lecturer integrity: %d/%d files verified.\n',sum([files.verified]),numel(files));
if ~report.all_verified
    error('MananStudy:SourceIntegrity','Missing/changed source: %s', ...
        strjoin({files(~[files.verified]).relative_path},', '));
end
end

function [hash,nBytes]=hash_file(file)
fid=fopen(file,'rb'); assert(fid>=0,'Cannot read %s.',file);
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8'); nBytes=numel(bytes);
digest=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
digest.update(typecast(bytes(:),'int8'));
raw=typecast(digest.digest(),'uint8');
hash=lower(reshape(dec2hex(raw,2).',1,[]));
end
