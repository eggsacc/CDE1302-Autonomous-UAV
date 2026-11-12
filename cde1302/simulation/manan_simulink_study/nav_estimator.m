function nav_estimator(block)
%NAV_ESTIMATOR Level-2 MATLAB S-function wrapper around nav_core.
% Inputs  1 true position (NED, 3)   2 true velocity (3)
%         3 true quaternion [w x y z] body->NED (4)   4 true body rates (3)
%         Truth reaches only the simulated sensors inside nav_core.
% Outputs 1 position error to add to feedback (3)
%         2 velocity error to add to feedback (3)
%         3 16-element column-major 4x4 matrix: q_feedback = M * q_true
%         4 navigation diagnostics (16, see nav_core('fields'))
%         5 sightings that arrived this step (nSlots x 12, NaN when none)
% Outputs computes from the committed state only; Update commits it, so a
% repeated evaluation at the same time cannot advance the estimator twice.
% The configuration is cached at initialisation (it carries pre-drawn noise;
% re-reading the dialog parameter every 20 ms is needlessly slow).
setup(block);
end

function setup(block)
block.NumDialogPrms=1; block.DialogPrmsTunable={'Nontunable'};
cfg=block.DialogPrm(1).Data; L=nav_core('layout',cfg);
block.NumInputPorts=4; block.NumOutputPorts=5;
block.SetPreCompPortInfoToDefaults;
dims=[3 3 4 3];
for k=1:4
    block.InputPort(k).Dimensions=dims(k); block.InputPort(k).DatatypeID=0;
    block.InputPort(k).Complexity='Real'; block.InputPort(k).DirectFeedthrough=true;
end
outDims=[3 3 16 L.diagLen L.sightLen];
for k=1:5
    block.OutputPort(k).Dimensions=outDims(k); block.OutputPort(k).DatatypeID=0;
    block.OutputPort(k).Complexity='Real';
end
block.SampleTimes=[cfg.nav.dt 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@postSetup);
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end

function postSetup(block)
L=nav_core('layout',block.DialogPrm(1).Data);
block.NumDworks=2;
names={'navState','navNext'};
for k=1:2
    block.Dwork(k).Name=names{k}; block.Dwork(k).Dimensions=L.total;
    block.Dwork(k).DatatypeID=0; block.Dwork(k).Complexity='Real';
    block.Dwork(k).UsedAsDiscState=(k==1);
end
end

function initialize(block)
cfg=block.DialogPrm(1).Data;
cfgCache('set',block.BlockHandle,cfg);
x=nav_core('initial',cfg);
block.Dwork(1).Data=x; block.Dwork(2).Data=x;
end

function outputs(block)
cfg=cfgCache('get',block.BlockHandle);
[dpos,dvel,qMat,diagOut,sight,next]=nav_core('step',block.CurrentTime, ...
    block.InputPort(1).Data,block.InputPort(2).Data,block.InputPort(3).Data, ...
    block.InputPort(4).Data,block.Dwork(1).Data,cfg);
block.OutputPort(1).Data=dpos; block.OutputPort(2).Data=dvel;
block.OutputPort(3).Data=qMat; block.OutputPort(4).Data=diagOut;
block.OutputPort(5).Data=sight; block.Dwork(2).Data=next;
nav_live('tick',block.CurrentTime,block.InputPort(1).Data, ...
    block.InputPort(3).Data,diagOut);   % display only; no-op outside the GUI
end

function update(block)
block.Dwork(1).Data=block.Dwork(2).Data;
end

function cfg=cfgCache(op,handle,cfg)
% Refreshed at every InitializeConditions, so a rebuilt model never sees a
% previous run's configuration.
persistent store
if isempty(store), store=containers.Map('KeyType','double','ValueType','any'); end
switch op
    case 'set', store(handle)=cfg;
    case 'get', cfg=store(handle);
end
end
