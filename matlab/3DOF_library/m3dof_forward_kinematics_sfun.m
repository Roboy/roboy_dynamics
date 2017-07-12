function msfuntmpl_basic(block)

setup(block);

function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        =  3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = [3,4];
block.OutputPort(2).Dimensions       = [3,4];
block.OutputPort(3).Dimensions       = [3,4];
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 3;

% Sample times
block.SampleTimes = [-1 0]; % Inherited


block.SimStateCompliance = 'DefaultSimState';


 
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

function Terminate(block)

%end Terminate

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode  = fd;
  block.OutputPort(3).SamplingMode  = fd;
  
%end SetIntPortFrameData

function Outputs(block)
% Read inputs and parameters
q = block.InputPort(1).Data;
l1 = block.DialogPrm(1).Data;
l2 = block.DialogPrm(1).Data;
l3 = block.DialogPrm(1).Data;

% Angles calculation
c1 = cos(q(1));
s1 = sin(q(1));
c12 = cos(q(1) + q(2));
s12 = sin(q(1) + q(2));
c123 = cos(q(1) + q(2) + q(3));
s123 = sin(q(1) + q(2) + q(3));

% Homogeneous matrices
T01 = [c1 -s1 0 l1*c1; s1 c1 0 l1*s1; 0 0 1 0];
T02 = [c12 -s12 0 (l1*c1 + l2*c12); s12 c12 0 (l1*s1 + l2*s12); 0 0 1 0];
T03 = [c123 -s123 0 (l1*c1 + l2*c12 + l3*c123); s123 c123 0 (l1*s1 + l2*s12 + l3*s123); 0 0 1 0];


% Write outputs
block.OutputPort(1).Data = T01;
block.OutputPort(2).Data = T02;
block.OutputPort(3).Data = T03;


%end Outputs


