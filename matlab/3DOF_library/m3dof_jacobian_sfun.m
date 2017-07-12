function msfuntmpl_basic(block)

setup(block);

function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        =  3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = [3,3];
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 3;

% Sample times
block.SampleTimes = [-1 0]; % Inherited


block.SimStateCompliance = 'DefaultSimState';


block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

function Terminate(block)

%end Terminate

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

% Jacobian calculation
J(1,1) = -l1*s1 -l2*s12 -l3*s123;
J(1,2) = -l2*s12 -l3*s123;
J(1,3) = -l3*s123;
J(2,1) = l1*c1 + l2*c12 + l3*c123;
J(2,2) = l2*c12 + l3*c123;
J(2,3) = l3*c123;
J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;
A = rand(2,2)*block.InputPort(1).Data(1);


% Write outputs
block.OutputPort(1).Data = J;


%end Outputs


