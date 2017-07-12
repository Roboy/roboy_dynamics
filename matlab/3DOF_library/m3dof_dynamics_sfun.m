function msfuntmpl_basic(block)

setup(block);

function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        =  3;
block.InputPort(2).Dimensions        =  3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = [3,3];
block.OutputPort(2).Dimensions       = [3,3];
block.OutputPort(3).Dimensions       = [3];
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 5;

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

function Outputs(block)
% Read inputs and parameters
q = block.InputPort(1).Data;
q_dot = block.InputPort(1).Data;
xi1 = block.DialogPrm(1).Data;
xi2 = block.DialogPrm(2).Data;
xi3 = block.DialogPrm(3).Data;
a = block.DialogPrm(4).Data;
eg = block.DialogPrm(5).Data;



% Write outputs
block.OutputPort(1).Data = mass(q,xi1,xi2,xi3,a);
block.OutputPort(2).Data = coriolis(q, q_dot,xi1, xi2, xi3, a);
block.OutputPort(3).Data = gravity(q,xi1,xi2,xi3,a,eg);


%end Outputs

function M = mass(q, xi1, xi2, xi3, a)

% Declaration and costants
M = zeros(3,3);
MAS = zeros(3,3);

% Support variables
t1 = a(1)^2;
t3 = cos(q(3));
t4 = cos(q(2));
t6 = a(1) * xi3(2);
t7 = t3 * t4 * t6;
t9 = sin(q(3));
t10 = sin(q(2));
t12 = t9 * t10 * t6;
t14 = a(2)^2;
t15 = t14 * xi3(1);
t16 = t4 * a(1);
t18 = t16 * a(2) * xi3(1);
t20 = t16 * xi2(2);
t23 = t3 * a(2) * xi3(2);
t24 = 0.2e1 * t23;
t26 = xi3(3) + t1 * xi2(1) + 0.2e1 * t7 - 0.2e1 * t12 + xi2(3) + xi1(3) + t15 + 0.2e1 * t18 + 0.2e1 * t20 + t24 + t1 * xi3(1);
t27 = xi2(3) + t20 + t15 + t18 - t12 + t24 + xi3(3) + t7;
t28 = xi3(3) - t12 + t7 + t23;
t30 = xi3(3) + t23;
MAS(1,1) = t26;
MAS(1,2) = t27;
MAS(1,3) = t28;
MAS(2,1) = t27;
MAS(2,2) = xi3(3) + xi2(3) + t24 + t15;
MAS(2,3) = t30;
MAS(3,1) = t28;
MAS(3,2) = t30;
MAS(3,3) = xi3(3);


% Inertia matrix
M = MAS';

function C = coriolis(q, q_dot,xi1, xi2, xi3, a)
% Declaration and costants
C = zeros(3,3);
CORIOLIS = zeros(3,3);


% Support variables
qp = q_dot;
t1 = cos(q(3));
t2 = sin(q(2));
t3 = t1 * t2;
t5 = qp(2) * a(1) * xi3(2);
t6 = t3 * t5;
t7 = sin(q(3));
t8 = cos(q(2));
t9 = t7 * t8;
t11 = qp(3) * a(1) * xi3(2);
t12 = t9 * t11;
t13 = t3 * t11;
t14 = t2 * qp(2);
t16 = a(1) * a(2) * xi3(1);
t17 = t14 * t16;
t19 = a(2) * xi3(2);
t20 = t7 * qp(3) * t19;
t21 = t9 * t5;
t22 = a(1) * xi2(2);
t23 = t14 * t22;
t25 = qp(1) * a(1);
t26 = t25 * xi3(2);
t27 = t9 * t26;
t28 = t2 * qp(1);
t29 = t28 * t22;
t30 = t28 * t16;
t31 = t3 * t26;
t32 = -t13 - t27 - t29 - t6 - t12 - t21 - t30 - t17 - t31 - t23 - t20;
t33 = qp(1) + qp(2) + qp(3);
CORIOLIS(1,1) = -t6 - t12 - t13 - t17 - t20 - t21 - t23;
CORIOLIS(1,2) = t32;
CORIOLIS(1,3) = -t33 * (t3 * a(1) + t7 * a(2) + t9 * a(1)) * xi3(2);
CORIOLIS(2,1) = t31 + t29 - t20 + t30 + t27;
CORIOLIS(2,2) = -t20;
CORIOLIS(2,3) = -t7 * t33 * t19;
CORIOLIS(3,1) = (t9 * t25 + t3 * t25 + t7 * qp(2) * a(2) + t7 * qp(1) * a(2)) * xi3(2);
CORIOLIS(3,2) = t7 * (qp(1) + qp(2)) * t19;
CORIOLIS(3,3) = 0.0e0;


% Coriolis Matrix
C = CORIOLIS;


function g = gravity(q,xi1,xi2,xi3,a,eg)
% Costants and declarations
g = zeros(3,1);
g0 = 9.81;


% Support variables
t1 = sin(q(1));
t2 = eg(1) * t1;
t4 = cos(q(1));
t5 = eg(1) * t4;
t6 = sin(q(2));
t7 = t6 * xi2(2);
t8 = t5 * t7;
t9 = cos(q(2));
t10 = t9 * xi2(2);
t11 = t2 * t10;
t12 = eg(2) * t1;
t13 = t12 * t7;
t14 = eg(2) * t4;
t15 = t14 * t10;
t16 = a(1) * xi2(1);
t19 = a(1) * xi3(1);
t22 = sin(q(3));
t23 = eg(1) * t22;
t24 = t1 * t6;
t25 = t24 * xi3(2);
t26 = t23 * t25;
t27 = cos(q(3));
t28 = eg(1) * t27;
t29 = t4 * t6;
t30 = t29 * xi3(2);
t31 = t28 * t30;
t32 = t2 * xi1(2) + t8 + t11 + t13 - t15 - t14 * t16 + t2 * t16 - t14 * t19 + t2 * t19 - t26 + t31;
t33 = t1 * t9;
t34 = t33 * xi3(2);
t35 = t28 * t34;
t37 = t9 * a(2) * xi3(1);
t38 = t2 * t37;
t39 = eg(2) * t22;
t40 = t39 * t30;
t42 = t6 * a(2) * xi3(1);
t43 = t12 * t42;
t44 = eg(2) * t27;
t45 = t4 * t9;
t46 = t45 * xi3(2);
t47 = t44 * t46;
t48 = t39 * t34;
t49 = t5 * t42;
t50 = t44 * t25;
t51 = t14 * t37;
t53 = t23 * t46;
t54 = t35 + t38 + t40 + t43 - t47 + t48 + t49 + t50 - t51 - t14 * xi1(2) + t53;
t57 = t53 - t26 + t50 + t8 + t13 + t35 + t49 - t15 + t40 + t48 + t11 - t51 - t47 + t43 + t38 + t31;

% Gravity vector
g(1) = g0 * (t32 + t54);
g(2) = g0 * t57;
g(3) = g0 * (t28 * t29 + t28 * t33 + t23 * t45 - t23 * t24 - t44 * t45 + t44 * t24 + t39 * t29 + t39 * t33) * xi3(2);


