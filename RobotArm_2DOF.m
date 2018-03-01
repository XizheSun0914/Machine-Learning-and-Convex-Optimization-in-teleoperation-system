function RobotArm_2DOF(block)
   
setup(block);
  
function setup(block)

  block.NumInputPorts  = 1; %% torque
  block.NumOutputPorts = 2; %% position, velocity
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  
  block.InputPort(1).Dimensions = 2;
  block.OutputPort(1).Dimensions = 2;
  block.OutputPort(2).Dimensions = 2;

  %% m1, m2, l1, l2, p10, p20, v10, v20
  block.NumDialogPrms = 8;

  block.NumContStates = 4;

  block.SampleTimes = [0 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Derivatives', @Derivatives);
  
  block.RegBlockMethod('Terminate', @Terminate);


function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode  = fd;
 
  
function Start(block)

  Q0 = [block.DialogPrm(5).Data; block.DialogPrm(6).Data];  
  Q_dot0 = [block.DialogPrm(7).Data; block.DialogPrm(8).Data];

  block.ContStates.Data = [Q0; Q_dot0];
   

function Outputs(block)
  
  block.OutputPort(1).Data = block.ContStates.Data(1:2);
  block.OutputPort(2).Data = block.ContStates.Data(3:4);
  

function Derivatives(block)

g = 9.8;
T = block.InputPort(1).Data;
Q = block.OutputPort(1).Data;
Q_dot = block.OutputPort(2).Data;
m1 = block.DialogPrm(1).Data;
m2 = block.DialogPrm(2).Data;
l1 = block.DialogPrm(3).Data;
l2 = block.DialogPrm(4).Data;
q1 = Q(1);
q2 = Q(2);
q1_dot = Q_dot(1);
q2_dot = Q_dot(2);
M11 = m2*l2^2+(m1+m2)*l1^2+2*m2*l1*l2*cos(q2);
M12 = m2*l2^2+m2*l1*l2*cos(q2);
M21 = M12;
M22 = m2*l2^2;
M = [M11, M12; M21, M22];
C11 = -2*m2*l1*l2*sin(q2)*q2_dot;
C12 = -m2*l1*l2*sin(q2)*q2_dot;
C21 = m2*l1*l2*sin(q2)*q1_dot;
C22 = 0;
C = [C11, C12; C21, C22];
g1 = m2*g*l2*cos(q1+q2)+l1*(m1+m2)*cos(q1);
g2 = m2*g*l2*cos(q1+q2);
G = [g1; g2];
Q_ddot = M\(T-C*Q_dot);

block.Derivatives.Data = [Q_dot; Q_ddot];

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

