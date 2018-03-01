function Force_to_Torque(block)
   
setup(block);
  
function setup(block)

  block.NumInputPorts  = 2; %% force in task space, position in joint space
  block.NumOutputPorts = 1; %% torque in joint space
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  
  block.InputPort(1).Dimensions = 2;
  block.OutputPort(1).Dimensions = 2;

  %% l1, l2
  block.NumDialogPrms = 2;

  block.NumContStates = 0;

  block.SampleTimes = [0 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

  block.RegBlockMethod('Outputs', @Outputs);
  
  block.RegBlockMethod('Terminate', @Terminate);


function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
   

function Outputs(block)
  
  F = block.InputPort(1).Data;
  Q = block.InputPort(2).Data;
  q1 = Q(1);
  q2 = Q(2);
  l1 = block.DialogPrm(1).Data;
  l2 = block.DialogPrm(2).Data;
  J11 = -l1*sin(q1)-l2*sin(q1+q2);
  J12 = -l2*sin(q1+q2);
  J21 = l1*cos(q1)+l2*cos(q1+q2);
  J22 = l2*cos(q1+q2);
  J = [J11, J12; J21, J22];
  T = J'*F;

  block.OutputPort(1).Data = T;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);