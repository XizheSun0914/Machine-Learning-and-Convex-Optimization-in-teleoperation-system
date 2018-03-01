function HapticDevice(block)
   
setup(block);
  

function setup(block)

  block.NumInputPorts  = 1;  % wrench
  block.NumOutputPorts = 2;  % pose_dot, pose
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';

  %% miDevice, bDevice, pose_dot0, pose0, dof
  block.NumDialogPrms     = 5;
  
  dof = block.DialogPrm(5).Data;
  block.InputPort(1).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= dof;
  
  block.NumContStates = 12;

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

  pose_dot0 = block.DialogPrm(3).Data;
  pose0 = block.DialogPrm(4).Data;

  block.ContStates.Data = [pose_dot0; pose0];
 
  
function Outputs(block)
  
  block.OutputPort(1).Data = block.ContStates.Data(1:6);
  block.OutputPort(2).Data = block.ContStates.Data(7:12);
  

function Derivatives(block)

  wrench = block.InputPort(1).Data;
  miDevice = diag(block.DialogPrm(1).Data);
  bDevice = diag(block.DialogPrm(2).Data);
  pose_dot = block.OutputPort(1).Data;
  pose_2dot = miDevice\(wrench-bDevice*pose_dot);

block.Derivatives.Data = [pose_2dot; pose_dot];

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
