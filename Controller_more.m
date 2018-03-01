function Controller_more(block)
   
setup(block);
  

function setup(block)

  block.NumInputPorts  = 2;  % wrench_e, pose
  block.NumOutputPorts = 3;  % wrench_c, xi, energy
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';

  %% bDevice, pose0, dof, tsampling
  block.NumDialogPrms     = 4;

  tsampling = block.DialogPrm(4).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  
  dof = block.DialogPrm(3).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(3).Dimensions = 1;
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 4;
  
  dof = block.DialogPrm(3).Data;
  
  block.Dwork(1).Name            = 'wrench_d_n_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'pose_n_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'xi_n_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'energy_n_1';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  pose0 = block.DialogPrm(2).Data;
  
  block.Dwork(1).Data = zeros(6,1);
  block.Dwork(2).Data = pose0; 
  block.Dwork(3).Data = 1;
  block.Dwork(4).Data = 0;
   

function Outputs(block)
  
  tsampling = block.DialogPrm(4).Data;
  Damping = diag(block.DialogPrm(1).Data/tsampling);
  
  wrench_e_n = block.InputPort(1).Data;
  wrench_d_n_1 = block.Dwork(1).Data;
  pose_n = block.InputPort(2).Data;
  pose_n_1 = block.Dwork(2).Data;
  dpose = pose_n - pose_n_1;
  xi_n_1 = block.Dwork(3).Data;
  CDamping = chol(Damping);
  phi = 2*xi_n_1*CDamping'*dpose - CDamping\wrench_d_n_1;
  if norm(wrench_e_n) ~= 0 
      u = wrench_e_n/norm(wrench_e_n);
      alpha = u'*(Damping\u);
      xi_n = (xi_n_1*alpha*norm(wrench_e_n)^2)/(phi'*phi);
  else
      u = zeros(6,1);
      alpha = 0;
      xi_n = xi_n_1;
  end
  
  if xi_n >1
      xi_n = 1;
  end
  
  if alpha ~= 0
      wrench_d_max = sqrt((xi_n*phi'*phi)/(alpha*xi_n_1));
  else
      wrench_d_max = 0;
  end
  wrench_d_min = -wrench_d_max;
  
  if norm(wrench_e_n) > wrench_d_max
      wrench_d_n = wrench_d_max*u;
  elseif norm(wrench_e_n) < wrench_d_min
      wrench_d_n = wrench_d_min*u;
  else
      wrench_d_n = wrench_e_n;
  end
  
  energy_n_1 = block.Dwork(4).Data;
  energy_n = energy_n_1 + dpose'*Damping*dpose - wrench_d_n_1'*dpose;
      
  block.OutputPort(1).Data = wrench_d_n;
  block.OutputPort(2).Data = xi_n;
  block.OutputPort(3).Data = energy_n;


function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  block.Dwork(4).Data = block.OutputPort(3).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

