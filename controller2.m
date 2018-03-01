function controller2(block)
% Level-2 MATLAB file S-Function for implementing Ryu's
% least conservative multi-dof energy bounding algorithm

setup(block);
  
%endfunction

% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)
  
  % dof, b_device, tsampling, x0_device
  block.NumDialogPrms = 4;
  dof= block.DialogPrm(1).Data;
  tsampling= block.DialogPrm(3).Data;
  
  % Register the number of ports.
  % F_wall, x_device
  block.NumInputPorts  = 2; 
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  
  % F_controller, fmax, Energy
  block.NumOutputPorts = 3; 
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(3).Dimensions = 1;
    
  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time    
  block.SampleTimes = [tsampling 0];
    
  block.InputPort(1).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode  = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode  = tsampling;
  block.OutputPort(3).SamplingMode  = tsampling;

  block.RegBlockMethod('Start', @Start);
  block.RegBlockMethod('Update', @Update);  
  block.RegBlockMethod('Outputs', @Outputs);
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  block.RegBlockMethod('Terminate', @Terminate);


  %endfunction
  
function Start(block)
  
  dof = block.DialogPrm(1).Data;
  p0 = block.DialogPrm(4).Data;
  block.Dwork(1).Data = zeros(dof,1);
  block.Dwork(2).Data = p0; 
  block.Dwork(3).Data = 0;
   
%endfunction

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(3).Data;
  
%endfunction

function Outputs(block)
    
  dof = block.DialogPrm(1).Data;
  b = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  
  B = b/tsampling * eye(3,3);
  
  x_n = block.InputPort(2).Data;
  x_n_1 = block.Dwork(2).Data;
  
  Fd_n_1 = block.Dwork(1).Data; % Fd(n-1)
  Fe_n = block.InputPort(1).Data; % Fe(n)
  E_n_1 = block.Dwork(3).Data;
  if norm(Fd_n_1) == 0
      % no contact at previous time step, ignore accumulated energy
      E_n_1 = 0;
  end;

  d = x_n - x_n_1; % dx = x(n)-x(n-1)
  E_n = E_n_1 + d'*B*d - Fd_n_1'*d; % E(n)
  
  
  %% ud(n) & alpha(n)
  if norm(Fe_n) ~= 0 
      % contact
      ud = Fe_n./norm(Fe_n);
      alpha = ud'*inv(B)*ud;
      fmax = nthroot(4*E_n/alpha,2); % fdmax(n) 
      fmax = sqrt(4*E_n/alpha); % fdmax(n) 
      if norm(Fe_n) > fmax
          fd = fmax;
      elseif norm(Fe_n) < -fmax
          fd = -fmax;
      else
          fd = norm(Fe_n);
      end
      Fd_n = fd*ud; % Fd(n)
  else
      % no contact
      fmax = 0;
      Fd_n = zeros(dof,1);
  end;

block.OutputPort(1).Data = Fd_n;
block.OutputPort(2).Data = fmax;
block.OutputPort(3).Data = E_n;
  
%endfunction

function DoPostPropSetup(block)
  
  dof= block.DialogPrm(1).Data;
  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'F_controller_n_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'xdevice_n_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;

  block.Dwork(3).Name            = 'Energy_n_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;


%endfunction


    
function Terminate(block)

%endfunction
 