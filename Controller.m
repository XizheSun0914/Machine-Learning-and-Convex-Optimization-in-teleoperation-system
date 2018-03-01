function Controller(block)
  
setup(block);
  
function setup(block)

  block.NumInputPorts  = 3;  % force, r, pose
  block.NumOutputPorts = 5;  % wrench_c
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(3).DatatypeID  = 0;  % double
  block.InputPort(3).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';
  block.OutputPort(4).DatatypeID  = 0; % double
  block.OutputPort(4).Complexity  = 'Real';
  block.OutputPort(5).DatatypeID  = 0; % double
  block.OutputPort(5).Complexity  = 'Real';

  %% bDevice, dof, tsampling, pose0, vertices
  block.NumDialogPrms     = 5;
  
  tsampling = block.DialogPrm(3).Data;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.InputPort(3).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  block.OutputPort(4).SamplingMode = tsampling;
  block.OutputPort(5).SamplingMode = tsampling;
  
  dof = block.DialogPrm(2).Data;
  
  block.InputPort(1).Dimensions = 8;
  block.InputPort(2).Dimensions = 8;
  block.InputPort(3).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(3).Dimensions = 1;
  block.OutputPort(4).Dimensions = 8;
  block.OutputPort(5).Dimensions = 1;
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 6;
  
  dof = block.DialogPrm(2).Data;
  
  block.Dwork(1).Name            = 'forces_c_n_1';
  block.Dwork(1).Dimensions      = 8;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'r_n_1';
  block.Dwork(2).Dimensions      = 8;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'pose_n_1';
  block.Dwork(3).Dimensions      = dof;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'E_n_1';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name            = 'xi_n_1';
  block.Dwork(5).Dimensions      = 1;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  block.Dwork(6).Name            = 'num_1';
  block.Dwork(6).Dimensions      = 1;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  pose0 = block.DialogPrm(4).Data;
  r0 = reshape(block.DialogPrm(5).Data,8,1)-reshape(pose0(1:2)*[1 1 1 1],8,1);
  
  block.Dwork(1).Data = zeros(8,1);
  block.Dwork(2).Data = r0; 
  block.Dwork(3).Data = pose0;
  block.Dwork(4).Data = 0; 
  block.Dwork(5).Data = 1; 
  block.Dwork(6).Data = 0;
   

function Outputs(block)

  forces_e_n = reshape(block.InputPort(1).Data,2,4);
  forces_c_n_1 = reshape(block.Dwork(1).Data,2,4);
  r_n = reshape(block.InputPort(2).Data,2,4);
  r_n_1 = reshape(block.Dwork(2).Data,2,4);
  pose_n = block.InputPort(3).Data;
  pose_n_1=block.Dwork(3).Data;
  dpose = pose_n-pose_n_1;
  delta_xy = dpose(1:2);
  delta_theta = dpose(3);
  xi_n_1 = block.Dwork(5).Data;
  
  num_f=0;
  for i=1:1:4
      if norm(forces_e_n(:,i))~=0
          num_f = num_f +1;
      end
  end
  
  if num_f ==0
      bDevice = block.DialogPrm(1).Data/4;
  else
      bDevice = block.DialogPrm(1).Data/num_f;
  end
  
  bDevice_total = block.DialogPrm(1).Data;
 
  tsampling = block.DialogPrm(3).Data;
  Bl_total = diag(bDevice_total(1:2)/tsampling);
  Ba_total = diag(bDevice_total(3)/tsampling);
  Bl = diag(bDevice(1:2)/tsampling);
  Ba = bDevice(3)/tsampling;
  
  E_n_1 = block.Dwork(4).Data;
  
  num_1 = block.Dwork(6).Data;
 
  if norm(forces_e_n)~=0
      num=num_1+1;
  else
      num=num_1;
  end
  if num==1
      E_n = 0;
  else
      E_n = E_n_1 + delta_xy'*Bl_total*delta_xy + delta_theta'*Ba_total*delta_theta;
  end
 
 %{ 
  num=num_1;
  E_n = E_n_1 + delta_xy'*Bl_total*delta_xy + delta_theta'*Ba_total*delta_theta;
  %}     
  
  E = 0;
  moment = 0;
  force = [0;0];
  
  for i=1:1:4
      Fd_i_n_1 = forces_c_n_1(:,i);
      r_i_n_1 = r_n_1(:,i);
      E_n = E_n - Fd_i_n_1'*delta_xy - cross([r_i_n_1;0],[Fd_i_n_1;0])'*[0;0;delta_theta];
      Fd_i_n = forces_e_n(:,i);
      r_i_n = r_n(:,i);
      M_i_n = cross([r_i_n;0],[Fd_i_n;0]);
      E = E + (Fd_i_n'*((Bl)\Fd_i_n)+M_i_n(3)'*((Ba)\M_i_n(3)))/4;
%{
      if num_f ~= 0
          if norm(forces_e_n(:,i))~=0
            E = E + (Fd_i_n'*((Bl)\Fd_i_n)+M_i_n(3)'*((Ba)\M_i_n(3)))/4;
          end
      else
          E = E + (Fd_i_n'*((Bl)\Fd_i_n)+M_i_n(3)'*((Ba)\M_i_n(3)))/4;
      end
 %}
  end
  
  if E ==0
      xi_n = xi_n_1;
  else  
      if E_n>=0
        xi_n = sqrt(E_n/E); 
      else
          xi_n=-sqrt(-E_n/E);
      end
      if xi_n >1
          xi_n = 1;
      elseif xi_n <-1
          xi_n=-1;
      end
  end
  
  forces_c_n = xi_n * forces_e_n;
  
  for i=1:1:4
      
      moment3 = cross([r_n(:,i); 0],[forces_c_n(:,i); 0]);
      moment = moment + moment3(3);
      force = force + forces_c_n(:,i);
      
  end

  block.OutputPort(1).Data = [force; moment];
  block.OutputPort(2).Data = E_n;
  block.OutputPort(3).Data = xi_n;
  block.OutputPort(4).Data = reshape(forces_c_n,8,1);
  block.OutputPort(5).Data = num;
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(4).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.InputPort(3).Data;
  block.Dwork(4).Data = block.OutputPort(2).Data;
  block.Dwork(5).Data = block.OutputPort(3).Data;
  block.Dwork(6).Data = block.OutputPort(5).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);