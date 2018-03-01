function VirtualEnvironment(block)
  
setup(block);

function setup(block)

  block.NumInputPorts  = 2;  % pose_dot, pose
  block.NumOutputPorts = 1;  % wrench_e,
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';


  %% kWall, bWall, nWall, pWall, vertices, dof, tsampling
  block.NumDialogPrms     = 7;
  
  tsampling = block.DialogPrm(7).Data;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  
  block.SampleTimes = [tsampling 0];
  
  dof = block.DialogPrm(6).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Terminate', @Terminate);
  

function Outputs(block)
  
  pose_dot_n = block.InputPort(1).Data;
  pose_n = block.InputPort(2).Data;
  
  kWall = diag(block.DialogPrm(1).Data);
  bWall = diag(block.DialogPrm(2).Data);
  nWall = block.DialogPrm(3).Data;
  pWall = block.DialogPrm(4).Data;
  vertices = block.DialogPrm(5).Data;
  dof = block.DialogPrm(6).Data;
  
  x = pose_n(1);
  y = pose_n(2);
  z = pose_n(3);
  theta_x = pose_n(4); 
  theta_y = pose_n(5);
  theta_z = pose_n(6);
  Trans = [cos(theta_z)*cos(theta_y), cos(theta_z)*sin(theta_y)*sin(theta_x)-sin(theta_z)*cos(theta_x), cos(theta_z)*sin(theta_y)*cos(theta_x)+sin(theta_z)*sin(theta_x), x;
           sin(theta_z)*cos(theta_y), sin(theta_z)*sin(theta_y)*sin(theta_x)+cos(theta_z)*cos(theta_x), sin(theta_z)*sin(theta_y)*cos(theta_x)-cos(theta_z)*sin(theta_x), y;
           -sin(theta_y),             cos(theta_y)*sin(theta_x),                                        cos(theta_y)*cos(theta_x)                                         z;
           0,                         0,                                                                0,                                                                1];
  wrench_e = zeros(dof,1);
  
  for i=1:1:3        
      for j=1:1:8      
          vertex_local = vertices(:,j);
          vertex = Trans*[vertex_local;1];
          vertex_global = vertex(1:3);
          pWall_vertex = vertex_global-pWall;

          r_global = vertex_global - pose_n(1:3);
          vr_global = cross(r_global, pose_dot_n(4:6));
          vt_global = pose_dot_n(1:3);
          v_global = vt_global + vr_global; 
          
          if (nWall(:,i)'*pWall_vertex) <= 0           
              force = -kWall*(nWall(:,i)'*pWall_vertex)*nWall(:,i)/(nWall(:,i)'*nWall(:,i)) - bWall*(nWall(:,i)'*v_global)*nWall(:,i)/(nWall(:,i)'*nWall(:,i));
              if (force'*nWall(:,i)) < 0
                  force = zeros(3,1);
              end
          else
              force = zeros(3,1);
          end    
          
          torque = cross(r_global,force);
          wrench_e = wrench_e + [force; torque];
      end    
  end

  block.OutputPort(1).Data = wrench_e;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);