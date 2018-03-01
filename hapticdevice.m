function [sys,x0,str,ts,simStateCompliance] = hapticdevice(t,x,u,flag,m,b,v0,p0)

switch flag,
 
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(m,b,v0,p0);

  case 1
        sys=mdlDerivatives(t,x,u,m,b,v0,p0);
 
  case 3
    sys=mdlOutputs(t,x,u);

  case { 2, 4, 9 }
    sys=[];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(m,b,v0,p0)

sizes = simsizes;
sizes.NumContStates  = 6;  %number of continuous states
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;  % six outputs, the first three are velocity in x,y,z and the last three are position in x,y,z
sizes.NumInputs      = 3;  % force in x,y,z
sizes.DirFeedthrough = 1;   % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [v0(1);v0(2);v0(3);p0(1);p0(2);p0(3)];  %set initial condition v0 and x0
ts  = [0 0];   % continuous


simStateCompliance = 'DefaultSimState';

%% compute the derivatives of x(velocity and position)----the acceleration and velocity
function sys= mdlDerivatives(t,x,u,m,b,v0,p0)

M   = [m 0 0;0 m 0;0 0 m];  % mass matrix
B   = [b 0 0;0 b 0;0 0 b];  % damping matrix
V   = [x(1);x(2);x(3)];  % velocity vector 3*1
A   = inv(M)*(u-B*V);  % acceleration vector 3*1
sys=[A(1);A(2);A(3);V(1);V(2);V(3)];


%% compute x(velocity and position)
function sys = mdlOutputs(t,x,u)

sys=[x(1);x(2);x(3);x(4);x(5);x(6)];  % output the velocity and position



