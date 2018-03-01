function [sys,x0,str,ts,simStateCompliance] = virtualenvironment(t,x,u,flag,kBall,bBall,rBall,cBall)

switch flag,
  
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 3
    sys=mdlOutputs(t,x,u,kBall,bBall,rBall,cBall);

  case { 1, 2, 4, 9 }
    sys=[];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;  % dynamically sized
sizes.NumInputs      = 6;  % dynamically sized
sizes.DirFeedthrough = 1;  % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   % sample time

% specify that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';


function sys = mdlOutputs(t,x,u,kBall,bBall,rBall,cBall)
KBall=[kBall 0 0;0 kBall 0;0 0 kBall];
BBall=[bBall 0 0;0 bBall 0;0 0 bBall];
x=[u(4);u(5);u(6)];
vel=[u(1);u(2);u(3)];
cx=[cBall'; x'];
dist=pdist(cx);
nBall=[x-cBall]./dist;
if dist<rBall
    f=(rBall-dist)*KBall*nBall-BBall*(vel'*nBall)*nBall;
    if f'*nBall<0
        f=[0;0;0];
    end
else
    f=[0;0;0];
end

sys = [f(1);f(2);f(3)];
