%% parameters for the system model
Dm=0.2; %maximum forward time delay 
Ds=0.1; %maximum backward time delay
P = 10; % proportional control gain in master and slave controllers
Km1 = 3; % damping in master and slave controllers
Km2 = 1;
Ks1 = 3;
Ks2 = 1;
B=0.1; % gain of the nonlinear term in my controller
Gm = 0.1;
Gs = 0.5;
Mm1=1; % mass of master robot link 1 
Mm2=1; % mass of master robot link 2
Ms1=2; % mass of slave robot link 1
Ms2=2; % mass of slave robot link 2
Lm1=0.5; % length of master robot link 1
Lm2=0.5; % length of master robot link 2
Ls1=0.5; % length of slave robot link 1
Ls2=0.5; % length of slave robot link 2
q0 = [0;0]; % initial position of master and slave robots
v0 = [0;0]; % initial velocity of master and slave robots

%% coefficients defined by myself to determine proper control gains
lambda_m=0.5;
lambda_s=0.5;
delta_m=1;
delta_s=1;
gamma=0.01;
k=10;
Mm=1.4571;
Ms=2.9142;