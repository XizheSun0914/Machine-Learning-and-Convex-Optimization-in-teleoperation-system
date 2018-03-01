clear all;
close all;

dof = 3;

%% Virtual Environment
kWall = 1000000;
bWall = 0;
nWall = 1;
xWall = 10;
rSpere = 10;
cSphere = [0; 0; 0];

%% Haptic Interface
m=1;
bHI=1;
p0=[0;0;15];
v0=[0;0;0];

%% Haptic Controller
A=0;

%% Sample time
tsampling=0.001;

F_hand = 10;
direction = [0; 0; -1];
