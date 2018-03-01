%{
load('Vm.mat');
load('Vs.mat');
plot(Vm,'linewidth',2);
hold on;
plot(Vs,'linewidth',2);
ylim([-0.3,0.3]);
xlabel('Time/(s)');
ylabel('Velocity/(m/s)');
legend({'Vm','Vs'},'Location','southeast','FontSize',15);
hold on;
%}
%{
load('Xm.mat');
load('Xs.mat');
plot(Xm,'linewidth',2);
hold on;
plot(Xs,'linewidth',2);
ylim([-0.2,0.6]);
xlabel('Time/(s)');
ylabel('Position/(m)');
legend({'Xm','Xs'},'Location','southeast','FontSize',15);
hold on;
%}
%{
load('Fh.mat');
load('Fs.mat');
plot(Fh,'linewidth',2);
hold on;
plot(Fs,'linewidth',2);
ylim([-2,3]);
xlabel('Time/(s)');
ylabel('Force/(N)');
legend({'Fh','Fs'},'Location','southeast','FontSize',15);
hold on;
%}
%{
load('Fe.mat');
load('Fm.mat');
plot(Fe,'linewidth',2);
hold on;
plot(Fm,'linewidth',2);
ylim([-7,1]);
xlabel('Time/(s)');
ylabel('Force/(N)');
legend({'Fe','Fm'},'Location','southeast','FontSize',15);
hold on;
%}