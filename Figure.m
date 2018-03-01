%{
load('Ym.mat');
load('Ys.mat');
plot(Ym,'-.','linewidth',2,'Color','r');
hold on;
plot(Ys,'-','linewidth',2,'Color','g');
hold on;
ylim([-0.1,0.45]);
xlabel('Time/(s)');
ylabel('Y/(m)');
legend({'Master','Slave'},'Location','southeast','FontSize',15);
%}
%{
load('Pmu.mat');
load('Pms.mat');
plot(Pmu,':','linewidth',2,'Color','m');
hold on;
plot(Pms,'-','linewidth',2,'Color','b');
hold on;
ylim([-3,0.5]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
legend({'P_m','SP_m'},'Location','southeast','FontSize',15);
%}
%{
load('Pdmu.mat');
load('Pdms.mat');
plot(Pdmu,':','linewidth',2,'Color','m');
hold on;
plot(Pdms,'-','linewidth',2,'Color','b');
hold on;
ylim([-0.8,0.4]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
legend({'SP+d_m','S_m(SP+d_m)'},'Location','southeast','FontSize',15);
%}
%{
load('Psu.mat');
load('Pss.mat');
plot(Psu,':','linewidth',2,'Color','m');
hold on;
plot(Pss,'-','linewidth',2,'Color','b');
hold on;
ylim([-1.5,2.5]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
legend({'P_s','SP_s'},'Location','southeast','FontSize',15);
%}
%{
load('Pdsu.mat');
load('Pdss.mat');
plot(Pdsu,':','linewidth',2,'Color','m');
hold on;
plot(Pdss,'-','linewidth',2,'Color','b');
hold on;
ylim([-0.3,0.5]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
legend({'SP+d_s','S_s(SP+d_s)'},'Location','southeast','FontSize',15);
%}
%{
load('Pmu.mat');
load('Pms.mat');
load('Psu.mat');
load('Pss.mat');
a=plot(Pmu,':','linewidth',2,'Color','m');
hold on;
b=plot(Pms,'-','linewidth',2,'Color','b');
hold on;
c=plot(Psu,':','linewidth',2,'Color','g');
hold on;
d=plot(Pss,'-','linewidth',2,'Color','r');
hold on;
ylim([-3,2.5]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
legend({'P_m','SP_m','P_s','SP_s'},'Location','southeast','FontSize',15,'Orientation','horizontal');
%}
%{
load('Pdmu.mat');
load('Pdms.mat');
load('Pdsu.mat');
load('Pdss.mat');
a=plot(Pdmu,'-','linewidth',2,'Color','k');
hold on;
b=plot(Pdms,'--','linewidth',2,'Color','m');
hold on;
c=plot(Pdsu,'-','linewidth',2,'Color','r');
hold on;
d=plot(Pdss,'--','linewidth',2,'Color','g');
hold on;
ylim([-0.8,0.5]);
xlabel('Time/(s)');
ylabel('Torque/(Nm)');
gridLegend([a,b,c,d],2,{'SP+d_m','S_m(SP+d_m)','SP+d_s','S_s(SP+d_s)'},'location','best','Fontsize',15);
%}