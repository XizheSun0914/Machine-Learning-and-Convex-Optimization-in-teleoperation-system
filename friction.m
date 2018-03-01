figure(1);
plot(m_f_2_x.time,m_f_2_x.signals.values,'black');
hold on;
plot(l_f_2_x.time,l_f_2_x.signals.values,'r');
xlabel('Time/s');
ylabel('X/cm');
figure(2);
plot(m_f_2_y.time,m_f_2_y.signals.values,'black');
plot(l_f_2_y.time,l_f_2_y.signals.values,'r');
xlabel('Time/s');
ylabel('Y/cm');