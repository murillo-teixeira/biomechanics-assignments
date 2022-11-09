clear;clc;

joints = table2array(readtable("Joints_Running.xlsx", 'FileType','spreadsheet'));
t=joints(9:166, 1) - joints(9, 1);
mass = 58.9;
n=1;

sgtitle('Moments of force during running gait', 'FontName', 'Times', 'FontSize', 11)

subplot(3, 2, 1)
title('Novacheck, 1998', 'FontName', 'Times', 'FontSize', 10)
data = table2array(readtable("Moment_Hip_Run.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-3, 3]);
ylabel({'Hip moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

subplot(3, 2, 3)
data = table2array(readtable("Moment_Knee_Run.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-3, 3]);
ylabel({'Knee moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)
subplot(3, 2, 5)
data = table2array(readtable("Moment_Ankle_Run.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-3, 3]);
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Ankle moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

i = 4;
subplot(3, 2, 2)
joint_n     = i;
body_n      = 1;
variable_n  = 3;
Mhip    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
Mhip= [Mhip(83:end); Mhip(1:82)];
hold on
plot(100*t/max(t),  Mhip/mass)
plot([0 100],[0 0], 'k')
hold off
title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
xlim([0, 100]);
ylim([-3, 3]);

i = 8;
subplot(3, 2, 4)
joint_n     = i;
variable_n  = 3;
body_n = 2;
Mknee    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
Mknee= [Mknee(83:end); Mknee(1:82)];
hold on
plot(100*t/max(t),  Mknee/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-3, 3]);

i = 9;
subplot(3, 2, 6)
joint_n     = i;
body_n      = 1;
variable_n  = 3;
Mankle    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
Mankle= [Mankle(83:end); Mankle(1:82)];
hold on
plot(100*t/max(t),  Mankle/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-3, 3]);
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
% 
% subplot(4, 2, 2)
% title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
% variable    = Mankle+Mankle+Mankle;
% hold on
% plot(100*t/max(t),  variable/mass)
% plot([0 100],[0 0], 'k')
% hold off
% xlim([0, 100]);
% ylim([-3, 3]);


% 
% clear;clc;
% 
% joints = table2array(readtable("Joints_Running.xlsx", 'FileType','spreadsheet'));
% t=joints(9:166, 1) - joints(9, 1);
% mass = 58.9;
% n=1;
% 
% figure(1)
% sgtitle('Running')
% i = 4;
% subplot(3, 1, 1)
% joint_n     = i;
% body_n      = 2;
% variable_n  = 3;
% variable    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
% variable = [variable(83:end); variable(1:82)];
% hold on
% plot(100*t/max(t),  variable/mass)
% plot([0 100],[0 0], 'k')
% hold off
% xlim([0, 100]);
% ylim([-3, 3]);
% title('Right hip')
% 
% i = 8;
% subplot(3, 1, 2)
% joint_n     = i;
% variable_n  = 3;
% body_n = 2;
% variable    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
% variable = [variable(83:end); variable(1:82)];
% hold on
% plot(100*t/max(t),  variable/mass)
% plot([0 100],[0 0], 'k')
% hold off
% xlim([0, 100]);
% ylim([-3, 3]);
% title('Right knee')
% 
% i = 9;
% subplot(3, 1, 3)
% joint_n     = i;
% body_n      = 1;
% variable_n  = 3;
% variable    = joints(9:166,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
% variable = [variable(83:end); variable(1:82)];
% hold on
% plot(100*t/max(t),  variable/mass)
% plot([0 100],[0 0], 'k')
% hold off
% xlim([0, 100]);
% ylim([-3, 3]);
% title('Right ankle')