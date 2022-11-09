clear;clc;

joints = table2array(readtable("Joints.xlsx", 'FileType','spreadsheet'));
t=joints(:, 1);
mass = 58.9;
n=1;

sgtitle('Moments of force during gait', 'FontName', 'Times', 'FontSize', 11)
subplot(4, 2, 1)
title('Winter, 1987', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Support moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)
data = table2array(readtable("Moment_Support_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-2, 2]);

subplot(4, 2, 3)
data = table2array(readtable("Moment_Hip_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-2, 2]);
ylabel({'Hip moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

subplot(4, 2, 5)
data = table2array(readtable("Moment_Knee_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-2, 2]);
ylabel({'Knee moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)
subplot(4, 2, 7)
data = table2array(readtable("Moment_Ankle_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-2, 2]);
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Ankle moment','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

i = 4;
subplot(4, 2, 4)
joint_n     = i;
body_n      = 1;
variable_n  = 3;
Mhip    = joints(:,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
hold on
plot(100*t/max(t),  Mhip/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-2, 2]);

i = 8;
subplot(4, 2, 6)
joint_n     = i;
variable_n  = 3;
body_n = 2;
Mknee    = joints(:,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
hold on
plot(100*t/max(t),  Mknee/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-2, 2]);

i = 9;
subplot(4, 2, 8)
joint_n     = i;
body_n      = 1;
variable_n  = 3;
Mankle    = joints(:,79+(joint_n-1)*6+(body_n-1)*3+(variable_n));
hold on
plot(100*t/max(t),  Mankle/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-2, 2]);
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)

subplot(4, 2, 2)
title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
variable    = Mankle+Mhip+Mknee;
hold on
plot(100*t/max(t),  variable/mass)
plot([0 100],[0 0], 'k')
hold off
xlim([0, 100]);
ylim([-2, 2]);