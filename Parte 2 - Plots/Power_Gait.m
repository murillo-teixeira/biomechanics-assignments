clear;clc;
biomec_data = table2array(readtable("BiomechanicalModel.out", 'FileType','text'));
joints = table2array(readtable("Joints.xlsx", 'FileType','spreadsheet'));
times = biomec_data(15:end-15, 1) - biomec_data(15, 1);
times = 100*times/max(times);
m = 58.9;

sgtitle('Power of joints during gait', 'FontName', 'Times', 'FontSize', 11)
subplot(3, 2, 1)
title('Winter, 1987', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Hip power','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)
data = table2array(readtable("Power_Hip_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-1, 4]);

subplot(3, 2, 3)
data = table2array(readtable("Power_Knee_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-5, 1]);
ylabel({'Knee power','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

subplot(3, 2, 5)
data = table2array(readtable("Power_Ankle_Gait.csv", 'Delimiter',';', 'DecimalSeparator',','));
hold on; plot(data(:,1), data(:,2));plot([0 100],[0 0], 'k'); hold off; 
xlim([0, 100]); ylim([-1, 4]);
ylabel({'Ankle power','/BW (N.m/kg)'},'FontName', 'Times', 'FontSize', 10)

xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
subplot(3, 2, 2)
b1_idx = 1; b2_idx = 7;
b1_Od = biomec_data(15:end-15, 9*(b1_idx-1)+1+6);
b2_Od = biomec_data(15:end-15, 9*(b2_idx-1)+1+6);
w = b1_Od - b2_Od;
title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
i = 4;
joint_n     = i;
body_n      = 2;
variable_n  = 3;
hold on;
power    = joints(15:end-15,79+(joint_n-1)*6+(body_n-1)*3+(variable_n)).*w/m;
plot(times, power)
plot([times(1) times(end)], [0 0], 'k')
ylim([-1, 4]);
hold on;

subplot(3, 2, 4)
b1_idx = 7; b2_idx = 8;
b1_Od = biomec_data(15:end-15, 9*(b1_idx-1)+1+6);
b2_Od = biomec_data(15:end-15, 9*(b2_idx-1)+1+6);
w = b1_Od - b2_Od;

i = 8;
joint_n     = i;
body_n      = 1;
variable_n  = 3;
hold on;
power    = joints(15:end-15,79+(joint_n-1)*6+(body_n-1)*3+(variable_n)).*w/m;
plot(times, power)
plot([times(1) times(end)], [0 0], 'k')
ylim([-5, 1]);
hold on;

subplot(3, 2, 6)
b1_idx = 8; b2_idx = 9;
b1_Od = biomec_data(15:end-15, 9*(b1_idx-1)+1+6);
b2_Od = biomec_data(15:end-15, 9*(b2_idx-1)+1+6);
w = b1_Od - b2_Od;
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
i = 9;
joint_n     = i;
body_n      = 1;
variable_n  = 3;
hold on;
power    = joints(15:end-15,79+(joint_n-1)*6+(body_n-1)*3+(variable_n)).*w/m;
plot(times, power)
plot([times(1) times(end)], [0 0], 'k')
ylim([-1, 4]);
hold on;