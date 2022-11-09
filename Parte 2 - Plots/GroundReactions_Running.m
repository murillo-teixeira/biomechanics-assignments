clear;clc;

m = 58.9;
reaction_forces_file = table2array(readtable("VarForceAppl_002.txt", 'Delimiter','\t'));
times = (reaction_forces_file(1:end, 1) - 0.05);
times = times;
x_reactions = reaction_forces_file(1:end, 2)/m;
y_reactions = reaction_forces_file(1:end, 3)/m;

sgtitle('Vertical ground reaction force during running gait', 'FontName', 'Times', 'FontSize', 11)

title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)

subplot(1, 2, 1)
grid minor;
gr_h_fast = table2array(readtable("GR_Running.csv", 'Delimiter',';', 'DecimalSeparator',','));
plot(gr_h_fast(:, 1), gr_h_fast(:, 2))

ylim([0 20])
title('Novacheck, 1998', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Vertical force','/body weight (N/kg)'},'FontName', 'Times', 'FontSize', 10)
xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 10)
subplot(1, 2, 2)
plot(times(90:end-90, :) - times(90, 1), y_reactions(90:end-90, :))

title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
ylim([0 20])
xlim([0 0.4])
xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 10)
