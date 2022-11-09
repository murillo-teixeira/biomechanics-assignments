clear;clc;

m = 62.5;
reaction_forces_file = table2array(readtable("VarForceAppl_002G.txt", 'Delimiter','\t'));
times = (reaction_forces_file(9:end, 1) - 0.05);
times = 100*times/max(times);
x_reactions = reaction_forces_file(9:end, 2)/m;
y_reactions = reaction_forces_file(9:end, 3)/m;

sgtitle('Ground reaction forces during gait', 'FontName', 'Times', 'FontSize', 11)

subplot(2, 2, 2)
plot(times, x_reactions)

xlim([0 100])
ylim([-3 3])

title('Data from LBL', 'FontName', 'Times', 'FontSize', 10)
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)

subplot(2, 2, 1)
gr_h_fast = table2array(readtable("GR_FastCadence_X.csv", 'Delimiter',';', 'DecimalSeparator',','));
gr_h_nat = table2array(readtable("GR_NaturalCadence_X.csv", 'Delimiter',';', 'DecimalSeparator',','));
gr_h_slow = table2array(readtable("GR_SlowCadence_X.csv", 'Delimiter',';', 'DecimalSeparator',','));
plot(gr_h_fast(:, 1), gr_h_fast(:, 2), gr_h_nat(:, 1), gr_h_nat(:, 2),  gr_h_slow(:, 1), gr_h_slow(:, 2))
ylim([-3 3])
title('Winter, 1987', 'FontName', 'Times', 'FontSize', 10)
ylabel({'Horizontal force','/body weight (N/kg)'},'FontName', 'Times', 'FontSize', 10)
leg = legend('Fast cad.', 'Natural cad.', 'Slow cad.', 'Location', 'Best');
leg.ItemTokenSize = [10, 20];
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
subplot(2, 2, 4)
plot(times, y_reactions)

ylim([0 14])
xlim([0 100])
xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)

subplot(2, 2, 3)
gr_h_fast = table2array(readtable("GR_FastCadence_Y.csv", 'Delimiter',';', 'DecimalSeparator',','));
gr_h_nat = table2array(readtable("GR_NaturalCadence_Y.csv", 'Delimiter',';', 'DecimalSeparator',','));
gr_h_slow = table2array(readtable("GR_SlowCadence_Y.csv", 'Delimiter',';', 'DecimalSeparator',','));
plot(gr_h_fast(:, 1), gr_h_fast(:, 2), gr_h_nat(:, 1), gr_h_nat(:, 2),  gr_h_slow(:, 1), gr_h_slow(:, 2))

xlabel('% of stride', 'FontName', 'Times', 'FontSize', 10)
ylim([0 14])
ylabel({'Vertical force','/body weight (N/kg)'},'FontName', 'Times', 'FontSize', 10)
leg = legend('Fast cad.', 'Natural cad.', 'Slow cad.', 'Location', 'Best');
leg.ItemTokenSize = [10, 20];
