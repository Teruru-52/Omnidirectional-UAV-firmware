clear al
close all

rpm_0_3V = 2040;
rpm_0_5V = 5120;
rpm_0_75V = 7220;
rpm_1V = 9250;
rpm_1_25V = 10720;
rpm_1_5V = 12050;
rpm_1_75V = 13500;
rpm_2V = 14480;
rpm_2_25V = 14900;

input = [0.3; 0.5; 0.75; 1.0; 1.25; 1.5; 1.75; 2.0; 2.25];
rpm = [rpm_0_3V; rpm_0_5V; rpm_0_75V; rpm_1V; rpm_1_25V; rpm_1_5V; rpm_1_75V; rpm_2V; rpm_2_25V];
rps = rpm / 60

%% Plot thrust and PPM (ESC signal)
p = polyfit(rps, input, 4)
[min_rps, max_rps] = bounds(rps);
RPS = min_rps:0.001:max_rps;
% 4次近似
% y = p(1) * RPS.^4 + p(2) * RPS.^3 + p(3) * RPS.^2 + p(4) * RPS + p(5);
y = p(1) * RPS.^4 + p(2) * RPS.^3 + p(3) * RPS.^2 + p(4) * RPS + p(5);

figure(1);
plot(rps, input,  'ro', 'MarkerSize', 8);
hold on;
grid on;
plot(RPS, y, 'b-', 'LineWidth', 2);
hold off;

h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;
ylabel('voltage [$V$]', 'Interpreter','latex');
xlabel('velocity [rps]', 'Interpreter','latex');
% ylim([1000 2000]);
legend('Experiment data', 'Fitting (4th order)', 'Interpreter','latex', 'Location','southeast');
set(legend, 'FontSize', 30);