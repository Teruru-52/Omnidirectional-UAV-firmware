clear all
close all
%% method1
step = 0.01;
alpha = 0:step:4*pi;

theta = -pi / 4;
phi = atan2(1, sqrt(2));

q0_des = zeros(length(alpha), 1);
q1_des = zeros(length(alpha), 1);
q2_des = zeros(length(alpha), 1);
q3_des = zeros(length(alpha), 1);
q_des = zeros(4, length(alpha));

q0_des(1) = cos(phi / 2) * cos(theta / 2);
q1_des(1) = sin(phi / 2) * cos(theta / 2);
q2_des(1) = cos(phi / 2) * sin(theta / 2);
q3_des(1) = -sin(phi / 2) * sin(theta / 2);
q_des(:, 1) = [q0_des(1); q1_des(1); q2_des(1); q3_des(1)];

for i = 2:length(alpha)
    q0_rot = cos(alpha(i) / 2);
%     q1_rot = (1 / sqrt(3)) * sin(alpha(i) / 2);
%     q2_rot = (1 / sqrt(3)) * sin(alpha(i) / 2); 
%     q3_rot = (1 / sqrt(3)) * sin(alpha(i) / 2);
    q1_rot = 0;
    q2_rot = 0; 
    q3_rot = sin(alpha(i) / 2);

%     Norm = norm([q0_rot, q1_rot, q2_rot, q3_rot]);
%     q0_rot = q0_rot / Norm;
%     q1_rot = q1_rot / Norm;
%     q2_rot = q2_rot / Norm;
%     q3_rot = q3_rot / Norm;

    q_des(:, i) = [q0_rot, -q1_rot, -q2_rot, -q3_rot;
                      q1_rot, q0_rot, -q3_rot, q2_rot;
                      q2_rot, q3_rot, q0_rot, -q1_rot;
                      q3_rot, -q2_rot, q1_rot, q0_rot] * q_des(:, 1);

%     q_rot = [q0_rot; q1_rot; q2_rot; q3_rot];
%     q_des(:, i) = [q_des(1, 1), -q_des(2, 1), -q_des(3, 1), -q_des(4, 1);
%                       q_des(2, 1), q_des(1, 1), -q_des(4, 1), q_des(3, 1);
%                       q_des(3, 1), q_des(4, 1), q_des(1, 1), -q_des(2, 1);
%                       q_des(4, 1), -q_des(3, 1), q_des(2, 1), q_des(1, 1)] * q_rot;

%     Norm = norm(q_des(:, i));
%     q_des(:, i) = q_des(:, i) / Norm;
    q0_des(i) = q_des(1, i);
    q1_des(i) = q_des(2, i);
    q2_des(i) = q_des(3, i);
    q3_des(i) = q_des(4, i);

end

roll = zeros(length(q0_des), 1);
pitch = zeros(length(q0_des), 1);
yaw = zeros(length(q0_des), 1);

% calclate Euler angle
for i = 1:length(q0_des)
    if (cos(theta) == 0)
        roll(i) = 0;
        yaw(i) = atan2(-2 * (q1_des(i)*q2_des(i) - q0_des(i)*q3_des(i)), (2*(q0_des(i)^2 + q2_des(i)^2)-1));
    else
         roll(i) = atan2(2 * (q2_des(i)*q3_des(i) + q0_des(i)*q1_des(i)), (2*(q0_des(i)^2 + q3_des(i)^2)-1));
        yaw(i) = atan2(2 * (q1_des(i)*q2_des(i) + q0_des(i)*q3_des(i)), (2*(q0_des(i)^2 + q1_des(i)^2)-1));
    end
    pitch(i) = asin(-2 * (q1_des(i)*q3_des(i) - q0_des(i)*q2_des(i)));
end

figure(1);
subplot(1, 2, 1);
plot(q0_des, 'LineWidth', 2);
hold on;
plot(q1_des, 'LineWidth', 2);
plot(q2_des, 'LineWidth', 2);
plot(q3_des, 'LineWidth', 2);
grid on;

legend('$q_0$', '$q_1$', '$q_2$', '$q_3$', 'Interpreter', 'latex', 'Location', 'best')
xlim([0 length(q0_des)])
h_axes = gca;
h_axes.XAxis.FontSize = 15;
h_axes.YAxis.FontSize = 15;
set(legend, 'FontSize', 20);

subplot(1, 2, 2);
plot(roll, 'LineWidth', 2);
hold on;
plot(pitch, 'LineWidth', 2);
plot(yaw, 'LineWidth', 2);
grid on;

legend('$\phi$', '$\theta$', '$\psi$', 'Interpreter', 'latex', 'Location', 'best')
xlim([0 length(q0_des)])
h_axes = gca;
h_axes.XAxis.FontSize = 15;
h_axes.YAxis.FontSize = 15;
set(legend, 'FontSize', 20);
%% method2
step = 0.01;

psi = 0:step:4*pi;
theta = -pi / 4;
phi = atan2(1, sqrt(2));

q0_des = zeros(length(psi), 1);
q1_des = zeros(length(psi), 1);
q2_des = zeros(length(psi), 1);
q3_des = zeros(length(psi), 1);

for i = 1:length(psi)
    q0_des(i) = sin(phi / 2) * sin(theta / 2) * sin(psi(i) / 2) + cos(phi / 2) * cos(theta / 2) * cos(psi(i) / 2);
    q1_des(i) = sin(phi / 2) * cos(theta / 2) * cos(psi(i) / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi(i) / 2);
    q2_des(i) = sin(phi / 2) * cos(theta / 2) * sin(psi(i) / 2) + cos(phi / 2) * sin(theta / 2) * cos(psi(i) / 2);
    q3_des(i) = -sin(phi / 2) * sin(theta / 2) * cos(psi(i) / 2) + cos(phi / 2) * cos(theta / 2) * sin(psi(i) / 2);

%     Norm = norm([q0_des(i), q1_des(i), q2_des(i), q3_des(i)]);
%     q0_des(i) = q0_des(i) / Norm;
%     q1_des(i) = q1_des(i) / Norm;
%     q2_des(i) = q2_des(i) / Norm;
%     q3_des(i) = q3_des(i) / Norm;
end

q_des = [q0_des, q1_des, q2_des, q3_des];

roll = zeros(length(q0_des), 1);
pitch = zeros(length(q0_des), 1);
yaw = zeros(length(q0_des), 1);

% calclate Euler angle
for i = 1:length(psi)
    if (cos(theta) == 0)
        roll(i) = 0;
        yaw(i) = atan2(-2 * (q1_des(i)*q2_des(i) - q0_des(i)*q3_des(i)), (2*(q0_des(i)^2 + q2_des(i)^2)-1));
    else
         roll(i) = atan2(2 * (q2_des(i)*q3_des(i) + q0_des(i)*q1_des(i)), (2*(q0_des(i)^2 + q3_des(i)^2)-1));
        yaw(i) = atan2(2 * (q1_des(i)*q2_des(i) + q0_des(i)*q3_des(i)), (2*(q0_des(i)^2 + q1_des(i)^2)-1));
    end
    pitch(i) = asin(-2 * (q1_des(i)*q3_des(i) - q0_des(i)*q2_des(i)));
end

figure(2);
subplot(1, 2, 1);
plot(q0_des, 'LineWidth', 2);
hold on;
plot(q1_des, 'LineWidth', 2);
plot(q2_des, 'LineWidth', 2);
plot(q3_des, 'LineWidth', 2);
grid on;

legend('$q_0$', '$q_1$', '$q_2$', '$q_3$', 'Interpreter', 'latex', 'Location', 'best')
xlim([0 length(q0_des)])
h_axes = gca;
h_axes.XAxis.FontSize = 15;
h_axes.YAxis.FontSize = 15;
set(legend, 'FontSize', 20);

subplot(1, 2, 2);
plot(roll, 'LineWidth', 2);
hold on;
plot(pitch, 'LineWidth', 2);
plot(yaw, 'LineWidth', 2);
grid on;

legend('$\phi$', '$\theta$', '$\psi$', 'Interpreter', 'latex', 'Location', 'best')
xlim([0 length(q0_des)])
h_axes = gca;
h_axes.XAxis.FontSize = 15;
h_axes.YAxis.FontSize = 15;
set(legend, 'FontSize', 20);