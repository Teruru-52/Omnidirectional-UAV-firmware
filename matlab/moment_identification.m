clear all

omega = csvread('expdata.csv');
omega = omega';

dt = 0.02;
t = 0:dt:length(omega)*dt-dt;
plot(t, omega, 'LineWidth', 2);
grid on;

%%
m = 0.1482; % [kg]
g = 9.81; % [kg・(m / s^2)]
h = 0.383; % [m]
D = 0.15 * sqrt(2); % [m] 
T = 0.18; % [s]

J = (m * g * D^2 * T^2) / (16 * h * pi^2)

% 平行軸の定理
Jo = J + m * (D / 2)^2