clear all
close all

dt = 0.01;
coeff_m = 0.1;
coeff_g = 9.81;
coeff_l = 0.155 / 2;
coeff_J = 0.0017;
%% generalized model
syms phi theta psi
syms omega_x omega_y omega_z
syms x [6 1] matrix
syms m l g J

% Rx = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
% Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
% Rz = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

R = Rx*Ry*Rz;

l_g = l * [1; 1; 1];
% m*R*[0; 0; -g]
tau_g = cross(l_g, (m*R*[0; 0; -g])) / J;

R_euler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);   0 cos(phi) -sin(phi);   0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
d_euler = R_euler * [omega_x; omega_y; omega_z];
dphi = d_euler(1);
dtheta = d_euler(2);
dpsi = d_euler(3);

dfdx = [diff(dphi, phi) diff(dphi, theta) diff(dphi, psi) diff(dphi, omega_x) diff(dphi, omega_y) diff(dphi, omega_z);
            diff(dtheta, phi) diff(dtheta, theta) diff(dtheta, psi) diff(dtheta, omega_x) diff(dtheta, omega_y) diff(dtheta, omega_z);
            diff(dpsi, phi) diff(dpsi, theta) diff(dpsi, psi) diff(dpsi, omega_x) diff(dpsi, omega_y) diff(dpsi, omega_z);
            diff(tau_g(1), phi) diff(tau_g(1), theta) diff(tau_g(1), psi) diff(tau_g(1), omega_x) diff(tau_g(1), omega_y) diff(tau_g(1), omega_z);
            diff(tau_g(2), phi) diff(tau_g(2), theta) diff(tau_g(2), psi) diff(tau_g(2), omega_x) diff(tau_g(2), omega_y) diff(tau_g(2), omega_z);
            diff(tau_g(3), phi) diff(tau_g(3), theta) diff(tau_g(3), psi) diff(tau_g(3), omega_x) diff(tau_g(3), omega_y) diff(tau_g(3), omega_z)]

% x = [phi; theta; psi; omega_x; omega_y; omega_z];
% f = [d_euler; tau_g];
% dfdx = diff(f, x)

% edge
% xd = [0; pi / 4; 0; 0; 0; 0];
% vertex
xd = [atan(1/sqrt(2)); pi / 4; 0; 0; 0; 0];

% linearized system matrix and input matrix
A = subs(dfdx, [phi, theta, psi, omega_x, omega_y, omega_z], [xd(1) xd(2) xd(3) xd(4) xd(5) xd(6)]);
B = 1 / coeff_J * [0, 0, 0; 0, 0, 0; 0, 0, 0; 1, 0, 0; 0, 1, 0; 0, 0, 1];
%% simulation (vertex, with yaw)
% control design
A = subs(A, [m, g, l, J], [coeff_m, coeff_g, coeff_l, coeff_J])
B = 1 / coeff_J * [0, 0, 0; 0, 0, 0; 0, 0, 0; 1, 0, 0; 0, 1, 0; 0, 0, 1];
V = ctrb(A, B);
rank(V)

% vertex
Q = diag([10, 10, 10, 10, 10, 10]);
R = diag([1; 1; 1]);

A = [                                                                                                                        0,                                                                                                                         0, 0, 1,         sin(5543748373244115/9007199254740992),         cos(5543748373244115/9007199254740992);
    0,                                                                                                                         0, 0, 0,         cos(5543748373244115/9007199254740992),        -sin(5543748373244115/9007199254740992);
    0,                                                                                                                         0, 0, 0, 2^(1/2)*sin(5543748373244115/9007199254740992), 2^(1/2)*cos(5543748373244115/9007199254740992);
    (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360 - (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360, (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360 + (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360, 0, 0,                                              0,                                              0;
    -(30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360,                                      - (30411*2^(1/2))/1360 - (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360, 0, 0,                                              0,                                              0;
    (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360,                                        (30411*2^(1/2))/1360 - (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360, 0, 0,                                              0,                                              0]
[F, P, E] = lqrd(A, B, Q, R, dt);
% [F, P, E] = lqr(A, B, Q, R);
F = -F

% simulation
tspan = [0.0:0.001:10];
x0 = [0.2; 0.3;  0.1; 0; 0; 0];
[t, x] = ode45(@(t, x) (A+B*F)*x, tspan, x0);

figure(1);
subplot(1, 2, 1);
plot(t, x(:, 1), "LineWidth", 2);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
hold on
plot(t, x(:, 2), "LineWidth", 2);
plot(t, x(:, 3), "LineWidth", 2);
ylabel("Euler angle [rad]", 'Interpreter', 'latex');
legend("$\phi$", "$\theta$", "$\psi$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 2, 2);
plot(t, x(:, 4), "LineWidth", 2);
grid on
hold on
plot(t, x(:, 5), "LineWidth", 2);
plot(t, x(:, 6), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("Angular velocity [rad/s]", 'Interpreter', 'latex');
legend("$\omega_x$", "$\omega_y$", "$\omega_z$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

%% simulation (vertex, without yaw)
Q = diag([250, 250, 10, 10, 10]);
R = diag([1; 1; 1]);

A = [                                                                                                                        0,                                                                                                                         0, 1,         sin(5543748373244115/9007199254740992),         cos(5543748373244115/9007199254740992);
    0,                                                                                                                         0, 0,         cos(5543748373244115/9007199254740992),        -sin(5543748373244115/9007199254740992);
    (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360 - (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360, (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360 + (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360, 0,                                              0,                                              0;
    -(30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360,                                      - (30411*2^(1/2))/1360 - (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360, 0,                                              0,                                              0;
    (30411*2^(1/2)*cos(5543748373244115/9007199254740992))/1360,                                        (30411*2^(1/2))/1360 - (30411*2^(1/2)*sin(5543748373244115/9007199254740992))/1360, 0,                                              0,                                              0]
B = 1 / coeff_J * [0, 0, 0; 0, 0, 0; 1, 0, 0; 0, 1, 0; 0, 0, 1];
V = ctrb(A, B);
rank(V)
[F, P, E] = lqrd(A, B, Q, R, dt);
% [F, P, E] = lqr(A, B, Q, R);
F = -F

% simulation
tspan = [0.0:0.001:10];
x0 = [0.2; 0.3; 0; 0; 0];
[t, x] = ode45(@(t, x) (A+B*F)*x, tspan, x0);

figure(1);
subplot(1, 2, 1);
plot(t, x(:, 1), "LineWidth", 2);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
hold on
plot(t, x(:, 2), "LineWidth", 2);
ylabel("$\phi$ [rad]", 'Interpreter', 'latex');
legend("$\phi$", "$\theta$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 2, 2);
plot(t, x(:, 3), "LineWidth", 2);
grid on
hold on
plot(t, x(:, 4), "LineWidth", 2);
plot(t, x(:, 5), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\omega_x$ [rad/s]", 'Interpreter', 'latex');
legend("$\omega_x$", "$\omega_y$", "$\omega_z$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

%% edge inverted model
syms theta
syms omega_y
syms m l g J

% Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

dtheta = omega_y;
l_g = l * [1; 1; 1];
% m*Ry*[0; 0; -g]
tau_g = cross(l_g, (m*Ry*[0; 0; -g])) / J;

dfdx = [diff(dtheta, theta) diff(dtheta, omega_y);
            diff(tau_g(2), theta) diff(tau_g(2), omega_y)]

xd = [pi / 4; 0];
% linearized system matrix and input matrix
A = subs(dfdx, [theta, omega_y], [xd(1) xd(2)])
B = 1 / coeff_J * [0; 1];


%% simulation (edge)
% control design
A = subs(A, [m, g, l, J], [coeff_m, coeff_g, coeff_l, coeff_J])
V = ctrb(A, B);
rank(V)

Q = diag([250, 2.5]);
R = [10];
A = [                   0, 1;
       -sqrt(2)*coeff_m*coeff_g*coeff_l / coeff_J, 0]
B = 1 / coeff_J * [0; 1];
[F, P, E] = lqrd(A, B, Q, R, dt);
% [F, P, E] = lqr(A, B, Q, R);
F = -F

% simulation
tspan = [0.0:0.001:10];
x0 = [0.1; 0];
[t, x] = ode45(@(t, x) (A+B*F)*x, tspan, x0);

figure(2);
subplot(1, 2, 1);
plot(t, x(:, 1), "LineWidth", 3);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\theta$ [rad]", 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 2, 2);
plot(t, x(:, 2), "LineWidth", 3);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\omega_y$ [rad/s]", 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);