function [u, flag] = ControlAllocation(tau_x, tau_y, tau_z) %#codegen   
coder.extrinsic('linprog');

f_max = 0.5;
motor_num = 8;
tau = [tau_x; tau_y; tau_z];
M = [0.0624   -0.2331    0.0781    0.0926    0.1192   -0.0213   -0.1337    0.0358 0;
    -0.2331    0.0926    0.0624    0.0781   -0.0213    0.0358    0.1192   -0.1337 0;
    0.1706    0.0411   -0.1139   -0.0979   -0.1706   -0.0411    0.1139    0.0979 0];

f  = [zeros(1,motor_num), 1];
A = [-eye(motor_num), zeros(motor_num,1);
    eye(motor_num), -f_max*ones(motor_num,1);
    zeros(1,motor_num), -1];
b = zeros(2*motor_num+1, 1);

[fr, ~, flag, ~] = linprog(f,A,b,M,tau,[],[]);
u = fr(1:motor_num);
end