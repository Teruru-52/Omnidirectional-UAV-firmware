clear all

% postion matrix
P = sqrt(1 / 3) * [1 -1 1 -1 1 -1 1 -1;
                           1 1 -1 -1 1 1 -1 -1;
                           1 1 1 1 -1 -1 -1 -1];

% orientation matrix
a = 1 / 2 + 1 / sqrt(12);
b = 1 / 2 - 1 / sqrt(12);
c = 1 / sqrt(3);

X = [-a b -b a a -b b -a;
        b a -a -b -b -a a b;
        c -c -c c c -c -c c];

% rotation matrix
syms theta
v1 = [1; 0; 0];
v2 = P(:, 1);
% v2 = P(:, 2);
% v2 = P(:, 3);
% v2 = P(:, 4);
% v2 = P(:, 5);
% v2 = P(:, 6);
% v2 = P(:, 7);
% v2 = P(:, 8);
v3 = cross(v1, v2);
n = v3 / norm(v3);

mat_n = ...
    [0 -n(3) n(2);
    n(3) 0 -n(1);
    -n(2) n(1) 0];
R = cos(theta) * eye(3) + (1 - cos(theta)) * (n * n') + sin(theta) * mat_n
% R * v1
theta = acos(v2(1));
subs(v2 - R * v1);

angle =  acos(v2(1));
R = cos(angle) * eye(3) + (1 - cos(angle)) * (n * n') + sin(angle) * mat_n
% R * v1

% rotation matrix from Euler angle
syms yaw pitch roll
ca = cos(roll);  sa = sin(roll);
cb = cos(pitch); sb = sin(pitch);
cc = cos(yaw);   sc = sin(yaw);

Rz = ...
    [cc, sc, 0;
    -sc, cc, 0
    0, 0, 1];
Ry = ...
    [cb, 0, -sb;
    0, 1, 0;
    sb, 0, cb];
Rx = ...
    [1, 0, 0;
    0, ca, sa;
    0, -sa, ca];
% R = Rz*Ry*Rx;

% motor orientation
syms alpha
v5 = [0; cos(alpha); sin(alpha)];
% eqn = R * v4 - X(:,1) == 0
ori = R * v5;
eqn = ori(1) + a == 0;
solve(eqn, alpha)
eqn = ori(2) - b == 0;
solve(eqn, alpha)
eqn = ori(3) - c == 0;
solve(eqn, alpha)

% alpha1 = 60; % degree
alpha1 = 1.0472;
alpha = alpha1;
v5 = [0; cos(alpha); sin(alpha)];
R = cos(angle) * eye(3) + (1 - cos(angle)) * (n * n') + sin(angle) * mat_n
R * v5
