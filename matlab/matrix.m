clear all

% postion matrix
r = 0.063;
% around the center of gravity
P = r / sqrt(3) * [1 -1 1 -1 1 -1 1 -1;
                           1 1 -1 -1 1 1 -1 -1;
                           1 1 1 1 -1 -1 -1 -1]

% around the center of rotation
L = 0.155;
ro = L  / 2 * [1; 1; 1];
P = ro + P
 
% orientation matrix
a = 1 / 2 + 1 / sqrt(12);
b = 1 / 2 - 1 / sqrt(12);
c = 1 / sqrt(3);

% default
X = [-a b -b a a -b b -a;
        b a -a -b -b -a a b;
        c -c -c c c -c -c c]

% reverse 1-4 directions
% X = [a -b b -a a -b b -a;
%         -b -a a b -b -a a b;
%         -c c c -c c -c -c c];

% reverse 5-8 directions
% X = [-a b -b a -a b -b a;
%         b a -a -b b a -a -b;
%         c -c -c c -c c c -c]

% X = [a -b b -a;
%        -b -a a b;
%         c -c -c c];
% P = P(:, 5:8);

% control allocation matrix
% the torque induced by the aerodynamic drag of the propeller is omitted
% because it is typically an order of magnitude smaller.
M = cross(P, X)
pinvM = pinv(M)