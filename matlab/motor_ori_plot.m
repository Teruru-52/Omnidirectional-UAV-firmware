close all
clear all

% postion matrix
r = 6; % [cm]
P = r / sqrt(3) * [1 -1 1 -1 1 -1 1 -1;
                           1 1 -1 -1 1 1 -1 -1;
                           1 1 1 1 -1 -1 -1 -1];

% orientation matrix
a = 1 / 2 + 1 / sqrt(12);
b = 1 / 2 - 1 / sqrt(12);
c = 1 / sqrt(3);

X = [-a b -b a a -b b -a;
        b a -a -b -b -a a b;
        c -c -c c c -c -c c];

figure(1);
for i = 1:8
    quiver3(P(1,i), P(2,i), P(3,i), X(1, i), X(2, i), X(3,i), 0, 'LineWidth', 5);
    hold on
end
axis equal
legend("motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7", "motor8");

% corner position matrix
l = 15 / 2; % [cm]
C = l * [1 -1 1 -1 1 -1 1 -1;
            1 1 -1 -1 1 1 -1 -1;
            1 1 1 1 -1 -1 -1 -1];
for i = 1:8
    quiver3(0, 0, 0, C(1,i), C(2,i), C(3,i),'k')
    hold on
end
