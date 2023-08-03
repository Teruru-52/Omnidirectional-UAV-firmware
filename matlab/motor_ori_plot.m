close all
clear all

%% default
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

% adjust quiver length
norm = 2;
text_labels = {1, 2, 3, 4, 5, 6, 7, 8};
dist = 0.5;

for i = 1:8
    quiver3(P(1,i), P(2,i), P(3,i), X(1, i)*norm, X(2, i)*norm, X(3,i)*norm, 0, 'LineWidth', 2.5);
    hold on
    text(P(1,i)+dist, P(2,i)+dist, P(3,i)+dist, text_labels(i), "FontName","Times New Roman", 'FontSize', 20);
end
axis equal
legend("motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7", "motor8");

% corner position matrix
l = 15 / 2; % [cm]
C = l * [1 -1 1 -1 1 -1 1 -1;
            1 1 -1 -1 1 1 -1 -1;
            1 1 1 1 -1 -1 -1 -1];
for i = 1:8
    quiver3(0, 0, 0, C(1,i), C(2,i), C(3,i),'k', 'LineWidth', 1.5)
    hold on
end
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
zlabel('$z$ [m]', 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

%% inverted pendulum
matrix;

figure(2);

% adjust quiver length
norm = 0.02;
text_labels = {1, 2, 3, 4, 5, 6, 7, 8};

dist = 0.005;

for i = 1:8
    quiver3(P(1,i), P(2,i), P(3,i), X(1, i)*norm, X(2, i)*norm, X(3,i)*norm, 0, 'LineWidth', 2.5);
    hold on
    text(P(1,i)+dist, P(2,i)+dist, P(3,i)+dist, text_labels(i), "FontName","Times New Roman", 'FontSize', 20);
end
axis equal
legend("motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7", "motor8");

% corner position matrix
C = L * 0.5  *[1 -1 1 -1 1 -1 1 -1;
            1 1 -1 -1 1 1 -1 -1;
            1 1 1 1 -1 -1 -1 -1];
for i = 1:8
    quiver3(L*0.5, L*0.5, L*0.5, C(1,i), C(2,i), C(3,i),'k', 'LineWidth', 1.5)
    hold on
end
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
zlabel('$z$ [m]', 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);