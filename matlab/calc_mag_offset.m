%% import data
clear

data = csvread('magdata.csv');
mx = data(:,1);
my = data(:,2);
mz = data(:,3);

%% least square method
n = length(mx);
A = zeros(4, 4);
B = zeros(4, 1);

for i = 1:n
    A(1, 1) = A(1, 1) + mx(i);
    A(1, 2) = A(1, 2) + mx(i)^2;
    A(1, 3) = A(1, 3) + mx(i) * my(i);
    A(1, 4) = A(1, 4) + mx(i) * mz(i);
    A(2, 1) = A(2, 1) + my(i);
    A(2, 3) = A(2, 3) + my(i)^2;
    A(2, 4) = A(2, 4) + my(i) * mz(i);
    A(3, 1) = A(3, 1) + mz(i);
    A(3, 4) = A(3, 4) + mz(i)^2;

    B(1, 1) = B(1, 1) - mx(i) * (mx(i)^2 + my(i)^2 + mz(i)^2);
    B(2, 1) = B(2, 1) - my(i) * (mx(i)^2 + my(i)^2 + mz(i)^2);
    B(3, 1) = B(3, 1) - mz(i) * (mx(i)^2 + my(i)^2 + mz(i)^2);
    B(4, 1) = B(4, 1) - (mx(i)^2 + my(i)^2 + mz(i)^2);
end

A(2, 2) = A(1, 3);
A(3, 2) = A(1, 4);
A(3, 3) = A(2, 4);
A(4, 1) = n;
A(4, 2) = A(1, 1);
A(4, 3) = A(2, 1);
A(4, 4) = A(3, 1);

x = A\B;
mx_offset = - 0.5 * x(2)
my_offset = - 0.5 * x(3)
mz_offset = - 0.5 * x(4)
radius = sqrt(mx_offset^2 + my_offset^2 + mz_offset^2 - x(1)^2);

%% plot
figure(1);
subplot(1,2,1);
f = @(x,y,z) (x - mx_offset)^2 +  (y - my_offset)^2 + (z - mz_offset)^2 - radius^2;
plot3(mx, my, mz, '.', 'Color', 'r', 'MarkerSize', 10);
hold on;
grid on;
fimplicit3(f,'EdgeColor','none','FaceAlpha',.2);
pbaspect([1 1 1]);
xlabel('$m_X$','Interpreter','latex');
ylabel('$m_Y$','Interpreter','latex');
zlabel('$m_Z$','Interpreter','latex');
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;
h_axes.ZAxis.FontSize = 25;
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1,2,2);
plot3(mx - mx_offset, my - my_offset, mz - mz_offset, '.', 'Color', 'b', 'MarkerSize', 10);
grid on;
pbaspect([1 1 1]);
xlabel('$m_X$','Interpreter','latex');
ylabel('$m_Y$','Interpreter','latex');
zlabel('$m_Z$','Interpreter','latex');
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;
h_axes.ZAxis.FontSize = 25;
set(gca, "FontName", "Times New Roman", "FontSize", 15);