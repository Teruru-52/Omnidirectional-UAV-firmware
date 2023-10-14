clear 
close all;

matrix;
N = 8;
% N = 4;

%% calculate vertex of torque set
input = [0; 1];
for i=2:N
    add = [0*ones(2^(i-1), 1); 1*ones(2^(i-1), 1)];
    input = [repmat(input, 2, 1), add];
end

fmax = 0.8;
% fmax = 0.6;
% fmax = 0.5;
% fmax = 1;
input = input * fmax

global vertex
vertex  = M*input';

%% plot

figure(1);
subplot(3, 4, [1,2,3,5,6,7,9,10,11])
plot_set(30);

subplot(3,4,4);
plot_set(20);
view(0,90);

subplot(3,4,8);
plot_set(20);
view(0,0);

subplot(3,4,12);
plot_set(20);
view(90,0);

function plot_set(fontsize)
global vertex
[k1,~] = convhull(vertex(1,:),vertex(2,:),vertex(3,:),'Simplify',true);
trisurf(k1,vertex(1,:),vertex(2,:),vertex(3,:),'FaceColor','cyan','FaceAlpha',0.1,'EdgeAlpha',0.4,'DisplayName','Torque set');
grid on;
daspect([1 1 1]);
xlim([-0.2 0.2]);
ylim([-0.2 0.2]);
zlim([-0.2 0.2]);
xlabel('$\tau_x$ [Nm]', 'Interpreter', 'latex');
ylabel('$\tau_y$ [Nm]', 'Interpreter', 'latex');
zlabel('$\tau_z$ [Nm]', 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", fontsize);
end