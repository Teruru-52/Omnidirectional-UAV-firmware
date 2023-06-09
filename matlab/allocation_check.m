%% import data
clear all

data = csvread('allocation_data.csv');
tau_x = data(:,1)';
tau_y = data(:,2)';
tau_z = data(:,3)';
f1 = data(:,4)';
f2 = data(:,5)';
f3 = data(:,6)';
f4 = data(:,7)';
f5 = data(:,8)';
f6 = data(:,9)';
f7 = data(:,10)';
f8 = data(:,11)';
objv = data(:,12);

tau = [tau_x; tau_y; tau_z];
f = [f1; f2; f3; f4; f5; f6; f7; f8];

M = [
    0.0417   -0.1556    0.0661    0.0478    0.0744   -0.0333   -0.0562    0.0151
   -0.1556    0.0478    0.0417    0.0661   -0.0333    0.0151    0.0744   -0.0562
    0.1139    0.0084   -0.0811   -0.0411   -0.1139   -0.0084    0.0811    0.0411];

error = tau - M*f;

error_norm = zeros(length(error), 1);
for i = 1:length(error)
    error_norm(i, 1) = norm(error(:, i));
end

objv - error_norm;