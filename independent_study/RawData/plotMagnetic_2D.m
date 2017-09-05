clear all;
close all;

lambda = 0.05;
load magneticData2
x = magneticData2(:,1);
m = size(magneticData2, 1);
meanx = mean(x);
sigmax = var(x);
y = magneticData2(:,2);
meany = mean(x);
sigmay = var(x);
z = magneticData2(:,3);
meanz = mean(z);
sigmaz = var(z);
xNor = (x - meanx) / sigmax;
yNor = (y - meany) / sigmay;
zNor = (z - meanz) / sigmaz;
%plot3(x, y, z, 'k.');
plot(xNor, yNor, 'k.');
hold on;
alpha = [0; 0];
A = [];
A = ones(m, 1);
A(:, 2) = xNor .^ 2;
b = yNor .^ 2;
alpha = pinv(A' * A + lambda * eye(size(A' * A, 1))) * A' * b;