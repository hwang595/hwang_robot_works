clear all;
close all;

load magneticData3
x = magneticData3(:,1);
meanx = mean(x);
sigmax = var(x);
y = magneticData3(:,2);
meany = mean(y);
sigmay = var(y);
z = magneticData3(:,3);
meanz = mean(z);
sigmaz = var(z);
xScaled = 0.001 * x; yScaled = 0.001 * y; zScaled = 0.001 *z;

plot3(xScaled, yScaled, zScaled, 'k.');
hold on;
[center, radii, evecs, v, chi2] = ellipsoid_fit([xScaled, yScaled, zScaled]);
a = radii(1); b = radii(2); c = radii(3);
[xE, yE, zE] = ellipsoid(center(1),center(2),center(3),a,b,c);
axis equal;
surf(xE, yE, zE);
alpha(0.5);
shading interp
plot3(center(1),center(2), center(3), '.r', 'MarkerSize', 10);
text(center(1),center(2), center(3), '(-0.6067, 0.1901, -0.5606)');
dataMatrix = [xScaled yScaled zScaled];
A = eye(3);
for i = 1:3,
    if i == 1,
        A(i, i) = a ^ 2;
    end
    if i == 2,
        A(i, i) = b ^ 2;
    end
    if i == 3,
        A(i, i) = c ^ 2;
    end
end
[~, S, V] = svd(A);
M = V * diag(1 ./ sqrt(diag(S))) * V';
for i = 1 : size(dataMatrix, 1),
    dataMatrix(i,:) = dataMatrix(i,:) - center';
end
calibratedData = (M * dataMatrix')';
[center1, radii1, evecs1, v1, chi21] = ellipsoid_fit([calibratedData(:, 1), calibratedData(:, 2), calibratedData(:, 3)]);
[xS, yS, zS] = ellipsoid(center1(1),center1(2),center1(3),radii1(1),radii1(2),radii1(3));
axis equal;
surf(xS, yS, zS);
shading interp
alpha(0.5);