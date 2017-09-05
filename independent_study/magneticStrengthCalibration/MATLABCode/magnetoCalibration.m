function Y = magnetoCalibration(X)
%this funtion is for calibration of input magnetic strengh data collected
%by MPU9250
%In this function, the X should be a scaled dataset with unit of G
%In general X can be represented by [mx, my, mz] scaled
%This code is used for CS 799 robotic project
%Author: Hongyi Wang
%Created on 10/3/2016
[center, radii, evecs, v, chi2] = ellipsoid_fit([X(:, 1) X(:,2) X(:, 3)]);
a = radii(1); b = radii(2); c = radii(3);
dataMatrix = [X(:, 1) X(:,2) X(:, 3)];
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
%[center1, radii1, evecs1, v1, chi21] = ellipsoid_fit([calibratedData(:, 1), calibratedData(:, 2), calibratedData(:, 3)]);
Y = calibratedData;