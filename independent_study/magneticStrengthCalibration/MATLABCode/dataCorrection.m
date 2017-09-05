load qDatainRawData3
load magneticData3
load mref
load qref
load oriEulerAngle
%%
%==========================================================================
oriYaw = oriEulerAngle(:,1);
oriPitch = oriEulerAngle(:, 2);
oriRoll = oriEulerAngle(:, 3);
%%Calculate reference value of magnatitude strength
%=========================================================================
magrefTmp = mref(:, 1:3);
qartrefTmp = qref(:, 1:4);
magref = mean(magrefTmp);
magref = [0 magref];
qartref = mean(qartrefTmp);
qartrefConj = quatconj(qartref);
refTmp1 = quatmultiply(qartrefConj, magref);
magRefIden = quatmultiply(refTmp1, qartref);
%%Calculate calibrated value of magnatitude strength
%=========================================================================
X = magneticData3;
X = X ./ 1000;
Y = magnetoCalibration(X); %calibrated values of magnetic strength
q = qDatainRawData3;
Y = [zeros(size(Y, 1), 1) Y];
qConj = [];
for i = 1 : size(q, 1),
    qConj(i, :) = quatconj(q(i, :));
end
yIden = [];
for j = 1 : size(q, 1),
   tmp1 = quatmultiply(qConj(j, :), Y(j, :));
   tmp2 = quatmultiply(tmp1, q(j, :));
   yIden(j, :) = tmp2; %identical orientation of magnetic strength
end
%%Implement the Yaw correction
%==========================================================================
alpha = 0.1;
Yaw = [];
Roll = [];
Pitch = [];
corrQ = [];
thetar = atan2(magRefIden(2), magRefIden(4));
for k = 1 : size(yIden, 1),
    mIden = yIden(k, :);
    theta = atan2(mIden(2), mIden(4));
    corrQ(k, :) = quatmultiply(quaternion([0, 1, 0], -alpha * (theta - thetar)), q(k, :));
end
%%Calculate Eulerian Deg using corrected qarternion
for l = 1 : size(corrQ, 1),
    [yaw, pitch, roll] = toEulerianAngle(corrQ(l, :));
    yawDeg = rad2deg(yaw);
    pitchDeg = rad2deg(pitch);
    rollDeg = rad2deg(roll);
    Yaw(l) = yawDeg;
    Pitch(l) = pitchDeg;
    Roll(l) = rollDeg;
end
CorrectedEulAngle = [Yaw' Pitch' Roll'];
p = 1:1:3009;
subplot(1, 3, 1); plot(p, Yaw, '-k'); hold on; plot(p, oriYaw, ':r'); axis([0, 3009, -180, 180]); xlabel('Yaw'); ylabel('Degree');
subplot(1, 3, 2); plot(p, Pitch, '-k'); hold on; plot(p, oriPitch, ':r'); axis([0, 3009, -90, 90]);xlabel('Pitch'); ylabel('Degree');
subplot(1, 3, 3); plot(p, Roll, '-k'); hold on; plot(p, oriRoll, ':r'); axis([0, 3009, -180, 180]);xlabel('Roll'); ylabel('Degree');
le1 = legend('Corrected Value', 'Raw Value'); set(le1,...
    'Position',[0.00458742080967715 0.00839147008891696 0.13896987114402 0.0654044734015929]);
set(le1,'Units','Normalized','FontUnits','Normalized');