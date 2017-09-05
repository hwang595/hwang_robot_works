load qDatainRawData3
load oriEulerAngle
q = qDatainRawData3;
Yaw = []; roll = []; pitch = [];
oriYaw = oriEulerAngle(:,1);
oriPitch = oriEulerAngle(:, 2);
oriRoll = oriEulerAngle(:, 3);
for i = 1 : size(q, 1),
   [yaw, pitch, roll] = toEulerianAngle(q(i, :));
   yawDeg = rad2deg(yaw);
   pitchDeg = rad2deg(pitch);
   rollDeg = rad2deg(roll);
   Yaw(i) = yawDeg;
   Pitch(i) = pitchDeg;
   Roll(i) = rollDeg;
end

EulAngle = [Yaw' Roll' Pitch'];
p = 1:1:3009;
subplot(1, 3, 1); plot(p, Yaw, '-k'); hold on; plot(p, oriYaw, ':r'); axis([0, 3009, -180, 180]); xlabel('Yaw'); ylabel('Degree'); 
subplot(1, 3, 2); plot(p, Pitch, '-k'); hold on; plot(p, oriPitch, ':r'); axis([0, 3009, -90, 90]);xlabel('Pitch'); ylabel('Degree');
subplot(1, 3, 3); plot(p, Roll, '-k'); hold on; plot(p, oriRoll, ':r'); axis([0, 3009, -180, 180]);xlabel('Roll'); ylabel('Degree');