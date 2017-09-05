function [yaw, pitch, roll] = toEulerianAngle(q)
%q is a qarternion q = [q0, q1, q2, q3]
ysqr = q(3) * q(3);
t0 = -2 * (ysqr + q(4) * q(4)) + 1;
t1 = 2 * (q(2) * q(3) + q(1) * q(4));
t2 = -2 * (q(2) * q(4) - q(1) * q(3));
t3 = 2 * (q(3) * q(4) + q(1) * q(2));
t4 = -2 * (q(2) * q(2) + ysqr) + 1;

if t2 > 1,
    t2 = 1;
end
if t2 < -1,
    t2 = -1;
end
% t2 = t2 > 1.0f ? 1.0f : t2;
% t2 = t2 < -1.0f ? -1.0f : t2;
pitch = asin(t2);
roll = atan2(t3, t4);
yaw = atan2(t1, t0);