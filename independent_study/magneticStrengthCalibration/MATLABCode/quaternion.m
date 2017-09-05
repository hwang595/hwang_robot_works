function q = quaternion(v, theta)
%this function returns a quaternion
%v is a vector, contains (vx, vy, vz), in which all of them are linearly
%velocity
q = zeros(1, 4);
q(1) = cos(theta / 2);
q(2) = v(1) * sin(theta / 2);
q(3) = v(2) * sin(theta / 2);
q(4) = v(3) * sin(theta / 2);
