function dq = carKinematics(q, v, delta, vehicle)

theta = q(3);
L = vehicle.wheelbase;

dx = v * cos(theta);
dy = v * sin(theta);
dtheta = (v / L) * tan(delta);

dq = [dx; dy; dtheta];

end