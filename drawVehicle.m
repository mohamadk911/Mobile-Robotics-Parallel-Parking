function bodyHandle = drawVehicle(pose, vehicle, delta, ax)
%drawVehicle Draw a rear-axle vehicle pose, optionally on explicit axes.
% H = drawVehicle(POSE,VEHICLE,DELTA,AX) returns the body patch for styling.
% The original three-argument call continues to use the current axes.
if nargin < 4
    ax = gca;
end

x = pose(1);
y = pose(2);
theta = pose(3);

% Vehicle body corners relative to rear axle midpoint
xBody = [-vehicle.rearOverhang, ...
          vehicle.wheelbase + vehicle.frontOverhang, ...
          vehicle.wheelbase + vehicle.frontOverhang, ...
         -vehicle.rearOverhang];

yBody = [-vehicle.width/2, ...
         -vehicle.width/2, ...
          vehicle.width/2, ...
          vehicle.width/2];

R = [cos(theta) -sin(theta);
     sin(theta)  cos(theta)];

body = R * [xBody; yBody];

body(1,:) = body(1,:) + x;
body(2,:) = body(2,:) + y;

bodyHandle = patch(ax, body(1,:), body(2,:), ...
      [0.7 0.7 0.7], ...
      'FaceAlpha',0.35, ...
      'LineWidth',1.5,'EdgeColor',[0.2,0.2,0.2]);

% Rear axle
rearLeft  = [0;  vehicle.width/2];
rearRight = [0; -vehicle.width/2];

% Front axle
frontLeft  = [vehicle.wheelbase;  vehicle.width/2];
frontRight = [vehicle.wheelbase; -vehicle.width/2];

drawWheel(rearLeft,  theta);
drawWheel(rearRight, theta);
drawWheel(frontLeft, theta + delta);
drawWheel(frontRight,theta + delta);

    function drawWheel(localPos, wheelAngle)

        p = R * localPos + [x; y];

        wheelLength = 0.55;
        wheelWidth = 0.18;

        wx = [-wheelLength/2 wheelLength/2 wheelLength/2 -wheelLength/2];
        wy = [-wheelWidth/2 -wheelWidth/2 wheelWidth/2 wheelWidth/2];

        RW = [cos(wheelAngle) -sin(wheelAngle);
              sin(wheelAngle)  cos(wheelAngle)];

        wheel = RW * [wx; wy];

        wheel(1,:) = wheel(1,:) + p(1);
        wheel(2,:) = wheel(2,:) + p(2);

        patch(ax, wheel(1,:), wheel(2,:), [0.2 0.2 0.2], ...
            'HandleVisibility','off','EdgeColor',[0.2,0.2,0.2]);

    end

end
