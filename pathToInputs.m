function traj = pathToInputs(path, scaling, vehicle)
%pathToInputs Derive bicycle commands from one analytic cubic geometry.
% Sampled path fields are caches and are not used to generate the commands.

validateattributes(path, {'struct'}, {'scalar'}, mfilename, 'path');
validateattributes(scaling, {'struct'}, {'scalar'}, mfilename, 'scaling');
validateattributes(vehicle, {'struct'}, {'scalar'}, mfilename, 'vehicle');
validateattributes(scaling.s, {'numeric'}, {'real','finite','row','nonempty'}, mfilename, 'scaling.s');
validateattributes(scaling.t, {'numeric'}, {'real','finite','size',size(scaling.s)}, mfilename, 'scaling.t');
validateattributes(scaling.sdot, {'numeric'}, {'real','finite','nonnegative','size',size(scaling.s)}, ...
    mfilename, 'scaling.sdot');
validateattributes(vehicle.wheelbase, {'numeric'}, {'real','finite','scalar','positive'}, ...
    mfilename, 'vehicle.wheelbase');
validateattributes(vehicle.maxSteering, {'numeric'}, {'real','finite','scalar','positive','<',pi/2}, ...
    mfilename, 'vehicle.maxSteering');
if any(diff(scaling.t) <= 0) || any(diff(scaling.s) < 0)
    error('pathToInputs:InvalidScaling', 'Times must increase strictly and path parameters must not decrease.');
end

geometry = evaluateCubicPath(path.coefficients, scaling.s, path.direction);
direction = geometry.direction;
s = geometry.s;
x = geometry.x;
y = geometry.y;
dxds = geometry.dxds;
dyds = geometry.dyds;
curvature = geometry.curvature;
thetaBody = geometry.thetaBody;

% Geometric rate
g = hypot(dxds, dyds);

% Signed longitudinal velocity
v = direction .* double(scaling.sdot) .* g;

% Yaw rate
omega = double(scaling.sdot) .* g .* curvature;

% Steering angle
delta = atan( ...
    double(vehicle.wheelbase) .* ...
    direction .* curvature);

if any(~isfinite([v,omega,delta]))
    error('pathToInputs:NonfiniteInputs', 'The timing and geometry produced nonfinite bicycle inputs.');
end

traj.t = double(scaling.t);
traj.s = s;

traj.x = x;
traj.y = y;
traj.theta = thetaBody;

traj.v = v;
traj.omega = omega;
traj.delta = delta;
traj.curvature = curvature;

traj.maxSteering = max(abs(delta));
traj.steeringWithinLimit = ...
    traj.maxSteering <= vehicle.maxSteering;
end
