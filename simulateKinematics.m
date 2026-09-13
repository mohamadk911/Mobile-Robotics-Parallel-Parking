function q = simulateKinematics(q0, t, v, delta, vehicle)
%simulateKinematics Explicit Euler with left-endpoint commands and actual times.
% On [t(j),t(j+1)), use v(j),delta(j) and q(:,j). The last command sample
% is retained but has no following interval. Nonuniform intervals are valid.
% Inputs may be row/column vectors of equal length; output is double 3-by-N.
% A single sample returns q0. No reference poses enter the integration.

validateattributes(q0,{'numeric'},{'real','finite','size',[3,1]},mfilename,'q0');
validateattributes(t,{'numeric'},{'real','finite','vector','nonempty'},mfilename,'t');
validateattributes(v,{'numeric'},{'real','finite','vector','numel',numel(t)},mfilename,'v');
validateattributes(delta,{'numeric'},{'real','finite','vector','numel',numel(t)},mfilename,'delta');
validateattributes(vehicle,{'struct'},{'scalar'},mfilename,'vehicle');
validateattributes(vehicle.wheelbase,{'numeric'},{'real','finite','scalar','positive'}, ...
    mfilename,'vehicle.wheelbase');
t = double(t(:).');
v = double(v(:).');
delta = double(delta(:).');
q0 = double(q0);
vehicle.wheelbase = double(vehicle.wheelbase);
if any(diff(t) <= 0) || any(~isfinite(diff(t)))
    error('simulateKinematics:InvalidTime','Time intervals must be finite and strictly positive.');
end

q = zeros(3, length(t));
q(:,1) = q0;

for k = 2:length(t)

    dt = t(k) - t(k-1);

    dq = carKinematics( ...
        q(:,k-1), ...
        v(k-1), ...
        delta(k-1), ...
        vehicle);

    q(:,k) = q(:,k-1) + dq * dt;

end

if any(~isfinite(q),'all')
    error('simulateKinematics:NonfiniteResult','Euler propagation produced nonfinite states.');
end

end
