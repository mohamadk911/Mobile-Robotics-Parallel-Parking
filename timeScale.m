function scaling = timeScale(T, dt)
%timeScale Sample cubic zero-end-speed timing, including the exact endpoint.
% T and dt are positive, finite, real numeric scalars in seconds. Outputs
% are double row vectors; the final time interval may be shorter than dt.

validateattributes(T, {'numeric'}, {'scalar','real','finite','positive'}, mfilename, 'T', 1);
validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'}, mfilename, 'dt', 2);
T = double(T);
dt = double(dt);

t = 0:dt:T;
% Preserve the existing interior grid and include T exactly once.
t = [t(t < T), T];
tau = t / T;

s = 3*tau.^2 - 2*tau.^3;

sdot = (6*tau - 6*tau.^2) / T;

scaling.t = t;
scaling.s = s;
scaling.sdot = sdot;

end
