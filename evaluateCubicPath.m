function curve = evaluateCubicPath(coefficients, s, direction)
%evaluateCubicPath Evaluate a regular planar cubic and its analytic geometry.
% Coefficients are 2-by-4, rows x/y and columns [s^3,s^2,s,1]. Query s may
% be a scalar or unordered vector in [0,1]; output samples are row vectors.
% Headings are continuous branches anchored at s=0, independent of query
% order/density. Body heading at s=0 is wrapped to the atan2 convention.
% Tangents below sqrt(eps)*max(1,max(abs(derivative coefficients))) are
% rejected: this metre-scale floor avoids ill-conditioned heading/curvature.
% This regularity guard is not a steering or curvature-limit feasibility test.

    validateattributes(coefficients, {'numeric'}, {'real','finite','size',[2,4]}, mfilename, 'coefficients');
    validateattributes(s, {'numeric'}, {'real','finite','vector','nonempty','>=',0,'<=',1}, mfilename, 's');
    validateattributes(direction, {'numeric'}, {'real','finite','scalar'}, mfilename, 'direction');
    if ~ismember(direction, [-1,1])
        error('evaluateCubicPath:InvalidDirection', 'Direction must be +1 or -1.');
    end
    coefficients = double(coefficients);
    s = double(s(:).');
    direction = double(direction);
    first = coefficients(:,1:3).*[3,2,1];
    second = first(:,1:2).*[2,1];
    if any(~isfinite([first(:); second(:)]))
        error('evaluateCubicPath:NonfiniteGeometry', 'Derivative coefficients overflowed.');
    end

    % Check tangent minima over the whole interval, so sparse queries cannot
    % conceal a stationary point at which a continuous heading is undefined.
    tangentScale = max(1, max(abs(first), [], 'all'));
    normalized = first/tangentScale;
    speedSquared = conv(normalized(1,:),normalized(1,:)) + ...
        conv(normalized(2,:),normalized(2,:));
    stationary = roots(polyder(speedSquared));
    realStationary = real(stationary(abs(imag(stationary)) <= 1e-10*max(1,abs(real(stationary)))));
    probes = [0,1,realStationary(realStationary > 0 & realStationary < 1).'];
    probeSpeed = hypot(polyval(normalized(1,:),probes), polyval(normalized(2,:),probes));
    if any(probeSpeed <= sqrt(eps))
        error('evaluateCubicPath:DegenerateTangent', ...
            'The curve has a zero or near-zero tangent on [0,1].');
    end

    x = polyval(coefficients(1,:), s);
    y = polyval(coefficients(2,:), s);
    dxds = polyval(first(1,:), s);
    dyds = polyval(first(2,:), s);
    d2xds2 = polyval(second(1,:), s);
    d2yds2 = polyval(second(2,:), s);
    speed = hypot(dxds, dyds);
    if any(speed/tangentScale <= sqrt(eps))
        error('evaluateCubicPath:DegenerateTangent', 'A query has a zero or near-zero tangent.');
    end
    % Normalize before the cross product to avoid unnecessary squared/cubed
    % magnitudes; this is (x_s*y_ss-y_s*x_ss)/hypot(x_s,y_s)^3.
    curvature = ((dxds./speed).*(d2yds2./speed) - ...
        (dyds./speed).*(d2xds2./speed))./speed;

    % Factor the complex tangent polynomial. Each factor's phase change
    % along real s is continuous without sampling-based unwrap assumptions.
    tangentRoots = roots(complex(normalized(1,:), normalized(2,:)));
    angleChange = zeros(size(s));
    for k = 1:numel(tangentRoots)
        angleChange = angleChange + angle((s-tangentRoots(k))/(-tangentRoots(k)));
    end
    tangentStart = atan2(first(2,3), first(1,3));
    bodyStart = tangentStart + (direction == -1)*pi;
    bodyStart = atan2(sin(bodyStart), cos(bodyStart));
    thetaTangent = tangentStart + angleChange;
    thetaBody = bodyStart + angleChange;
    if any(~isfinite([x,y,dxds,dyds,d2xds2,d2yds2,curvature,thetaTangent,thetaBody]))
        error('evaluateCubicPath:NonfiniteGeometry', 'Curve evaluation produced nonfinite geometry.');
    end

    curve.coefficients = coefficients;
    curve.direction = direction;
    curve.s = s;
    curve.x = x;
    curve.y = y;
    curve.dxds = dxds;
    curve.dyds = dyds;
    curve.d2xds2 = d2xds2;
    curve.d2yds2 = d2yds2;
    curve.thetaTangent = thetaTangent;
    curve.thetaBody = thetaBody;
    curve.curvature = curvature;
end
