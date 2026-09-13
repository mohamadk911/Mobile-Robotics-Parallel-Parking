function path = cubicPath(startPose, endPose, ks, ke, nSamples, direction)
%cubicPath Construct a cubic Hermite path from rear-axle endpoint poses.
% Poses are 3-by-1 [x;y;heading], in metres/radians. Positive ks and ke
% specify endpoint tangent magnitudes (metres per unit s). Direction is
% +1 forward or -1 reverse. Coefficients are the geometry source of truth;
% sampled fields are analytic evaluations, with continuous body headings.

    validateattributes(startPose, {'numeric'}, {'real','finite','size',[3,1]}, mfilename, 'startPose');
    validateattributes(endPose, {'numeric'}, {'real','finite','size',[3,1]}, mfilename, 'endPose');
    validateattributes(ks, {'numeric'}, {'real','finite','scalar','positive'}, mfilename, 'ks');
    validateattributes(ke, {'numeric'}, {'real','finite','scalar','positive'}, mfilename, 'ke');
    validateattributes(nSamples, {'numeric'}, {'real','finite','scalar','integer','>=',2}, mfilename, 'nSamples');
    validateattributes(direction, {'numeric'}, {'real','finite','scalar'}, mfilename, 'direction');
    if ~ismember(direction, [-1,1])
        error('cubicPath:InvalidDirection', 'Direction must be +1 or -1.');
    end

    startPose = double(startPose);
    endPose = double(endPose);
    direction = double(direction);
    offset = (direction == -1)*pi;
    tangentS = double(ks)*[cos(startPose(3)+offset); sin(startPose(3)+offset)];
    tangentE = double(ke)*[cos(endPose(3)+offset); sin(endPose(3)+offset)];
    displacement = endPose(1:2) - startPose(1:2);
    % Hermite coefficients, formed from displacement to avoid subtracting
    % large world-coordinate terms when computing geometric derivatives.
    coefficients = [-2*displacement+tangentS+tangentE, ...
        3*displacement-2*tangentS-tangentE, tangentS, startPose(1:2)];

    path = evaluateCubicPath(coefficients, linspace(0,1,double(nSamples)), direction);
    path.startPose = startPose;
    path.endPose = endPose;
end
