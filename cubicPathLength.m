function lengths = cubicPathLength(segments, absTol, relTol)
%cubicPathLength Arc lengths of regular coefficient-based cubic segments.
% LENGTHS is 1-by-M, in metres. Adaptive MATLAB integral quadrature evaluates
% hypot(x_s,y_s) directly, independent of caches, timing and simulation.
% Defaults: absolute 1e-9 m per segment, relative 1e-9. These are requested
% quadrature error tolerances, not a claim of interval-certified accuracy.
    if nargin < 2
        absTol = 1e-9;
    end
    if nargin < 3
        relTol = 1e-9;
    end
    validateattributes(segments,{'struct'},{'vector','nonempty'},mfilename,'segments');
    validateattributes(absTol,{'numeric'},{'real','finite','scalar','positive'},mfilename,'absTol');
    validateattributes(relTol,{'numeric'},{'real','finite','scalar','>=',100*eps},mfilename,'relTol');
    lengths = zeros(1,numel(segments));
    for k = 1:numel(segments)
        % Production evaluator supplies complete-interval regularity checks.
        evaluateCubicPath(segments(k).coefficients,[0,1],segments(k).direction);
        first = double(segments(k).coefficients(:,1:3)).*[3,2,1];
        lengths(k) = integral(@(s) hypot(polyval(first(1,:),s), ...
            polyval(first(2,:),s)),0,1,AbsTol=double(absTol),RelTol=double(relTol));
    end
    if any(~isfinite(lengths))
        error('cubicPathLength:NonfiniteLength','Arc-length integration produced nonfinite values.');
    end
end
