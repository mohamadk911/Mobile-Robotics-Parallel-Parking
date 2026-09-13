function result = cubicPathDistance(segments, points)
%cubicPathDistance Shortest XY distances to a complete piecewise cubic path.
% POINTS is finite real 2-by-N (metres). For each point z, consider every
% segment p(s), both endpoints and stationary points of squared distance:
% (p(s)-z)'*p'(s)=0, a polynomial of degree at most five on [0,1].
% Recursive derivative partitions bracket sign-changing roots; derivative
% candidates are retained too, including repeated-root locations. Extra
% candidates lie on the path and cannot underestimate its minimum distance.
% No cached path samples, time correspondence or single local search is used.
%
% Bisection widths 1e-12 and 1e-14 in dimensionless s must yield distances
% agreeing within 1e-9 m + 1e-12*distance, on EVERY segment/point pair.
% These are floating-point resolution/convergence checks, not certified
% interval error bounds. Ill-conditioned polynomial arithmetic can limit
% accuracy; nonfinite or nonconverged calculations throw rather than pass.
% Result includes distance, nearestXY, s, segmentIndex and numerical details.
% Exact ties choose the first segment and then the lowest candidate s.
    validateattributes(segments,{'struct'},{'row','nonempty'},mfilename,'segments');
    validateattributes(points,{'numeric'},{'real','finite','2d','nrows',2,'nonempty'},mfilename,'points');
    points = double(points);
    n = size(points,2);
    result = struct('distance',inf(1,n),'nearestXY',nan(2,n),'s',nan(1,n), ...
        'segmentIndex',nan(1,n),'numerics',struct('method','polynomial stationary points', ...
        'rootWidth',1e-12,'refinedRootWidth',1e-14,'distanceAbsTolerance',1e-9, ...
        'distanceRelTolerance',1e-12,'maxRefinementDifference',0,'converged',false));
    for k = 1:numel(segments)
        evaluateCubicPath(segments(k).coefficients,[0,1],segments(k).direction);
        c = double(segments(k).coefficients);
        for j = 1:n
            residual = c;
            residual(:,4) = residual(:,4)-points(:,j);
            assert(all(isfinite(residual),'all'),'cubicPathDistance:NonfiniteResult', ...
                'Point-relative polynomial coefficients overflowed.');
            scale = max(abs(residual),[],'all');
            normalized = residual/scale; % Regular paths imply scale > 0.
            derivative = normalized(:,1:3).*[3,2,1];
            stationary = conv(normalized(1,:),derivative(1,:))+ ...
                conv(normalized(2,:),derivative(2,:));
            coarse = candidates(stationary,result.numerics.rootWidth);
            fine = candidates(stationary,result.numerics.refinedRootWidth);
            coarseDistance = min(hypot(polyval(residual(1,:),coarse),polyval(residual(2,:),coarse)));
            [distance,index] = min(hypot(polyval(residual(1,:),fine),polyval(residual(2,:),fine)));
            difference = abs(distance-coarseDistance);
            assert(isfinite(distance) && isfinite(difference), ...
                'cubicPathDistance:NonfiniteResult','Distance calculation overflowed.');
            assert(difference <= result.numerics.distanceAbsTolerance+ ...
                result.numerics.distanceRelTolerance*distance, ...
                'cubicPathDistance:NotConverged','Distance changed excessively under root refinement.');
            result.numerics.maxRefinementDifference = max(result.numerics.maxRefinementDifference,difference);
            if distance < result.distance(j)
                s = fine(index);
                result.distance(j) = distance;
                result.s(j) = s;
                result.segmentIndex(j) = k;
                result.nearestXY(:,j) = [polyval(c(1,:),s);polyval(c(2,:),s)];
            end
        end
    end
    assert(all(isfinite(result.nearestXY),'all'),'cubicPathDistance:NonfiniteResult', ...
        'Nearest-point coordinates overflowed.');
    result.numerics.converged = true;
end

function points = candidates(p,width)
% Low-degree polynomial root candidates, including all derivative partitions.
% On each interval between derivative roots the polynomial is monotone.
    first = find(p ~= 0,1);
    if isempty(first)
        points = [0,1];
        return
    end
    p = p(first:end)/max(abs(p));
    if isscalar(p)
        points = [0,1];
        return
    end
    partitions = candidates(polyder(p),width);
    points = partitions;
    for k = 1:numel(partitions)-1
        lo = partitions(k); hi = partitions(k+1);
        flo = polyval(p,lo); fhi = polyval(p,hi);
        if sign(flo)*sign(fhi) < 0
            while hi-lo > width
                mid = (lo+hi)/2;
                fm = polyval(p,mid);
                if fm == 0
                    lo = mid; hi = mid;
                elseif sign(fm) == sign(flo)
                    lo = mid; flo = fm;
                else
                    hi = mid;
                end
            end
            points(end+1) = (lo+hi)/2; %#ok<AGROW>
        end
    end
    points = unique(points);
end
