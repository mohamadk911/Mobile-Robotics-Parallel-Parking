function comparison = compareParkingStates(a,b)
%compareParkingStates Differences A-B on IDENTICAL times; no interpolation.
% XY in metres, heading wrapped radians, RMSE trapezoidal on actual t.
    validateattributes(a.t,{'double'},{'real','finite','row','nonempty'});
    assert(numel(a.t)>=2 && all(diff(a.t)>0) && isequal(a.t,b.t), ...
        'compareParkingStates:TimeMismatch','Identical increasing times and positive duration required.');
    validateattributes(a.q,{'double'},{'real','finite','size',[3,numel(a.t)]});
    validateattributes(b.q,{'double'},{'real','finite','size',size(a.q)});
    error = a.q-b.q;
    heading = atan2(sin(error(3,:)),cos(error(3,:)));
    xy = hypot(error(1,:),error(2,:));
    duration = a.t(end)-a.t(1);
    assert(isfinite(duration) && duration>0,'compareParkingStates:InvalidDuration','Invalid duration.');
    comparison = struct('finalXY',xy(end),'finalHeading',abs(heading(end)), ...
        'maxXY',max(xy),'timeWeightedXYRMSE',sqrt(trapz(a.t,xy.^2)/duration), ...
        'maxHeading',max(abs(heading)),'t',a.t,'xyError',error(1:2,:), ...
        'headingError',heading);
    assert(all(isfinite([comparison.finalXY,comparison.finalHeading,comparison.maxXY, ...
        comparison.timeWeightedXYRMSE,comparison.maxHeading])), ...
        'compareParkingStates:NonfiniteMetrics','Difference metrics overflowed.');
end
