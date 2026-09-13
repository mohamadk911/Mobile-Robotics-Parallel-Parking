function convergence = parkingConvergence(dt, errors, baselineDt)
%parkingConvergence Adjacent coarse/fine ratios and log-ratio observed order.
% Positive finite errors only yield ratios/order. Zero/NaN/Inf errors yield
% NaN rather than a fabricated order. Last row has no finer comparison.
    validateattributes(dt,{'numeric'},{'real','finite','row','nonempty','positive'});
    validateattributes(errors,{'numeric'},{'real','row','size',size(dt)});
    assert(all(errors>=0 | isnan(errors)),'parkingConvergence:InvalidErrors','Errors must be nonnegative or unavailable.');
    assert(all(diff(dt)<0),'parkingConvergence:GridOrder','dt must strictly decrease.');
    baseline = find(dt==baselineDt);
    assert(isscalar(baseline),'parkingConvergence:MissingBaseline','Baseline must appear exactly once.');
    ratio = nan(size(dt)); order = ratio; relative = ratio;
    for k = 1:numel(dt)-1
        if all(isfinite(errors(k:k+1))) && all(errors(k:k+1)>0)
            ratio(k) = errors(k)/errors(k+1);
            order(k) = log(ratio(k))/log(dt(k)/dt(k+1));
        end
    end
    if isfinite(errors(baseline)) && errors(baseline)>0
        relative = errors/errors(baseline);
    end
    convergence = table(dt.',errors.',ratio.',order.',relative.', ...
        'VariableNames',{'dt','error','ratioToFiner','observedOrder','relativeToBaseline'});
end
