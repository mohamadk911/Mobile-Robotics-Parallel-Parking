function result = validateCubicGeometry(segments, vehicle, joinTypes)
%validateCubicGeometry Validate complete cubic paths before time assignment.
% RESULT = validateCubicGeometry(SEGMENTS, VEHICLE, JOINTYPES) accepts a
% nonempty struct vector with coefficients (2-by-4, descending powers),
% startPose/endPose (3-by-1, rear axle, m/rad) and direction (+1/-1).
% JOINTYPES is "stopped" (default), "moving", or one string per join.
% This is a declaration of intended joins, not verification of actual speed.
% Timing must later enforce stops or continuous signed speed at moving joins.
%
% Only coefficients define geometry. Optional geometric caches are checked
% for finite numeric data but never used for extrema, poses or joins. Other
% metadata (including duration) is ignored. No timing, simulation or plots.
% Invalid data/geometry returns failed checks; unavailable metrics are NaN.
% A straight regular curve has minTurningRadius=Inf. Limits derive from the
% supplied wheelbase and maxSteering; minTurningRadius metadata is not used.
%
% For velocity polynomial u, Q=u.u and N=det(u,u'), kappa=N/Q^(3/2).
% Tangent minima occur at endpoints or Q'=0; nonzero curvature extrema at
% endpoints or H=2*N'*Q-3*N*Q'=0 (degree <=5). Recursive derivative partitions
% and bisection locate real roots without a sample grid; derivative partition
% points are also retained to cover repeated roots. See polynomialCandidates.
% Repeat root isolation at widths 1e-12 and 1e-14 in s. Require agreement in
% max curvature to 1e-10*max(1,max curvature) and normalized min tangent to
% 1e-12. This is a floating-point convergence check, not interval certification.
% Unstable/overflowing calculations fail rather than claim feasibility.
%
% Numerical tolerances are returned in result.tolerances. Endpoint/join
% agreement uses 1e-8 m, 1e-10 rad; moving curvature uses 1e-10 1/m.
% Tangent floor is sqrt(eps)*max(1,max(abs(derivative coefficients))), matching
% the analytic evaluator's regularity convention. Physical limits are strict:
% no curvature/steering clipping and no added tolerance on the vehicle limit.

    if nargin < 3
        joinTypes = "stopped";
    end
    tol = struct('position',1e-8,'heading',1e-10,'joinCurvature',1e-10, ...
        'rootWidth',1e-12,'refinedRootWidth',1e-14, ...
        'curvatureConvergence',1e-10,'tangentConvergence',1e-12, ...
        'relativeTangentFloor',sqrt(eps));
    result = struct('passed',false,'status','fail','code','invalid_input', ...
        'message','','segments',[],'joins',[],'checks',emptyChecks(), ...
        'maxAbsCurvature',NaN,'minTurningRadius',NaN,'maxAbsSteering',NaN, ...
        'curvatureLimit',NaN,'vehicleMinTurningRadius',NaN,'tolerances',tol);
    validCollection = isstruct(segments) && isvector(segments) && ~isempty(segments);
    validVehicle = isstruct(vehicle) && isscalar(vehicle) && ...
        all(isfield(vehicle,{'wheelbase','maxSteering'})) && ...
        finiteArray(vehicle.wheelbase,[1,1]) && vehicle.wheelbase > 0 && ...
        finiteArray(vehicle.maxSteering,[1,1]) && ...
        vehicle.maxSteering > 0 && vehicle.maxSteering < pi/2;
    result.checks = [check('input',0,'segmentCollection',validCollection,1,validCollection, ...
        'Expected a nonempty struct vector.'), ...
        check('input',0,'vehicle',validVehicle,1,validVehicle, ...
        'Wheelbase must be positive; steering limit must lie in (0,pi/2).')];
    if ~validCollection || ~validVehicle
        result.message = failureMessage(result.checks);
        return
    end
    L = double(vehicle.wheelbase);
    steeringLimit = double(vehicle.maxSteering);
    result.curvatureLimit = tan(steeringLimit)/L;
    result.vehicleMinTurningRadius = 1/result.curvatureLimit;
    validLimits = isfinite(result.curvatureLimit) && result.curvatureLimit > 0 && ...
        isfinite(result.vehicleMinTurningRadius) && result.vehicleMinTurningRadius > 0;
    result.checks(end+1) = check('input',0,'finiteVehicleLimits',validLimits,1,validLimits, ...
        'Derived vehicle limits must be finite and positive.');
    count = numel(segments);
    [joinTypes,validJoins] = normalizeJoins(joinTypes,count-1);
    result.checks(end+1) = check('input',0,'joinTypes',validJoins,1,validJoins, ...
        'Use stopped/moving, either scalar or one entry per join.');
    if ~validJoins || ~validLimits
        result.message = failureMessage(result.checks);
        return
    end

    reports = repmat(segmentReport(),1,count);
    for k = 1:count
        reports(k) = validateSegment(segments(k),L,steeringLimit,result.curvatureLimit,tol,k);
        result.checks = [result.checks,reports(k).checks];
    end
    joins = repmat(struct('type',"",'passed',false,'positionError',NaN, ...
        'headingError',NaN,'curvatureJump',NaN,'steeringJump',NaN, ...
        'checks',emptyChecks()),1,count-1);
    for k = 1:count-1
        joins(k) = validateJoin(reports(k),reports(k+1),joinTypes(k), ...
            L,steeringLimit,result.curvatureLimit,tol,k);
        result.checks = [result.checks,joins(k).checks];
    end
    result.segments = reports;
    result.joins = joins;
    % Never hide an unavailable segment behind the maximum of valid segments.
    if all(isfinite([reports.maxAbsCurvature]))
        result.maxAbsCurvature = max([reports.maxAbsCurvature]);
        result.minTurningRadius = 1/result.maxAbsCurvature;
        result.maxAbsSteering = max([reports.maxAbsSteering]);
    end
    result.passed = all([result.checks.passed]);
    result.code = 'infeasible';
    if result.passed
        result.status = 'pass';
        result.code = 'feasible';
        result.message = 'All segment and declared join geometry checks passed.';
    else
        result.message = failureMessage(result.checks);
        if any(strcmp({reports.code},'invalid_input'))
            result.code = 'invalid_input';
        elseif any(strcmp({reports.code},'numerical_failure'))
            result.code = 'numerical_failure';
        end
    end
end

function report = validateSegment(segment,L,steeringLimit,curvatureLimit,tol,index)
    report = segmentReport();
    sourceOK = all(isfield(segment,{'coefficients','startPose','endPose','direction'})) && ...
        finiteArray(segment.coefficients,[2,4]) && finiteArray(segment.startPose,[3,1]) && ...
        finiteArray(segment.endPose,[3,1]) && finiteArray(segment.direction,[1,1]);
    report.checks = check('segment',index,'finiteSourceData',sourceOK,1,sourceOK, ...
        'Require finite real coefficients, endpoint poses and scalar direction of the documented sizes.');
    if ~sourceOK
        return
    end
    report.direction = double(segment.direction);
    directionOK = ismember(report.direction,[-1,1]);
    cacheOK = finiteCaches(segment);
    report.checks(end+1) = check('segment',index,'direction',report.direction,[-1,1],directionOK, ...
        'Direction must be +1 or -1.');
    report.checks(end+1) = check('segment',index,'finiteCaches',cacheOK,1,cacheOK, ...
        'Supplied geometric sample caches must contain finite real numeric data.');
    if ~directionOK || ~cacheOK
        return
    end
    report.code = 'numerical_failure';
    coefficients = double(segment.coefficients);
    first = coefficients(:,1:3).*[3,2,1];
    derivativeOK = all(isfinite(first),'all');
    report.checks(end+1) = check('segment',index,'finiteDerivatives',derivativeOK,1,derivativeOK, ...
        'Derivative coefficients overflowed.');
    if ~derivativeOK
        return
    end
    scale = max(1,max(abs(first),[],'all'));
    u = first/scale;
    Q = conv(u(1,:),u(1,:)) + conv(u(2,:),u(2,:));
    % The cubic term in det(u,u') cancels identically. Form the quadratic
    % directly so roundoff cannot introduce a spurious highest-order term.
    cross2 = @(a,b) a(1)*b(2)-a(2)*b(1);
    N = [-cross2(u(:,1),u(:,2)),2*cross2(u(:,3),u(:,1)),cross2(u(:,3),u(:,2))];
    hA = 2*conv(polyder(N),Q);
    hB = 3*conv(N,polyder(Q));
    sizeH = max(numel(hA),numel(hB));
    H = [zeros(1,sizeH-numel(hA)),hA] - [zeros(1,sizeH-numel(hB)),hB];
    coarseTangent = polynomialCandidates(polyder(Q),tol.rootWidth);
    fineTangent = polynomialCandidates(polyder(Q),tol.refinedRootWidth);
    coarseSpeed = min(hypot(polyval(u(1,:),coarseTangent),polyval(u(2,:),coarseTangent)));
    fineSpeed = hypot(polyval(u(1,:),fineTangent),polyval(u(2,:),fineTangent));
    [minimum,where] = min(fineSpeed);
    report.minTangent = scale*minimum;
    report.minTangentS = fineTangent(where);
    report.tangentThreshold = scale*tol.relativeTangentFloor;
    regular = minimum > tol.relativeTangentFloor;
    report.checks(end+1) = check('segment',index,'regularTangent',report.minTangent, ...
        report.tangentThreshold,regular,'Zero or near-zero tangent on the complete segment.');
    if ~regular
        report.code = 'degenerate_tangent';
        return
    end
    coarse = polynomialCandidates(H,tol.rootWidth);
    fine = polynomialCandidates(H,tol.refinedRootWidth);
    % Include Q critical points as extra probes near a small denominator.
    % Coordinate extrema also check for nonfinite polynomial positions.
    coarse = unique([coarse,coarseTangent,polynomialCandidates(u(1,:),tol.rootWidth), ...
        polynomialCandidates(u(2,:),tol.rootWidth)]);
    fine = unique([fine,fineTangent,polynomialCandidates(u(1,:),tol.refinedRootWidth), ...
        polynomialCandidates(u(2,:),tol.refinedRootWidth)]);
    try
        geometry = evaluateCubicPath(coefficients,fine,report.direction);
        coarseGeometry = evaluateCubicPath(coefficients,coarse,report.direction);
    catch exception
        if ~startsWith(exception.identifier,'evaluateCubicPath:')
            rethrow(exception)
        end
        report.checks(end+1) = check('segment',index,'finiteGeometry',false,1,false,exception.message);
        return
    end
    [maximum,where] = max(abs(geometry.curvature));
    curvatureDifference = abs(maximum-max(abs(coarseGeometry.curvature)));
    stable = curvatureDifference <= tol.curvatureConvergence*max(1,maximum) && ...
        abs(minimum-coarseSpeed) <= tol.tangentConvergence;
    report.checks(end+1) = check('segment',index,'extremaConvergence',curvatureDifference, ...
        tol.curvatureConvergence*max(1,maximum),stable, ...
        'Root refinement did not give stable curvature/tangent extrema.');
    if ~stable
        return
    end
    report.code = 'infeasible';
    report.candidateS = fine;
    report.maxCurvatureS = fine(where);
    report.maxAbsCurvature = maximum;
    report.minTurningRadius = 1/maximum;
    report.maxAbsSteering = atan(L*maximum);
    report.startPose = [geometry.x(1);geometry.y(1);geometry.thetaBody(1)];
    report.endPose = [geometry.x(end);geometry.y(end);geometry.thetaBody(end)];
    report.endpointCurvature = geometry.curvature([1,end]);
    report.endpointSteering = atan(L*report.direction*report.endpointCurvature);
    report.checks(end+1) = check('segment',index,'finiteCurvature',true,1,true, ...
        'Regular polynomial geometry has bounded continuous curvature.');
    report.checks = [report.checks,poseChecks(report.startPose,double(segment.startPose), ...
        tol,'segment',index,'start'),poseChecks(report.endPose,double(segment.endPose), ...
        tol,'segment',index,'end')];
    report.checks(end+1) = check('segment',index,'curvatureLimit',maximum,curvatureLimit, ...
        maximum <= curvatureLimit,'Complete-segment curvature exceeds the vehicle limit.');
    report.checks(end+1) = check('segment',index,'steeringLimit',report.maxAbsSteering,steeringLimit, ...
        report.maxAbsSteering <= steeringLimit,'Required steering exceeds the vehicle limit.');
    report.passed = all([report.checks.passed]);
    if report.passed
        report.code = 'feasible';
    end
end

function join = validateJoin(left,right,type,L,steeringLimit,curvatureLimit,tol,index)
    join = struct('type',type,'passed',false,'positionError',NaN,'headingError',NaN, ...
        'curvatureJump',NaN,'steeringJump',NaN,'checks',emptyChecks());
    available = all(isfinite([left.endPose;right.startPose; ...
        left.endpointCurvature(:);right.endpointCurvature(:)]));
    join.checks = check('join',index,'availableGeometry',available,1,available, ...
        'Cannot validate join because adjacent endpoint geometry is unavailable.');
    if ~available
        return
    end
    join.positionError = norm(left.endPose(1:2)-right.startPose(1:2));
    join.headingError = wrappedError(left.endPose(3),right.startPose(3));
    sides = [left.endpointCurvature(end),right.endpointCurvature(1)];
    steering = [left.endpointSteering(end),right.endpointSteering(1)];
    join.curvatureJump = abs(diff(sides));
    join.steeringJump = abs(diff(steering));
    join.checks = [join.checks,poseChecks(left.endPose,right.startPose,tol,'join',index,'join')];
    join.checks(end+1) = check('join',index,'oneSidedCurvature',max(abs(sides)),curvatureLimit, ...
        all(abs(sides) <= curvatureLimit),'One-sided join curvature exceeds the vehicle limit.');
    join.checks(end+1) = check('join',index,'oneSidedSteering',max(abs(steering)),steeringLimit, ...
        all(abs(steering) <= steeringLimit),'One-sided join steering exceeds the vehicle limit.');
    if type == "moving"
        compatible = left.direction == right.direction;
        join.checks(end+1) = check('join',index,'movingDirection',compatible,1,compatible, ...
            'Moving joins cannot reverse direction.');
        join.checks(end+1) = check('join',index,'movingCurvature',join.curvatureJump,tol.joinCurvature, ...
            join.curvatureJump <= tol.joinCurvature,'Moving joins require signed curvature continuity.');
        % atan is 1-Lipschitz: curvature tolerance implies this steering bound.
        join.checks(end+1) = check('join',index,'movingSteering',join.steeringJump,L*tol.joinCurvature, ...
            join.steeringJump <= L*tol.joinCurvature,'Moving joins require steering continuity.');
    end
    join.passed = all([join.checks.passed]);
end

function points = polynomialCandidates(p,width)
% Return endpoints, all isolated real roots, and derivative partition points.
% Between successive derivative roots a real polynomial is monotone, hence
% every simple root is bracketed by a sign change. Retaining all derivative
% candidates also retains even-multiplicity roots and avoids a complex-root
% cutoff. Extra candidates are harmless in a global maximum/minimum search.
    firstNonzero = find(p ~= 0,1);
    if isempty(firstNonzero)
        points = [0,1];
        return
    end
    p = p(firstNonzero:end);
    p = p/max(abs(p));
    if isscalar(p)
        points = [0,1];
        return
    end
    partitions = polynomialCandidates(polyder(p),width);
    points = partitions;
    for k = 1:numel(partitions)-1
        lo = partitions(k);
        hi = partitions(k+1);
        flo = polyval(p,lo);
        fhi = polyval(p,hi);
        if sign(flo)*sign(fhi) < 0
            while hi-lo > width
                mid = (lo+hi)/2;
                fm = polyval(p,mid);
                if fm == 0
                    lo = mid;
                    hi = mid;
                elseif sign(fm) == sign(flo)
                    lo = mid;
                    flo = fm;
                else
                    hi = mid;
                end
            end
            points(end+1) = (lo+hi)/2; %#ok<AGROW>
        end
    end
    points = unique(points);
end

function report = segmentReport()
    report = struct('passed',false,'code','invalid_input','direction',NaN, ...
        'maxAbsCurvature',NaN,'minTurningRadius',NaN,'maxAbsSteering',NaN, ...
        'minTangent',NaN,'minTangentS',NaN,'tangentThreshold',NaN, ...
        'maxCurvatureS',NaN,'candidateS',[], ...
        'startPose',nan(3,1),'endPose',nan(3,1), ...
        'endpointCurvature',[NaN,NaN],'endpointSteering',[NaN,NaN],'checks',emptyChecks());
end

function checks = poseChecks(actual,expected,tol,scope,index,prefix)
    position = norm(actual(1:2)-expected(1:2));
    heading = wrappedError(actual(3),expected(3));
    checks = [check(scope,index,[prefix,'Position'],position,tol.position,position <= tol.position, ...
        'Rear-axle positions disagree.'), ...
        check(scope,index,[prefix,'Heading'],heading,tol.heading,heading <= tol.heading, ...
        'Body headings disagree modulo 2*pi.')];
end

function value = wrappedError(a,b)
    value = abs(atan2(sin(a-b),cos(a-b)));
end

function valid = finiteArray(value,dimensions)
    valid = isnumeric(value) && isreal(value) && isequal(size(value),dimensions) && ...
        all(isfinite(value),'all');
end

function valid = finiteCaches(segment)
    caches = {'s','x','y','dxds','dyds','d2xds2','d2yds2','thetaTangent','thetaBody','curvature'};
    valid = true;
    for k = 1:numel(caches)
        if isfield(segment,caches{k})
            value = segment.(caches{k});
            valid = valid && isnumeric(value) && isreal(value) && all(isfinite(value),'all');
        end
    end
end

function [types,valid] = normalizeJoins(types,count)
    valid = (isstring(types) && isvector(types)) || (ischar(types) && isrow(types));
    if count == 0 && isempty(types)
        valid = true;
    end
    if ~valid
        return
    end
    types = string(types);
    valid = ~any(ismissing(types)) && all(ismember(types,["stopped","moving"])) && ...
        (isscalar(types) || numel(types) == count);
    if valid && isscalar(types)
        types = repmat(types,1,count);
    end
    types = reshape(types,1,[]);
end

function item = check(scope,index,name,value,limit,passed,reason)
    if passed
        reason = '';
    end
    item = struct('scope',scope,'index',index,'name',name,'value',value, ...
        'limit',limit,'passed',logical(passed),'reason',reason);
end

function checks = emptyChecks()
    checks = struct('scope',{},'index',{},'name',{},'value',{},'limit',{},'passed',{},'reason',{});
end

function message = failureMessage(checks)
    failed = checks(~[checks.passed]);
    descriptions = arrayfun(@(c) sprintf('%s %d %s: %s',c.scope,c.index,c.name,c.reason), ...
        failed,UniformOutput=false);
    message = strjoin(descriptions,'; ');
end
