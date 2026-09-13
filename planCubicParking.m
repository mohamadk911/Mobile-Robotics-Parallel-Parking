function plan = planCubicParking(vehicle, scenario, settings)
%planCubicParking Automatically select two reverse cubics in parking-local space.
% PLAN = planCubicParking(VEHICLE,SCENARIO,plannerSettings()) returns segments,
% status (feasible/not_found/invalid_input/solver_error) and diagnostics.
% Only independently validated geometry is returned. Feasible means geometric
% admissibility in this bounded family, not global optimality, collision-free
% parking or simulated trajectory-following success. Joins are declared stopped.
% No timing is assigned. scenario1Path and historical fixtures are never used.
%
% Seven independent variables: intermediate x/y/body heading and four positive
% endpoint tangent magnitudes, in segment order. Exact endpoint/join poses are
% imposed by cubicPath construction. Fixed-point curvature constraints guide
% SQP; violating complete-segment extrema are inserted between solves. A
% separate validateCubicGeometry check of solver iterates and terminal
% candidates is the acceptance gate, regardless of exit
% flag. The shortest validated candidate across all attempts is selected.
% Diagnostics distinguish terminal solver results from a retained earlier
% feasible iterate. Failure always returns an empty segment collection.

    plan = struct('segments',struct([]),'status',struct('code','invalid_input','message',''), ...
        'diagnostics',struct('exitflag',NaN,'objectiveValue',NaN,'iterations',0, ...
        'initialGuessCount',0,'selectedParameters',[],'maxConstraintViolation',NaN, ...
        'validation',[],'attempts',[]));
    if nargin ~= 3
        plan.status.message = 'Supply vehicle, scenario and plannerSettings().';
        return
    end
    try
        validateInputs(vehicle,scenario,settings);
        q0 = double(scenario.initialPose);
        qf = double(scenario.targetPose);
        distance = norm(q0(1:2)-qf(1:2));
        if distance <= sqrt(eps)*max(1,vehicle.length)
            error('planCubicParking:InvalidInput','Distinct endpoint positions are required by this search.');
        end
        scale = [distance;distance;1;repmat(distance,4,1)];
        padding = settings.bounds.lateralPaddingWidths*vehicle.width;
        lb = [min(q0(1),qf(1));min(q0(2),qf(2))-padding; ...
            settings.bounds.heading(1);repmat(settings.bounds.tangentDistanceFactors(1)*distance,4,1)]./scale;
        ub = [max(q0(1),qf(1));max(q0(2),qf(2))+padding; ...
            settings.bounds.heading(2);repmat(settings.bounds.tangentDistanceFactors(2)*distance,4,1)]./scale;
        seeds = initialGuesses(q0,qf,settings)./scale;
        seeds = min(ub,max(lb,seeds)); % Project SEARCH seeds to search bounds.
        if any(~isfinite([lb;ub;seeds(:)]))
            error('planCubicParking:InvalidInput','Search coordinates overflowed.');
        end
    catch exception
        plan.status.message = exception.message;
        return
    end
    bestLength = Inf;
    bestSegments = struct([]);
    bestValidation = [];
    bestParameters = [];
    bestAttempt = 0;
    bestIteration = 0;
    bestRefinement = 0;
    attemptIndex = 0;
    refinement = 0;
    constraintPoints = {[],[]};
    template = struct('initialParameters',[],'exitflag',NaN,'objectiveValue',NaN, ...
        'iterations',0,'terminalParameters',[],'maxConstraintViolation',NaN, ...
        'validationPassed',false,'rejectionReason','','solverMessage','', ...
        'validatedCandidateCount',0,'bestValidatedLength',Inf,'solverError','', ...
        'refinements',0,'solveExitflags',[],'solveObjectives',[], ...
        'maxAbsCurvature',NaN,'minTurningRadius',NaN,'maxAbsSteering',NaN);
    attempts = repmat(template,1,size(seeds,2));
    limit = tan(vehicle.maxSteering)/vehicle.wheelbase;
    try
        options = solverOptions(settings.solverOptions);
        options.OutputFcn = @retainIterate;
    catch exception
        plan.status.code = 'solver_error';
        plan.status.message = exception.message;
        return
    end
    for attemptIndex = 1:size(seeds,2)
        attempts(attemptIndex).initialParameters = seeds(:,attemptIndex).*scale;
        constraintPoints = {linspace(0,1,settings.constraintRefinement.initialPointCount), ...
            linspace(0,1,settings.constraintRefinement.initialPointCount)};
        try
            objective(seeds(:,attemptIndex));
            constraints(seeds(:,attemptIndex));
            z = seeds(:,attemptIndex);
            for refinement = 0:settings.constraintRefinement.maxRefinements
                [z,fval,flag,output] = fmincon(@objective,z, ...
                    [],[],[],[],lb,ub,@constraints,options);
                attempts(attemptIndex).solveExitflags(end+1) = flag;
                attempts(attemptIndex).solveObjectives(end+1) = fval;
                attempts(attemptIndex).iterations = attempts(attemptIndex).iterations+output.iterations;
                attempts(attemptIndex).refinements = refinement;
                [valid,reason] = consider(z,output.iterations);
                if valid || flag <= 0
                    break
                end
                report = validateCubicGeometry(construct(z),vehicle,'stopped');
                if ~all(isfinite([report.segments.maxAbsCurvature]))
                    break
                end
                for segmentIndex = 1:2
                    constraintPoints{segmentIndex} = unique([constraintPoints{segmentIndex}, ...
                        report.segments(segmentIndex).maxCurvatureS]);
                end
            end
            attempts(attemptIndex).exitflag = flag;
            attempts(attemptIndex).objectiveValue = fval;
            attempts(attemptIndex).solverMessage = output.message;
            attempts(attemptIndex).terminalParameters = z.*scale;
            terminalReport = validateCubicGeometry(construct(z),vehicle,'stopped');
            attempts(attemptIndex).maxAbsCurvature = terminalReport.maxAbsCurvature;
            attempts(attemptIndex).minTurningRadius = terminalReport.minTurningRadius;
            attempts(attemptIndex).maxAbsSteering = terminalReport.maxAbsSteering;
            [c,~] = constraints(z);
            attempts(attemptIndex).maxConstraintViolation = max([0;c;lb-z;z-ub]);
            attempts(attemptIndex).validationPassed = valid;
            attempts(attemptIndex).rejectionReason = reason;
        catch exception
            attempts(attemptIndex).solverError = exception.identifier;
            attempts(attemptIndex).rejectionReason = exception.message;
        end
    end
    plan.diagnostics.initialGuessCount = numel(attempts);
    plan.diagnostics.attempts = attempts;
    plan.diagnostics.bounds = struct('lower',lb.*scale,'upper',ub.*scale);
    if isempty(bestSegments)
        plan.status.code = 'not_found';
        plan.status.message = 'No validated path found in this two-reverse-cubic family and bounded multistart search; this is not proof of physical impossibility.';
        if all(~cellfun(@isempty,{attempts.solverError}))
            plan.status.code = 'solver_error';
            plan.status.message = 'All solver attempts raised exceptions; inspect diagnostics.attempts.';
        end
        return
    end
    % Final independent gate on precisely the geometry about to be returned.
    validation = validateCubicGeometry(bestSegments,vehicle,'stopped');
    if ~validation.passed
        plan.status.code = 'not_found';
        plan.status.message = validation.message;
        plan.diagnostics.validation = validation;
        return
    end
    plan.segments = bestSegments;
    plan.status.code = 'feasible';
    plan.status.message = 'Selected the shortest independently validated candidate found; stopped join required. Global optimality is not established.';
    selected = attempts(bestAttempt);
    plan.diagnostics.exitflag = selected.exitflag;
    plan.diagnostics.objectiveValue = bestLength;
    plan.diagnostics.iterations = selected.iterations;
    plan.diagnostics.selectedParameters = bestParameters;
    plan.diagnostics.maxConstraintViolation = max(0,bestValidation.maxAbsCurvature/limit-1);
    plan.diagnostics.validation = validation;
    plan.diagnostics.selectedAttempt = bestAttempt;
    plan.diagnostics.selectedIteration = bestIteration;
    plan.diagnostics.selectedRefinement = bestRefinement;
    plan.diagnostics.totalLength = bestLength;
    plan.diagnostics.selectedIsTerminal = isequal(bestParameters,selected.terminalParameters);

    function segments = construct(z)
        p = z.*scale;
        segments = [cubicPath(q0,p(1:3),p(4),p(5),2,-1), ...
            cubicPath(p(1:3),qf,p(6),p(7),2,-1)];
    end

    function value = objective(z)
        try
            value = sum(cubicPathLength(construct(z), ...
                settings.lengthIntegration.AbsTol,settings.lengthIntegration.RelTol));
        catch exception
            if ~startsWith(exception.identifier,'evaluateCubicPath:')
                rethrow(exception)
            end
            value = 1e6*distance; % Invalid geometry penalty, never acceptance.
        end
    end

    function [c,ceq] = constraints(z)
        ceq = [];
        c = ones(2*sum(cellfun(@numel,constraintPoints)),1)*1e6;
        try
            segments = construct(z);
            offset = 0;
            for k = 1:2
                geometry = evaluateCubicPath(segments(k).coefficients,constraintPoints{k},-1);
                values = [geometry.curvature.'/limit-1;-geometry.curvature.'/limit-1];
                c(offset+(1:numel(values))) = values;
                offset = offset+numel(values);
            end
        catch exception
            if ~startsWith(exception.identifier,'evaluateCubicPath:')
                rethrow(exception)
            end
        end
    end

    function stop = retainIterate(z,optimValues,state)
        stop = false;
        if strcmp(state,'init') || strcmp(state,'iter')
            consider(z,optimValues.iteration);
        end
    end

    function [valid,reason] = consider(z,iteration)
        valid = false;
        reason = '';
        try
            segments = construct(z);
            report = validateCubicGeometry(segments,vehicle,'stopped');
            valid = report.passed && all(z >= lb & z <= ub);
            if ~valid
                reason = report.message;
                if report.passed
                    reason = 'Candidate lies outside the declared search bounds.';
                end
                return
            end
            lengthValue = sum(cubicPathLength(segments, ...
                settings.lengthIntegration.AbsTol,settings.lengthIntegration.RelTol));
            attempts(attemptIndex).validatedCandidateCount = attempts(attemptIndex).validatedCandidateCount+1;
            attempts(attemptIndex).bestValidatedLength = min(attempts(attemptIndex).bestValidatedLength,lengthValue);
            if lengthValue < bestLength
                bestLength = lengthValue;
                bestSegments = segments;
                bestParameters = z.*scale;
                bestValidation = report;
                bestAttempt = attemptIndex;
                bestIteration = iteration;
                bestRefinement = refinement;
            end
        catch exception
            if ~startsWith(exception.identifier,'evaluateCubicPath:')
                rethrow(exception)
            end
            reason = exception.message;
        end
    end
end

function seeds = initialGuesses(q0,qf,settings)
    fractions = settings.initialization.chordFractions;
    factors = settings.initialization.headingFactors;
    reverseChord = atan2(q0(2)-qf(2),q0(1)-qf(1));
    seeds = zeros(7,numel(fractions)*numel(factors));
    index = 0;
    for fraction = fractions
        midpoint = (1-fraction)*q0(1:2)+fraction*qf(1:2);
        lengths = [norm(midpoint-q0(1:2));norm(qf(1:2)-midpoint)];
        for factor = factors
            index = index+1;
            seeds(:,index) = [midpoint;factor*reverseChord;lengths(1)*ones(2,1);lengths(2)*ones(2,1)];
        end
    end
end

function validateInputs(vehicle,scenario,settings)
    validateattributes(scenario,{'struct'},{'scalar'});
    validateattributes(settings,{'struct'},{'scalar'});
    required = {'id','spaceLength','framePoseWorld','startOffsetFront', ...
        'rearCarPose','frontCarPose','initialPose','targetPose'};
    assert(all(isfield(scenario,required)),'planCubicParking:InvalidInput','Incomplete scenario.');
    reference = scenarioParameters(vehicle,scenario.id,scenario.framePoseWorld,scenario.spaceLength);
    for k = 4:numel(required)
        value = scenario.(required{k});
        validateattributes(value,{'numeric'},{'real','finite','size',[3,1]});
        difference = double(value)-reference.(required{k});
        difference(3) = atan2(sin(difference(3)),cos(difference(3)));
        assert(norm(difference) <= 1e-10, ...
            'planCubicParking:InvalidInput','Scenario poses conflict with its defining geometry.');
    end
    validateattributes(vehicle.maxSteering,{'numeric'},{'real','finite','scalar','positive','<',pi/2});
    assert(isfinite(tan(vehicle.maxSteering)/vehicle.wheelbase), ...
        'planCubicParking:InvalidInput','Nonfinite physical curvature limit.');
    validateattributes(vehicle.minTurningRadius,{'numeric'},{'real','finite','scalar','positive'});
    radius = double(vehicle.wheelbase)/tan(double(vehicle.maxSteering));
    assert(abs(double(vehicle.minTurningRadius)-radius) <= 64*eps(max(radius,double(vehicle.minTurningRadius))), ...
        'planCubicParking:InvalidInput','Vehicle minimum-radius metadata conflicts with wheelbase/steering.');
    defaults = plannerSettings();
    assert(isequal(sort(fieldnames(settings)),sort(fieldnames(defaults))), ...
        'planCubicParking:InvalidInput','Settings fields must match plannerSettings().');
    groups = {'bounds','objectiveWeights','initialization','solverOptions', ...
        'lengthIntegration','constraintRefinement'};
    for k = 1:numel(groups)
        group = settings.(groups{k});
        assert(isstruct(group) && isscalar(group) && ...
            isequal(sort(fieldnames(group)),sort(fieldnames(defaults.(groups{k})))), ...
            'planCubicParking:InvalidInput','Settings substruct fields must match plannerSettings().');
    end
    assert(isequal(settings.solver,'fmincon') && isequal(settings.directionPattern,[-1,-1]), ...
        'planCubicParking:InvalidInput','Only fmincon with directionPattern [-1,-1] is supported.');
    assert(isequal(settings.objectiveWeights,defaults.objectiveWeights) && ...
        isequal(settings.feasibilityTolerances,defaults.feasibilityTolerances), ...
        'planCubicParking:InvalidInput','Length-only objective and fixed validator tolerances are required.');
    validateattributes(settings.bounds.lateralPaddingWidths,{'numeric'},{'real','finite','scalar','nonnegative'});
    validateattributes(settings.bounds.heading,{'numeric'},{'real','finite','size',[1,2],'>=',-pi/2,'<=',pi/2});
    validateattributes(settings.bounds.tangentDistanceFactors,{'numeric'},{'real','finite','size',[1,2],'positive'});
    assert(diff(settings.bounds.heading)>0 && diff(settings.bounds.tangentDistanceFactors)>0, ...
        'planCubicParking:InvalidInput','Search bound intervals must increase.');
    validateattributes(settings.initialization.chordFractions,{'numeric'},{'real','finite','row','nonempty','>',0,'<',1});
    validateattributes(settings.initialization.headingFactors,{'numeric'},{'real','finite','row','nonempty','positive'});
    validateattributes(settings.lengthIntegration.AbsTol,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(settings.lengthIntegration.RelTol,{'numeric'},{'real','finite','scalar','>=',100*eps});
    validateattributes(settings.constraintRefinement.initialPointCount,{'numeric'},{'real','finite','scalar','integer','>=',3});
    validateattributes(settings.constraintRefinement.maxRefinements,{'numeric'},{'real','finite','scalar','integer','nonnegative'});
    assert(isequal(sort(fieldnames(settings.solverOptions)),sort(fieldnames(defaults.solverOptions))), ...
        'planCubicParking:InvalidInput','Use the documented solverOptions fields; callbacks/plotting are not accepted.');
    solverOptions(settings.solverOptions); % Validate installed solver option values.
end

function options = solverOptions(values)
    options = optimoptions('fmincon');
    names = fieldnames(values);
    for k = 1:numel(names)
        options.(names{k}) = values.(names{k});
    end
end
