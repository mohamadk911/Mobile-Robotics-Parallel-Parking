function evaluation = evaluateParkingExperiment(plan, trajectory, simulation, scenario, vehicle, settings)
%evaluateParkingExperiment Quantitative local parking POC evaluation and gate.
% E = evaluateParkingExperiment(PLAN,TRAJECTORY,SIMULATION,SCENARIO,VEHICLE, ...
%                            evaluationSettings())
% Geometry is revalidated from coefficients; stored planner diagnostics and
% timed sample maxima cannot certify steering/curvature or joins. Final
% simulation errors refer to scenario.targetPose, separately from plan-target
% checks. RMSE uses trapz on squared XY error and the actual global grid.
% cubicPathDistance measures sampled states against the full geometric path.
% No replanning, integration, plots, world conversion or file writes occur.
%
% All named checks are mandatory; unavailable checks fail with NaN values.
% Missing/unsuccessful stages return not_evaluated; malformed data and failed
% checks return fail. All unavailable metrics are NaN, except a known vehicle
% radius can still be reported. Straight regular paths legitimately have Inf
% radius. Headings compare modulo 2*pi. Numerical consistency uses existing
% geometry tolerances for poses, 1e-10 relative/absolute for command series.
% Passing assesses these supplied data, not provenance, collision clearance,
% feedback control or continuous-time simulation error between samples.
    if nargin < 6
        settings = evaluationSettings();
    end
    metricNames = {'finalPositionError','finalHeadingError','maxTimeMatchedXYError', ...
        'timeWeightedXYRMSE','maxGeometricPathDeviation','maxSteering', ...
        'maxAbsCurvature','minTurningRadius','vehicleMinTurningRadius'};
    names = {'settingsValid','vehicleValid','plannerFeasible','timingSuccess','simulationSuccess', ...
        'geometryValid','planTargetPosition','planTargetHeading','planInitialPosition', ...
        'planInitialHeading','finiteData','timeAlignment','referenceConsistent', ...
        'commandsConsistent','simulationInitialPose','simulationJoinContinuity', ...
        'stoppedBoundaries','steeringLimit','curvatureLimit','turningRadiusLimit', ...
        'geometricDistanceConverged','finiteMetrics', ...
        'finalPositionError','finalHeadingError','maxTimeMatchedXYError','timeWeightedXYRMSE'};
    checks = struct('name',names,'value',NaN,'limit',NaN,'passed',false, ...
        'reason','Not evaluated because prerequisites are unavailable.');
    evaluation = struct('metrics',cell2struct(num2cell(nan(size(metricNames))),metricNames,2), ...
        'errors',struct('t',[],'xError',[],'yError',[],'xyErrorMagnitude',[], ...
        'headingError',[],'geometricPathDeviation',[]),'checks',checks,'passed',false, ...
        'status','not_evaluated','message','','geometryValidation',[], ...
        'geometricDistance',[],'settings',settings);
    stage = 'settingsValid';
    try
        defaults = evaluationSettings();
        assert(isstruct(settings) && isscalar(settings) && ...
            isequal(sort(fieldnames(settings)),sort(fieldnames(defaults))), ...
            'evaluateParkingExperiment:InvalidSettings','Supply the four quality threshold fields only.');
        qualityNames = fieldnames(defaults);
        for k = 1:numel(qualityNames)
            name = qualityNames{k};
            validateattributes(settings.(name),{'numeric'},{'real','finite','scalar','positive'});
            index = strcmp(names,name);
            evaluation.checks(index).limit = double(settings.(name));
        end
        record(stage,true,true,true,'Valid quality thresholds.');
        stage = 'vehicleValid';
        validateattributes(vehicle,{'struct'},{'scalar'});
        validateattributes(vehicle.wheelbase,{'numeric'},{'real','finite','scalar','positive'});
        validateattributes(vehicle.maxSteering,{'numeric'},{'real','finite','scalar','positive','<',pi/2});
        curvatureLimit = tan(double(vehicle.maxSteering))/double(vehicle.wheelbase);
        vehicleRadius = 1/curvatureLimit;
        assert(isfinite(curvatureLimit) && isfinite(vehicleRadius) && vehicleRadius > 0, ...
            'evaluateParkingExperiment:InvalidVehicle','Invalid derived vehicle limits.');
        evaluation.metrics.vehicleMinTurningRadius = vehicleRadius;
        record(stage,true,true,true,'Vehicle limits derived from wheelbase and steering.');
    catch exception
        record(stage,false,true,false,exception.message);
        finish('fail');
        return
    end
    [plannerCode,plannerReason] = stageStatus(plan,true);
    [timingCode,timingReason] = stageStatus(trajectory,false);
    [simulationCode,simulationReason] = stageStatus(simulation,false);
    record('plannerFeasible',plannerCode,'feasible',strcmp(plannerCode,'feasible'),plannerReason);
    record('timingSuccess',timingCode,'success',strcmp(timingCode,'success'),timingReason);
    record('simulationSuccess',simulationCode,'success',strcmp(simulationCode,'success'),simulationReason);
    if ~all([evaluation.checks(3:5).passed])
        finish('not_evaluated');
        return
    end
    stage = 'geometryValid';
    try
        assert(isfield(plan,'segments') && isstruct(plan.segments) && ...
            isrow(plan.segments) && ~isempty(plan.segments), ...
            'evaluateParkingExperiment:MissingPlan','No nonempty plan segment row is available.');
        g = validateCubicGeometry(plan.segments,vehicle,'stopped');
        evaluation.geometryValidation = g;
        evaluation.metrics.maxSteering = g.maxAbsSteering;
        evaluation.metrics.maxAbsCurvature = g.maxAbsCurvature;
        evaluation.metrics.minTurningRadius = g.minTurningRadius;
        record('geometryValid',g.passed,true,g.passed,g.message);
        record('steeringLimit',g.maxAbsSteering,vehicle.maxSteering, ...
            g.maxAbsSteering <= vehicle.maxSteering,'Continuous geometric steering exceeds the vehicle limit or is unavailable.');
        record('curvatureLimit',g.maxAbsCurvature,curvatureLimit, ...
            g.maxAbsCurvature <= curvatureLimit,'Continuous curvature exceeds the vehicle limit or is unavailable.');
        record('turningRadiusLimit',g.minTurningRadius,vehicleRadius, ...
            g.minTurningRadius >= vehicleRadius,'Path minimum radius is below the vehicle radius or unavailable.');
        stage = 'finiteData';
        validateattributes(scenario,{'struct'},{'scalar'});
        validateattributes(scenario.targetPose,{'numeric'},{'real','finite','size',[3,1]});
        validateattributes(scenario.initialPose,{'numeric'},{'real','finite','size',[3,1]});
        count = numel(plan.segments);
        referenceFields = {'t','globalT','s','q','x','y','theta','v','omega','delta','curvature','segmentIndex'};
        simulationFields = {'t','globalT','q','v','delta','segmentIndex'};
        reference = checkedCollection(trajectory,referenceFields,count,g.tolerances.heading);
        simulated = checkedCollection(simulation,simulationFields,count,g.tolerances.heading);
        record(stage,true,true,true,'Finite arrays and consistent combined/per-segment data.');
        stage = 'timeAlignment';
        assert(isequal(reference.t,simulated.t) && isequal(reference.segmentIndex,simulated.segmentIndex), ...
            'evaluateParkingExperiment:TimeMismatch','Reference and simulation times/segment indices must match exactly.');
        duration = reference.t(end)-reference.t(1);
        assert(isfinite(duration) && duration > 0, ...
            'evaluateParkingExperiment:InvalidDuration','Total duration must be finite and positive.');
        record(stage,true,true,true,'Identical strictly increasing global grids and positive duration.');
        tol = g.tolerances;
        start = g.segments(1).startPose;
        endpoint = g.segments(end).endPose;
        target = double(scenario.targetPose);
        initial = double(scenario.initialPose);
        record('planTargetPosition',norm(endpoint(1:2)-target(1:2)),tol.position, ...
            norm(endpoint(1:2)-target(1:2)) <= tol.position,'Analytic plan endpoint does not match target position.');
        record('planTargetHeading',abs(wrapped(endpoint(3)-target(3))),tol.heading, ...
            abs(wrapped(endpoint(3)-target(3))) <= tol.heading,'Analytic plan endpoint does not match target heading.');
        record('planInitialPosition',norm(start(1:2)-initial(1:2)),tol.position, ...
            norm(start(1:2)-initial(1:2)) <= tol.position,'Plan start does not match scenario initial position.');
        record('planInitialHeading',abs(wrapped(start(3)-initial(3))),tol.heading, ...
            abs(wrapped(start(3)-initial(3))) <= tol.heading,'Plan start does not match scenario initial heading.');
        record('simulationInitialPose',poseAgrees(simulated.q(:,1),initial,tol),true, ...
            poseAgrees(simulated.q(:,1),initial,tol),'Simulation initial state differs from the scenario.');
        stage = 'referenceConsistent';
        [referenceOK,commandsOK,stopsOK,joinsOK] = inspectSegments(plan,trajectory,simulation,vehicle,tol);
        record(stage,referenceOK,true,referenceOK,'Timed reference does not match analytic plan and cubic timing.');
        record('commandsConsistent',commandsOK,true,commandsOK, ...
            'Reference or applied commands disagree with analytic bicycle inputs or exceed steering limits.');
        record('stoppedBoundaries',stopsOK,true,stopsOK,'Expected zero-speed endpoints and stopped instantaneous steering policy.');
        record('simulationJoinContinuity',joinsOK,true,joinsOK,'Actual simulated terminal state was not retained at a join.');
        stage = 'finiteMetrics';
        error = simulated.q-reference.q;
        error(3,:) = wrapped(error(3,:));
        xy = hypot(error(1,:),error(2,:));
        terminal = simulated.q(:,end)-target;
        values = [norm(terminal(1:2)),abs(wrapped(terminal(3))),max(xy), ...
            sqrt(trapz(reference.t,xy.^2)/duration)];
        assert(all(isfinite([error(:);values(:)])), ...
            'evaluateParkingExperiment:NonfiniteMetrics','Error arithmetic produced nonfinite values.');
        for k = 1:4
            name = metricNames{k};
            evaluation.metrics.(name) = values(k);
            record(name,values(k),settings.(name),values(k) <= settings.(name), ...
                'Measured error exceeds the configured POC acceptance threshold.');
        end
        evaluation.errors = struct('t',reference.t,'xError',error(1,:), ...
            'yError',error(2,:),'xyErrorMagnitude',xy,'headingError',error(3,:), ...
            'geometricPathDeviation',[]);
        stage = 'geometricDistanceConverged';
        distance = cubicPathDistance(plan.segments,simulated.q(1:2,:));
        evaluation.geometricDistance = distance;
        evaluation.errors.geometricPathDeviation = distance.distance;
        evaluation.metrics.maxGeometricPathDeviation = max(distance.distance);
        record(stage,distance.numerics.converged,true,distance.numerics.converged, ...
            'Point-to-cubic distance did not converge under polynomial root refinement.');
        m = evaluation.metrics;
        finite = all(isfinite([m.finalPositionError,m.finalHeadingError,m.maxTimeMatchedXYError, ...
            m.timeWeightedXYRMSE,m.maxGeometricPathDeviation,m.maxSteering,m.maxAbsCurvature,m.vehicleMinTurningRadius])) && ...
            (isfinite(m.minTurningRadius) || (isinf(m.minTurningRadius) && m.maxAbsCurvature == 0));
        record('finiteMetrics',finite,true,finite,'Required metrics are nonfinite or unavailable.');
    catch exception
        record(stage,false,true,false,sprintf('%s: %s',exception.identifier,exception.message));
        finish('fail');
        return
    end
    finish('fail'); % Promoted to pass only if every mandatory check passed.

    function record(name,value,limit,passed,reason)
        index = strcmp(names,name);
        if passed
            reason = 'Satisfied.';
        end
        evaluation.checks(index) = struct('name',name,'value',value,'limit',limit, ...
            'passed',logical(passed),'reason',reason);
    end
    function finish(failureStatus)
        evaluation.passed = all([evaluation.checks.passed]);
        evaluation.status = failureStatus;
        if evaluation.passed
            evaluation.status = 'pass';
            evaluation.message = 'All quantitative parking POC checks passed.';
        else
            failed = evaluation.checks(~[evaluation.checks.passed]);
            messages = arrayfun(@(c) [c.name,': ',c.reason],failed,UniformOutput=false);
            evaluation.message = strjoin(messages,'; ');
        end
    end
end

function [code,reason] = stageStatus(data,planner)
    code = 'missing'; reason = 'Required stage result/status is missing or malformed.';
    if ~isstruct(data) || ~isscalar(data) || ~isfield(data,'status')
        return
    end
    status = data.status;
    if planner
        if ~isstruct(status) || ~isscalar(status) || ~all(isfield(status,{'code','message'}))
            return
        end
        message = status.message;
        status = status.code;
    else
        if ~isfield(data,'message')
            return
        end
        message = data.message;
    end
    if ~textScalar(status) || ~textScalar(message)
        return
    end
    code = char(status);
    reason = [code,': ',char(message)];
end

function combined = checkedCollection(data,fields,count,headingTolerance)
    assert(isfield(data,'segments') && isstruct(data.segments) && ...
        isequal(size(data.segments),[1,count]), ...
        'evaluateParkingExperiment:SegmentCount','Timed/simulated segment count differs from the plan.');
    combined = combineTimedSegments(data.segments,fields);
    names = fieldnames(combined);
    for k = 1:numel(names)
        name = names{k};
        value = data.(name);
        validateattributes(value,{'double'},{'real','finite','size',size(combined.(name))});
        difference = value-combined.(name);
        if strcmp(name,'q')
            difference(3,:) = wrapped(difference(3,:));
            agrees = all(difference(1:2,:) == 0,'all') && all(abs(difference(3,:)) <= headingTolerance);
        elseif strcmp(name,'theta')
            agrees = all(abs(wrapped(difference)) <= headingTolerance);
        else
            agrees = isequal(value,combined.(name));
        end
        assert(agrees,'evaluateParkingExperiment:InconsistentArrays', ...
            'Combined and per-segment %s arrays disagree.',name);
    end
end

function [referenceOK,commandsOK,stopsOK,joinsOK] = inspectSegments(plan,timed,simulated,vehicle,tol)
    referenceOK = true; commandsOK = true; joinsOK = true;
    stopsOK = isfield(timed,'joinPolicy') && textScalar(timed.joinPolicy) && ...
        strcmp(timed.joinPolicy,'stopInstantaneousSteering');
    for k = 1:numel(plan.segments)
        a = timed.segments(k); b = simulated.segments(k);
        assert(isequal(a.t,b.t),'evaluateParkingExperiment:TimeMismatch','Segment time grids differ.');
        tau = a.t/a.t(end);
        scaling = struct('t',a.t,'s',3*tau.^2-2*tau.^3,'sdot',(6*tau-6*tau.^2)/a.t(end));
        expected = pathToInputs(plan.segments(k),scaling,vehicle);
        referenceOK = referenceOK && seriesAgrees(a.s,expected.s) && ...
            poseAgrees(a.q,[expected.x;expected.y;expected.theta],tol) && ...
            poseAgrees(a.q,[a.x;a.y;a.theta],tol);
        commandsOK = commandsOK && seriesAgrees(a.v,expected.v) && ...
            seriesAgrees(a.delta,expected.delta) && seriesAgrees(a.omega,expected.omega) && ...
            seriesAgrees(a.curvature,expected.curvature) && seriesAgrees(b.v,a.v) && ...
            seriesAgrees(b.delta,a.delta) && all(abs([a.delta,b.delta]) <= vehicle.maxSteering);
        stopsOK = stopsOK && isequal(a.s([1,end]),[0,1]) && ...
            all([a.v([1,end]),b.v([1,end])] == 0);
        if k > 1
            joinsOK = joinsOK && poseAgrees(b.q(:,1),simulated.segments(k-1).q(:,end),tol);
        end
    end
end

function yes = poseAgrees(a,b,tol)
    difference = a-b;
    yes = all(hypot(difference(1,:),difference(2,:)) <= tol.position) && ...
        all(abs(wrapped(difference(3,:))) <= tol.heading);
end
function yes = seriesAgrees(a,b)
    yes = all(abs(a-b) <= 1e-10*max(1,abs(b)));
end
function angle = wrapped(angle)
    angle = atan2(sin(angle),cos(angle));
end
function valid = textScalar(value)
    valid = (ischar(value) && isrow(value)) || ...
        (isstring(value) && isscalar(value) && ~ismissing(value));
end
