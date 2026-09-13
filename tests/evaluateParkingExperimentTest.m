classdef evaluateParkingExperimentTest < matlab.unittest.TestCase
%evaluateParkingExperimentTest Step 9 metrics, gate mutations and real scenarios.
    properties
        Experiments
        Evaluations
    end
    properties (TestParameter)
        scenarioId = {'nominal','changedStart'};
        metricName = {'finalPositionError','finalHeadingError','maxTimeMatchedXYError','timeWeightedXYRMSE'};
        failedStage = {'planner','timing','simulation'};
        badValue = {NaN,Inf,1i};
        badTime = {[0,0,1],[0,1,0.5],[0,0,0]};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fileparts(fileparts(mfilename('fullpath')))));
            ids = {'nominal','tight','changedStart'};
            for k = 1:numel(ids)
                id = ids{k}; v = vehicleParameters(); sc = scenarioParameters(v,id);
                p = planCubicParking(v,sc,plannerSettings());
                t = timeParkingPlan(p,v,timingSettings());
                s = simulateParkingPlan(t,sc,v);
                testCase.Experiments.(id) = {p,t,s,sc,v};
                testCase.Evaluations.(id) = evaluateParkingExperiment(p,t,s,sc,v,evaluationSettings());
            end
        end
    end
    methods (Test)
        function freezesOnlyFourQualityThresholds(testCase)
            testCase.verifyEqual(evaluationSettings(),struct('finalPositionError',0.05, ...
                'finalHeadingError',pi/180,'maxTimeMatchedXYError',0.05,'timeWeightedXYRMSE',0.02));
        end
        function exactFinalPositionUsesTarget(testCase)
            data = simpleExperiment([0,0.5,1]);
            data = withError(data,[0,0,0.03;0,0,0.04;0,0,0]);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.finalPositionError,0.05,AbsTol=1e-14);
            data{4}.targetPose(1:2) = data{3}.q(1:2,end);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.finalPositionError,0,AbsTol=1e-14);
            testCase.verifyFalse(checkPassed(e,'planTargetPosition'));
            testCase.verifyTrue(checkPassed(e,'finalPositionError'));
            testCase.verifyFalse(e.passed);
        end
        function wrapsHeadingAcrossPi(testCase)
            data = simpleExperiment([0,0.5,1],pi-0.001);
            data{3}.q(3,end) = -pi+0.001;
            data{3}.segments.q = data{3}.q;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.finalHeadingError,0.002,AbsTol=1e-12);
            testCase.verifyEqual(e.errors.headingError(end),0.002,AbsTol=1e-12);
            testCase.verifyTrue(e.passed,e.message);
        end
        function wholeHeadingTurnsDoNotChangeEvaluation(testCase)
            data = simpleExperiment([0,0.5,1]);
            data{3}.q(3,:) = data{3}.q(3,:)+2*pi;
            data{3}.segments.q = data{3}.q;
            data{4}.targetPose(3) = 4*pi;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyLessThan(e.metrics.finalHeadingError,1e-12);
            testCase.verifyTrue(e.passed,e.message);
        end
        function integratesKnownLinearSquaredError(testCase)
            times = [0,0.5,1];
            data = withError(simpleExperiment(times),[zeros(1,3);0.01*sqrt(times);zeros(1,3)]);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.timeWeightedXYRMSE,0.01/sqrt(2),AbsTol=1e-14);
            testCase.verifyEqual(e.errors.xyErrorMagnitude,0.01*sqrt(times),AbsTol=1e-14);
            testCase.verifyTrue(e.passed,e.message);
        end
        function integratesNonuniformGridWithoutSampleMean(testCase)
            times = [0,0.1,0.6,1];
            data = withError(simpleExperiment(times),[zeros(1,4);0,0.01,0.02,0;zeros(1,4)]);
            e = evaluateParkingExperiment(data{:});
            % Areas: 0.1*(0+.0001)/2 + .5*(.0001+.0004)/2 + .4*(.0004+0)/2.
            testCase.verifyEqual(e.metrics.timeWeightedXYRMSE,sqrt(0.00021),AbsTol=1e-14);
            testCase.verifyGreaterThan(abs(e.metrics.timeWeightedXYRMSE-sqrt(0.000125)),0.001);
            testCase.verifyTrue(e.passed,e.message);
        end
        function distinguishesTimeMatchedAndGeometricError(testCase)
            data = withError(simpleExperiment([0,0.5,1]),[0,-0.1,0;zeros(2,3)]);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.maxTimeMatchedXYError,0.1,AbsTol=1e-14);
            testCase.verifyLessThan(e.metrics.maxGeometricPathDeviation,1e-12);
            testCase.verifyFalse(checkPassed(e,'maxTimeMatchedXYError'));
            testCase.verifyFalse(e.passed);
        end
        function individualThresholdPass(testCase,metricName)
            data = thresholdExperiment(metricName,0.99);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyTrue(checkPassed(e,metricName),e.message);
            testCase.verifyTrue(e.passed,e.message);
        end
        function individualThresholdFail(testCase,metricName)
            data = thresholdExperiment(metricName,1.01);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,metricName));
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'fail');
            testCase.verifyTrue(contains(e.message,metricName));
        end
        function acceptsInclusiveXYLimit(testCase)
            data = withError(simpleExperiment([0,0.999,1]),[zeros(1,3);0,0,0.05;zeros(1,3)]);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.finalPositionError,0.05);
            testCase.verifyTrue(checkPassed(e,'finalPositionError'));
            testCase.verifyTrue(checkPassed(e,'maxTimeMatchedXYError'));
            testCase.verifyTrue(e.passed,e.message);
        end
        function evaluatesRequiredPipeline(testCase,scenarioId)
            e = testCase.Evaluations.(scenarioId);
            testCase.verifyTrue(e.passed,e.message);
            testCase.verifyEqual(e.status,'pass');
            testCase.verifyTrue(all([e.checks.passed]));
            testCase.verifyEqual(e.metrics.maxSteering,e.geometryValidation.maxAbsSteering);
            testCase.verifyEqual(e.metrics.maxAbsCurvature,e.geometryValidation.maxAbsCurvature);
            testCase.verifyEqual(e.metrics.minTurningRadius,e.geometryValidation.minTurningRadius);
            testCase.verifyLessThanOrEqual(e.metrics.maxGeometricPathDeviation,e.metrics.maxTimeMatchedXYError+1e-9);
            testCase.verifySize(e.errors.xError,[1,281]);
            testCase.verifyTrue(e.geometricDistance.numerics.converged);
            printEvaluation(scenarioId,e);
        end
        function tightRetainsUnavailableMetricsAndReason(testCase)
            e = testCase.Evaluations.tight;
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'not_evaluated');
            testCase.verifyFalse(checkPassed(e,'plannerFeasible'));
            testCase.verifyTrue(all(isnan(struct2array(rmfield(e.metrics,'vehicleMinTurningRadius')))));
            testCase.verifyEqual(e.metrics.vehicleMinTurningRadius,vehicleParameters().minTurningRadius,AbsTol=1e-14);
            testCase.verifyEmpty(e.errors.t);
            testCase.verifyTrue(contains(e.message,'not_found'));
            testCase.verifyTrue(contains(e.message,'not proof of physical impossibility'));
        end
        function failedStagesCannotPass(testCase,failedStage)
            data = failStage(simpleExperiment([0,0.5,1]),failedStage);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'not_evaluated');
            testCase.verifyTrue(isnan(e.metrics.finalPositionError));
            testCase.verifyEmpty(e.errors.xError);
            testCase.verifyTrue(contains(e.message,'Deliberate stage failure'));
        end
        function missingPlannerCannotFabricateZeros(testCase)
            data = simpleExperiment([0,0.5,1]); data{1} = [];
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'not_evaluated');
            testCase.verifyTrue(isnan(e.metrics.finalPositionError));
            testCase.verifyFalse(checkPassed(e,'plannerFeasible'));
        end
        function emptyFeasiblePlanCannotPass(testCase)
            data = simpleExperiment([0,0.5,1]); data{1}.segments = struct([]);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyTrue(isnan(e.metrics.finalPositionError));
            testCase.verifyFalse(checkPassed(e,'geometryValid'));
        end
        function continuousSteeringAndCurvatureCannotBeHiddenBySamples(testCase)
            data = simpleExperiment([0,0.5,1]);
            limit = tan(data{5}.maxSteering)/data{5}.wheelbase;
            peak = 0.3725; kappa = limit*(1+1e-8);
            data{1}.segments = segmentFromCoefficients([0,0,1,0;0,kappa/2,-kappa*peak,kappa*peak^2/2]);
            % Even 201 caches miss the violation; forged prior PASS is ignored.
            curve = evaluateCubicPath(data{1}.segments.coefficients,linspace(0,1,201),1);
            data{1}.diagnostics.validation = struct('passed',true);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyLessThan(max(abs(curve.curvature)),limit);
            testCase.verifyGreaterThan(e.metrics.maxAbsCurvature,limit);
            testCase.verifyGreaterThan(e.metrics.maxSteering,data{5}.maxSteering);
            testCase.verifyFalse(checkPassed(e,'geometryValid'));
            testCase.verifyFalse(checkPassed(e,'curvatureLimit'));
            testCase.verifyFalse(checkPassed(e,'steeringLimit'));
            testCase.verifyFalse(checkPassed(e,'turningRadiusLimit'));
            testCase.verifyFalse(e.passed);
        end
        function planHeadingAndSimulationHeadingAreSeparate(testCase)
            data = simpleExperiment([0,0.5,1]);
            data{4}.targetPose(3) = 0.001;
            data{3}.q(3,end) = 0.001; data{3}.segments.q = data{3}.q;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,'planTargetHeading'));
            testCase.verifyTrue(checkPassed(e,'finalHeadingError'));
            testCase.verifyFalse(e.passed);
        end
        function invalidFiniteDataFailsSafely(testCase,badValue)
            data = simpleExperiment([0,0.5,1]); data{3}.q(1,2) = badValue;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'fail');
            testCase.verifyFalse(checkPassed(e,'finiteData'));
            testCase.verifyTrue(isnan(e.metrics.finalPositionError));
        end
        function invalidGlobalTimesFailSafely(testCase,badTime)
            data = simpleExperiment([0,0.5,1]); data{2}.segments.t = badTime;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'fail');
            testCase.verifyTrue(isnan(e.metrics.timeWeightedXYRMSE));
        end
        function mismatchedTimesAndArrayLengthsFail(testCase)
            data = simpleExperiment([0,0.5,1]); data{3}.q = data{3}.q(:,1:2);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.passed);
            testCase.verifyFalse(checkPassed(e,'finiteData'));
            data = simpleExperiment([0,0.5,1]);
            data{3}.t(2) = 0.4; data{3}.globalT = data{3}.t; data{3}.localT = data{3}.t;
            data{3}.segments.t = data{3}.t; data{3}.segments.globalT = data{3}.t;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,'timeAlignment'));
            testCase.verifyTrue(isnan(e.metrics.timeWeightedXYRMSE));
        end
        function badReferenceCannotPassWithMatchingSimulation(testCase)
            data = simpleExperiment([0,0.5,1]);
            data{2}.q(2,2) = 0.1; data{2}.y(2) = 0.1;
            data{2}.segments.q = data{2}.q; data{2}.segments.y = data{2}.y;
            data{3}.q = data{2}.q; data{3}.segments.q = data{3}.q;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.maxTimeMatchedXYError,0);
            testCase.verifyFalse(checkPassed(e,'referenceConsistent'));
            testCase.verifyFalse(e.passed);
        end
        function appliedSteeringViolationFailsEvenOnStraightGeometry(testCase)
            data = simpleExperiment([0,0.5,1]);
            data{3}.delta(2) = 0.7; data{3}.segments.delta = data{3}.delta;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyTrue(checkPassed(e,'steeringLimit')); % Geometric requirement remains zero.
            testCase.verifyFalse(checkPassed(e,'commandsConsistent'));
            testCase.verifyFalse(e.passed);
        end
        function invalidSettingsFailSafely(testCase)
            data = simpleExperiment([0,0.5,1]); settings = evaluationSettings();
            settings.finalPositionError = NaN;
            e = evaluateParkingExperiment(data{:},settings);
            testCase.verifyFalse(checkPassed(e,'settingsValid'));
            testCase.verifyFalse(e.passed);
            testCase.verifyEqual(e.status,'fail');
        end
        function malformedVehicleAndTargetFailSafely(testCase)
            data = simpleExperiment([0,0.5,1]); data{5}.wheelbase = 0;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,'vehicleValid'));
            testCase.verifyFalse(e.passed);
            data = simpleExperiment([0,0.5,1]); data{4}.targetPose(1) = NaN;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,'finiteData'));
            testCase.verifyFalse(e.passed);
        end
        function straightPathInfiniteRadiusIsValid(testCase)
            data = simpleExperiment([0,0.5,1]); e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e.metrics.minTurningRadius,Inf);
            testCase.verifyEqual(e.metrics.maxAbsCurvature,0,AbsTol=1e-14);
            testCase.verifyTrue(e.passed,e.message);
        end
        function frameMetadataDoesNotChangeEvaluation(testCase,scenarioId)
            data = testCase.Experiments.(scenarioId);
            data{4}.framePoseWorld = [50;-20;2.7];
            e = evaluateParkingExperiment(data{:});
            testCase.verifyEqual(e,testCase.Evaluations.(scenarioId));
        end
        function brokenGeometricJoinCannotPass(testCase)
            data = testCase.Experiments.nominal;
            data{1}.segments(2).coefficients(1,4) = data{1}.segments(2).coefficients(1,4)+0.01;
            data{1}.segments(2).startPose(1) = data{1}.segments(2).startPose(1)+0.01;
            data{1}.segments(2).endPose(1) = data{1}.segments(2).endPose(1)+0.01;
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(e.geometryValidation.joins.passed);
            testCase.verifyFalse(checkPassed(e,'geometryValid'));
            testCase.verifyFalse(e.passed);
        end
        function detectsResetAtSimulationJoin(testCase)
            data = testCase.Experiments.nominal;
            data{3}.segments(2).q(:,1) = data{1}.segments(2).startPose;
            data{3}.q(:,141) = data{3}.segments(2).q(:,1);
            e = evaluateParkingExperiment(data{:});
            testCase.verifyFalse(checkPassed(e,'simulationJoinContinuity'));
            testCase.verifyFalse(e.passed);
        end
    end
end

function data = simpleExperiment(times,heading)
% Analytic reference with an exact matching synthetic simulation. Metrics
% tests may perturb states; these are test data, not claimed Euler outputs.
    if nargin < 2
        heading = 0;
    end
    v = vehicleParameters();
    start = [0;0;heading]; target = [cos(heading);sin(heading);heading];
    p = struct('segments',cubicPath(start,target,1,1,2,1), ...
        'status',struct('code','feasible','message','Synthetic metric fixture.'),'diagnostics',struct());
    tau = times/times(end);
    scaling = struct('t',times,'s',3*tau.^2-2*tau.^3,'sdot',(6*tau-6*tau.^2)/times(end));
    a = pathToInputs(p.segments,scaling,v); a.q = [a.x;a.y;a.theta];
    a.globalT = a.t; a.segmentIndex = ones(size(times));
    t = a; t.localT = a.t; t.segments = a;
    t.status = 'success'; t.message = 'Synthetic timing.'; t.joinPolicy = 'stopInstantaneousSteering';
    b = struct('t',a.t,'globalT',a.t,'q',a.q,'v',a.v,'delta',a.delta,'segmentIndex',a.segmentIndex);
    s = b; s.localT = a.t; s.segments = b; s.status = 'success'; s.message = 'Synthetic states.';
    sc = struct('initialPose',start,'targetPose',target,'framePoseWorld',[0;0;0]);
    data = {p,t,s,sc,v};
end
function data = withError(data,error)
    data{3}.q = data{3}.q+error;
    data{3}.segments.q = data{3}.q;
end
function data = thresholdExperiment(name,factor)
    settings = evaluationSettings(); amplitude = factor*settings.(name);
    switch name
        case 'finalPositionError'
            data = withError(simpleExperiment([0,0.999,1]),[zeros(1,3);0,0,amplitude;zeros(1,3)]);
        case 'finalHeadingError'
            data = withError(simpleExperiment([0,0.5,1]),[zeros(2,3);0,0,amplitude]);
        case 'maxTimeMatchedXYError'
            data = withError(simpleExperiment([0,0.49,0.5,0.51,1]), ...
                [zeros(1,5);0,0,amplitude,0,0;zeros(1,5)]);
        case 'timeWeightedXYRMSE'
            data = withError(simpleExperiment([0,0.1,0.9,1]), ...
                [zeros(1,4);0,amplitude/sqrt(0.9),amplitude/sqrt(0.9),0;zeros(1,4)]);
    end
end
function data = failStage(data,name)
    switch name
        case 'planner'
            data{1}.status = struct('code','not_found','message','Deliberate stage failure.');
        case 'timing'
            data{2}.status = 'failed'; data{2}.message = 'Deliberate stage failure.';
        case 'simulation'
            data{3}.status = 'failed'; data{3}.message = 'Deliberate stage failure.';
    end
end
function segment = segmentFromCoefficients(c)
    g = evaluateCubicPath(c,[0,1],1);
    segment = struct('coefficients',c,'direction',1, ...
        'startPose',[g.x(1);g.y(1);g.thetaBody(1)],'endPose',[g.x(end);g.y(end);g.thetaBody(end)]);
end
function passed = checkPassed(e,name)
    passed = e.checks(strcmp({e.checks.name},name)).passed;
end
function printEvaluation(id,e)
    fprintf('\n%s Step 9: %s\n',id,e.status);
    names = fieldnames(e.metrics);
    for k = 1:numel(names)
        fprintf('  %s = %.16g\n',names{k},e.metrics.(names{k}));
    end
    fprintf('  All %d mandatory checks passed: %d\n',numel(e.checks),all([e.checks.passed]));
    fprintf('  Root refinement distance difference = %.16g m\n',e.geometricDistance.numerics.maxRefinementDifference);
end
