classdef planCubicParkingTest < matlab.unittest.TestCase
%planCubicParkingTest Shared automatic search, geometric acceptance and failures.
% Class setup runs the three required searches once; assertions reuse those
% results. Repeated/world-transform tests deliberately make fresh calls.
% No expected optimized midpoint/tangents are frozen: improvements are allowed.
    properties
        Vehicle
        Settings
        Scenarios
        Plans
    end
    properties (TestParameter)
        scenarioId = {'nominal','changedStart'};
        allScenarioId = {'nominal','tight','changedStart'};
        badScenario = struct('missing',struct('mode','remove','field','initialPose','value',[]), ...
            'nan',struct('mode','set','field','initialPose','value',[NaN;2.8;0]), ...
            'row',struct('mode','set','field','targetPose','value',[5.5,0,0]), ...
            'inconsistent',struct('mode','set','field','targetPose','value',[5.6;0;0]), ...
            'frame',struct('mode','set','field','framePoseWorld','value',[0;Inf;0]), ...
            'identifier',struct('mode','set','field','id','value','nomnal'), ...
            'gap',struct('mode','set','field','spaceLength','value',-1));
        badVehicle = struct('nanWheelbase',struct('field','wheelbase','value',NaN), ...
            'inconsistentLength',struct('field','length','value',5), ...
            'negativeWidth',struct('field','width','value',-1), ...
            'invalidSteering',struct('field','maxSteering','value',pi/2), ...
            'nonfiniteRadius',struct('field','minTurningRadius','value',NaN), ...
            'inconsistentRadius',struct('field','minTurningRadius','value',1));
        badSettings = struct('direction',struct('field','directionPattern','value',[-1,1]), ...
            'solver',struct('field','solver','value','fminsearch'), ...
            'tolerances',struct('field','feasibilityTolerances','value',1), ...
            'objective',struct('field','objectiveWeights','value',struct('length',0)));
    end
    methods (TestClassSetup)
        function preparePlans(testCase)
            folder = fileparts(mfilename('fullpath'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(folder)));
            testCase.Vehicle = vehicleParameters();
            testCase.Settings = plannerSettings();
            ids = {'nominal','tight','changedStart'};
            for k = 1:numel(ids)
                scenario = scenarioParameters(testCase.Vehicle,ids{k});
                testCase.Scenarios.(ids{k}) = scenario;
                testCase.Plans.(ids{k}) = planCubicParking(testCase.Vehicle,scenario,testCase.Settings);
            end
        end
    end
    methods (Test)
        function independentlyValidatesRequiredScenario(testCase,scenarioId)
            plan = testCase.Plans.(scenarioId);
            testCase.assertEqual(plan.status.code,'feasible',plan.status.message);
            result = validateCubicGeometry(plan.segments,testCase.Vehicle,'stopped');
            testCase.verifyTrue(result.passed,result.message);
            testCase.verifyEqual(result,plan.diagnostics.validation);
            testCase.verifyLessThanOrEqual(result.maxAbsSteering,testCase.Vehicle.maxSteering);
            testCase.verifyLessThanOrEqual(result.maxAbsCurvature, ...
                tan(testCase.Vehicle.maxSteering)/testCase.Vehicle.wheelbase);
            testCase.verifyGreaterThanOrEqual(result.minTurningRadius,testCase.Vehicle.minTurningRadius);
            testCase.verifyEqual([plan.segments.direction],[-1,-1]);
            testCase.verifyTrue(all([result.segments.minTangent] > [result.segments.tangentThreshold]));
        end
        function satisfiesRequiredPosesAndStoppedJoin(testCase,scenarioId)
            plan = testCase.Plans.(scenarioId);
            scenario = testCase.Scenarios.(scenarioId);
            testCase.assertEqual(plan.status.code,'feasible',plan.status.message);
            testCase.verifySize(plan.segments,[1,2]);
            testCase.verifyEqual(plan.segments(1).startPose,scenario.initialPose,AbsTol=1e-12);
            testCase.verifyEqual(plan.segments(2).endPose,scenario.targetPose,AbsTol=1e-12);
            testCase.verifyEqual(plan.segments(1).endPose,plan.segments(2).startPose,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(plan.diagnostics.validation.joins.positionError,1e-8);
            testCase.verifyLessThanOrEqual(plan.diagnostics.validation.joins.headingError,1e-10);
            testCase.verifyEqual(plan.diagnostics.validation.joins.type,"stopped");
            testCase.verifyFalse(any(isfield(plan.segments,{'duration','dt','v','delta'})));
        end
        function reportsSearchAndStableLength(testCase,scenarioId)
            plan = testCase.Plans.(scenarioId);
            testCase.assertEqual(plan.status.code,'feasible',plan.status.message);
            expectedCount = numel(testCase.Settings.initialization.chordFractions)* ...
                numel(testCase.Settings.initialization.headingFactors);
            testCase.verifyEqual(plan.diagnostics.initialGuessCount,expectedCount);
            testCase.verifySize(plan.diagnostics.attempts,[1,expectedCount]);
            testCase.verifyTrue(all(isfinite([plan.diagnostics.attempts.exitflag])));
            testCase.verifyEqual(plan.diagnostics.objectiveValue, ...
                sum(cubicPathLength(plan.segments,1e-12,1e-12)),AbsTol=1e-8);
            testCase.verifyEqual(plan.diagnostics.maxConstraintViolation,0,AbsTol=eps);
            testCase.verifyGreaterThan(plan.diagnostics.objectiveValue,0);
            testCase.verifySize(plan.diagnostics.selectedParameters,[7,1]);
            testCase.verifyEqual(plan.diagnostics.selectedParameters(1:3),plan.segments(1).endPose,AbsTol=1e-12);
            testCase.verifyEqual(plan.diagnostics.selectedParameters(4:7), ...
                [hypot(plan.segments(1).dxds,plan.segments(1).dyds), ...
                hypot(plan.segments(2).dxds,plan.segments(2).dyds)].',AbsTol=1e-11);
            printEvidence(scenarioId,plan,testCase.Vehicle);
        end
        function initializesFromGeometryWithoutHistoricalPlanner(testCase,allScenarioId)
            scenario = testCase.Scenarios.(allScenarioId);
            attempts = testCase.Plans.(allScenarioId).diagnostics.attempts;
            fraction = testCase.Settings.initialization.chordFractions(1);
            midpoint = (1-fraction)*scenario.initialPose(1:2)+fraction*scenario.targetPose(1:2);
            chordHeading = atan2(scenario.initialPose(2)-scenario.targetPose(2), ...
                scenario.initialPose(1)-scenario.targetPose(1));
            seed = attempts(1).initialParameters;
            testCase.verifyEqual(seed(1:3),[midpoint;chordHeading],AbsTol=1e-12);
            testCase.verifyEqual(seed(4:7),[repmat(norm(midpoint-scenario.initialPose(1:2)),2,1); ...
                repmat(norm(midpoint-scenario.targetPose(1:2)),2,1)],AbsTol=1e-12);
            source = [fileread(which('planCubicParking')),fileread(which('plannerSettings'))];
            testCase.verifyEmpty(regexp(source,'8\.63|48\.19|3\.59|legacyNominalFixture\(','once'));
            testCase.verifyEmpty(regexp(source,'scenario1Path\s*\(','once'));
        end
        function repeatsDeterministically(testCase)
            repeated = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,testCase.Settings);
            original = testCase.Plans.nominal;
            testCase.assertEqual(repeated.status.code,'feasible');
            % Same MATLAB environment, deterministic SQP and start ordering.
            testCase.verifyEqual(repeated.diagnostics.selectedParameters, ...
                original.diagnostics.selectedParameters,AbsTol=1e-10);
            testCase.verifyEqual(repeated.diagnostics.objectiveValue,original.diagnostics.objectiveValue,AbsTol=1e-10);
        end
        function ignoresWorldPlacement(testCase)
            scenario = scenarioParameters(testCase.Vehicle,'nominal',[123.4;-45.6;2.1]);
            placed = planCubicParking(testCase.Vehicle,scenario,testCase.Settings);
            original = testCase.Plans.nominal;
            testCase.assertEqual(placed.status.code,'feasible');
            testCase.verifyEqual(placed.diagnostics.selectedParameters,original.diagnostics.selectedParameters,AbsTol=1e-10);
            testCase.verifyEqual([placed.segments.coefficients],[original.segments.coefficients],AbsTol=1e-10);
        end
        function doesNotForceChangedStartSymmetry(testCase)
            plan = testCase.Plans.changedStart;
            testCase.assertEqual(plan.status.code,'feasible');
            p = plan.diagnostics.selectedParameters;
            testCase.verifyGreaterThan(norm(p(4:5)-flipud(p(6:7))),1e-3);
        end
        function comparesHistoricalGeometryWithoutReplacingIt(testCase)
            f = legacyNominalFixture();
            historical = [cubicPath(f.scenario.initialPose,f.midPose,f.ks,f.ke,2,-1), ...
                cubicPath(f.midPose,f.scenario.targetPose,f.ks,f.ke,2,-1)];
            plan = testCase.Plans.nominal;
            testCase.assertEqual(plan.status.code,'feasible');
            testCase.verifyLessThanOrEqual(plan.diagnostics.totalLength,sum(cubicPathLength(historical)));
            testCase.verifyGreaterThan(norm(plan.segments(1).endPose-f.midPose),1e-3);
        end
        function tightSearchReportsUnresolvedCurvatureConstraint(testCase)
            plan = testCase.Plans.tight;
            testCase.verifyEqual(plan.status.code,'not_found',plan.status.message);
            testCase.verifyEmpty(plan.segments);
            testCase.verifyEqual(plan.diagnostics.initialGuessCount,6);
            testCase.verifyFalse(any([plan.diagnostics.attempts.validationPassed]));
            testCase.verifyEqual([plan.diagnostics.attempts.validatedCandidateCount],zeros(1,6));
            testCase.verifyTrue(all([plan.diagnostics.attempts.maxAbsCurvature] > ...
                tan(testCase.Vehicle.maxSteering)/testCase.Vehicle.wheelbase));
            testCase.verifyTrue(all(contains({plan.diagnostics.attempts.rejectionReason},'curvature')));
            testCase.verifyTrue(contains(plan.status.message,'not proof of physical impossibility'));
            fprintf('tight: %s; best terminal max curvature=%.12g 1/m, minimum required steering=%.12g deg; all %d starts rejected\n', ...
                plan.status.code,min([plan.diagnostics.attempts.maxAbsCurvature]), ...
                rad2deg(min([plan.diagnostics.attempts.maxAbsSteering])),plan.diagnostics.initialGuessCount);
        end
        function rejectsMalformedScenario(testCase,badScenario)
            scenario = modifyScenario(testCase.Scenarios.nominal,badScenario);
            plan = planCubicParking(testCase.Vehicle,scenario,testCase.Settings);
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
            testCase.verifyNotEmpty(plan.status.message);
        end
        function rejectsMalformedVehicle(testCase,badVehicle)
            vehicle = testCase.Vehicle;
            vehicle.(badVehicle.field) = badVehicle.value;
            plan = planCubicParking(vehicle,testCase.Scenarios.nominal,testCase.Settings);
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
        end
        function rejectsUnsupportedSettings(testCase,badSettings)
            settings = testCase.Settings;
            settings.(badSettings.field) = badSettings.value;
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
        end
        function rejectsMalformedSearchData(testCase)
            settings = testCase.Settings;
            settings.initialization.chordFractions = NaN;
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
        end
        function rejectsUnrecognizedNestedSettings(testCase)
            settings = testCase.Settings;
            settings.bounds.worldOffset = 100;
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
        end
        function rejectsMissingArguments(testCase)
            plan = planCubicParking();
            testCase.verifyEqual(plan.status.code,'invalid_input');
            testCase.verifyEmpty(plan.segments);
        end
        function reportsLimitedSearchFailure(testCase)
            settings = testCase.Settings;
            settings.bounds.heading = [-0.001,0.001];
            settings.solverOptions.MaxIterations = 1;
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyEqual(plan.status.code,'not_found');
            testCase.verifyEmpty(plan.segments);
            testCase.verifyFalse(any([plan.diagnostics.attempts.validationPassed]));
            testCase.verifyTrue(all(~cellfun(@isempty,{plan.diagnostics.attempts.rejectionReason})));
        end
        function solverSuccessCannotOverrideIndependentFailure(testCase)
            settings = testCase.Settings;
            settings.bounds.heading = [-0.001,0.001];
            % Deliberately meaningless solver stopping tolerances: production
            % physical acceptance must remain strict even if SQP says success.
            settings.solverOptions.ConstraintTolerance = 1e9;
            settings.solverOptions.OptimalityTolerance = 1e9;
            settings.solverOptions.MaxIterations = 1;
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyTrue(any([plan.diagnostics.attempts.exitflag] > 0));
            testCase.verifyEqual(plan.status.code,'not_found');
            testCase.verifyEmpty(plan.segments);
            testCase.verifyFalse(any([plan.diagnostics.attempts.validationPassed]));
            testCase.verifyTrue(any(contains({plan.diagnostics.attempts.rejectionReason},'curvature')));
        end
        function returnsStructuredSolverException(testCase)
            settings = testCase.Settings;
            % Valid MATLAB option, incompatible with nonlinear constraints
            % and finite-difference derivatives: fmincon itself must reject it.
            settings.solverOptions.Algorithm = 'trust-region-reflective';
            plan = planCubicParking(testCase.Vehicle,testCase.Scenarios.nominal,settings);
            testCase.verifyEqual(plan.status.code,'solver_error');
            testCase.verifyEmpty(plan.segments);
            testCase.verifyTrue(all(~cellfun(@isempty,{plan.diagnostics.attempts.solverError})));
        end
    end
end

function scenario = modifyScenario(scenario,change)
    if strcmp(change.mode,'remove')
        scenario = rmfield(scenario,change.field);
    else
        scenario.(change.field) = change.value;
    end
end

function printEvidence(id,plan,vehicle)
    d = plan.diagnostics;
    r = d.validation;
    fprintf('%s: %s; parameters=%s; length=%.12g m; kappa=%.12g 1/m; radius=%.12g m; steering=%.12g deg; margin=%.12g deg; exit=%g; objective=%.12g; validated=%d\n', ...
        id,plan.status.code,mat2str(d.selectedParameters.',12),d.totalLength, ...
        r.maxAbsCurvature,r.minTurningRadius,rad2deg(r.maxAbsSteering), ...
        rad2deg(vehicle.maxSteering-r.maxAbsSteering),d.exitflag,d.objectiveValue,r.passed);
end
