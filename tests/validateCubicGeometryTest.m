classdef validateCubicGeometryTest < matlab.unittest.TestCase
%validateCubicGeometryTest Complete-segment geometry and declared join checks.
% Synthetic parabolas have known global curvature maxima, independent of the
% validator's polynomial root search. No timing or simulation is needed.

    properties (TestParameter)
        boundary = struct('below',struct('factor',1-1e-6,'passed',true), ...
            'exact',struct('factor',1,'passed',true), ...
            'above',struct('factor',1+1e-6,'passed',false));
        peak = struct('start',0,'interior',0.3725,'end',1);
        degenerate = struct('constant',zeros(2,4), ...
            'small',[0,0,1e-12,0;0,0,0,0], ...
            'hiddenCusp',[0,1,-0.746,0;0,0,0,0], ...
            'hiddenNearCusp',[0,1,-0.746,0;0,0,1e-12,0], ...
            'doubleRoot',[1/3,-0.371,0.371^2,0;0,0,0,0]);
        endpoint = struct('startPosition',struct('field','startPose','row',1), ...
            'endPosition',struct('field','endPose','row',2), ...
            'startHeading',struct('field','startPose','row',3), ...
            'endHeading',struct('field','endPose','row',3));
        invalidDirection = struct('zero',0,'fraction',0.5,'tooLarge',2, ...
            'nan',NaN,'text','reverse','logical',true);
        invalidData = struct('nanCoefficients',struct('field','coefficients','value',nan(2,4)), ...
            'infiniteCoefficients',struct('field','coefficients','value',inf(2,4)), ...
            'wrongCoefficientSize',struct('field','coefficients','value',ones(4,2)), ...
            'complexCoefficients',struct('field','coefficients','value',1i*ones(2,4)), ...
            'nanStart',struct('field','startPose','value',[NaN;0;0]), ...
            'infiniteEnd',struct('field','endPose','value',[0;Inf;0]), ...
            'rowPose',struct('field','startPose','value',[0,0,0]), ...
            'nanCache',struct('field','curvature','value',NaN));
        invalidVehicle = struct('zeroWheelbase',struct('wheelbase',0,'maxSteering',0.6), ...
            'nanWheelbase',struct('wheelbase',NaN,'maxSteering',0.6), ...
            'negativeLimit',struct('wheelbase',2.7,'maxSteering',-0.6), ...
            'rightAngle',struct('wheelbase',2.7,'maxSteering',pi/2), ...
            'infiniteLimit',struct('wheelbase',2.7,'maxSteering',Inf), ...
            'missingLimit',struct('wheelbase',2.7), ...
            'overflowLimit',struct('wheelbase',realmin/100,'maxSteering',pi/2-eps));
        badJoins = struct('unknown',"stop",'wrongCount',["stopped","moving"], ...
            'numeric',1,'missing',string(missing));
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            folder = fileparts(mfilename('fullpath'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(folder)));
        end
    end

    methods (Test)
        function acceptsNominalAnalyticGeometry(testCase)
            f = legacyNominalFixture();
            a = cubicPath(f.scenario.initialPose,f.midPose,f.ks,f.ke,f.nSamples,-1);
            b = cubicPath(f.midPose,f.scenario.targetPose,f.ks,f.ke,f.nSamples,-1);
            result = validateCubicGeometry([a,b],vehicleParameters());
            testCase.assertTrue(result.passed,result.message);
            testCase.verifyEqual(result.code,'feasible');
            testCase.verifySize(result.segments,[1,2]);
            testCase.verifyTrue(result.joins.passed);
            % Prior audit estimates are a sanity check, not tuned constants.
            testCase.verifyEqual(result.maxAbsCurvature,0.238655,AbsTol=1e-6);
            testCase.verifyEqual(result.minTurningRadius,4.19014,AbsTol=1e-5);
            testCase.verifyEqual(rad2deg(result.maxAbsSteering),32.7965,AbsTol=1e-4);
            testCase.verifyEqual(result.vehicleMinTurningRadius,3.856,AbsTol=1e-6);
            testCase.verifyGreaterThan(result.minTurningRadius,result.vehicleMinTurningRadius);
            fprintf('Nominal geometry: kappa=%.17g 1/m, radius=%.17g m, steering=%.17g rad (%.12g deg), vehicleRadius=%.17g m\n', ...
                result.maxAbsCurvature,result.minTurningRadius,result.maxAbsSteering, ...
                rad2deg(result.maxAbsSteering),result.vehicleMinTurningRadius);
        end

        function straightSegmentHasInfiniteRadius(testCase)
            segment = makeSegment([0,0,3,1;0,0,4,-2],1);
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.assertTrue(result.passed,result.message);
            testCase.verifyEqual(result.maxAbsCurvature,0,AbsTol=1e-15);
            testCase.verifyEqual(result.maxAbsSteering,0,AbsTol=1e-15);
            testCase.verifyEqual(result.minTurningRadius,Inf);
            testCase.verifyEqual(result.segments.minTangent,5,AbsTol=1e-13);
        end

        function enforcesKnownCurvatureMaximum(testCase,boundary,peak)
            vehicle = vehicleParameters();
            limit = tan(vehicle.maxSteering)/vehicle.wheelbase;
            kappa = limit*boundary.factor;
            % x=s, y=kappa/2*(s-peak)^2: max |curvature|=kappa at peak.
            segment = parabola(kappa,peak);
            result = validateCubicGeometry(segment,vehicle);
            testCase.verifyEqual(result.passed,boundary.passed,result.message);
            testCase.verifyEqual(result.maxAbsCurvature,kappa,AbsTol=2e-13);
            testCase.verifyEqual(result.segments.maxCurvatureS,peak,AbsTol=1e-12);
            testCase.verifyEqual(result.minTurningRadius,1/kappa,AbsTol=2e-12);
            testCase.verifyEqual(result.maxAbsSteering,atan(vehicle.wheelbase*kappa),AbsTol=1e-13);
            testCase.verifyEqual(result.curvatureLimit,limit,AbsTol=1e-15);
        end

        function detectsPeakMissedEvenBy201Samples(testCase)
            vehicle = vehicleParameters();
            limit = tan(vehicle.maxSteering)/vehicle.wheelbase;
            peakS = 0.3725;
            segment = parabola(limit*(1+1e-8),peakS);
            cache = evaluateCubicPath(segment.coefficients,linspace(0,1,201),1);
            segment.s = cache.s;
            segment.curvature = cache.curvature;
            result = validateCubicGeometry(segment,vehicle);
            testCase.verifyLessThan(max(abs(cache.curvature)),limit);
            testCase.verifyFalse(result.passed);
            testCase.verifyGreaterThan(result.maxAbsCurvature,limit);
            testCase.verifyEqual(result.segments.maxCurvatureS,peakS,AbsTol=1e-12);
            testCase.verifyTrue(hasFailure(result,'curvatureLimit'));
        end

        function ignoresCoarseAndStaleCaches(testCase)
            segment = parabola(0.2,0.37);
            original = validateCubicGeometry(segment,vehicleParameters());
            segment.s = [0,1];
            segment.x = [100,100];
            segment.curvature = [0,0];
            segment.thetaBody = [0,0];
            coarse = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyEqual(coarse,original);
        end

        function rejectsDegenerateTangents(testCase,degenerate)
            segment = makeSegment(degenerate,1);
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(hasFailure(result,'regularTangent'));
            testCase.verifyTrue(isnan(result.maxAbsCurvature));
        end

        function rejectsEndpointMismatch(testCase,endpoint)
            segment = parabola(0.1,0);
            segment.(endpoint.field)(endpoint.row) = segment.(endpoint.field)(endpoint.row)+0.01;
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.code,'infeasible');
        end

        function acceptsEndpointToleranceAndWrappedHeading(testCase)
            segment = makeSegment([0,0,1,0;0,0,0,0],1);
            segment.startPose(1) = 5e-9;
            segment.endPose(3) = 2*pi+5e-11;
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyTrue(result.passed,result.message);
        end

        function rejectsJoinPositionMismatch(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1.01;0,0,0,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(all([result.segments.passed]));
            testCase.verifyTrue(hasFailure(result,'joinPosition'));
        end

        function rejectsJoinHeadingMismatch(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0,0.1,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(all([result.segments.passed]));
            testCase.verifyTrue(hasFailure(result,'joinHeading'));
        end

        function stoppedJoinAllowsCurvatureJump(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0.05,0,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters(),"stopped");
            testCase.verifyTrue(result.passed,result.message);
            testCase.verifyEqual(result.joins.curvatureJump,0.1,AbsTol=1e-14);
            testCase.verifyGreaterThan(result.joins.steeringJump,0);
        end

        function stoppedJoinRejectsExcessiveOneSidedCurvature(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0.2,0,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters(),"stopped");
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(hasFailure(result,'oneSidedCurvature'));
            testCase.verifyTrue(hasFailure(result,'oneSidedSteering'));
        end

        function movingJoinRejectsCurvatureJump(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0.05,0,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters(),"moving");
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(all([result.segments.passed]));
            testCase.verifyTrue(hasFailure(result,'movingCurvature'));
        end

        function movingJoinAcceptsContinuousCurvature(testCase)
            a = makeSegment([0,0,1,0;0,0.05,0,0],1);
            b = makeSegment([0,0,1,1;0,0.05,0.1,0.05],1);
            result = validateCubicGeometry([a,b],vehicleParameters(),"moving");
            testCase.verifyTrue(result.passed,result.message);
            testCase.verifyEqual(result.joins.curvatureJump,0,AbsTol=1e-14);
        end

        function movingJoinChecksSignedCurvature(testCase)
            a = makeSegment([0,0,1,0;0,0.05,0,0],1);
            b = makeSegment([0,0,1,1;0,-0.05,0.1,0.05],1);
            stopped = validateCubicGeometry([a,b],vehicleParameters(),"stopped");
            moving = validateCubicGeometry([a,b],vehicleParameters(),"moving");
            testCase.verifyTrue(stopped.passed,stopped.message);
            testCase.verifyFalse(moving.passed);
            testCase.verifyTrue(hasFailure(moving,'movingCurvature'));
            testCase.verifyEqual(moving.joins.curvatureJump,0.2/(1.01^1.5),AbsTol=1e-13);
        end

        function onlyStoppedJoinsAllowDirectionReversal(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,-1,1;0,0,0,0],-1);
            stopped = validateCubicGeometry([a,b],vehicleParameters(),"stopped");
            moving = validateCubicGeometry([a,b],vehicleParameters(),"moving");
            testCase.verifyTrue(stopped.passed,stopped.message);
            testCase.verifyFalse(moving.passed);
            testCase.verifyTrue(hasFailure(moving,'movingDirection'));
        end

        function supportsVariableLengthAndMixedJoins(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0,0,0],1);
            c = makeSegment([0,0,-1,2;0,0,0,0],-1);
            result = validateCubicGeometry([a;b;c],vehicleParameters(),["moving","stopped"]);
            testCase.verifyTrue(result.passed,result.message);
            testCase.verifySize(result.segments,[1,3]);
            testCase.verifySize(result.joins,[1,2]);
        end

        function rejectsInvalidDirections(testCase,invalidDirection)
            segment = parabola(0.1,0);
            segment.direction = invalidDirection;
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.code,'invalid_input');
        end

        function rejectsInvalidSegmentData(testCase,invalidData)
            segment = parabola(0.1,0);
            segment.(invalidData.field) = invalidData.value;
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.code,'invalid_input');
            testCase.verifyTrue(isnan(result.maxAbsCurvature));
        end

        function rejectsInvalidVehicle(testCase,invalidVehicle)
            result = validateCubicGeometry(parabola(0.1,0),invalidVehicle);
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.code,'invalid_input');
        end

        function rejectsInvalidJoinDeclarations(testCase,badJoins)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment([0,0,1,1;0,0,0,0],1);
            result = validateCubicGeometry([a,b],vehicleParameters(),badJoins);
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(hasFailure(result,'joinTypes'));
        end

        function rejectsMissingEmptyAndNonstructSegments(testCase)
            empty = validateCubicGeometry(struct([]),vehicleParameters());
            missingFields = validateCubicGeometry(struct('direction',1),vehicleParameters());
            numeric = validateCubicGeometry(1,vehicleParameters());
            testCase.verifyFalse(empty.passed);
            testCase.verifyFalse(missingFields.passed);
            testCase.verifyFalse(numeric.passed);
            testCase.verifyTrue(isnan(empty.maxAbsCurvature));
        end

        function rejectsOverflowWithoutSuccessfulMetrics(testCase)
            segment = parabola(0.1,0);
            segment.coefficients(1,1) = realmax;
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.code,'numerical_failure');
            testCase.verifyTrue(isnan(result.maxAbsCurvature));
        end

        function failedSegmentCannotBeHiddenInAggregate(testCase)
            a = makeSegment([0,0,1,0;0,0,0,0],1);
            b = makeSegment(zeros(2,4),1);
            result = validateCubicGeometry([a,b],vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyTrue(result.segments(1).passed);
            testCase.verifyTrue(isnan(result.maxAbsCurvature));
            testCase.verifyTrue(hasFailure(result,'availableGeometry'));
        end

        function preservesGeometryUnderRigidTransform(testCase)
            segment = parabola(0.2,0.37);
            phi = 0.7;
            R = [cos(phi),-sin(phi);sin(phi),cos(phi)];
            coefficients = R*segment.coefficients;
            coefficients(:,4) = coefficients(:,4)+[20;-4];
            moved = makeSegment(coefficients,1);
            base = validateCubicGeometry(segment,vehicleParameters());
            result = validateCubicGeometry(moved,vehicleParameters());
            testCase.verifyTrue(result.passed,result.message);
            testCase.verifyEqual(result.maxAbsCurvature,base.maxAbsCurvature,AbsTol=1e-12);
            testCase.verifyEqual(result.minTurningRadius,base.minTurningRadius,AbsTol=1e-11);
        end

        function derivesLimitsAndIgnoresTimingMetadata(testCase)
            segment = parabola(0.2,0.37);
            vehicle = struct('wheelbase',3,'maxSteering',atan(0.45));
            result = validateCubicGeometry(segment,vehicle);
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.curvatureLimit,0.15,AbsTol=1e-14);
            testCase.verifyEqual(result.vehicleMinTurningRadius,1/0.15,AbsTol=1e-13);
            segment.duration = NaN;
            segment.dt = Inf;
            withMetadata = validateCubicGeometry(segment,vehicle);
            testCase.verifyEqual(withMetadata,result);
        end

        function detectsNarrowInteriorPeak(testCase)
            % x=0.01*s, y=(s-0.373)^2/2 has max curvature 1/0.01^2.
            segment = makeSegment([0,0,0.01,0;0,0.5,-0.373,0.373^2/2],1);
            result = validateCubicGeometry(segment,vehicleParameters());
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.maxAbsCurvature,10000,AbsTol=1e-6);
            testCase.verifyEqual(result.segments.maxCurvatureS,0.373,AbsTol=1e-11);
        end
    end
end

function segment = parabola(kappa,peak)
    segment = makeSegment([0,0,1,0;0,kappa/2,-kappa*peak,kappa*peak^2/2],1);
end

function segment = makeSegment(coefficients,direction)
% Only construct endpoint metadata; no test-local curvature/extrema solver.
% This also allows intentionally degenerate curves rejected by cubicPath.
    points = [polyval(coefficients(1,:),[0,1]);polyval(coefficients(2,:),[0,1])];
    dx = polyval(polyder(coefficients(1,:)),[0,1]);
    dy = polyval(polyder(coefficients(2,:)),[0,1]);
    heading = atan2(dy,dx)+(direction == -1)*pi;
    segment = struct('coefficients',coefficients,'startPose',[points(:,1);heading(1)], ...
        'endPose',[points(:,2);heading(2)],'direction',direction);
end

function yes = hasFailure(result,name)
    yes = any(strcmp({result.checks.name},name) & ~[result.checks.passed]);
end
