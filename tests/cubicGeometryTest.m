classdef cubicGeometryTest < matlab.unittest.TestCase
%cubicGeometryTest Analytic cubic geometry, reverse motion and rejection tests.
% Run with runtests('tests/cubicGeometryTest.m') from the project root.

    properties (TestParameter)
        direction = struct('forward',1,'reverse',-1);
        invalidDirection = struct('zero',0,'fraction',0.5,'tooLarge',2);
        invalidPose = struct('row',[0,0,0],'short',[0;0], ...
            'long',[0;0;0;0],'matrix',zeros(3,2));
        nonfinite = struct('nan',NaN,'positiveInfinity',Inf,'negativeInfinity',-Inf);
        invalidCount = struct('one',struct('value',1,'id','notGreaterEqual'), ...
            'zero',struct('value',0,'id','notGreaterEqual'), ...
            'fraction',struct('value',2.5,'id','expectedInteger'), ...
            'vector',struct('value',[3,4],'id','expectedScalar'));
        invalidQuery = struct('below',struct('value',-0.1,'id','notGreaterEqual'), ...
            'above',struct('value',1.1,'id','notLessEqual'), ...
            'empty',struct('value',[],'id','expectedVector'), ...
            'matrix',struct('value',ones(2),'id','expectedVector'), ...
            'complex',struct('value',0.5+1i,'id','expectedReal'));
        degenerate = struct('constant',zeros(2,4), ...
            'tinyTangent',[0,0,1e-12,0;0,0,0,0], ...
            'interiorStop',[0,1,-1,0;0,0,0,0], ...
            'interiorNearStop',[0,1,-1,0;0,0,1e-12,0], ...
            'doubleRoot',[1/3,-0.5,0.25,0;0,0,0,0]);
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            folder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
        end
    end

    methods (Test)
        function preservesEndpointConditions(testCase, direction)
            offset = (direction == -1)*pi;
            startPose = [0;0;offset];
            endPose = [1;1;atan(2)+offset];
            path = cubicPath(startPose,endPose,1,sqrt(5),9,direction);

            testCase.verifyEqual([path.x([1,end]);path.y([1,end])], [0,1;0,1], AbsTol=2e-14);
            headingError = path.thetaBody([1,end]) - [startPose(3),endPose(3)];
            testCase.verifyEqual(atan2(sin(headingError),cos(headingError)), [0,0], AbsTol=2e-14);
            testCase.verifyEqual([path.dxds([1,end]);path.dyds([1,end])], [1,1;0,2], AbsTol=2e-14);
            testCase.verifyEqual(path.coefficients, [0,0,1,0;0,1,0,0], AbsTol=2e-14);
            testCase.verifyEqual(path.direction, direction);
        end

        function evaluatesKnownCubicAtArbitraryQueries(testCase)
            s = [0.83,0,0.137,1,0.83,0.5];
            curve = evaluateCubicPath([1,0,1,0;0,1,0,0],s,1);
            testCase.verifyEqual(curve.s, s, AbsTol=1e-15);
            testCase.verifyEqual(curve.x, s+s.^3, AbsTol=2e-14);
            testCase.verifyEqual(curve.y, s.^2, AbsTol=2e-14);
            testCase.verifyEqual(curve.dxds, 1+3*s.^2, AbsTol=2e-14);
            testCase.verifyEqual(curve.dyds, 2*s, AbsTol=2e-14);
            testCase.verifyEqual(curve.d2xds2, 6*s, AbsTol=2e-14);
            testCase.verifyEqual(curve.d2yds2, 2*ones(size(s)), AbsTol=2e-14);
            expectedCurvature = (2-6*s.^2)./((1+3*s.^2).^2+4*s.^2).^(3/2);
            testCase.verifyEqual(curve.curvature, expectedCurvature, AbsTol=2e-14);
            testCase.verifyEqual(curve.thetaTangent, atan2(2*s,1+3*s.^2), AbsTol=2e-14);
        end

        function derivativesMatchIndependentDifferences(testCase)
            coefficients = [0.7,-0.2,2,4;-0.3,0.8,0.4,-2];
            s = 0.37;
            h = 1e-5;
            curve = evaluateCubicPath(coefficients,s,1);
            plus = evaluateCubicPath(coefficients,s+h,1);
            minus = evaluateCubicPath(coefficients,s-h,1);
            testCase.verifyEqual([curve.dxds;curve.dyds], ...
                ([plus.x;plus.y]-[minus.x;minus.y])/(2*h), AbsTol=2e-9);
            testCase.verifyEqual([curve.d2xds2;curve.d2yds2], ...
                ([plus.dxds;plus.dyds]-[minus.dxds;minus.dyds])/(2*h), AbsTol=2e-9);
            headingRate = (plus.thetaBody-minus.thetaBody)/(2*h);
            testCase.verifyEqual(headingRate, hypot(curve.dxds,curve.dyds)*curve.curvature, AbsTol=2e-9);
        end

        function maintainsContinuousHeadingAcrossWrap(testCase, direction)
            path = cubicPath([0;0;deg2rad(179)],[-4*direction;0;deg2rad(-179)],4,4,3,direction);
            scalar = evaluateCubicPath(path.coefficients,1,direction);
            unordered = evaluateCubicPath(path.coefficients,[1;0;0.5;1],direction);
            testCase.verifyEqual(rad2deg(path.thetaBody), [179,180,181], AbsTol=1e-10);
            testCase.verifyEqual(scalar.thetaBody, path.thetaBody(end), AbsTol=1e-14);
            testCase.verifyEqual(unordered.thetaBody, path.thetaBody([3,1,2,3]), AbsTol=1e-14);
            testCase.verifySize(unordered.s, [1,4]);
        end

        function resolvesLargeHeadingSweepWithoutDenseQueries(testCase)
            coefficients = [1/3,-0.5,0.24,0;0,0.1,-0.1,0];
            sparseCurve = evaluateCubicPath(coefficients,[0,1],1);
            denseCurve = evaluateCubicPath(coefficients,linspace(0,1,501),1);
            scalar = evaluateCubicPath(coefficients,1,1);
            testCase.verifyEqual(diff(sparseCurve.thetaBody), -4*atan(5), AbsTol=1e-12);
            testCase.verifyEqual(sparseCurve.thetaBody, denseCurve.thetaBody([1,end]), AbsTol=1e-12);
            testCase.verifyEqual(scalar.thetaBody, sparseCurve.thetaBody(end), AbsTol=1e-12);
            independentHeading = unwrap(atan2(denseCurve.dyds,denseCurve.dxds));
            testCase.verifyEqual(denseCurve.thetaBody, independentHeading, AbsTol=1e-12);
        end

        function straightPathHasZeroCurvature(testCase)
            curve = evaluateCubicPath([0,0,3,1;0,0,4,-2],[0,0.23,1],1);
            testCase.verifyEqual(curve.curvature, zeros(1,3), AbsTol=1e-15);
            testCase.verifyEqual(curve.thetaBody, atan2(4,3)*ones(1,3), AbsTol=1e-15);
        end

        function respectsTranslationAndRotation(testCase)
            coefficients = [1,0,1,0;0,1,0,0];
            phi = 0.4;
            R = [cos(phi),-sin(phi);sin(phi),cos(phi)];
            moved = R*coefficients;
            moved(:,4) = moved(:,4) + [100;-50];
            base = evaluateCubicPath(coefficients,[0,0.3,1],1);
            transformed = evaluateCubicPath(moved,[0,0.3,1],1);
            testCase.verifyEqual([transformed.x;transformed.y], ...
                R*[base.x;base.y]+[100;-50], AbsTol=1e-12);
            testCase.verifyEqual(transformed.curvature, base.curvature, AbsTol=1e-12);
            testCase.verifyEqual(transformed.thetaBody, base.thetaBody+phi, AbsTol=1e-12);
        end

        function timedInputsMatchBicycleKinematics(testCase, direction)
            offset = (direction == -1)*pi;
            path = cubicPath([0;0;offset],[1;1;atan(2)+offset],1,sqrt(5),2,direction);
            scaling = timeScale(2,0.2);
            vehicle = vehicleParameters();
            traj = pathToInputs(path,scaling,vehicle);
            testCase.verifyEqual(traj.x, scaling.s, AbsTol=2e-14);
            testCase.verifyEqual(traj.y, scaling.s.^2, AbsTol=2e-14);
            testCase.verifyEqual(traj.v, direction*scaling.sdot.*sqrt(1+4*scaling.s.^2), AbsTol=2e-14);
            testCase.verifyEqual(traj.omega, 2*scaling.sdot./(1+4*scaling.s.^2), AbsTol=2e-14);
            testCase.verifyEqual(traj.delta, atan(direction*vehicle.wheelbase*2./ ...
                (1+4*scaling.s.^2).^(3/2)), AbsTol=2e-14);
            q = [traj.x(4);traj.y(4);traj.theta(4)];
            dq = carKinematics(q,traj.v(4),traj.delta(4),vehicle);
            testCase.verifyEqual(dq, [scaling.sdot(4); ...
                2*scaling.s(4)*scaling.sdot(4);traj.omega(4)], AbsTol=2e-14);
        end

        function normalizesIntegerDirection(testCase)
            path = struct('coefficients',[0,0,1,0;0,1,0,0],'direction',int8(-1));
            scaling = timeScale(2,0.2);
            traj = pathToInputs(path,scaling,vehicleParameters());
            testCase.verifyEqual(traj.v, -scaling.sdot.*sqrt(1+4*scaling.s.^2), AbsTol=2e-14);
            testCase.verifyClass(traj.v, 'double');
        end

        function ignoresSampleCachesAndRespectsCoefficients(testCase)
            path = cubicPath([0;0;0],[1;1;atan(2)],1,sqrt(5),2,1);
            scaling = timeScale(2,0.1);
            vehicle = vehicleParameters();
            original = pathToInputs(path,scaling,vehicle);
            path = rmfield(path, {'s','x','y','dxds','dyds','d2xds2','d2yds2', ...
                'thetaTangent','thetaBody','curvature'});
            withoutCaches = pathToInputs(path,scaling,vehicle);
            testCase.verifyEqual(withoutCaches, original, AbsTol=1e-14);
            path.coefficients(2,:) = [0,2,0,0];
            changed = pathToInputs(path,scaling,vehicle);
            testCase.verifyEqual(changed.y, 2*scaling.s.^2, AbsTol=1e-14);
            testCase.verifyGreaterThan(max(abs(changed.y-original.y)), 0.9);
        end

        function rejectsDirections(testCase, invalidDirection)
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,1,3,invalidDirection), ...
                'cubicPath:InvalidDirection');
            testCase.verifyError(@() evaluateCubicPath([0,0,1,0;0,0,0,0],0.5,invalidDirection), ...
                'evaluateCubicPath:InvalidDirection');
        end

        function rejectsPoseSizes(testCase, invalidPose)
            testCase.verifyError(@() cubicPath(invalidPose,[1;0;0],1,1,3,1), 'MATLAB:cubicPath:incorrectSize');
            testCase.verifyError(@() cubicPath([0;0;0],invalidPose,1,1,3,1), 'MATLAB:cubicPath:incorrectSize');
        end

        function rejectsNonfiniteInputs(testCase, nonfinite)
            testCase.verifyError(@() cubicPath([nonfinite;0;0],[1;0;0],1,1,3,1), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() cubicPath([0;0;0],[1;nonfinite;0],1,1,3,1), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],nonfinite,1,3,1), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,nonfinite,3,1), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,1,nonfinite,1), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,1,3,nonfinite), ...
                'MATLAB:cubicPath:expectedFinite');
            testCase.verifyError(@() evaluateCubicPath([0,0,1,nonfinite;0,0,0,0],0,1), ...
                'MATLAB:evaluateCubicPath:expectedFinite');
            testCase.verifyError(@() evaluateCubicPath([0,0,1,0;0,0,0,0],nonfinite,1), ...
                'MATLAB:evaluateCubicPath:expectedFinite');
            testCase.verifyError(@() evaluateCubicPath([0,0,1,0;0,0,0,0],0,nonfinite), ...
                'MATLAB:evaluateCubicPath:expectedFinite');
        end

        function rejectsInvalidSampleCounts(testCase, invalidCount)
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,1,invalidCount.value,1), ...
                ['MATLAB:cubicPath:',invalidCount.id]);
        end

        function rejectsInvalidQueries(testCase, invalidQuery)
            testCase.verifyError(@() evaluateCubicPath([0,0,1,0;0,0,0,0],invalidQuery.value,1), ...
                ['MATLAB:evaluateCubicPath:',invalidQuery.id]);
        end

        function rejectsInvalidCoefficientsAndTangents(testCase)
            testCase.verifyError(@() evaluateCubicPath(zeros(4,2),0.5,1), ...
                'MATLAB:evaluateCubicPath:incorrectSize');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],0,1,3,1), 'MATLAB:cubicPath:expectedPositive');
            testCase.verifyError(@() cubicPath([0;0;0],[1;0;0],1,-1,3,1), 'MATLAB:cubicPath:expectedPositive');
            testCase.verifyError(@() evaluateCubicPath([realmax,0,1,0;0,0,0,0],0.5,1), ...
                'evaluateCubicPath:NonfiniteGeometry');
        end

        function rejectsDegeneracyBetweenQueries(testCase, degenerate)
            testCase.verifyError(@() evaluateCubicPath(degenerate,[0,1],1), ...
                'evaluateCubicPath:DegenerateTangent');
        end

        function rejectsInvalidTimingAndVehicleInputs(testCase)
            path = cubicPath([0;0;0],[1;0;0],1,1,3,1);
            vehicle = vehicleParameters();
            badTime = struct('t',[0,0],'s',[0,1],'sdot',[0,0]);
            badS = struct('t',[0,1],'s',[0,1.1],'sdot',[0,0]);
            badRate = struct('t',[0,1],'s',[0,1],'sdot',[NaN,0]);
            testCase.verifyError(@() pathToInputs(path,badTime,vehicle), 'pathToInputs:InvalidScaling');
            testCase.verifyError(@() pathToInputs(path,badS,vehicle), 'MATLAB:evaluateCubicPath:notLessEqual');
            testCase.verifyError(@() pathToInputs(path,badRate,vehicle), 'MATLAB:pathToInputs:expectedFinite');
            vehicle.wheelbase = Inf;
            testCase.verifyError(@() pathToInputs(path,timeScale(1,0.1),vehicle), ...
                'MATLAB:pathToInputs:expectedFinite');
        end
    end
end
