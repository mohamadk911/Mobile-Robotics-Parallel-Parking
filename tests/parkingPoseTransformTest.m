classdef parkingPoseTransformTest < matlab.unittest.TestCase
%parkingPoseTransformTest Coordinate mathematics only; no planner is invoked.

    properties (TestParameter)
        scenarioId = {'nominal','tight','changedStart'};
        frame = struct('identity',[0;0;0],'translation',[17.3;-42.1;0], ...
            'rotation',[0;0;0.83],'combined',[-8.1;12.7;-2.3], ...
            'nearWrap',[3;-2;pi-1e-8]);
        badFrame = struct('row',struct('value',[0,0,0],'error','incorrectSize'), ...
            'short',struct('value',[0;0],'error','incorrectSize'), ...
            'matrix',struct('value',zeros(3,2),'error','incorrectSize'), ...
            'nan',struct('value',[0;NaN;0],'error','expectedFinite'), ...
            'inf',struct('value',[0;0;Inf],'error','expectedFinite'), ...
            'complex',struct('value',[0;0;1i],'error','expectedReal'));
        badPoses = struct('row',struct('value',[0,0,0],'error','incorrectNumrows'), ...
            'short',struct('value',[0;0],'error','incorrectNumrows'), ...
            'long',struct('value',zeros(4,2),'error','incorrectNumrows'), ...
            'empty',struct('value',zeros(3,0),'error','expectedNonempty'), ...
            'nan',struct('value',[0;0;NaN],'error','expectedFinite'), ...
            'inf',struct('value',[Inf;0;0],'error','expectedFinite'), ...
            'complex',struct('value',[0;1i;0],'error','expectedReal'), ...
            'text',struct('value','pose','error','invalidType'), ...
            'volume',struct('value',zeros(3,2,2),'error','expected2D'));
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            folder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
        end
    end

    methods (Test)
        function transformsWholeSceneAndRoundTrips(testCase,scenarioId,frame)
            scenario = scenarioParameters(vehicleParameters(),scenarioId,frame);
            local = [scenario.rearCarPose,scenario.frontCarPose,scenario.initialPose,scenario.targetPose];
            world = parkingLocalToWorld(local,scenario.framePoseWorld);
            recovered = parkingWorldToLocal(world,scenario.framePoseWorld);
            testCase.verifyEqual(recovered,local,AbsTol=1e-12);
            testCase.verifyEqual(world(:,1),frame,AbsTol=1e-12);
            testCase.verifyEqual(world(3,:),local(3,:)+frame(3),AbsTol=1e-12);
            % Relative positions expressed in the placed front-car frame
            % recover the same staging offset for every scene placement.
            relativeInitial = parkingWorldToLocal(world(:,3),world(:,2));
            testCase.verifyEqual(relativeInitial,scenario.startOffsetFront,AbsTol=1e-12);
            localDifferences = local(1:2,2:end)-local(1:2,1:end-1);
            worldDifferences = world(1:2,2:end)-world(1:2,1:end-1);
            testCase.verifyEqual(vecnorm(worldDifferences),vecnorm(localDifferences),AbsTol=1e-12);
        end

        function translationMovesAllPositionsEqually(testCase)
            local = [0,11,11.76,5.5;0,0,2.8,0;0,0,0.2,0];
            world = parkingLocalToWorld(local,[17.3;-42.1;0]);
            testCase.verifyEqual(world(1:2,:),local(1:2,:)+[17.3;-42.1],AbsTol=1e-12);
            testCase.verifyEqual(world(3,:),local(3,:),AbsTol=1e-12);
        end

        function rotationHasCorrectAxisAndSign(testCase)
            local = [1,0,-2;0,1,3;0,pi/2,-pi/4];
            world = parkingLocalToWorld(local,[4;-7;pi/2]);
            testCase.verifyEqual(world,[4,3,1;-6,-7,-9;pi/2,pi,pi/4],AbsTol=1e-12);
            testCase.verifyEqual(parkingWorldToLocal(world,[4;-7;pi/2]),local,AbsTol=1e-12);
        end

        function arbitraryRotationMatchesIndependentComplexArithmetic(testCase)
            local = [2,-4,7;3,8,-2;0.1,-2,4];
            phi = 0.83;
            world = parkingLocalToWorld(local,[0;0;phi]);
            expectedXY = exp(1i*phi)*complex(local(1,:),local(2,:));
            testCase.verifyEqual(world(1:2,:),[real(expectedXY);imag(expectedXY)],AbsTol=1e-12);
        end

        function preservesHeadingBranchesAcrossWrap(testCase)
            local = [zeros(2,3);deg2rad([179,181,539])];
            framePose = [0;0;deg2rad(10)];
            world = parkingLocalToWorld(local,framePose);
            testCase.verifyEqual(rad2deg(world(3,:)),[189,191,549],AbsTol=1e-12);
            testCase.verifyEqual(diff(world(3,:)),diff(local(3,:)),AbsTol=1e-12);
            testCase.verifyEqual(parkingWorldToLocal(world,framePose),local,AbsTol=1e-12);
            world(3,:) = atan2(sin(world(3,:)),cos(world(3,:)));
            recovered = parkingWorldToLocal(world,framePose);
            error = recovered(3,:)-local(3,:);
            testCase.verifyEqual(atan2(sin(error),cos(error)),zeros(1,3),AbsTol=1e-12);
        end

        function inverseSubtractsRotationAcrossNegativeWrap(testCase)
            world = [0;0;deg2rad(-179)];
            local = parkingWorldToLocal(world,[0;0;deg2rad(10)]);
            testCase.verifyEqual(rad2deg(local(3)),-189,AbsTol=1e-12);
            testCase.verifyEqual(parkingLocalToWorld(local,[0;0;deg2rad(10)]),world,AbsTol=1e-12);
        end

        function rigidTransformPreservesCubicCurvatureAndSteering(testCase)
            % Transform a known polynomial; no path search or retuning.
            coefficients = [0.2,0,2,0;0,0.1,0,0];
            framePose = [20;-4;0.7];
            basis = parkingLocalToWorld([eye(2);zeros(1,2)],framePose);
            R = basis(1:2,:)-framePose(1:2);
            worldCoefficients = R*coefficients;
            worldCoefficients(:,4) = worldCoefficients(:,4)+framePose(1:2);
            local = evaluateCubicPath(coefficients,[0,0.17,0.49,0.88,1],-1);
            world = evaluateCubicPath(worldCoefficients,local.s,-1);
            transformedPoses = parkingLocalToWorld([local.x;local.y;local.thetaBody],framePose);
            testCase.verifyEqual([world.x;world.y],transformedPoses(1:2,:),AbsTol=1e-12);
            testCase.verifyEqual(world.curvature,local.curvature,AbsTol=1e-12);
            vehicle = vehicleParameters();
            testCase.verifyEqual(atan(-vehicle.wheelbase*world.curvature), ...
                atan(-vehicle.wheelbase*local.curvature),AbsTol=1e-12);
            headingError = world.thetaBody-transformedPoses(3,:);
            testCase.verifyEqual(atan2(sin(headingError),cos(headingError)),zeros(1,5),AbsTol=1e-12);
        end

        function acceptsNumericTypesAndSinglePose(testCase)
            world = parkingLocalToWorld(int32([1;2;0]),single([3;4;0]));
            local = parkingWorldToLocal(int32([4;6;0]),single([3;4;0]));
            testCase.verifyClass(world,'double');
            testCase.verifyEqual(world,[4;6;0],AbsTol=1e-12);
            testCase.verifyEqual(local,[1;2;0],AbsTol=1e-12);
        end

        function rejectsInvalidFrame(testCase,badFrame)
            testCase.verifyError(@() parkingLocalToWorld([0;0;0],badFrame.value), ...
                ['MATLAB:parkingLocalToWorld:',badFrame.error]);
            testCase.verifyError(@() parkingWorldToLocal([0;0;0],badFrame.value), ...
                ['MATLAB:parkingWorldToLocal:',badFrame.error]);
        end

        function rejectsInvalidPoses(testCase,badPoses)
            testCase.verifyError(@() parkingLocalToWorld(badPoses.value,[0;0;0]), ...
                ['MATLAB:parkingLocalToWorld:',badPoses.error]);
            testCase.verifyError(@() parkingWorldToLocal(badPoses.value,[0;0;0]), ...
                ['MATLAB:parkingWorldToLocal:',badPoses.error]);
        end

        function rejectsNonfiniteResults(testCase)
            testCase.verifyError(@() parkingLocalToWorld([realmax;0;0],[realmax;0;0]), ...
                'parkingLocalToWorld:NonfiniteResult');
            testCase.verifyError(@() parkingWorldToLocal([-realmax;0;0],[realmax;0;0]), ...
                'parkingWorldToLocal:NonfiniteResult');
        end
    end
end
