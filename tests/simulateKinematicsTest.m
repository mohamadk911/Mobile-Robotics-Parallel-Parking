classdef simulateKinematicsTest < matlab.unittest.TestCase
%simulateKinematicsTest Independent Euler interval and input-contract checks.
    properties (TestParameter)
        badTime = struct('duplicate',[0,0,1],'decreasing',[0,1,0.5], ...
            'overflow',[-realmax,realmax/2,realmax]);
    end
    methods (TestClassSetup)
        function addPath(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fileparts(fileparts(mfilename('fullpath')))));
        end
    end
    methods (Test)
        function usesLeftCommandAndPreviousHeading(testCase)
            vehicle = vehicleParameters();
            q = simulateKinematics([0;0;0],[0,0.2,0.7],[2,3,99], ...
                [atan(vehicle.wheelbase/2),0,-1],vehicle);
            testCase.verifyEqual(q,[0,0.4,0.4+1.5*cos(0.2); ...
                0,0,1.5*sin(0.2);0,0.2,0.2],AbsTol=1e-14);
        end
        function zeroSpeedStopsDespiteSteeringJump(testCase)
            q = simulateKinematics([1;2;0.3],[0,0.1,0.4],[0,0,0],[-0.6,0.6,0],vehicleParameters());
            testCase.verifyEqual(q,repmat([1;2;0.3],1,3));
        end
        function singleSampleAndColumns(testCase)
            q = simulateKinematics([1;2;0],0,1,0,vehicleParameters());
            column = simulateKinematics([0;0;0],[0;0.2;0.3],[1;2;9],[0;0;0],vehicleParameters());
            testCase.verifyEqual(q,[1;2;0]);
            testCase.verifyEqual(column(1,:),[0,0.2,0.4],AbsTol=1e-15);
        end
        function rejectsNonIncreasingTimes(testCase,badTime)
            testCase.verifyError(@() simulateKinematics([0;0;0],badTime,[0,0,0],[0,0,0],vehicleParameters()), ...
                'simulateKinematics:InvalidTime');
        end
        function rejectsLengthsAndNonfiniteInputs(testCase)
            testCase.verifyError(@() simulateKinematics([0;0;0],[0,1],[1,1,1],[0,0],vehicleParameters()), ...
                'MATLAB:simulateKinematics:incorrectNumel');
            testCase.verifyError(@() simulateKinematics([0;0;0],[0,1],[1,1],[NaN,0],vehicleParameters()), ...
                'MATLAB:simulateKinematics:expectedFinite');
            testCase.verifyError(@() simulateKinematics([0,0,0],[0,1],[1,1],[0,0],vehicleParameters()), ...
                'MATLAB:simulateKinematics:incorrectSize');
        end
        function rejectsNonfiniteStates(testCase)
            testCase.verifyError(@() simulateKinematics([realmax;0;0],[0,2],[realmax,0],[0,0],vehicleParameters()), ...
                'simulateKinematics:NonfiniteResult');
        end
    end
end
