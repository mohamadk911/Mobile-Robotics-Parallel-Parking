classdef cubicPathLengthTest < matlab.unittest.TestCase
%cubicPathLengthTest Closed-form reference lengths and quadrature stability.
    methods (TestClassSetup)
        function addProjectPath(testCase)
            folder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
        end
    end
    methods (Test)
        function measuresStraightSegment(testCase)
            segment = cubicPath([0;0;atan2(4,3)],[3;4;atan2(4,3)],5,5,2,1);
            testCase.verifyEqual(cubicPathLength(segment),5,AbsTol=1e-12);
        end
        function measuresKnownParabola(testCase)
            segment = cubicPath([0;0;0],[1;1;atan(2)],1,sqrt(5),2,1);
            expected = sqrt(5)/2+asinh(2)/4;
            testCase.verifyEqual(cubicPathLength(segment),expected,AbsTol=1e-11);
        end
        function supportsCollectionAndIgnoresCaches(testCase)
            segment = cubicPath([0;0;pi],[-1;1;pi-atan(2)],1,sqrt(5),2,1);
            original = cubicPathLength(segment);
            segment.x = [1e6,-1e6];
            segment.curvature = [0,0];
            testCase.verifyEqual(cubicPathLength([segment;segment]),[original,original],AbsTol=1e-12);
        end
        function rejectsDegenerateGeometry(testCase)
            segment = struct('coefficients',zeros(2,4),'direction',-1);
            testCase.verifyError(@() cubicPathLength(segment),'evaluateCubicPath:DegenerateTangent');
        end
        function rejectsInvalidTolerance(testCase)
            segment = cubicPath([0;0;0],[1;0;0],1,1,2,1);
            testCase.verifyError(@() cubicPathLength(segment,NaN), 'MATLAB:cubicPathLength:expectedFinite');
            testCase.verifyError(@() cubicPathLength(segment,1e-9,0), 'MATLAB:cubicPathLength:notGreaterEqual');
        end
    end
end
