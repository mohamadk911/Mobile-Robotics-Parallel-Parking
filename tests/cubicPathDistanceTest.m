classdef cubicPathDistanceTest < matlab.unittest.TestCase
%cubicPathDistanceTest Closed-form distances and global stationary-point cases.
    properties (TestParameter)
        nearestS = {0.3725,0.5,0.937};
        badPoints = {NaN(2,1),Inf(2,1),zeros(3,2),zeros(2,0),1i*ones(2,1)};
    end
    methods (TestClassSetup)
        function addPath(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fileparts(fileparts(mfilename('fullpath')))));
        end
    end
    methods (Test)
        function straightLineProjectionAndEndpoints(testCase)
            line = struct('coefficients',[0,0,2,0;0,0,0,0],'direction',1);
            r = cubicPathDistance(line,[0.7,-1,3;0.3,0,4]);
            testCase.verifyEqual(r.distance,[0.3,1,sqrt(17)],AbsTol=1e-12);
            testCase.verifyEqual(r.s,[0.35,0,1],AbsTol=1e-12);
            testCase.verifyEqual(r.nearestXY,[0.7,0,2;0,0,0],AbsTol=1e-12);
        end
        function trueCubicHasKnownUniqueNearestPoint(testCase,nearestS)
            % p=(s,s^3), z=p(a)+d*(3*a^2,-1), with z_y<0.
            % Distance derivative is strictly increasing on s>=0, so a is
            % the unique global minimum, not just a stationary point.
            a = nearestS; d = 1;
            curve = struct('coefficients',[0,0,1,0;1,0,0,0],'direction',1);
            z = [a+3*a^2*d;a^3-d];
            r = cubicPathDistance(curve,z);
            testCase.verifyEqual(r.s,a,AbsTol=1e-12);
            testCase.verifyEqual(r.distance,d*sqrt(1+9*a^4),AbsTol=1e-11);
        end
        function findsAllLocalMinima(testCase)
            % p=(s,10*(s-0.5)^2), z=(0.5,1). Two equal global minima.
            curve = struct('coefficients',[0,0,1,0;0,10,-10,2.5],'direction',1);
            r = cubicPathDistance(curve,[0.5;1]);
            testCase.verifyEqual(r.distance,sqrt(0.0975),AbsTol=1e-12);
            testCase.verifyEqual(abs(r.s-0.5),sqrt(19/200),AbsTol=1e-12);
        end
        function retainsRepeatedStationaryRoot(testCase)
            % At z_y=0.05 the distance derivative is 200*(s-0.5)^3.
            curve = struct('coefficients',[0,0,1,0;0,10,-10,2.5],'direction',1);
            r = cubicPathDistance(curve,[0.5;0.05]);
            testCase.verifyEqual(r.distance,0.05,AbsTol=1e-12);
            % Flat minimum makes s less well conditioned than distance.
            testCase.verifyEqual(r.s,0.5,AbsTol=1e-5);
        end
        function joinAndEverySegmentAreConsidered(testCase)
            a = struct('coefficients',[0,0,1,0;0,0,0,0],'direction',1);
            b = struct('coefficients',[0,0,1,1;0,0,0,0],'direction',1);
            c = struct('coefficients',[0,0,1,2;0,0,0,0],'direction',1);
            r = cubicPathDistance([a,b,c],[1,2.8;0.2,0.3]);
            testCase.verifyEqual(r.distance,[0.2,0.3],AbsTol=1e-12);
            testCase.verifyEqual(r.nearestXY,[1,2.8;0,0],AbsTol=1e-12);
            testCase.verifyEqual(r.segmentIndex,[1,3]);
        end
        function ignores201PointCaches(testCase)
            a = 0.3725;
            curve = struct('coefficients',[0,0,1,0;0.1,0,0,0],'direction',1, ...
                's',linspace(0,1,201),'x',zeros(1,201),'y',100*ones(1,201));
            r = cubicPathDistance(curve,[a;0.1*a^3]);
            testCase.verifyLessThan(r.distance,1e-12);
            testCase.verifyEqual(r.s,a,AbsTol=1e-12);
            testCase.verifyTrue(r.numerics.converged);
            testCase.verifyEqual([r.numerics.rootWidth,r.numerics.refinedRootWidth],[1e-12,1e-14]);
        end
        function translationAndRotationPreserveDistance(testCase)
            c = [0,0,1,0;0.1,0,0,0]; points = [0.37,1.2;0.2,-0.1];
            phi = 0.73; R = [cos(phi),-sin(phi);sin(phi),cos(phi)];
            moved = R*c; moved(:,4) = moved(:,4)+[20;-4];
            base = cubicPathDistance(struct('coefficients',c,'direction',1),points);
            placed = cubicPathDistance(struct('coefficients',moved,'direction',1),R*points+[20;-4]);
            testCase.verifyEqual(placed.distance,base.distance,AbsTol=1e-12);
        end
        function rejectsInvalidPoints(testCase,badPoints)
            curve = struct('coefficients',[0,0,1,0;0,0,0,0],'direction',1);
            testCase.verifyTrue(throws(@() cubicPathDistance(curve,badPoints)));
        end
        function rejectsDegenerateAndOverflowGeometry(testCase)
            curve = struct('coefficients',zeros(2,4),'direction',1);
            testCase.verifyError(@() cubicPathDistance(curve,[0;0]),'evaluateCubicPath:DegenerateTangent');
            curve.coefficients = [0,0,1,-realmax;0,0,0,0];
            testCase.verifyError(@() cubicPathDistance(curve,[realmax;0]),'cubicPathDistance:NonfiniteResult');
        end
    end
end
function yes = throws(action)
    yes = false;
    try
        action();
    catch
        yes = true;
    end
end
