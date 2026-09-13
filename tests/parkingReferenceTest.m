classdef parkingReferenceTest < matlab.unittest.TestCase
%parkingReferenceTest Independent numerical formulas and propagation contracts.
    methods (TestClassSetup)
        function prepare(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fileparts(fileparts(mfilename('fullpath')))));
        end
    end
    methods (Test)
        function knownNonuniformComparisonAndWrappedHeading(testCase)
            a = struct('t',[0,0.1,0.6,1],'q',[zeros(1,4);0,0.01,0.02,0.04; ...
                pi-0.001,pi-0.001,pi-0.001,-pi+0.001]);
            b = a; b.q(2,:)=0; b.q(3,:)=pi-0.001;
            c = compareParkingStates(a,b);
            testCase.verifyEqual(c.finalXY,0.04);
            testCase.verifyEqual(c.finalHeading,0.002,AbsTol=1e-12);
            testCase.verifyEqual(c.maxXY,0.04);
            testCase.verifyEqual(c.timeWeightedXYRMSE,sqrt(0.00053),AbsTol=1e-14);
        end
        function observedOrderUsesGridRatio(testCase)
            c = parkingConvergence([0.1,0.05,0.025,0.0125],[0.04,0.02,0.01,0.005],0.05);
            testCase.verifyEqual(c.ratioToFiner(1:3),[2;2;2]);
            testCase.verifyEqual(c.observedOrder(1:3),[1;1;1]);
            testCase.verifyEqual(c.relativeToBaseline,[2;1;0.5;0.25]);
            testCase.verifyTrue(isnan(c.observedOrder(end)));
            other = parkingConvergence([0.1,0.025],[0.01,0.000625],0.1);
            testCase.verifyEqual(other.observedOrder(1),2);
        end
        function zeroAndUnavailableErrorsHaveNoInventedOrder(testCase)
            c = parkingConvergence([0.1,0.05,0.025],[0,0,NaN],0.05);
            testCase.verifyTrue(all(isnan(c.observedOrder)));
            testCase.verifyTrue(all(isnan(c.relativeToBaseline)));
        end
        function invalidComparisonGridsAndValuesFail(testCase)
            a = struct('t',[0,1],'q',zeros(3,2)); b = a; b.t=[0,0.9];
            testCase.verifyError(@() compareParkingStates(a,b),'compareParkingStates:TimeMismatch');
            b = a; b.q(1,2)=NaN;
            testCase.verifyError(@() compareParkingStates(a,b),'MATLAB:expectedFinite');
            testCase.verifyError(@() parkingConvergence([0.05,0.1],[1,2],0.05), ...
                'parkingConvergence:GridOrder');
        end
        function straightAnalyticInputReferenceIsExact(testCase)
            [p,sc,v] = straightPlan(1);
            timing = timingSettings(1,0.3);
            r = integrateParkingReference(p,sc,v,timing);
            t = timeParkingPlan(p,v,timing);
            sampled = sampleParkingReference(r,t);
            testCase.verifyEqual(sampled.q,t.q,AbsTol=1e-10);
            testCase.verifyEqual(sampled.t,[0,0.3,0.6,0.9,1],AbsTol=1e-14);
        end
        function referenceActuallyPropagatesPerturbedInitialStateAcrossThreeSegments(testCase)
            [p,sc,v] = straightPlan(3);
            sc.initialPose = [0;0.1;0.02];
            timing = timingSettings([1,1,1],0.3);
            r = integrateParkingReference(p,sc,v,timing);
            testCase.verifyEqual(r.segments(2).initialPose,r.segments(1).terminalPose);
            testCase.verifyEqual(r.segments(3).initialPose,r.segments(2).terminalPose);
            testCase.verifyGreaterThan(norm(r.segments(2).initialPose-p.segments(2).startPose),0.1);
            testCase.verifyEqual(r.segments(3).terminalPose, ...
                [3*cos(0.02);0.1+3*sin(0.02);0.02],AbsTol=1e-10);
        end
        function referenceSolutionDoesNotDependOnRequestedDt(testCase)
            [p,sc,v] = straightPlan(1);
            a = integrateParkingReference(p,sc,v,timingSettings(1,0.1));
            b = integrateParkingReference(p,sc,v,timingSettings(1,0.03));
            testCase.verifyEqual(a.segments.solution.x,b.segments.solution.x);
            testCase.verifyEqual(a.segments.solution.y,b.segments.solution.y);
        end
        function exactHeldArcMatchesClosedForm(testCase)
            v = vehicleParameters(); sc.initialPose=[0;0;0];
            part = struct('t',[0,0.2,0.7],'globalT',[0,0.2,0.7], ...
                'v',[2,2,0],'delta',[atan(v.wheelbase/2),atan(v.wheelbase/2),0], ...
                'segmentIndex',[1,1,1]);
            t = struct('status','success','message','','segments',part);
            s = simulateHeldParkingPlan(t,sc,v);
            testCase.verifyEqual(s.q(:,end),[2*sin(0.7);2*(1-cos(0.7));0.7],AbsTol=1e-14);
            e = simulateKinematics(sc.initialPose,part.t,part.v,part.delta,v);
            testCase.verifyGreaterThan(norm(s.q(1:2,end)-e(1:2,end)),0.1);
        end
        function heldStraightMotionAndStoppedIntervals(testCase)
            [p,sc,v] = straightPlan(3);
            t = timeParkingPlan(p,v,timingSettings([1,1,1],0.3));
            held = simulateHeldParkingPlan(t,sc,v);
            euler = simulateParkingPlan(t,sc,v);
            testCase.verifyEqual(held.q,euler.q,AbsTol=1e-14);
            testCase.verifyEqual(held.segments(2).q(:,1),held.segments(1).q(:,end));
        end
        function failedPlanCannotProduceAccurateReference(testCase)
            [p,sc,v] = straightPlan(1); p.status.code='not_found'; p.segments=struct([]);
            r = integrateParkingReference(p,sc,v,timingSettings());
            t = timeParkingPlan(p,v,timingSettings());
            sampled = sampleParkingReference(r,t);
            testCase.verifyEqual(r.status,'not_run');
            testCase.verifyEmpty(r.segments);
            testCase.verifyEmpty(sampled.q);
        end
    end
end

function [p,sc,v] = straightPlan(count)
    pieces = cell(1,count);
    for k = 1:count
        pieces{k} = cubicPath([k-1;0;0],[k;0;0],1,1,2,1);
    end
    p = struct('segments',[pieces{:}],'status',struct('code','feasible','message','Synthetic straight path.'), ...
        'diagnostics',struct());
    sc = struct('initialPose',[0;0;0],'targetPose',[count;0;0]);
    v = vehicleParameters();
end
