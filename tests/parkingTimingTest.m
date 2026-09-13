classdef parkingTimingTest < matlab.unittest.TestCase
%parkingTimingTest Step 8 data/timing/propagation contract, no quality gate.
    properties
        Vehicle
        Scenarios
        Plans
        Trajectories
        Simulations
    end
    properties (TestParameter)
        scenarioId = {'nominal','changedStart'};
        segmentCount = {1,3,4};
        badNumber = struct('zero',0,'negative',-1,'nan',NaN,'inf',Inf, ...
            'complex',1i,'text','bad','logical',true,'empty',[]);
        failedCode = {'not_found','invalid_input','solver_error'};
        badPlan = struct('numeric',1,'missing',struct(), ...
            'badStatus',struct('segments',struct([]),'status',struct('code','unknown','message',''), ...
            'diagnostics',struct()));
        badSegment = struct('coefficients',struct('field','coefficients','value',nan(2,4)), ...
            'direction',struct('field','direction','value',0), ...
            'endpoint',struct('field','endPose','value',[100;0;0]), ...
            'degenerate',struct('field','coefficients','value',zeros(2,4)));
    end
    methods (TestClassSetup)
        function prepare(testCase)
            folder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
            testCase.Vehicle = vehicleParameters();
            settings = plannerSettings();
            ids = {'nominal','tight','changedStart'};
            for k = 1:numel(ids)
                id = ids{k};
                scenario = scenarioParameters(testCase.Vehicle,id);
                plan = planCubicParking(testCase.Vehicle,scenario,settings);
                trajectory = timeParkingPlan(plan,testCase.Vehicle,timingSettings());
                simulation = simulateParkingPlan(trajectory,scenario,testCase.Vehicle);
                testCase.Scenarios.(id) = scenario;
                testCase.Plans.(id) = plan;
                testCase.Trajectories.(id) = trajectory;
                testCase.Simulations.(id) = simulation;
            end
        end
    end
    methods (Test)
        function defaultsContainOnlyTiming(testCase)
            settings = timingSettings();
            testCase.verifyEqual(settings,struct('segmentDurations',[7,7], ...
                'dt',0.05,'joinPolicy','stopInstantaneousSteering'));
        end
        function processesRequiredScenario(testCase,scenarioId)
            t = testCase.Trajectories.(scenarioId);
            s = testCase.Simulations.(scenarioId);
            testCase.verifyEqual(t.status,'success',t.message);
            testCase.verifyEqual(s.status,'success',s.message);
            testCase.verifySize(t.segments,[1,numel(testCase.Plans.(scenarioId).segments)]);
            testCase.verifySize(s.segments,size(t.segments));
            testCase.verifySize(t.q,[3,281]);
            testCase.verifySize(s.q,[3,281]);
            testCase.verifyEqual(t.t([1,end]),[0,14]);
            testCase.verifyEqual(s.t,t.t);
            testCase.verifyEqual(s.method,'euler');
            testCase.verifyEqual(s.q(:,1),testCase.Scenarios.(scenarioId).initialPose);
            testCase.verifyTrue(all(isfinite([t.q(:);t.v(:);t.omega(:);t.delta(:); ...
                t.curvature(:);s.q(:);s.v(:);s.delta(:)])));
            printDiagnostics(scenarioId,t,s);
        end
        function exactBoundariesAndAnalyticCommands(testCase,scenarioId)
            t = testCase.Trajectories.(scenarioId);
            a = t.segments(1); b = t.segments(2);
            testCase.verifyEqual([a.t([1,end]),b.t([1,end])],[0,7,0,7]);
            testCase.verifyEqual([a.s([1,end]),b.s([1,end])],[0,1,0,1]);
            testCase.verifyEqual([a.v([1,end]),b.v([1,end])],zeros(1,4));
            expected = pathToInputs(testCase.Plans.(scenarioId).segments(2), ...
                timeScale(7,0.05),testCase.Vehicle);
            testCase.verifyEqual(b.delta,expected.delta);
            testCase.verifyEqual(b.omega,expected.omega);
            testCase.verifyEqual(b.x,expected.x);
            testCase.verifyEqual(b.globalT,7+b.t);
        end
        function joinOnceWithOutgoingCommands(testCase,scenarioId)
            t = testCase.Trajectories.(scenarioId);
            s = testCase.Simulations.(scenarioId);
            a = t.segments(1); b = t.segments(2);
            join = find(t.t == 7);
            testCase.verifyNumElements(join,1);
            testCase.verifyGreaterThan(diff(t.t),0);
            testCase.verifyEqual(t.delta(join),b.delta(1));
            testCase.verifyEqual(t.curvature(join),b.curvature(1));
            testCase.verifyEqual(t.segmentIndex(join),2);
            testCase.verifyEqual(t.localT(join),0);
            testCase.verifyEqual(t.s(join),0);
            testCase.verifyEqual(t.q(:,join),b.q(:,1));
            testCase.verifyEqual(s.delta(join),b.delta(1));
            testCase.verifyEqual(s.q(:,join),s.segments(1).q(:,end));
            testCase.verifyGreaterThan(abs(a.delta(end)-b.delta(1)),0.1);
            testCase.verifyEqual([a.v(end),b.v(1)],[0,0]);
            testCase.verifyEqual(a.q(:,end),b.q(:,1),AbsTol=1e-12);
        end
        function actualStatePropagatesAndResetWouldDiffer(testCase,scenarioId)
            t = testCase.Trajectories.(scenarioId);
            s = testCase.Simulations.(scenarioId);
            plannedMidpoint = testCase.Plans.(scenarioId).segments(2).startPose;
            reset = simulateKinematics(plannedMidpoint,t.segments(2).t, ...
                t.segments(2).v,t.segments(2).delta,testCase.Vehicle);
            testCase.verifyEqual(s.segments(2).q(:,1),s.segments(1).q(:,end));
            testCase.verifyGreaterThan(norm(s.segments(2).q(:,1)-plannedMidpoint),1e-4);
            testCase.verifyGreaterThan(norm(s.q(:,end)-reset(:,end)),1e-4);
            % These are mutation-detection bounds, never scenario quality limits.
        end
        function supportsVariableSegmentCounts(testCase,segmentCount)
            [plan,scenario] = straightPlan(segmentCount);
            t = timeParkingPlan(plan,testCase.Vehicle,timingSettings(ones(1,segmentCount),0.3));
            s = simulateParkingPlan(t,scenario,testCase.Vehicle);
            testCase.verifySize(t.segments,[1,segmentCount]);
            testCase.verifySize(s.segments,[1,segmentCount]);
            testCase.verifyNumElements(t.t,4*segmentCount+1);
            testCase.verifyEqual(t.t(end),double(segmentCount));
            testCase.verifyEqual(unique(t.segmentIndex),1:segmentCount);
            testCase.verifyGreaterThan(diff(t.t),0);
            testCase.verifyEqual(s.q(1,end),0.864*sum([plan.segments.direction]),AbsTol=1e-14);
            starts = arrayfun(@(part) part.q(:,1),s.segments(2:end),UniformOutput=false);
            ends = arrayfun(@(part) part.q(:,end),s.segments(1:end-1),UniformOutput=false);
            testCase.verifyEqual(starts,ends);
        end
        function shorterLastIntervalUsesItsActualLength(testCase)
            [plan,scenario] = straightPlan(1);
            t = timeParkingPlan(plan,testCase.Vehicle,timingSettings(1,0.3));
            s = simulateParkingPlan(t,scenario,testCase.Vehicle);
            testCase.verifyEqual(t.t,[0,0.3,0.6,0.9,1],AbsTol=1e-15);
            testCase.verifyEqual(t.segments.t(end),1);
            testCase.verifyEqual(s.q(1,:),[0,0,0.378,0.81,0.864],AbsTol=1e-14);
        end
        function stepLargerThanDurationRetainsStoppedEulerBehavior(testCase)
            [plan,scenario] = straightPlan(3);
            t = timeParkingPlan(plan,testCase.Vehicle,timingSettings([0.2,0.3,0.4],1));
            s = simulateParkingPlan(t,scenario,testCase.Vehicle);
            testCase.verifyEqual(t.t,[0,0.2,0.5,0.9],AbsTol=1e-15);
            testCase.verifyEqual([t.segments.v],zeros(1,6));
            testCase.verifyEqual(s.q,repmat(scenario.initialPose,1,4));
            testCase.verifyGreaterThan(norm(t.q(:,end)-s.q(:,end)),0.5);
        end
        function framePlacementDoesNotAffectProcessing(testCase,scenarioId)
            scenario = testCase.Scenarios.(scenarioId);
            scenario.framePoseWorld = [123;-45;2.1];
            t = timeParkingPlan(testCase.Plans.(scenarioId),testCase.Vehicle,timingSettings());
            s = simulateParkingPlan(t,scenario,testCase.Vehicle);
            testCase.verifyEqual(t,testCase.Trajectories.(scenarioId));
            testCase.verifyEqual(s,testCase.Simulations.(scenarioId));
        end
        function tightHasNoFabricatedData(testCase)
            t = testCase.Trajectories.tight; s = testCase.Simulations.tight;
            testCase.verifyEqual(t.plannerStatus.code,'not_found');
            testCase.verifyEqual(t.status,'not_run');
            testCase.verifyEqual(s.status,'not_run');
            testCase.verifyEmpty(t.segments); testCase.verifyEmpty(s.segments);
            testCase.verifyEmpty(t.q); testCase.verifyEmpty(s.q);
            testCase.verifyEmpty(t.t); testCase.verifyEmpty(s.t);
            testCase.verifyEmpty(t.v); testCase.verifyEmpty(s.delta);
            testCase.verifyTrue(contains(s.message,'not_found'));
            testCase.verifyFalse(isfield(s,'metrics'));
        end
        function failedStatusNeverProcessesEvenWithSegments(testCase,failedCode)
            plan = testCase.Plans.nominal;
            plan.status = struct('code',failedCode,'message','Deliberate failure.');
            t = timeParkingPlan(plan,testCase.Vehicle,struct());
            s = simulateParkingPlan(t,struct(),testCase.Vehicle);
            testCase.verifyEqual(t.status,'not_run');
            testCase.verifyEqual(s.status,'not_run');
            testCase.verifyEmpty(t.q); testCase.verifyEmpty(s.q);
            testCase.verifyTrue(contains(s.message,failedCode));
        end
        function emptyFeasiblePlanDoesNotRun(testCase)
            plan = testCase.Plans.nominal; plan.segments = struct([]);
            t = timeParkingPlan(plan,testCase.Vehicle,timingSettings());
            s = simulateParkingPlan(t,testCase.Scenarios.nominal,testCase.Vehicle);
            testCase.verifyEqual(s.status,'not_run');
            testCase.verifyEmpty(t.t); testCase.verifyEmpty(s.segments);
            testCase.verifyTrue(contains(t.message,'segment count 0'));
        end
        function rejectsDurationCount(testCase)
            testCase.verifyError(@() timeParkingPlan(testCase.Plans.nominal, ...
                testCase.Vehicle,timingSettings(7)), 'timeParkingPlan:DurationCount');
        end
        function rejectsInvalidDt(testCase,badNumber)
            settings = timingSettings(); settings.dt = badNumber;
            testCase.verifyTrue(throws(@() timeParkingPlan(testCase.Plans.nominal,testCase.Vehicle,settings)));
        end
        function rejectsInvalidDurations(testCase,badNumber)
            settings = timingSettings(); settings.segmentDurations = badNumber;
            testCase.verifyTrue(throws(@() timeParkingPlan(testCase.Plans.nominal,testCase.Vehicle,settings)));
        end
        function rejectsColumnDurationsAndNonScalarDt(testCase)
            testCase.verifyTrue(throws(@() timingSettings([7;7],0.05)));
            testCase.verifyTrue(throws(@() timingSettings([7,7],[0.05,0.1])));
        end
        function rejectsMalformedPlans(testCase,badPlan)
            testCase.verifyError(@() timeParkingPlan(badPlan,testCase.Vehicle,timingSettings()), ...
                'timeParkingPlan:InvalidPlan');
        end
        function rejectsMalformedGeometry(testCase,badSegment)
            plan = testCase.Plans.nominal;
            plan.segments(1).(badSegment.field) = badSegment.value;
            testCase.verifyError(@() timeParkingPlan(plan,testCase.Vehicle,timingSettings()), ...
                'timeParkingPlan:InvalidGeometry');
        end
        function rejectsUnsupportedPolicyAndExtraSettings(testCase)
            settings = timingSettings(); settings.joinPolicy = 'moving';
            testCase.verifyError(@() timeParkingPlan(testCase.Plans.nominal,testCase.Vehicle,settings), ...
                'timeParkingPlan:InvalidJoinPolicy');
            settings = timingSettings(); settings.threshold = 1;
            testCase.verifyError(@() timeParkingPlan(testCase.Plans.nominal,testCase.Vehicle,settings), ...
                'timeParkingPlan:InvalidSettings');
        end
        function rejectsInvalidInitialPose(testCase)
            scenario = testCase.Scenarios.nominal; scenario.initialPose = [NaN;0;0];
            testCase.verifyError(@() simulateParkingPlan(testCase.Trajectories.nominal,scenario,testCase.Vehicle), ...
                'MATLAB:simulateParkingPlan:expectedFinite');
        end
        function rejectsInconsistentAndNonfiniteTimedData(testCase)
            t = testCase.Trajectories.nominal; t.v(2) = t.v(2)+1;
            testCase.verifyError(@() simulateParkingPlan(t,testCase.Scenarios.nominal,testCase.Vehicle), ...
                'simulateParkingPlan:InconsistentTrajectory');
            t = testCase.Trajectories.nominal; t.segments(2).delta = [0,0];
            testCase.verifyTrue(throws(@() simulateParkingPlan(t,testCase.Scenarios.nominal,testCase.Vehicle)));
            t = testCase.Trajectories.nominal; t.segments(1).q(1,1) = Inf;
            testCase.verifyTrue(throws(@() simulateParkingPlan(t,testCase.Scenarios.nominal,testCase.Vehicle)));
        end
        function rejectsUnrepresentableGlobalGrid(testCase)
            [plan,~] = straightPlan(3);
            testCase.verifyError(@() timeParkingPlan(plan,testCase.Vehicle,timingSettings([1e16,1,1],1e16)), ...
                'parkingTiming:InvalidTime');
        end
    end
end

function [plan,scenario] = straightPlan(count)
% Alternate forward/reverse at stops; arbitrary M, no parking planner family.
    x = 0; pieces = cell(1,count);
    for k = 1:count
        direction = (-1)^(k+1);
        pieces{k} = cubicPath([x;0;0],[x+direction;0;0],1,1,2,direction);
        x = x+direction;
    end
    plan = struct('segments',[pieces{:}],'status',struct('code','feasible','message','Synthetic.'), ...
        'diagnostics',struct());
    scenario = struct('initialPose',[0;0;0]);
end

function yes = throws(action)
% Test helper for rejection cases whose low-level validation ID may differ.
    yes = false;
    try
        action();
    catch
        yes = true;
    end
end

function printDiagnostics(id,t,s)
    difference = s.q(:,end)-t.q(:,end);
    fprintf('%s Step 8: duration=%.12g, samples=%d, planned=%s, simulated=%s, positionDifference=%.15g m, wrappedHeadingDifference=%.15g rad, maxSteering=%.15g deg, actualJoin=%d\n', ...
        id,t.t(end),numel(t.t),mat2str(t.q(:,end).',15),mat2str(s.q(:,end).',15), ...
        norm(difference(1:2)),atan2(sin(difference(3)),cos(difference(3))), ...
        rad2deg(max(abs([t.segments.delta]))),isequal(s.segments(2).q(:,1),s.segments(1).q(:,end)));
end
