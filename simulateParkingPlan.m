function simulation = simulateParkingPlan(trajectory, scenario, vehicle)
%simulateParkingPlan Forward Euler propagation of a complete timed local plan.
% S = simulateParkingPlan(timeParkingPlan(PLAN,V,TIMING),SCENARIO,V).
% First state is scenario.initialPose. Each subsequent segment starts at the
% ACTUAL preceding terminal state. Reference poses never initialize a join.
% On [t(j),t(j+1)), apply v(j),delta(j) with explicit Euler at q(:,j).
% Terminal commands have no following interval within their segment.
% Both stopped-join sides are retained in S.segments; combined commands are
% outgoing/right-hand, without steering interpolation or added dwell time.
% framePoseWorld is unused. 'success' means propagation ran, not quality pass.
    simulation = struct('segments',struct([]),'t',[],'localT',[],'globalT',[], ...
        'q',zeros(3,0),'v',[],'delta',[],'segmentIndex',[], ...
        'method','euler','status','not_run','message','');
    assert(isstruct(trajectory) && isscalar(trajectory) && ...
        all(isfield(trajectory,{'status','message','segments'})), ...
        'simulateParkingPlan:InvalidTrajectory','Supply a timeParkingPlan result.');
    assert(any(strcmp(trajectory.status,{'success','failed','not_run'})), ...
        'simulateParkingPlan:InvalidTrajectory','Unknown timing status.');
    if ~strcmp(trajectory.status,'success') || isempty(trajectory.segments)
        simulation.message = sprintf('Simulation not run: timing %s; %s', ...
            trajectory.status,trajectory.message);
        return
    end
    assert(isstruct(scenario) && isscalar(scenario) && isfield(scenario,'initialPose'), ...
        'simulateParkingPlan:InvalidScenario','Scenario must provide initialPose.');
    validateattributes(scenario.initialPose,{'numeric'}, ...
        {'real','finite','size',[3,1]},mfilename,'scenario.initialPose');
    assert(isstruct(trajectory.segments) && isrow(trajectory.segments) && ...
        strcmp(trajectory.joinPolicy,'stopInstantaneousSteering'), ...
        'simulateParkingPlan:InvalidTrajectory','Expected stopped timed segment struct row.');
    fields = {'t','globalT','s','q','x','y','theta','v','omega','delta','curvature','segmentIndex'};
    combined = combineTimedSegments(trajectory.segments,fields);
    names = fieldnames(combined);
    for k = 1:numel(names)
        assert(isequal(trajectory.(names{k}),combined.(names{k})), ...
            'simulateParkingPlan:InconsistentTrajectory','Combined and per-segment arrays disagree.');
    end
    count = numel(trajectory.segments);
    pieces = cell(1,count);
    q0 = double(scenario.initialPose);
    for k = 1:count
        part = trajectory.segments(k);
        assert(all(part.v([1,end]) == 0) && isequal(part.s([1,end]),[0,1]) && ...
            isequal(part.q,[part.x;part.y;part.theta]), ...
            'simulateParkingPlan:InvalidTrajectory','Invalid stopped boundaries or reference pose arrays.');
        q = simulateKinematics(q0,part.t,part.v,part.delta,vehicle);
        pieces{k} = struct('t',part.t,'globalT',part.globalT,'q',q, ...
            'v',part.v,'delta',part.delta,'segmentIndex',part.segmentIndex);
        q0 = q(:,end);
    end
    segments = [pieces{:}];
    combined = combineTimedSegments(segments,{'t','globalT','q','v','delta','segmentIndex'});
    names = fieldnames(combined);
    for k = 1:numel(names)
        simulation.(names{k}) = combined.(names{k});
    end
    simulation.segments = segments;
    simulation.status = 'success';
    simulation.message = 'Euler propagation completed using actual terminal states; quality not evaluated.';
end
