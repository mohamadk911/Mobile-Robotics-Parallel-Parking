function trajectory = timeParkingPlan(plan, vehicle, settings)
%timeParkingPlan Convert validated local cubic segments into analytic commands.
% T = timeParkingPlan(PLAN,VEHICLE,timingSettings()) supports any M>=1.
% T.segments(k) retains local t, globalT and both endpoints. Combined t is
% elapsed time; localT and s restart at joins. q=[x;y;theta]. All scalar
% series are rows. At a join the outgoing sample owns all combined fields.
% Headings use continuous branches across segments (whole-turn offsets only).
% A failed/empty plan returns status='not_run', its reason, and empty arrays.
% Malformed inputs throw. The existing Step 5 geometric validator is reused;
% no simulation quality criteria or evaluation metrics are introduced.
    trajectory = struct('segments',struct([]),'t',[],'localT',[],'globalT',[], ...
        's',[],'q',zeros(3,0),'x',[],'y',[],'theta',[],'v',[],'omega',[], ...
        'delta',[],'curvature',[],'segmentIndex',[],'status','not_run', ...
        'message','','plannerStatus',struct(),'joinPolicy','stopInstantaneousSteering');
    assert(isstruct(plan) && isscalar(plan) && ...
        all(isfield(plan,{'segments','status','diagnostics'})) && ...
        isstruct(plan.diagnostics) && isscalar(plan.diagnostics) && ...
        isstruct(plan.status) && isscalar(plan.status) && ...
        all(isfield(plan.status,{'code','message'})), ...
        'timeParkingPlan:InvalidPlan','Expected plan segments, status and diagnostics structs.');
    assert(textScalar(plan.status.code) && textScalar(plan.status.message) && ...
        any(strcmp(plan.status.code,{'feasible','not_found','invalid_input','solver_error'})), ...
        'timeParkingPlan:InvalidPlan','Invalid planner status code/message.');
    assert(isstruct(plan.segments) && (isempty(plan.segments) || isrow(plan.segments)), ...
        'timeParkingPlan:InvalidPlan','Segments must be a 1-by-M struct array.');
    trajectory.plannerStatus = plan.status;
    if ~strcmp(plan.status.code,'feasible') || isempty(plan.segments)
        trajectory.message = sprintf('Timing not run: planner %s; %s; segment count %d.', ...
            plan.status.code,plan.status.message,numel(plan.segments));
        return
    end
    assert(isstruct(settings) && isscalar(settings) && ...
        isequal(sort(fieldnames(settings)),sort({'segmentDurations';'dt';'joinPolicy'})), ...
        'timeParkingPlan:InvalidSettings','Use timingSettings fields only.');
    assert(textScalar(settings.joinPolicy) && strcmp(settings.joinPolicy,'stopInstantaneousSteering'), ...
        'timeParkingPlan:InvalidJoinPolicy','Only stopInstantaneousSteering is supported.');
    settings = timingSettings(settings.segmentDurations,settings.dt);
    count = numel(plan.segments);
    assert(numel(settings.segmentDurations) == count, ...
        'timeParkingPlan:DurationCount','Supply one positive duration per segment.');
    validation = validateCubicGeometry(plan.segments,vehicle,'stopped');
    assert(validation.passed,'timeParkingPlan:InvalidGeometry','%s',validation.message);
    pieces = cell(1,count);
    offset = 0;
    heading = double(plan.segments(1).startPose(3));
    for k = 1:count
        part = pathToInputs(plan.segments(k), ...
            timeScale(settings.segmentDurations(k),settings.dt),vehicle);
        part.theta = part.theta+2*pi*round((heading-part.theta(1))/(2*pi));
        part.q = [part.x;part.y;part.theta];
        part.globalT = offset+part.t;
        part.segmentIndex = repmat(k,size(part.t));
        assert(isequal(part.s([1,end]),[0,1]) && all(part.v([1,end]) == 0), ...
            'timeParkingPlan:InvalidBoundary','Each segment must span s=0 to 1 with zero endpoint speed.');
        pieces{k} = part;
        offset = part.globalT(end);
        heading = part.theta(end);
    end
    segments = [pieces{:}];
    combined = combineTimedSegments(segments, ...
        {'t','globalT','s','q','x','y','theta','v','omega','delta','curvature','segmentIndex'});
    names = fieldnames(combined);
    for k = 1:numel(names)
        trajectory.(names{k}) = combined.(names{k});
    end
    trajectory.segments = segments;
    trajectory.status = 'success';
    trajectory.message = 'Analytic timing completed; simulation quality has not been evaluated.';
end

function valid = textScalar(value)
    valid = (ischar(value) && (isrow(value) || isempty(value))) || ...
        (isstring(value) && isscalar(value) && ~ismissing(value));
end
