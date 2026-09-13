function reference = integrateParkingReference(plan, scenario, vehicle, timing, tolerances)
%integrateParkingReference ode45 propagation of continuous analytic commands.
% R = integrateParkingReference(PLAN,SCENARIO,VEHICLE,TIMING,TOLERANCES).
% TOLERANCES has RelTol and 3-by-1 AbsTol for x/y metres and heading radians.
% Defaults: sensitivitySettings().reference. Each segment is solved ONCE on
% [0,T], independently of output dt; use sampleParkingReference for queries.
% At RHS query t, evaluate the accepted cubic s(t), sdot(t), then pathToInputs
% analytically. No sampled-command interpolation or planned-state resets.
% Separate ode45 calls stop exactly at joins; actual terminal q initializes
% the next solve, so a steering jump is never treated as smooth. No dwell.
% Dense-output solutions and individual initial/terminal states are retained.
    if nargin < 5
        settings = sensitivitySettings(); tolerances = settings.reference;
    end
    validateattributes(tolerances,{'struct'},{'scalar'});
    validateattributes(tolerances.RelTol,{'numeric'},{'real','finite','scalar','>=',100*eps,'<',1});
    validateattributes(tolerances.AbsTol,{'numeric'},{'real','finite','size',[3,1],'positive'});
    reference = struct('status','not_run','message','','method','ode45 analytic commands', ...
        'tolerances',tolerances,'segments',struct([]),'planSegments',plan.segments, ...
        'segmentDurations',timing.segmentDurations,'joinPolicy',timing.joinPolicy);
    % Reuse accepted plan/timing validation; this does not optimize geometry.
    validated = timeParkingPlan(plan,vehicle,timing);
    if ~strcmp(validated.status,'success')
        reference.message = validated.message;
        return
    end
    validateattributes(scenario.initialPose,{'numeric'},{'real','finite','size',[3,1]});
    q0 = double(scenario.initialPose);
    parts = cell(1,numel(plan.segments));
    options = odeset('RelTol',tolerances.RelTol,'AbsTol',tolerances.AbsTol);
    for k = 1:numel(parts)
        duration = timing.segmentDurations(k);
        segment = plan.segments(k);
        solution = ode45(@(t,q) analyticRHS(t,q,segment,duration,vehicle),[0,duration],q0,options);
        assert(solution.x(end) == duration && all(isfinite(solution.y),'all'), ...
            'integrateParkingReference:Incomplete','Reference solver did not reach a finite terminal state.');
        terminal = solution.y(:,end);
        parts{k} = struct('solution',solution,'initialPose',q0,'terminalPose',terminal, ...
            'duration',duration,'segmentIndex',k);
        q0 = terminal;
    end
    reference.segments = [parts{:}];
    reference.status = 'success';
    reference.message = 'Separate ode45 solves completed with actual-state propagation at joins.';
end

function dq = analyticRHS(t,q,segment,duration,vehicle)
    tau = t/duration;
    scaling = struct('t',t,'s',3*tau.^2-2*tau.^3,'sdot',(6*tau-6*tau.^2)/duration);
    command = pathToInputs(segment,scaling,vehicle);
    dq = carKinematics(q,command.v,command.delta,vehicle);
end
