function simulation = simulateHeldParkingPlan(trajectory, scenario, vehicle)
%simulateHeldParkingPlan Exact circular-arc propagation for LEFT-HELD commands.
% Diagnostic only; the accepted Euler implementation remains unchanged.
% On each actual interval, v/delta are constant. Their bicycle solution is
% distance*sinc(turn/2) at heading theta+turn/2, where sinc(z)=sin(z)/z.
% This isolates Euler state-integration error from command sampling: compare
% Euler to this held solution, and this held solution to continuous ode45.
    simulation = struct('status','not_run','message',trajectory.message, ...
        'segments',struct([]),'t',[],'q',zeros(3,0),'method','exact left-held commands');
    if ~strcmp(trajectory.status,'success')
        return
    end
    validateattributes(scenario.initialPose,{'numeric'},{'real','finite','size',[3,1]});
    validateattributes(vehicle.wheelbase,{'numeric'},{'real','finite','scalar','positive'});
    q0 = double(scenario.initialPose);
    parts = cell(1,numel(trajectory.segments));
    for k = 1:numel(parts)
        part = trajectory.segments(k);
        intervals = diff(part.t);
        assert(all(isfinite(intervals)) && all(intervals > 0), ...
            'simulateHeldParkingPlan:InvalidTime','Intervals must be finite and positive.');
        validateattributes(part.v,{'double'},{'real','finite','size',size(part.t)});
        validateattributes(part.delta,{'double'},{'real','finite','size',size(part.t)});
        q = zeros(3,numel(part.t)); q(:,1) = q0;
        for j = 1:numel(intervals)
            distance = part.v(j)*intervals(j);
            turn = distance*tan(part.delta(j))/vehicle.wheelbase;
            z = turn/2;
            if abs(z) < 1e-4
                factor = 1-z^2/6+z^4/120-z^6/5040;
            else
                factor = sin(z)/z;
            end
            angle = q(3,j)+z;
            q(:,j+1) = q(:,j)+[distance*factor*[cos(angle);sin(angle)];turn];
        end
        parts{k} = struct('t',part.t,'globalT',part.globalT,'q',q, ...
            'v',part.v,'delta',part.delta,'segmentIndex',part.segmentIndex);
        q0 = q(:,end);
    end
    simulation = combineTimedSegments([parts{:}],{'t','globalT','q','v','delta','segmentIndex'});
    simulation.segments = [parts{:}];
    simulation.method = 'exact left-held commands';
    simulation.status = 'success';
    simulation.message = 'Exact held-command arcs, actual intervals and actual-state joins.';
end
