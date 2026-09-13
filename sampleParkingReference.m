function simulation = sampleParkingReference(reference, trajectory)
%sampleParkingReference Evaluate an ode45 solution on a timed plan's grid.
% No reintegration: deval uses ode45's continuous extension within each
% segment, including exact endpoint states. Combined join uses outgoing data.
    simulation = struct('status','not_run','message','','segments',struct([]), ...
        't',[],'q',zeros(3,0),'method','ode45 analytic commands');
    if ~strcmp(reference.status,'success') || ~strcmp(trajectory.status,'success')
        simulation.message = 'Reference or timing unavailable; no samples generated.';
        return
    end
    count = numel(trajectory.segments);
    assert(count == numel(reference.segments),'sampleParkingReference:SegmentCount','Segment counts differ.');
    parts = cell(1,count);
    for k = 1:count
        t = trajectory.segments(k);
        r = reference.segments(k);
        assert(t.t(1) == 0 && t.t(end) == r.duration, ...
            'sampleParkingReference:DurationMismatch','Sample grid must span the frozen duration exactly.');
        q = deval(r.solution,t.t);
        q(:,1) = r.initialPose;
        q(:,end) = r.terminalPose;
        parts{k} = struct('t',t.t,'globalT',t.globalT,'q',q, ...
            'v',t.v,'delta',t.delta,'segmentIndex',t.segmentIndex);
    end
    simulation = combineTimedSegments([parts{:}],{'t','globalT','q','v','delta','segmentIndex'});
    simulation.segments = [parts{:}];
    simulation.method = 'ode45 analytic commands';
    simulation.status = 'success';
    simulation.message = 'Dense ode45 output sampled at identical reference/Euler times.';
end
