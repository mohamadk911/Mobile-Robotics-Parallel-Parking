function combined = combineTimedSegments(segments, fields)
%combineTimedSegments Keep outgoing samples at shared stopped joins.
% Validate each local/global grid and every requested numeric sample series.
% Drop each nonfinal segment's terminal sample, retaining it in segment data.
    combined = struct();
    count = numel(segments);
    offset = 0;
    for k = 1:count
        segment = segments(k);
        validateattributes(segment.t,{'double'},{'real','finite','row','nonempty'});
        n = numel(segment.t);
        assert(n >= 2 && segment.t(1) == 0 && all(diff(segment.t) > 0), ...
            'parkingTiming:InvalidTime','Each segment needs increasing local time starting at zero.');
        assert(isequal(segment.globalT,offset+segment.t), ...
            'parkingTiming:InvalidTime','Global time must equal local time plus elapsed duration.');
        assert(isequal(segment.segmentIndex,repmat(k,1,n)), ...
            'parkingTiming:InvalidIndex','Segment indices do not match segment order.');
        keep = 1:n-(k < count);
        for j = 1:numel(fields)
            name = fields{j};
            rows = 1+2*strcmp(name,'q');
            value = segment.(name);
            validateattributes(value,{'double'},{'real','finite','size',[rows,n]});
            if k == 1
                combined.(name) = value(:,keep);
            else
                combined.(name) = [combined.(name),value(:,keep)];
            end
        end
        offset = segment.globalT(end);
    end
    combined.localT = combined.t;
    combined.t = combined.globalT;
    assert(all(diff(combined.t) > 0), 'parkingTiming:InvalidTime', ...
        'Combined elapsed time must be strictly increasing at floating-point precision.');
end
