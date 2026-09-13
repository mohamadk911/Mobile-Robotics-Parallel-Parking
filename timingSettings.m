function settings = timingSettings(segmentDurations, dt)
%timingSettings Stop-to-stop timing only, in seconds.
% Defaults are [7,7] and 0.05. Supply one duration per selected segment;
% durations are never broadcast or inferred from geometry.
    if nargin < 1
        segmentDurations = [7,7];
    end
    if nargin < 2
        dt = 0.05;
    end
    validateattributes(segmentDurations,{'numeric'}, ...
        {'real','finite','row','nonempty','positive'},mfilename,'segmentDurations');
    validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'},mfilename,'dt');
    settings = struct('segmentDurations',double(segmentDurations), ...
        'dt',double(dt),'joinPolicy','stopInstantaneousSteering');
end
