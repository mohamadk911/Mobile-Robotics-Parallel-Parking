function scenario = scenarioParameters(vehicle, id, framePoseWorld, spaceLength)
%scenarioParameters Define aligned parking geometry in the parking-local frame.
% S = scenarioParameters(VEHICLE) retains the original nominal geometry/name.
% S = scenarioParameters(VEHICLE, ID) selects 'nominal', 'tight' or
% 'changedStart' (case-sensitive character row or scalar string).
% S = scenarioParameters(VEHICLE, ID, FRAMEPOSEWORLD) sets [ox;oy;phi],
% default [0;0;0], in metres/radians. It does not change any local pose.
% S = scenarioParameters(VEHICLE, ID, FRAMEPOSEWORLD, SPACELENGTH) explicitly
% overrides the preset positive bumper-to-bumper gap, for geometry experiments.
% Omit optional arguments for defaults; explicit empty values are invalid.
%
% Presets: nominal = 6.5 m, tight = 5.5 m, changedStart = 6.5 m with +10 deg
% initial heading. All starts use [0.76;2.8;heading] relative to the front car.
% Rear parked car rear axle is the local origin; +x is forward, +y outward.
% All output poses are rear-axle 3-by-1 columns in LOCAL coordinates. Use
% parkingLocalToWorld / parkingWorldToLocal to transform poses externally.
% No planner parameters, path, timing or feasibility decision are included.
% A positive gap smaller than the car remains a defined scene, not a claim
% that the car fits. The required experiments use the unmodified presets.

    if nargin < 2
        id = 'nominal';
    end
    if nargin < 3
        framePoseWorld = [0;0;0];
    end
    validId = (ischar(id) && isrow(id) && ~isempty(id)) || ...
        (isstring(id) && isscalar(id) && ~ismissing(id));
    if ~validId || ~any(strcmp(id,{'nominal','tight','changedStart'}))
        error('scenarioParameters:InvalidIdentifier', ...
            'ID must be nominal, tight or changedStart (case-sensitive).');
    end
    id = char(id);
    validateattributes(framePoseWorld,{'numeric'},{'real','finite','size',[3,1]}, ...
        mfilename,'framePoseWorld');
    dimensions = {'length','width','wheelbase','rearOverhang','frontOverhang'};
    if ~isstruct(vehicle) || ~isscalar(vehicle) || ~all(isfield(vehicle,dimensions))
        error('scenarioParameters:InvalidVehicle','Vehicle must provide all five dimensions as scalar fields.');
    end
    values = zeros(1,numel(dimensions));
    for k = 1:numel(dimensions)
        value = vehicle.(dimensions{k});
        if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
            error('scenarioParameters:InvalidVehicle','Vehicle dimensions must be finite real numeric scalars.');
        end
        values(k) = double(value);
    end
    if any(values(1:3) <= 0) || any(values(4:5) < 0)
        error('scenarioParameters:InvalidVehicle', ...
            'Length, width and wheelbase must be positive; overhangs must be nonnegative.');
    end
    bodyLength = values(3)+values(4)+values(5);
    % Internal dimension agreement: a roundoff allowance, not a fit tolerance.
    if ~isfinite(bodyLength) || abs(bodyLength-values(1)) > 64*eps(max(bodyLength,values(1)))
        error('scenarioParameters:InconsistentVehicle', ...
            'Vehicle length must equal wheelbase plus front and rear overhangs.');
    end

    heading = 0;
    presetGap = 6.5;
    switch id
        case 'nominal'
            name = 'Standard Parking';
        case 'tight'
            name = 'Tight Parking';
            presetGap = 5.5;
        case 'changedStart'
            name = 'Changed Start Pose';
            heading = deg2rad(10);
    end
    if nargin < 4
        spaceLength = presetGap;
    end
    validateattributes(spaceLength,{'numeric'},{'real','finite','scalar','positive'}, ...
        mfilename,'spaceLength');
    gap = double(spaceLength);

    scenario.id = id;
    scenario.name = name;
    scenario.spaceLength = gap;
    scenario.framePoseWorld = double(framePoseWorld);
    scenario.startOffsetFront = [0.76;2.8;heading];
    scenario.rearCarPose = [0;0;0];
    % Gap begins at the rear car's front bumper. The front car's rear axle
    % lies one rear overhang beyond the other end of that gap.
    rearCarFrontBumper = values(3)+values(5);
    scenario.frontCarPose = [values(1)+gap;0;0];
    scenario.initialPose = scenario.frontCarPose+scenario.startOffsetFront;
    scenario.targetPose = [rearCarFrontBumper+(gap-values(1))/2+values(4);0;0];
    if any(~isfinite([scenario.frontCarPose;scenario.initialPose;scenario.targetPose]))
        error('scenarioParameters:NonfiniteGeometry','Derived scene coordinates overflowed.');
    end
end
