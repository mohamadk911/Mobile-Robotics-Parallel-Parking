function vehicle = vehicleParameters()

vehicle.length = 4.50;              % m
vehicle.width = 1.80;               % m
vehicle.wheelbase = 2.70;           % m

vehicle.rearOverhang = 0.90;        % m
vehicle.frontOverhang = 0.90;       % m

vehicle.maxSteering = deg2rad(35);  % rad

vehicle.minTurningRadius = ...
    vehicle.wheelbase / tan(vehicle.maxSteering);
end