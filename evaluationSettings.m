function settings = evaluationSettings()
%evaluationSettings Frozen POC quality thresholds, metres/radians.
% These are project acceptance criteria, NOT professor-specified values,
% automotive production tolerances or external standards. At car scale,
% 5 cm bounds terminal/transient XY disagreement; 1 degree bounds terminal
% orientation disagreement. A tighter 2 cm time-weighted RMS bound detects
% persistent error that a terminal check could miss. They assess numerical
% agreement of planned and open-loop simulated kinematics, not safe parking.
% Geometry tolerances belong to validateCubicGeometry; numerical distance
% tolerances belong to cubicPathDistance. No timing/planning data belongs here.
    settings = struct('finalPositionError',0.05,'finalHeadingError',pi/180, ...
        'maxTimeMatchedXYError',0.05,'timeWeightedXYRMSE',0.02);
end
