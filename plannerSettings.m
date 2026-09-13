function settings = plannerSettings()
%plannerSettings Shared deterministic search rules for two reverse cubics.
% No scene, timing, simulation or quality-gate data belongs in this struct.
% Variables are [midX;midY;midHeading;k1Start;k1End;k2Start;k2End].
% Positions/tangents are scaled by start-to-target distance for fmincon.
% Midpoint x stays between endpoint x values; y extends their interval by
% one vehicle width on either side. Heading covers the rearward-parking
% branch [-pi/2,pi/2]. Tangents span 0.05 to 3 times endpoint distance.
% These are search restrictions, not a physical infeasibility certificate.
%
% Objective: total arc length only (weight 1); no smoothness penalty.
% At a declared stopped join, curvature continuity is not required.
% Six starts: chord fractions 0.4/0.5/0.6, intermediate heading equal to
% 1 or 2 times the reverse chord body heading, tangent = segment chord.
% This does not impose equal optimized tangents or nominal symmetry.
%
% SQP constrains signed curvature at 33 points per segment, then inserts any
% violating analytic extrema from independent validation and resolves (up to
% 4 refinements). Fixed point constraints within each solve avoid the kinks
% of optimizing a maximum directly. No sampled check establishes acceptance.
% ConstraintTolerance is a solver stopping test, NEVER a physical allowance.
% Retain independently validated iterates, including when a final iterate
% lies slightly outside a strict vehicle limit. No limit is changed.

    settings.solver = 'fmincon';
    settings.directionPattern = [-1,-1];
    settings.bounds = struct('lateralPaddingWidths',1, ...
        'heading',[-pi/2,pi/2],'tangentDistanceFactors',[0.05,3]);
    settings.objectiveWeights = struct('length',1);
    settings.initialization = struct('chordFractions',[0.4,0.5,0.6], ...
        'headingFactors',[1,2]);
    settings.solverOptions = struct('Algorithm','sqp','Display','off', ...
        'MaxIterations',150,'MaxFunctionEvaluations',2500, ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-7, ...
        'StepTolerance',1e-10,'FiniteDifferenceType','forward');
    settings.lengthIntegration = struct('AbsTol',1e-9,'RelTol',1e-9);
    settings.constraintRefinement = struct('initialPointCount',33,'maxRefinements',4);
    % The validator owns its immutable geometric tolerances; do not override.
    settings.feasibilityTolerances = 'validateCubicGeometry defaults';
end
