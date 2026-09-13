function settings = sensitivitySettings()
%sensitivitySettings Predeclared Step 11 grid and numerical evidence criteria.
% Frozen before the sensitivity run; these do not replace POC quality limits.
% Use whole-maneuver XY RMS for order, avoiding terminal cancellation.
% Baseline justification requires the four unchanged POC budgets (fraction<=1),
% not a claim of extra safety margin or suitability for a physical controller.
    settings.dt = [0.1,0.05,0.025,0.0125];
    settings.baselineDt = 0.05;
    settings.diagnosticDt = 0.03;
    settings.reference = struct('RelTol',1e-10,'AbsTol',[1e-12;1e-12;1e-13]);
    settings.refinedReference = struct('RelTol',1e-12,'AbsTol',[1e-14;1e-14;1e-14]);
    settings.criteria = struct('orderRange',[0.8,1.2], ...
        'referenceMaxXYChange',1e-7,'referenceMaxHeadingChange',1e-8, ...
        'referenceFractionOfFinestRMS',1e-3,'baselineQualityBudgetFraction',1);
end
