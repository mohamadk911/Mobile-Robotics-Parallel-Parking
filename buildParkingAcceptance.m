function acceptance = buildParkingAcceptance(step10,step11)
%buildParkingAcceptance Consolidate audited Step 10/11 results without planning.
% A = buildParkingAcceptance(STEP10,STEP11) takes the results structs returned
% by the accepted runners (or loaded from their MAT files). No files are
% written. Metadata/scenario/geometry/metric discrepancies raise an error;
% they are never repaired. Failed gates produce NOT_ACCEPTED, unavailable
% plans produce NO_VALIDATED_PLAN, and both successful gates are required for
% ACCEPTED_POC. This is a POC evidence classification, not safety certification.
% Full precision is retained; steering presentation columns use degrees.
    vehicle = vehicleParameters(); planning = plannerSettings();
    timing = timingSettings(); quality = evaluationSettings();
    settings = sensitivitySettings();
    expected = struct('vehicle',vehicle,'plannerSettings',planning, ...
        'timingSettings',timing,'evaluationSettings',quality);
    names = fieldnames(expected);
    for k = 1:numel(names)
        name = names{k};
        require(isequal(step10.metadata.(name),expected.(name)) && ...
            isequal(step11.metadata.(name),expected.(name)),['Settings mismatch: ',name]);
    end
    require(isequal(step11.settings,settings),'Sensitivity settings mismatch.');
    ids = ["nominal";"tight";"changedStart"];
    require(numel(step10.experiments)==3 && numel(step11.studies)==3 && ...
        isequal(step10.summary.scenarioId,ids),'Expected exactly the three required scenarios.');
    summary = step10.summary;
    classification = repmat("NOT_ACCEPTED",3,1);
    convergenceStatus = strings(3,1);
    metricNames = fieldnames(step10.experiments(1).evaluation.metrics);
    for k = 1:3
        e = step10.experiments(k); s = step11.studies(k);
        require(isequal(e.scenario,scenarioParameters(vehicle,ids(k))) && ...
            isequal(s.scenario,e.scenario),'Scenario definition mismatch.');
        require(strcmp(e.plan.status.code,s.plan.status.code) && ...
            isequal(e.plan.segments,s.plan.segments),'Step 10/11 plan mismatch.');
        require(strcmp(summary.plannerStatus(k),e.plan.status.code) && ...
            strcmp(summary.evaluationStatus(k),e.evaluation.status) && ...
            summary.passed(k)==e.evaluation.passed,'Summary status mismatch.');
        require(isequal(e.evaluation.settings,quality),'Evaluation threshold mismatch.');
        for j = 1:numel(metricNames)
            name = metricNames{j};
            require(isequaln(summary.(name)(k),e.evaluation.metrics.(name)), ...
                ['Summary metric mismatch: ',name]);
        end
        convergenceStatus(k) = string(s.status);
        if ~strcmp(e.plan.status.code,'feasible') || isempty(e.plan.segments)
            unavailable = rmfield(e.evaluation.metrics,'vehicleMinTurningRadius');
            require(isempty(e.plan.segments) && isempty(e.trajectory.q) && ...
                isempty(e.simulation.q) && isempty(s.runs) && isempty(s.summary) && ...
                isempty(s.reference) && ~e.evaluation.passed && ~s.passed && ...
                strcmp(e.evaluation.status,'not_evaluated') && ...
                strcmp(s.status,'not_applicable') && isnan(summary.pathLength(k)) && ...
                all(isnan(cell2mat(struct2cell(unavailable)))), ...
                'Failed plan contains fabricated successful evidence or metrics.');
            classification(k) = "NO_VALIDATED_PLAN";
            continue
        end
        require(isequal([s.runs.dt],settings.dt) && isequal(s.summary.dt,settings.dt.'), ...
            'Sensitivity grid mismatch.');
        g = validateCubicGeometry(e.plan.segments,vehicle,'stopped');
        fresh = evaluateParkingExperiment(e.plan,e.trajectory,e.simulation,e.scenario,vehicle,quality);
        require(isequaln(fresh.metrics,e.evaluation.metrics),'Reevaluated metrics differ from saved data.');
        require(isequaln(summary.pathLength(k),sum(cubicPathLength(e.plan.segments, ...
            planning.lengthIntegration.AbsTol,planning.lengthIntegration.RelTol))), ...
            'Path-length mismatch.');
        baseline = find(settings.dt==timing.dt);
        b = s.runs(baseline);
        require(isequal(b.timing,timing) && isequal(b.trajectory,e.trajectory) && ...
            isequal(b.euler,e.simulation) && isequaln(b.evaluation.metrics,fresh.metrics), ...
            'Step 10/11 baseline mismatch.');
        for j = 1:numel(s.runs)
            run = s.runs(j);
            require(isequal(run.planSegments,e.plan.segments) && ...
                isequal(run.timing,timingSettings(timing.segmentDurations,settings.dt(j))) && ...
                isequal(run.evaluation.settings,quality),'Frozen study geometry/settings mismatch.');
        end
        recalculated = parkingConvergence(settings.dt,s.summary.numericalXYRMSE.',timing.dt);
        require(isequaln(recalculated,s.convergence),'Convergence table mismatch.');
        qualityPass = strcmp(e.evaluation.status,'pass') && e.evaluation.passed && ...
            numel(e.evaluation.checks)==26 && all([e.evaluation.checks.passed]) && fresh.passed;
        convergencePass = strcmp(s.status,'pass') && s.passed && s.baselineJustified && ...
            numel(s.checks)==6 && all([s.checks.passed]) && ...
            all(s.summary.passed(baseline:end)) && ...
            all(diff(s.summary.numericalXYRMSE)<0) && ...
            all(recalculated.observedOrder(1:end-1)>=settings.criteria.orderRange(1) & ...
                recalculated.observedOrder(1:end-1)<=settings.criteria.orderRange(2));
        if g.passed && strcmp(e.trajectory.status,'success') && ...
                strcmp(e.simulation.status,'success') && qualityPass && convergencePass
            classification(k) = "ACCEPTED_POC";
        end
    end
    summary.finalClassification = classification;
    summary.convergenceStatus = convergenceStatus;
    summary.maxSteeringDeg = rad2deg(summary.maxSteering);
    summary.steeringMarginDeg = rad2deg(vehicle.maxSteering-summary.maxSteering);
    summary.Properties.VariableUnits(end-3:end) = {'','','deg','deg'};
    decision = 'POC OBJECTIVES NOT YET SATISFIED';
    if isequal(classification,["ACCEPTED_POC";"NO_VALIDATED_PLAN";"ACCEPTED_POC"]) && ...
            strcmp(summary.plannerStatus(2),'not_found')
        decision = 'POC OBJECTIVES SATISFIED WITH DOCUMENTED LIMITATIONS';
    end
    acceptance = struct('summary',summary,'decision',decision, ...
        'settings',expected,'sensitivitySettings',settings, ...
        'sources',{{'results/step10/parking_experiments.mat','results/step11/parking_sensitivity.mat'}});
end

function require(condition,message)
    assert(condition,'buildParkingAcceptance:EvidenceMismatch','%s',message);
end
