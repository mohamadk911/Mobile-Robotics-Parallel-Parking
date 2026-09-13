function results = runParkingExperiments(outputDirectory, overwrite)
%runParkingExperiments Reproduce the three accepted Step 10 experiments.
% R = runParkingExperiments() writes to results/step10 beside this function.
% R = runParkingExperiments(FOLDER) selects an output folder, independent of
% the current workspace/directory. R = runParkingExperiments(FOLDER,true)
% explicitly permits replacing the named Step 10 outputs in that folder.
% Existing named outputs otherwise cause an error BEFORE running experiments.
% Unrelated files are untouched. No planner/timing/evaluation overrides exist.
%
% R.summary is a three-row table (SI units, including radians). R.experiments
% retains every scenario, plan, timed reference, simulation and evaluation.
% R.metadata contains MATLAB version, all common settings and local scenarios.
% Six invisible figures are exported as 300-dpi PNG and vector PDF and closed.
% Tight gets a status text file, never a fabricated trajectory. A failed
% quality gate is reported as such, without suppressing available evidence.

    if nargin < 1
        outputDirectory = fullfile(fileparts(mfilename('fullpath')),'results','step10');
    end
    if nargin < 2
        overwrite = false;
    end
    validateattributes(overwrite,{'logical'},{'scalar'},mfilename,'overwrite');
    validFolder = (ischar(outputDirectory) && isrow(outputDirectory) && ~isempty(outputDirectory)) || ...
        (isstring(outputDirectory) && isscalar(outputDirectory) && ...
        ~ismissing(outputDirectory) && strlength(outputDirectory) > 0);
    if ~validFolder
        error('runParkingExperiments:InvalidDirectory','Output directory must be a nonempty path.');
    end
    outputDirectory = char(outputDirectory);
    names = {'scenario_summary.csv','parking_experiments.mat','tight_status.txt'};
    successfulIds = {'nominal','changedStart'};
    kinds = {'xy','errors','inputs'};
    for k = 1:numel(successfulIds)
        for j = 1:numel(kinds)
            base = [successfulIds{k},'_',kinds{j}];
            names = [names,{[base,'.png'],[base,'.pdf']}]; %#ok<AGROW>
        end
    end
    files = cellfun(@(name) fullfile(outputDirectory,name),names,UniformOutput=false);
    occupied = cellfun(@(path) isfile(path) || isfolder(path),files);
    if any(occupied) && ~overwrite
        error('runParkingExperiments:OutputExists', ...
            'Step 10 output already exists. Choose a new folder or explicitly pass overwrite=true.');
    end
    if any(cellfun(@isfolder,files))
        error('runParkingExperiments:OutputIsDirectory','An output filename is occupied by a directory.');
    end
    if ~isfolder(outputDirectory)
        [ok,message] = mkdir(outputDirectory);
        if ~ok
            error('runParkingExperiments:CreateDirectory','%s',message);
        end
    end
    [ok,folderInfo] = fileattrib(outputDirectory);
    if ~ok
        error('runParkingExperiments:DirectoryUnavailable','Cannot resolve output directory.');
    end
    outputDirectory = folderInfo.Name;
    if any(occupied)
        fprintf('Replacing %d explicitly authorized Step 10 outputs in %s\n',sum(occupied),outputDirectory);
    end

    vehicle = vehicleParameters();
    planning = plannerSettings();
    timing = timingSettings();
    quality = evaluationSettings();
    ids = {'nominal','tight','changedStart'};
    experimentCells = cell(1,numel(ids));
    rowCells = cell(1,numel(ids));
    for k = 1:numel(ids)
        scenario = scenarioParameters(vehicle,ids{k});
        plan = planCubicParking(vehicle,scenario,planning);
        trajectory = timeParkingPlan(plan,vehicle,timing);
        simulation = simulateParkingPlan(trajectory,scenario,vehicle);
        evaluation = evaluateParkingExperiment(plan,trajectory,simulation,scenario,vehicle,quality);
        experimentCells{k} = struct('scenario',scenario,'plan',plan, ...
            'trajectory',trajectory,'simulation',simulation,'evaluation',evaluation);
        pathLength = NaN;
        if strcmp(plan.status.code,'feasible') && ~isempty(plan.segments)
            pathLength = sum(cubicPathLength(plan.segments, ...
                planning.lengthIntegration.AbsTol,planning.lengthIntegration.RelTol));
        end
        row = struct('scenarioId',string(scenario.id),'scenarioName',string(scenario.name), ...
            'plannerStatus',string(plan.status.code),'evaluationStatus',string(evaluation.status), ...
            'passed',evaluation.passed,'pathLength',pathLength);
        metricNames = fieldnames(evaluation.metrics);
        for j = 1:numel(metricNames)
            row.(metricNames{j}) = evaluation.metrics.(metricNames{j});
        end
        rowCells{k} = row;
        fprintf('%s: planner=%s, evaluation=%s\n',scenario.id,plan.status.code,evaluation.status);
    end
    experiments = [experimentCells{:}];
    summary = struct2table([rowCells{:}]);
    summary.Properties.VariableUnits = {'','','','','','m','m','rad','m','m','m','rad','1/m','m','m'};
    metadata = struct('matlabVersion',version,'matlabRelease',version('-release'), ...
        'generatedAt',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd HH:mm:ss Z')), ...
        'vehicle',vehicle,'plannerSettings',planning,'timingSettings',timing, ...
        'evaluationSettings',quality,'scenarios',[experiments.scenario], ...
        'stateCoordinates','parking-local; framePoseWorld is applied only by the XY plot', ...
        'summaryUnits','metres, seconds, radians, curvature in 1/metre', ...
        'integration','Euler, left command sample over each actual time interval');
    results = struct('summary',summary,'experiments',experiments,'metadata',metadata, ...
        'outputDirectory',outputDirectory,'artifactNames',{names});

    for k = 1:numel(experiments)
        experiment = experiments(k);
        if strcmp(experiment.trajectory.status,'success') && strcmp(experiment.simulation.status,'success')
            figures = plotParkingExperiment(experiment,vehicle);
            cleanup = onCleanup(@() closeFigures(figures));
            for j = 1:numel(kinds)
                base = fullfile(outputDirectory,[experiment.scenario.id,'_',kinds{j}]);
                exportgraphics(figures.(kinds{j}),[base,'.png'],'Resolution',300,'Padding',12);
                exportgraphics(figures.(kinds{j}),[base,'.pdf'],'ContentType','vector','Padding',12);
            end
            clear cleanup
        end
    end
    tight = experiments(strcmp(ids,'tight'));
    statusPath = fullfile(outputDirectory,'tight_status.txt');
    fid = fopen(statusPath,'w');
    if fid < 0
        error('runParkingExperiments:WriteStatus','Cannot write %s.',statusPath);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'Step 10 | %s | bumper gap %.1f m\n',tight.scenario.name,tight.scenario.spaceLength);
    fprintf(fid,'Planner: %s\n%s\n',tight.plan.status.code,tight.plan.status.message);
    fprintf(fid,'Evaluation: %s | passed: %d\n', ...
        tight.evaluation.status,tight.evaluation.passed);
    fprintf(fid,['No accepted trajectory or simulation. Unavailable metrics are NaN.\n', ...
        'not_found describes the selected bounded two-cubic search; it does not prove physical impossibility.\n']);
    clear cleanup
    writetable(summary,fullfile(outputDirectory,'scenario_summary.csv'));
    save(fullfile(outputDirectory,'parking_experiments.mat'),'results');
    disp(summary);
    fprintf('Summary uses SI units (heading/steering in rad). Artifacts: %s\n',outputDirectory);
end

function closeFigures(figures)
    handles = struct2cell(figures);
    for k = 1:numel(handles)
        if isgraphics(handles{k},'figure')
            close(handles{k});
        end
    end
end
