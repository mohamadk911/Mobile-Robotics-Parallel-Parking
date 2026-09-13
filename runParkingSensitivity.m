function results = runParkingSensitivity(outputDirectory,overwrite)
%runParkingSensitivity Reproduce ONLY the Step 11 numerical study.
% R = runParkingSensitivity() writes to results/step11 beside this function.
% R = runParkingSensitivity(FOLDER,true) explicitly replaces named outputs.
% Plans each scenario ONCE, then passes frozen geometry and durations to
% studyParkingPlan. No Step 10 runner/files or accepted settings are changed.
    if nargin < 1
        outputDirectory = fullfile(fileparts(mfilename('fullpath')),'results','step11');
    end
    if nargin < 2
        overwrite = false;
    end
    validateattributes(overwrite,{'logical'},{'scalar'});
    assert((ischar(outputDirectory) && isrow(outputDirectory) && ~isempty(outputDirectory)) || ...
        (isstring(outputDirectory) && isscalar(outputDirectory) && ~ismissing(outputDirectory) && strlength(outputDirectory)>0), ...
        'runParkingSensitivity:InvalidDirectory','Supply a nonempty output directory.');
    outputDirectory = char(outputDirectory);
    names = {'nominal_sensitivity.csv','nominal_convergence.csv','nominal_metric_changes.csv', ...
        'changedStart_sensitivity.csv','changedStart_convergence.csv','changedStart_metric_changes.csv', ...
        'convergence.png','convergence.pdf','parking_sensitivity.mat','tight_status.txt'};
    paths = cellfun(@(n) fullfile(outputDirectory,n),names,UniformOutput=false);
    occupied = cellfun(@(p) isfile(p) || isfolder(p),paths);
    assert(~any(occupied) || overwrite,'runParkingSensitivity:OutputExists', ...
        'Named output exists; choose another folder or explicitly pass overwrite=true.');
    assert(~any(cellfun(@isfolder,paths)),'runParkingSensitivity:OutputIsDirectory','An output name is a directory.');
    % Avoid replacing any Step 10 artifact even if the wrong folder is supplied.
    assert(~isfile(fullfile(outputDirectory,'parking_experiments.mat')), ...
        'runParkingSensitivity:Step10Directory','Choose a separate Step 11 output folder.');
    if ~isfolder(outputDirectory)
        [ok,message] = mkdir(outputDirectory);
        assert(ok,'runParkingSensitivity:CreateDirectory','%s',message);
    end
    [ok,info] = fileattrib(outputDirectory);
    assert(ok,'runParkingSensitivity:DirectoryUnavailable','Cannot resolve output folder.');
    outputDirectory = info.Name;
    if any(occupied)
        fprintf('Replacing %d explicitly authorized Step 11 outputs.\n',sum(occupied));
    end
    vehicle = vehicleParameters(); planning = plannerSettings(); timing = timingSettings();
    quality = evaluationSettings(); settings = sensitivitySettings();
    ids = {'nominal','tight','changedStart'};
    cases = cell(1,numel(ids));
    for k = 1:numel(ids)
        scenario = scenarioParameters(vehicle,ids{k});
        plan = planCubicParking(vehicle,scenario,planning); % Exactly once per case.
        cases{k} = studyParkingPlan(plan,scenario,vehicle,timing,quality,settings);
        fprintf('%s: planner=%s, sensitivity=%s\n',ids{k},plan.status.code,cases{k}.status);
    end
    results = struct('studies',[cases{:}],'settings',settings, ...
        'metadata',struct('matlabVersion',version,'vehicle',vehicle,'plannerSettings',planning, ...
        'timingSettings',timing,'evaluationSettings',quality, ...
        'generatedAt',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd HH:mm:ss Z')), ...
        'plannerCallsPerScenario',1,'coordinates','parking-local', ...
        'units','dt in s; XY errors in m; heading/steering in rad; curvature in 1/m'), ...
        'outputDirectory',outputDirectory,'artifactNames',{names});
    for k = [1,3]
        study = results.studies(k);
        if isempty(study.summary)
            continue
        end
        writetable(study.summary,fullfile(outputDirectory,[ids{k},'_sensitivity.csv']));
        writetable(study.convergence,fullfile(outputDirectory,[ids{k},'_convergence.csv']));
        writetable(study.metricChanges,fullfile(outputDirectory,[ids{k},'_metric_changes.csv']));
        disp(study.summary); disp(study.convergence); disp(struct2table(study.checks));
    end
    fig = convergenceFigure(results.studies,settings);
    cleanup = onCleanup(@() close(fig));
    exportgraphics(fig,fullfile(outputDirectory,'convergence.png'),'Resolution',300,'Padding',12);
    exportgraphics(fig,fullfile(outputDirectory,'convergence.pdf'),'ContentType','vector','Padding',12);
    clear cleanup
    tight = results.studies(2);
    fid = fopen(fullfile(outputDirectory,'tight_status.txt'),'w');
    assert(fid>=0,'runParkingSensitivity:WriteStatus','Cannot write tight status.');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'Tight: planner %s; sensitivity %s.\n%s\n',tight.plan.status.code,tight.status,tight.message);
    fprintf(fid,'No dt sensitivity is performed without a validated plan. No physical-impossibility claim.\n');
    clear cleanup
    save(fullfile(outputDirectory,'parking_sensitivity.mat'),'results');
end

function fig = convergenceFigure(studies,settings)
    fig = figure('Visible','off','Color','white','Position',[80,80,1050,490]);
    layout = tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
    limits = [];
    for k = [1,3]
        s = studies(k).summary;
        if ~isempty(s)
            limits = [limits;s.numericalXYRMSE;s.stateIntegrationXYRMSE;s.commandSamplingXYRMSE]; %#ok<AGROW>
        end
    end
    for k = [1,3]
        ax = nexttile(layout);
        set(ax,'Color','white','XColor',[0.15,0.15,0.15],'YColor',[0.15,0.15,0.15], ...
            'XScale','log','YScale','log','FontName','Arial','FontSize',11,'Box','on');
        hold(ax,'on'); grid(ax,'on');
        study = studies(k);
        if isempty(study.summary)
            title(ax,[study.scenario.name,' | unavailable'],'Color',[0.15,0.15,0.15]);
            continue
        end
        s = study.summary;
        loglog(ax,s.dt,s.numericalXYRMSE,'o-','Color',[0,0.35,0.7],'LineWidth',1.8, ...
            'DisplayName','Euler vs continuous reference');
        loglog(ax,s.dt,s.stateIntegrationXYRMSE,'s--','Color',[0.85,0.3,0.05],'LineWidth',1.4, ...
            'DisplayName','Euler vs exact held commands');
        loglog(ax,s.dt,s.commandSamplingXYRMSE,'d:','Color',[0.1,0.55,0.4],'LineWidth',1.6, ...
            'DisplayName','Held vs continuous reference');
        loglog(ax,s.dt,s.numericalXYRMSE(end)*s.dt/s.dt(end),'--','Color',[0.4,0.4,0.4], ...
            'LineWidth',1,'DisplayName','First-order guide');
        xline(ax,settings.baselineDt,':','HandleVisibility','off','Color',[0.4,0.4,0.4]);
        xlim(ax,[min(settings.dt)*0.85,max(settings.dt)*1.15]);
        ylim(ax,[min(limits)*0.75,max(limits)*1.3]);
        xticks(ax,sort(settings.dt)); xticklabels(ax,compose('%.4g',sort(settings.dt)));
        xlabel(ax,'Sample interval dt (s)'); ylabel(ax,'Time-weighted XY RMS difference (m)');
        title(ax,study.scenario.name,'Color',[0.15,0.15,0.15],'FontSize',13);
        legend(ax,'Location','southoutside','FontSize',9,'TextColor',[0.15,0.15,0.15],'Color','white');
    end
    set(findall(fig,'Type','text'),'Color',[0.15,0.15,0.15]);
end
