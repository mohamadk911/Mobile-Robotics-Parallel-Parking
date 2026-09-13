classdef parkingExperimentRunnerTest < matlab.unittest.TestCase
%parkingExperimentRunnerTest Reproducible Step 10 execution and plot contracts.
    properties
        First
        Repeated
        Folder
        Figures = cell(1,3)
        TransformedFigures = cell(1,3)
        RunnerLeftFigures
    end
    properties (TestParameter)
        successfulIndex = {1,3};
        plotKind = {'xy','errors','inputs'};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(project));
            temporary = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(temporary.Folder));
            testCase.Folder = fullfile(temporary.Folder,'outputs');
            before = findall(groot,'Type','figure');
            testCase.First = cleanScopeRun(testCase.Folder,false);
            writelines('unrelated sentinel',fullfile(testCase.Folder,'keep.txt'));
            testCase.Repeated = cleanScopeRun(testCase.Folder,true);
            testCase.RunnerLeftFigures = setdiff(findall(groot,'Type','figure'),before);
            for index = [1,3]
                experiment = testCase.First.experiments(index);
                original = plotParkingExperiment(experiment,testCase.First.metadata.vehicle);
                testCase.addTeardown(@closePlots,original);
                testCase.Figures{index} = original;
                experiment.scenario.framePoseWorld = [20;-4;pi/6];
                transformed = plotParkingExperiment(experiment,testCase.First.metadata.vehicle);
                testCase.addTeardown(@closePlots,transformed);
                testCase.TransformedFigures{index} = transformed;
            end
        end
    end
    methods (Test)
        function cleanScopeRunsAndAllScenariosAppearOnce(testCase)
            testCase.verifyEqual(testCase.First.summary.scenarioId,["nominal";"tight";"changedStart"]);
            testCase.verifyNumElements(testCase.First.experiments,3);
            testCase.verifyNumElements(unique(testCase.First.summary.scenarioId),3);
        end
        function successfulScenarioPassesAllChecks(testCase,successfulIndex)
            e = testCase.First.experiments(successfulIndex);
            testCase.verifyEqual(e.plan.status.code,'feasible');
            testCase.verifyEqual(e.evaluation.status,'pass');
            testCase.verifyTrue(e.evaluation.passed);
            testCase.verifyTrue(all([e.evaluation.checks.passed]));
        end
        function tightRemainsUnavailable(testCase)
            e = testCase.First.experiments(2);
            testCase.verifyEqual(e.plan.status.code,'not_found');
            testCase.verifyEqual(e.evaluation.status,'not_evaluated');
            testCase.verifyFalse(e.evaluation.passed);
            testCase.verifyEmpty(e.plan.segments);
            testCase.verifyEmpty(e.trajectory.q);
            testCase.verifyEmpty(e.simulation.q);
        end
        function summaryStatusesAndUnavailableValues(testCase)
            s = testCase.First.summary;
            testCase.verifyEqual(s.plannerStatus,["feasible";"not_found";"feasible"]);
            testCase.verifyEqual(s.evaluationStatus,["pass";"not_evaluated";"pass"]);
            testCase.verifyEqual(s.passed,[true;false;true]);
            testCase.verifyTrue(all(isnan(s{2,6:14})));
            testCase.verifyEqual(s.vehicleMinTurningRadius(2),testCase.First.metadata.vehicle.minTurningRadius);
        end
        function availableMetricsAreFiniteAndMatchEvaluator(testCase,successfulIndex)
            s = testCase.First.summary;
            testCase.verifyTrue(all(isfinite(s{successfulIndex,6:end})));
            metrics = testCase.First.experiments(successfulIndex).evaluation.metrics;
            testCase.verifyEqual(s{successfulIndex,7:end},cell2mat(struct2cell(metrics)).');
            testCase.verifyGreaterThan(s.pathLength(successfulIndex),0);
        end
        function repeatsSameNumericalResults(testCase)
            testCase.verifyEqual(testCase.First.summary,testCase.Repeated.summary,AbsTol=1e-12);
            testCase.verifyEqual({testCase.First.experiments.plan}, ...
                {testCase.Repeated.experiments.plan},AbsTol=1e-12);
        end
        function repeatedRunPreservesUnrelatedFile(testCase)
            testCase.verifyEqual(strtrim(fileread(fullfile(testCase.Folder,'keep.txt'))),'unrelated sentinel');
        end
        function settingsAndScenariosAreRecordedUnchanged(testCase)
            m = testCase.First.metadata;
            testCase.verifyEqual(m.vehicle,vehicleParameters());
            testCase.verifyEqual(m.plannerSettings,plannerSettings());
            testCase.verifyEqual(m.timingSettings,timingSettings());
            testCase.verifyEqual(m.evaluationSettings,evaluationSettings());
            testCase.verifyEqual(m.scenarios,[testCase.First.experiments.scenario]);
            testCase.verifyEqual(m.matlabVersion,version);
            testCase.verifyNotEmpty(m.generatedAt);
        end
        function deterministicFilenamesAndDirectory(testCase)
            expected = {'scenario_summary.csv','parking_experiments.mat','tight_status.txt', ...
                'nominal_xy.png','nominal_xy.pdf','nominal_errors.png','nominal_errors.pdf', ...
                'nominal_inputs.png','nominal_inputs.pdf','changedStart_xy.png','changedStart_xy.pdf', ...
                'changedStart_errors.png','changedStart_errors.pdf','changedStart_inputs.png','changedStart_inputs.pdf'};
            testCase.verifyEqual(testCase.First.artifactNames,expected);
            testCase.verifyEqual(testCase.First.outputDirectory,testCase.Repeated.outputDirectory);
            testCase.verifyEqual(testCase.First.artifactNames,testCase.Repeated.artifactNames);
            entries = dir(testCase.Folder);
            testCase.verifyTrue(all(ismember(expected,{entries.name})));
            testCase.verifyGreaterThan([entries(ismember({entries.name},expected)).bytes],0);
        end
        function csvAndMatPreserveSummary(testCase)
            csv = readtable(fullfile(testCase.Folder,'scenario_summary.csv'),'TextType','string');
            testCase.verifyEqual(csv.scenarioId,testCase.First.summary.scenarioId);
            testCase.verifyEqual(csv.evaluationStatus,testCase.First.summary.evaluationStatus);
            testCase.verifyEqual(csv{:,6:end},testCase.First.summary{:,6:end},AbsTol=1e-12);
            saved = load(fullfile(testCase.Folder,'parking_experiments.mat'),'results');
            testCase.verifyEqual(saved.results.summary,testCase.Repeated.summary);
            testCase.verifyEqual(saved.results.metadata,testCase.Repeated.metadata);
        end
        function noRejectedTightTrajectoryOrFigure(testCase)
            testCase.verifyFalse(isfile(fullfile(testCase.Folder,'tight_xy.png')));
            status = fileread(fullfile(testCase.Folder,'tight_status.txt'));
            testCase.verifySubstring(status,'not_found');
            testCase.verifySubstring(status,'not_evaluated');
            testCase.verifySubstring(status,'passed: 0');
            testCase.verifySubstring(status,'does not prove physical impossibility');
            testCase.verifyError(@() plotParkingExperiment(testCase.First.experiments(2), ...
                testCase.First.metadata.vehicle),'plotParkingExperiment:UnavailableTrajectory');
        end
        function runnerClosesOwnedFigures(testCase)
            testCase.verifyEmpty(testCase.RunnerLeftFigures);
        end
        function plotHasNoWorkspaceOrVisibilityDependency(testCase,successfulIndex,plotKind)
            fig = testCase.Figures{successfulIndex}.(plotKind);
            testCase.verifyTrue(isgraphics(fig,'figure'));
            testCase.verifyEqual(char(fig.Visible),'off');
            testCase.verifyNotEmpty(findall(fig,'Type','axes'));
        end
        function plotExportColorsIgnoreDesktopTheme(testCase,successfulIndex,plotKind)
            fig = testCase.Figures{successfulIndex}.(plotKind);
            axes = findall(fig,'Type','axes');
            testCase.verifyEqual(vertcat(axes.Color),ones(numel(axes),3));
            testCase.verifyEqual(vertcat(axes.XColor),repmat([0.15,0.15,0.15],numel(axes),1));
            textObjects = findall(fig,'Type','text');
            testCase.verifyEqual(vertcat(textObjects.Color),repmat([0.15,0.15,0.15],numel(textObjects),1));
        end
        function xyPreservesRearAxleGeometryAndEqualScaling(testCase,successfulIndex)
            e = testCase.First.experiments(successfulIndex);
            fig = testCase.Figures{successfulIndex}.xy;
            line = findall(fig,'Tag','plannedXY');
            testCase.verifyEqual([line.XData;line.YData],e.trajectory.q(1:2,:));
            testCase.verifyEqual(line.Parent.DataAspectRatio,[1,1,1]);
            simulated = findall(fig,'Tag','simulatedXY');
            testCase.verifyNotEqual(line.LineStyle,simulated.LineStyle);
            testCase.verifyNotEqual(line.Color,simulated.Color);
            parked = findall(fig,'DisplayName','Parked vehicles');
            v = testCase.First.metadata.vehicle;
            testCase.verifyEqual(parked.XData(:).',[-v.rearOverhang, ...
                v.wheelbase+v.frontOverhang,v.wheelbase+v.frontOverhang,-v.rearOverhang]);
        end
        function frameOnlyTransformsPresentation(testCase,successfulIndex)
            e = testCase.First.experiments(successfulIndex);
            frame = [20;-4;pi/6]; e.scenario.framePoseWorld = frame;
            evaluated = evaluateParkingExperiment(e.plan,e.trajectory,e.simulation, ...
                e.scenario,testCase.First.metadata.vehicle,testCase.First.metadata.evaluationSettings);
            testCase.verifyEqual(evaluated.metrics,e.evaluation.metrics);
            testCase.verifyEqual(evaluated.passed,e.evaluation.passed);
            line = findall(testCase.TransformedFigures{successfulIndex}.xy,'Tag','plannedXY');
            expected = parkingLocalToWorld(e.trajectory.q,frame);
            testCase.verifyEqual([line.XData;line.YData],expected(1:2,:));
            errors = findall(testCase.TransformedFigures{successfulIndex}.errors,'Tag','xyError');
            testCase.verifyEqual(errors.YData,e.evaluation.errors.xyErrorMagnitude);
        end
        function errorPlotUsesEvaluatedDataAndThreshold(testCase,successfulIndex)
            fig = testCase.Figures{successfulIndex}.errors;
            line = findall(fig,'Tag','xyError');
            e = testCase.First.experiments(successfulIndex).evaluation;
            testCase.verifyEqual(line.XData,e.errors.t);
            testCase.verifyEqual(line.YData,e.errors.xyErrorMagnitude);
            limit = findall(fig,'Tag','xyErrorLimit');
            testCase.verifyEqual(limit.Value,e.settings.maxTimeMatchedXYError);
        end
        function inputsRetainSeparateStoppedJoinSidesAndLimits(testCase,successfulIndex)
            fig = testCase.Figures{successfulIndex}.inputs;
            t = testCase.First.experiments(successfulIndex).trajectory;
            layout = fig.Children(1);
            testCase.verifyEqual(layout.Title.Color,[0.15,0.15,0.15]);
            testCase.verifyNumElements(findall(fig,'Tag','segmentSteering'),numel(t.segments));
            incoming = findall(fig,'Tag','incomingSteering');
            outgoing = findall(fig,'Tag','outgoingSteering');
            testCase.verifyEqual(incoming.XData,t.segments(1).globalT(end));
            testCase.verifyEqual(outgoing.XData,incoming.XData);
            testCase.verifyEqual(incoming.YData,rad2deg(t.segments(1).delta(end)));
            testCase.verifyEqual(outgoing.YData,rad2deg(t.segments(2).delta(1)));
            positive = findall(fig,'Tag','positiveSteeringLimit');
            negative = findall(fig,'Tag','negativeSteeringLimit');
            testCase.verifyEqual([positive.Value,negative.Value],[35,-35],AbsTol=1e-12);
        end
        function refusesImplicitOverwriteBeforeAnyRun(testCase)
            testCase.verifyError(@() runParkingExperiments(testCase.Folder), ...
                'runParkingExperiments:OutputExists');
            testCase.verifyEqual(strtrim(fileread(fullfile(testCase.Folder,'keep.txt'))),'unrelated sentinel');
        end
        function protectsUnrecognizedExistingOutput(testCase)
            folder = fullfile(fileparts(testCase.Folder),'unrelated');
            mkdir(folder);
            filename = fullfile(folder,'scenario_summary.csv');
            writelines('old unrelated data',filename);
            testCase.verifyError(@() runParkingExperiments(folder),'runParkingExperiments:OutputExists');
            testCase.verifyEqual(strtrim(fileread(filename)),'old unrelated data');
        end
        function drawVehicleUsesExplicitAxes(testCase)
            fig = figure('Visible','off');
            testCase.addTeardown(@close,fig);
            layout = tiledlayout(fig,1,2);
            requested = nexttile(layout); decoy = nexttile(layout);
            body = drawVehicle([0;0;0],testCase.First.metadata.vehicle,0,requested);
            testCase.verifyEqual(body.Parent,requested);
            testCase.verifyNumElements(findall(requested,'Type','patch'),5);
            testCase.verifyEmpty(decoy.Children);
        end
    end
end

function results = cleanScopeRun(folder,overwrite)
% A separate function scope with only output preferences, no scenario variables.
    results = runParkingExperiments(folder,overwrite);
end

function closePlots(figures)
    handles = struct2cell(figures);
    for k = 1:numel(handles)
        if isgraphics(handles{k},'figure')
            close(handles{k});
        end
    end
end
