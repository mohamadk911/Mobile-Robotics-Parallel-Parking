classdef parkingSensitivityTest < matlab.unittest.TestCase
%parkingSensitivityTest Frozen-plan study, grids, comparisons and output safety.
    properties
        Results
        Output
        Step10Before
        Step10After
    end
    properties (TestParameter)
        caseIndex = {1,3};
        gridIndex = {1,2,3,4};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(project));
            temporary = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(temporary.Folder));
            testCase.Output = fullfile(temporary.Folder,'step11');
            testCase.Step10Before = readArtifacts(fullfile(project,'results','step10'));
            testCase.Results = runParkingSensitivity(testCase.Output);
            testCase.Step10After = readArtifacts(fullfile(project,'results','step10'));
        end
    end
    methods (Test)
        function requestedGridAndBaselineAreExact(testCase,caseIndex)
            s = testCase.Results.studies(caseIndex);
            testCase.verifyEqual(s.summary.dt,[0.1;0.05;0.025;0.0125]);
            testCase.verifyEqual(s.summary.samples,[141;281;561;1121]);
            testCase.verifyEqual(testCase.Results.settings.baselineDt,0.05);
            testCase.verifyEqual(testCase.Results.settings,sensitivitySettings());
        end
        function eachGridUsesFrozenGeometryAndDurations(testCase,caseIndex,gridIndex)
            s = testCase.Results.studies(caseIndex); r = s.runs(gridIndex);
            testCase.verifyEqual(r.planSegments,s.plan.segments);
            testCase.verifyEqual(r.timing.segmentDurations,[7,7]);
            expected = timeParkingPlan(s.plan,testCase.Results.metadata.vehicle,r.timing);
            testCase.verifyEqual(r.trajectory,expected);
            testCase.verifyEqual(r.euler,simulateParkingPlan(expected,s.scenario,testCase.Results.metadata.vehicle));
        end
        function exactEndpointsAndUniqueJoins(testCase,caseIndex,gridIndex)
            r = testCase.Results.studies(caseIndex).runs(gridIndex);
            parts = r.trajectory.segments;
            testCase.verifyEqual([parts(1).t([1,end]),parts(2).t([1,end])],[0,7,0,7]);
            testCase.verifyEqual([parts(1).v([1,end]),parts(2).v([1,end])],zeros(1,4));
            testCase.verifyEqual(r.euler.t([1,end]),[0,14]);
            testCase.verifyEqual(nnz(r.euler.t==7),1);
            testCase.verifyEqual(nnz(r.euler.t==14),1);
            testCase.verifyGreaterThan(diff(r.euler.t),0);
        end
        function comparisonsAreAlignedAndDistinct(testCase,caseIndex,gridIndex)
            r = testCase.Results.studies(caseIndex).runs(gridIndex);
            testCase.verifyEqual(r.accurate.t,r.euler.t);
            testCase.verifyEqual(r.held.t,r.euler.t);
            testCase.verifyEqual(r.numerical,compareParkingStates(r.euler,r.accurate));
            testCase.verifyEqual(r.stateIntegration,compareParkingStates(r.euler,r.held));
            testCase.verifyEqual(r.commandSampling,compareParkingStates(r.held,r.accurate));
            testCase.verifyEqual(r.numerical.xyError, ...
                r.stateIntegration.xyError+r.commandSampling.xyError,AbsTol=1e-14);
        end
        function geometryAuthorityIsIdenticalAcrossDt(testCase,caseIndex)
            s = testCase.Results.studies(caseIndex);
            g = validateCubicGeometry(s.plan.segments,testCase.Results.metadata.vehicle,'stopped');
            testCase.verifyEqual(s.summary.maxSteering,repmat(g.maxAbsSteering,4,1));
            testCase.verifyEqual(s.summary.maxAbsCurvature,repmat(g.maxAbsCurvature,4,1));
            testCase.verifyEqual(s.summary.minTurningRadius,repmat(g.minTurningRadius,4,1));
            testCase.verifyEqual(s.runs(1).evaluation.geometryValidation,s.runs(4).evaluation.geometryValidation);
            testCase.verifyTrue(all([g.checks(strcmp({g.checks.name},'extremaConvergence')).passed]));
        end
        function independentReferenceIsSplitAndPropagated(testCase,caseIndex)
            s = testCase.Results.studies(caseIndex);
            ref = s.refinedReference;
            testCase.verifyNumElements(ref.segments,numel(s.plan.segments));
            testCase.verifyEqual(ref.planSegments,s.plan.segments);
            testCase.verifyEqual(ref.segments(1).solution.x([1,end]),[0,7]);
            testCase.verifyEqual(ref.segments(2).solution.x([1,end]),[0,7]);
            testCase.verifyEqual(ref.segments(2).initialPose,ref.segments(1).terminalPose);
            testCase.verifyEqual(ref.segments(2).solution.y(:,1),ref.segments(1).terminalPose);
            testCase.verifyEqual(ref.tolerances,testCase.Results.settings.refinedReference);
        end
        function tighterReferenceAndAnalyticPlanAgree(testCase,caseIndex)
            r = testCase.Results.studies(caseIndex).runs(end);
            testCase.verifyLessThanOrEqual(r.referenceStability.maxXY,1e-7);
            testCase.verifyLessThanOrEqual(r.referenceStability.maxHeading,1e-8);
            testCase.verifyLessThan(r.referenceToPlan.maxXY,1e-7);
            testCase.verifyLessThan(r.referenceToPlan.maxHeading,1e-8);
        end
        function nonDivisibleDiagnosticIsStable(testCase,caseIndex)
            r = testCase.Results.studies(caseIndex).diagnostic;
            testCase.verifyEqual(r.dt,0.03);
            testCase.verifyNumElements(r.euler.t,469);
            testCase.verifyEqual(r.trajectory.segments(1).t(end),7);
            testCase.verifyEqual(diff(r.trajectory.segments(1).t(end-1:end)),0.01,AbsTol=1e-14);
            testCase.verifyEqual([r.trajectory.segments(1).v([1,end]), ...
                r.trajectory.segments(2).v([1,end])],zeros(1,4));
            testCase.verifyEqual(r.euler.t(end),14);
            testCase.verifyNumElements(unique(r.euler.t),469);
            testCase.verifyTrue(all(isfinite([r.euler.q,r.accurate.q]),'all'));
            testCase.verifyTrue(r.evaluation.passed);
        end
        function requiredScenariosCompleteAndCriteriaPass(testCase,caseIndex)
            s = testCase.Results.studies(caseIndex);
            testCase.verifyEqual(s.status,'pass',s.message);
            testCase.verifyTrue(s.passed);
            testCase.verifyTrue(s.baselineJustified);
            testCase.verifyTrue(all([s.checks.passed]));
            testCase.verifyEqual(s.summary.qualityStatus,["fail";"pass";"pass";"pass"]);
            testCase.verifyGreaterThan(s.convergence.observedOrder(1:3),0.8);
            testCase.verifyLessThan(s.convergence.observedOrder(1:3),1.2);
        end
        function baselineIsAcceptedStep10Numerics(testCase)
            testCase.verifyEqual(testCase.Results.studies(1).summary.timeWeightedXYRMSE(2), ...
                0.01887321386739633,AbsTol=1e-12);
            testCase.verifyEqual(testCase.Results.studies(3).summary.timeWeightedXYRMSE(2), ...
                0.01569928765299206,AbsTol=1e-12);
        end
        function tightHasNoSensitivityArrays(testCase)
            s = testCase.Results.studies(2);
            testCase.verifyEqual(s.plan.status.code,'not_found');
            testCase.verifyEqual(s.status,'not_applicable');
            testCase.verifyFalse(s.passed);
            testCase.verifyEmpty(s.runs);
            testCase.verifyEmpty(s.summary);
            testCase.verifyEmpty(s.reference);
            testCase.verifyEmpty(s.diagnostic);
            testCase.verifyFalse(isfile(fullfile(testCase.Output,'tight_sensitivity.csv')));
        end
        function allSettingsRemainAccepted(testCase)
            m = testCase.Results.metadata;
            testCase.verifyEqual(m.vehicle,vehicleParameters());
            testCase.verifyEqual(m.plannerSettings,plannerSettings());
            testCase.verifyEqual(m.timingSettings,timingSettings());
            testCase.verifyEqual(m.evaluationSettings,evaluationSettings());
            testCase.verifyEqual(m.plannerCallsPerScenario,1);
        end
        function savesDeterministicArtifactsAndData(testCase)
            names = testCase.Results.artifactNames;
            testCase.verifyNumElements(names,10);
            testCase.verifyTrue(all(cellfun(@(n) isfile(fullfile(testCase.Output,n)),names)));
            saved = load(fullfile(testCase.Output,'parking_sensitivity.mat'),'results');
            testCase.verifyEqual(saved.results.studies(1).summary,testCase.Results.studies(1).summary);
            csv = readtable(fullfile(testCase.Output,'nominal_convergence.csv'));
            testCase.verifyEqual(csv.error,testCase.Results.studies(1).convergence.error,AbsTol=1e-12);
        end
        function step10ArtifactsRemainUntouched(testCase)
            testCase.verifyEqual(testCase.Step10After,testCase.Step10Before);
        end
        function rejectsOverwriteAndWrongOutputFolder(testCase)
            testCase.verifyError(@() runParkingSensitivity(testCase.Output), ...
                'runParkingSensitivity:OutputExists');
            folder = fullfile(fileparts(testCase.Output),'fakeStep10'); mkdir(folder);
            writelines('Step 10 sentinel',fullfile(folder,'parking_experiments.mat'));
            testCase.verifyError(@() runParkingSensitivity(folder,true), ...
                'runParkingSensitivity:Step10Directory');
            testCase.verifyEqual(strtrim(fileread(fullfile(folder,'parking_experiments.mat'))),'Step 10 sentinel');
        end
    end
end

function artifacts = readArtifacts(folder)
    listing = dir(folder); listing = listing(~[listing.isdir]);
    artifacts = cell(numel(listing),2);
    for k = 1:numel(listing)
        fid = fopen(fullfile(folder,listing(k).name),'r');
        cleanup = onCleanup(@() fclose(fid));
        artifacts{k,1} = listing(k).name;
        artifacts{k,2} = fread(fid,Inf,'*uint8');
        clear cleanup
    end
end
