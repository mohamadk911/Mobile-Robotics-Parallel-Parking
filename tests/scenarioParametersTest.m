classdef scenarioParametersTest < matlab.unittest.TestCase
%scenarioParametersTest Fixed experiments, derived geometry and input errors.

    properties (TestParameter)
        experiment = struct( ...
            'nominal',struct('id','nominal','gap',6.5,'frontX',11,'initialX',11.76, ...
                'targetX',5.5,'heading',0,'clearance',1), ...
            'tight',struct('id','tight','gap',5.5,'frontX',10,'initialX',10.76, ...
                'targetX',5,'heading',0,'clearance',0.5), ...
            'changedStart',struct('id','changedStart','gap',6.5,'frontX',11,'initialX',11.76, ...
                'targetX',5.5,'heading',deg2rad(10),'clearance',1));
        badId = struct('misspelled','nomnal','wrongCase','changedstart','blank','', ...
            'emptyString',"",'numeric',1,'cell',{{'nominal'}}, ...
            'multiple',["nominal","tight"],'missing',string(missing));
        badGap = struct('zero',struct('value',0,'error','expectedPositive'), ...
            'negative',struct('value',-1,'error','expectedPositive'), ...
            'nan',struct('value',NaN,'error','expectedFinite'), ...
            'inf',struct('value',Inf,'error','expectedFinite'), ...
            'vector',struct('value',[5,6],'error','expectedScalar'), ...
            'empty',struct('value',[],'error','expectedScalar'), ...
            'complex',struct('value',1+1i,'error','expectedReal'), ...
            'text',struct('value','6.5','error','invalidType'), ...
            'logical',struct('value',true,'error','invalidType'));
        badDimension = struct( ...
            'lengthNaN',struct('field','length','value',NaN), ...
            'widthInf',struct('field','width','value',Inf), ...
            'wheelbaseNaN',struct('field','wheelbase','value',NaN), ...
            'rearInf',struct('field','rearOverhang','value',Inf), ...
            'frontNaN',struct('field','frontOverhang','value',NaN), ...
            'negativeLength',struct('field','length','value',-1), ...
            'zeroWidth',struct('field','width','value',0), ...
            'zeroWheelbase',struct('field','wheelbase','value',0), ...
            'negativeRear',struct('field','rearOverhang','value',-0.1), ...
            'negativeFront',struct('field','frontOverhang','value',-0.1), ...
            'vectorLength',struct('field','length','value',[4,5]), ...
            'textWidth',struct('field','width','value','1.8'));
        badFrame = struct('row',struct('value',[0,0,0],'error','incorrectSize'), ...
            'short',struct('value',[0;0],'error','incorrectSize'), ...
            'matrix',struct('value',zeros(3,2),'error','incorrectSize'), ...
            'nan',struct('value',[NaN;0;0],'error','expectedFinite'), ...
            'infiniteHeading',struct('value',[0;0;Inf],'error','expectedFinite'), ...
            'complex',struct('value',[0;0;1i],'error','expectedReal'));
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            folder = fileparts(mfilename('fullpath'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(folder)));
        end
    end

    methods (Test)
        function generatesExactExperiments(testCase,experiment)
            vehicle = vehicleParameters();
            scenario = scenarioParameters(vehicle,experiment.id);
            testCase.verifyEqual(scenario.id,experiment.id);
            testCase.verifyEqual(scenario.spaceLength,experiment.gap,AbsTol=1e-12);
            testCase.verifyEqual(scenario.framePoseWorld,zeros(3,1),AbsTol=1e-12);
            testCase.verifyEqual(scenario.rearCarPose,zeros(3,1),AbsTol=1e-12);
            testCase.verifyEqual(scenario.frontCarPose,[experiment.frontX;0;0],AbsTol=1e-12);
            testCase.verifyEqual(scenario.startOffsetFront,[0.76;2.8;experiment.heading],AbsTol=1e-12);
            testCase.verifyEqual(scenario.initialPose,[experiment.initialX;2.8;experiment.heading],AbsTol=1e-12);
            testCase.verifyEqual(scenario.targetPose,[experiment.targetX;0;0],AbsTol=1e-12);
            rearMargin = scenario.targetPose(1)-vehicle.rearOverhang - ...
                (scenario.rearCarPose(1)+vehicle.wheelbase+vehicle.frontOverhang);
            frontMargin = scenario.frontCarPose(1)-vehicle.rearOverhang - ...
                (scenario.targetPose(1)+vehicle.wheelbase+vehicle.frontOverhang);
            testCase.verifyEqual([rearMargin,frontMargin],experiment.clearance*[1,1],AbsTol=1e-12);
            testCase.verifyEqual(scenario.initialPose-scenario.frontCarPose, ...
                scenario.startOffsetFront,AbsTol=1e-12);
            testCase.verifyFalse(any(isfield(scenario, ...
                {'midPose','ks','ke','coefficients','plannerSettings','segments','duration','dt'})));
        end

        function preservesNominalCallAndLegacyFields(testCase)
            fixture = legacyNominalFixture();
            default = scenarioParameters(vehicleParameters());
            explicit = scenarioParameters(vehicleParameters(),"nominal");
            testCase.verifyEqual(default,explicit);
            legacyFields = rmfield(default,{'id','framePoseWorld','startOffsetFront'});
            testCase.verifyEqual(legacyFields,fixture.scenario,AbsTol=fixture.tolerance.configuration);
        end

        function derivesCoordinatesFromVehicleAndGap(testCase)
            % Synthetic dimension changes test the derivation, not a new
            % required vehicle or a planning/feasibility experiment.
            vehicle = vehicleParameters();
            vehicle.length = 5;
            vehicle.width = 2;
            vehicle.wheelbase = 3;
            vehicle.frontOverhang = 1.25;
            vehicle.rearOverhang = 0.75;
            scenario = scenarioParameters(vehicle,'nominal',[0;0;0],7.2);
            testCase.verifyEqual(scenario.frontCarPose,[12.2;0;0],AbsTol=1e-12);
            testCase.verifyEqual(scenario.targetPose,[6.1;0;0],AbsTol=1e-12);
            testCase.verifyEqual(scenario.initialPose,[12.96;2.8;0],AbsTol=1e-12);
            rearBumper = scenario.targetPose(1)-vehicle.rearOverhang;
            frontBumper = scenario.targetPose(1)+vehicle.wheelbase+vehicle.frontOverhang;
            testCase.verifyEqual([rearBumper-4.25,11.45-frontBumper],[1.1,1.1],AbsTol=1e-12);
        end

        function frameChangesOnlyFrameMetadata(testCase,experiment)
            local = scenarioParameters(vehicleParameters(),experiment.id);
            placed = scenarioParameters(vehicleParameters(),experiment.id,[23.7;-8.2;0.83]);
            testCase.verifyEqual(rmfield(local,'framePoseWorld'),rmfield(placed,'framePoseWorld'));
            testCase.verifyEqual(placed.framePoseWorld,[23.7;-8.2;0.83],AbsTol=1e-12);
        end

        function gapOverrideRetainsStagingAndHeading(testCase)
            scenario = scenarioParameters(vehicleParameters(),'changedStart',[0;0;0],8);
            testCase.verifyEqual(scenario.spaceLength,8,AbsTol=1e-12);
            testCase.verifyEqual(scenario.frontCarPose,[12.5;0;0],AbsTol=1e-12);
            testCase.verifyEqual(scenario.targetPose,[6.25;0;0],AbsTol=1e-12);
            testCase.verifyEqual(scenario.initialPose,[13.26;2.8;deg2rad(10)],AbsTol=1e-12);
        end

        function positiveSmallGapDefinesGeometryWithoutFeasibilityClaim(testCase)
            scenario = scenarioParameters(vehicleParameters(),'nominal',[0;0;0],1);
            testCase.verifyEqual(scenario.spaceLength,1,AbsTol=1e-12);
            testCase.verifyFalse(any(isfield(scenario,{'passed','feasible','status'})));
        end

        function rejectsInvalidIdentifiers(testCase,badId)
            testCase.verifyError(@() scenarioParameters(vehicleParameters(),badId), ...
                'scenarioParameters:InvalidIdentifier');
        end

        function rejectsInvalidGap(testCase,badGap)
            testCase.verifyError(@() scenarioParameters(vehicleParameters(),'nominal',[0;0;0],badGap.value), ...
                ['MATLAB:scenarioParameters:',badGap.error]);
        end

        function rejectsInvalidDimensions(testCase,badDimension)
            vehicle = vehicleParameters();
            vehicle.(badDimension.field) = badDimension.value;
            testCase.verifyError(@() scenarioParameters(vehicle),'scenarioParameters:InvalidVehicle');
        end

        function rejectsInvalidFramePose(testCase,badFrame)
            testCase.verifyError(@() scenarioParameters(vehicleParameters(),'nominal',badFrame.value), ...
                ['MATLAB:scenarioParameters:',badFrame.error]);
        end

        function rejectsIncompleteAndInconsistentVehicle(testCase)
            vehicle = vehicleParameters();
            missing = rmfield(vehicle,'rearOverhang');
            vehicle.length = 5;
            testCase.verifyError(@() scenarioParameters(missing),'scenarioParameters:InvalidVehicle');
            testCase.verifyError(@() scenarioParameters([]),'scenarioParameters:InvalidVehicle');
            testCase.verifyError(@() scenarioParameters(vehicle),'scenarioParameters:InconsistentVehicle');
        end

        function rejectsOverflowingDerivedGeometry(testCase)
            vehicle = vehicleParameters();
            vehicle.length = realmax;
            vehicle.wheelbase = realmax;
            vehicle.frontOverhang = 0;
            vehicle.rearOverhang = 0;
            testCase.verifyError(@() scenarioParameters(vehicle,'nominal',[0;0;0],realmax), ...
                'scenarioParameters:NonfiniteGeometry');
        end
    end
end
