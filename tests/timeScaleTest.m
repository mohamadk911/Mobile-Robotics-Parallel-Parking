classdef timeScaleTest < matlab.unittest.TestCase
%timeScaleTest Endpoint, timing-profile and input-validation checks.
% Run from the project root with runtests('tests/timeScaleTest.m').

    properties (TestParameter)
        validCase = struct( ...
            'nominal', struct('T',7,'dt',0.05,'expectedTime',0:0.05:7), ...
            'nonDivisibleNominal', struct('T',7,'dt',0.03,'expectedTime',[0:0.03:6.99,7]), ...
            'nonDivisible', struct('T',1,'dt',0.3,'expectedTime',[0,0.3,0.6,0.9,1]), ...
            'equalDuration', struct('T',1,'dt',1,'expectedTime',[0,1]), ...
            'longerThanDuration', struct('T',1,'dt',2,'expectedTime',[0,1]), ...
            'decimalEndpoint', struct('T',0.3,'dt',0.1,'expectedTime',[0,0.1,0.2,0.3]), ...
            'nearBelow', struct('T',1-eps(1),'dt',0.25,'expectedTime',[0,0.25,0.5,0.75,1-eps(1)]), ...
            'nearAbove', struct('T',1+eps(1),'dt',0.25,'expectedTime',[0,0.25,0.5,0.75,1+eps(1)]), ...
            'smallDuration', struct('T',1e-6,'dt',3e-7,'expectedTime',[0,3e-7,6e-7,9e-7,1e-6]), ...
            'singleInputs', struct('T',single(1),'dt',single(0.25),'expectedTime',[0,0.25,0.5,0.75,1]), ...
            'integerInputs', struct('T',int32(2),'dt',uint8(1),'expectedTime',[0,1,2]));
        invalidCase = struct( ...
            'zero', struct('value',0,'error','expectedPositive'), ...
            'negative', struct('value',-1,'error','expectedPositive'), ...
            'infinite', struct('value',Inf,'error','expectedFinite'), ...
            'negativeInfinite', struct('value',-Inf,'error','expectedFinite'), ...
            'notANumber', struct('value',NaN,'error','expectedFinite'), ...
            'empty', struct('value',[],'error','expectedScalar'), ...
            'rowVector', struct('value',[1,2],'error','expectedScalar'), ...
            'columnVector', struct('value',[1;2],'error','expectedScalar'), ...
            'matrix', struct('value',ones(2),'error','expectedScalar'), ...
            'complex', struct('value',1+1i,'error','expectedReal'), ...
            'text', struct('value','7','error','invalidType'), ...
            'logical', struct('value',true,'error','invalidType'));
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectFolder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(projectFolder));
        end
    end

    methods (Test)
        function includesExactEndpoints(testCase, validCase)
            scaling = timeScale(validCase.T, validCase.dt);
            T = double(validCase.T);
            timeTolerance = 8*eps(T);

            testCase.verifyClass(scaling.t, 'double');
            testCase.assertSize(scaling.t, size(validCase.expectedTime));
            testCase.assertSize(scaling.s, size(scaling.t));
            testCase.assertSize(scaling.sdot, size(scaling.t));
            testCase.verifyEqual(scaling.t, validCase.expectedTime, AbsTol=timeTolerance);
            % Exact comparisons here enforce the explicitly exact endpoint contract.
            testCase.verifyEqual(scaling.t([1,end]), [0,T]);
            testCase.verifyEqual(scaling.s([1,end]), [0,1]);
            testCase.verifyEqual(scaling.sdot([1,end]), [0,0]);
            testCase.verifyEqual(nnz(scaling.t == T), 1);
            testCase.verifyGreaterThan(diff(scaling.t), 0);
            testCase.verifyLessThanOrEqual(diff(scaling.t), double(validCase.dt) + timeTolerance);
            testCase.verifyTrue(all(isfinite([scaling.t,scaling.s,scaling.sdot])));
        end

        function preservesCubicProfile(testCase)
            scaling = timeScale(2, 0.5);
            testCase.verifyEqual(scaling.s, [0,0.15625,0.5,0.84375,1], AbsTol=1e-14);
            testCase.verifyEqual(scaling.sdot, [0,0.5625,0.75,0.5625,0], AbsTol=1e-14);
        end

        function preservesProfileWithShortFinalInterval(testCase)
            scaling = timeScale(1, 0.3);
            testCase.verifyEqual(scaling.s, [0,0.216,0.648,0.972,1], AbsTol=1e-14);
            testCase.verifyEqual(scaling.sdot, [0,1.26,1.44,0.54,0], AbsTol=1e-14);
            testCase.verifyEqual(diff(scaling.t), [0.3,0.3,0.3,0.1], AbsTol=1e-14);
        end

        function rejectsInvalidDuration(testCase, invalidCase)
            testCase.verifyError(@() timeScale(invalidCase.value, 0.05), ...
                ['MATLAB:timeScale:',invalidCase.error]);
        end

        function rejectsInvalidStep(testCase, invalidCase)
            testCase.verifyError(@() timeScale(7, invalidCase.value), ...
                ['MATLAB:timeScale:',invalidCase.error]);
        end
    end
end
