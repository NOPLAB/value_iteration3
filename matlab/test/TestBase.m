classdef (Abstract) TestBase < matlab.unittest.TestCase
%TESTBASE Shared path setup for MATLAB unit tests.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            matlab_root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(matlab_root, 'src')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(matlab_root, 'src', 'bitboard')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(matlab_root, 'test')));
        end
    end
end
