classdef (Abstract) TestBase < matlab.unittest.TestCase
%TESTBASE Shared path setup for MATLAB unit tests.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            matlab_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            original_path = path();
            testCase.addTeardown(@() path(original_path));
            addpath(genpath(fullfile(matlab_root, 'src')));
            addpath(genpath(fullfile(matlab_root, 'workflows', 'validation', 'tests')));
        end
    end
end
