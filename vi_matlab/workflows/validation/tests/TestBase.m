classdef (Abstract) TestBase < matlab.unittest.TestCase
%TESTBASE Shared path setup for MATLAB unit tests.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            matlab_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            original_path = path();
            testCase.addTeardown(@() path(original_path));
            addpath(matlab_root);
            setup_matlab_paths('src', 'tests');
        end
    end
end
