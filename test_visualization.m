% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_visualization.m
% Description: Verification script for the rover_visualization module.
%
% Tests performed:
%   1. Execution of rover_visualization() without errors.
%   2. Validation of returned figure handle.
%   3. Verification of file creation: results/mission_map.png.
%   4. Verification that image file has non-zero size.
% =========================================================================

clear;
clc;
close all;

fprintf('====================================================\n');
fprintf('  PLANETARY ROVER VISUALIZATION: VERIFICATION TEST\n');
fprintf('====================================================\n\n');

totalTests = 0;
passedTests = 0;

%% Test 1: Run rover_visualization module
totalTests = totalTests + 1;
try
    figHandle = rover_visualization();
    if isgraphics(figHandle, 'figure')
        passedTests = passedTests + 1;
        fprintf('[PASS] Test 1 - Visualization Function: Executed successfully, valid figure handle returned.\n');
    else
        fprintf('[FAIL] Test 1 - Visualization Function: Return value is not a valid figure.\n');
    end
catch ME
    fprintf('[FAIL] Test 1 - Visualization Function Error: %s\n', ME.message);
end

%% Test 2: Check Output File Existence
totalTests = totalTests + 1;
currentDir = fileparts(mfilename('fullpath'));
outputImagePath = fullfile(currentDir, 'results', 'mission_map.png');

if exist(outputImagePath, 'file') == 2
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 2 - Output Image File: File found at results/mission_map.png.\n');
else
    fprintf('[FAIL] Test 2 - Output Image File: results/mission_map.png was not found.\n');
end

%% Test 3: Check Image File Size
totalTests = totalTests + 1;
if exist(outputImagePath, 'file') == 2
    fileInfo = dir(outputImagePath);
    if fileInfo.bytes > 1000
        passedTests = passedTests + 1;
        fprintf('[PASS] Test 3 - Output Image Integrity: File size is valid (%d bytes).\n', fileInfo.bytes);
    else
        fprintf('[FAIL] Test 3 - Output Image Integrity: File size too small (%d bytes).\n', fileInfo.bytes);
    end
else
    fprintf('[FAIL] Test 3 - Output Image Integrity: File does not exist.\n');
end

%% Summary
fprintf('\n----------------------------------------------------\n');
fprintf('Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: VISUALIZATION MODULE VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: ISSUES DETECTED IN VISUALIZATION MODULE.\n');
end
fprintf('====================================================\n');
