% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_path_visualization.m
% Description: Verification suite for A* pathfinding and visualization integration.
%
% Tests performed:
%   1. End-to-end integration with standard Mars environment.
%   2. Verification of total path cells and movement cost.
%   3. Verification of image file generation in results/ folder.
%   4. Edge-case test: Visualizing environment with NO valid path.
% =========================================================================

clear;
clc;
close all;

fprintf('============================================================\n');
fprintf('  PATH VISUALIZATION INTEGRATION: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

%% Test 1: Standard Environment Integration
fprintf('[TEST 1] Testing standard Mars environment path generation...\n');
totalTests = totalTests + 1;

[map, start, goal] = environment();
[path, cost] = astar_path_planning(map, start, goal);

if ~isempty(path) && cost > 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1.1 - Route Calculation: Found route with %d cells (Cost: %d steps).\n', ...
        size(path, 1), cost);
else
    fprintf('  [FAIL] Test 1.1 - Route calculation failed.\n');
end

% Render visualization
totalTests = totalTests + 1;
try
    fig1 = rover_visualization(map, start, goal, path, cost);
    if isgraphics(fig1, 'figure')
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 1.2 - Figure Rendering: Visualized terrain and blue planned route.\n');
    else
        fprintf('  [FAIL] Test 1.2 - Figure handle invalid.\n');
    end
catch ME
    fprintf('  [FAIL] Test 1.2 - Visualization error: %s\n', ME.message);
end

%% Test 2: Output Image Verification in results/
fprintf('\n[TEST 2] Verifying saved image files...\n');
totalTests = totalTests + 1;

currentDir = fileparts(mfilename('fullpath'));
imagePath = fullfile(currentDir, 'results', 'planned_route.png');

if exist(imagePath, 'file') == 2
    fInfo = dir(imagePath);
    if fInfo.bytes > 5000
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 2.1 - Image Output: results/planned_route.png generated (%d bytes).\n', fInfo.bytes);
    else
        fprintf('  [FAIL] Test 2.1 - Image file size too small (%d bytes).\n', fInfo.bytes);
    end
else
    fprintf('  [FAIL] Test 2.1 - results/planned_route.png not found.\n');
end

%% Test 3: Edge Case - Handling Environment With No Valid Route
fprintf('\n[TEST 3] Testing visualization when NO valid route exists...\n');
totalTests = totalTests + 1;

blockedMap = zeros(6, 6);
blockedStart = [1, 1];
blockedGoal  = [6, 6];
blockedMap(5, 5) = 1;
blockedMap(5, 6) = 1;
blockedMap(6, 5) = 1;

[emptyPath, infCost] = astar_path_planning(blockedMap, blockedStart, blockedGoal);

try
    fig2 = rover_visualization(blockedMap, blockedStart, blockedGoal, emptyPath, infCost);
    if isgraphics(fig2, 'figure')
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 3.1 - No-Path Handling: Cleanly handled empty route without errors.\n');
    else
        fprintf('  [FAIL] Test 3.1 - Figure handle invalid for blocked environment.\n');
    end
catch ME
    fprintf('  [FAIL] Test 3.1 - Visualization crashed on empty path: %s\n', ME.message);
end

%% Summary
fprintf('\n------------------------------------------------------------\n');
fprintf('Integration Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: A* AND VISUALIZATION INTEGRATION VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
