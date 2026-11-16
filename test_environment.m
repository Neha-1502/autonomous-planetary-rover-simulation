% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_environment.m
% Description: Verification and unit test script for the environment module.
%
% Tests performed:
%   1. Grid Dimensions: Verifies 20x20 matrix size.
%   2. Starting Position: Verifies start coordinates are [2, 2].
%   3. Destination Position: Verifies goal coordinates are [19, 19].
%   4. Start & Goal Clearance: Verifies cells at start and goal equal 0.
%   5. Obstacle Count & Value Validity: Verifies binary values and obstacle total.
%   6. Reproducibility: Verifies identical output across subsequent calls.
% =========================================================================

clear;
clc;

fprintf('====================================================\n');
fprintf('  PLANETARY ROVER ENVIRONMENT: VERIFICATION TEST\n');
fprintf('====================================================\n\n');

% 1. Call environment module
[map, start, goal] = environment();

totalTests = 0;
passedTests = 0;

%% Test 1: Grid Dimensions
totalTests = totalTests + 1;
[rows, cols] = size(map);
expectedRows = 20;
expectedCols = 20;

if rows == expectedRows && cols == expectedCols
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 1 - Grid Dimensions: %d x %d (Expected: %d x %d)\n', ...
        rows, cols, expectedRows, expectedCols);
else
    fprintf('[FAIL] Test 1 - Grid Dimensions: %d x %d (Expected: %d x %d)\n', ...
        rows, cols, expectedRows, expectedCols);
end

%% Test 2: Starting Position
totalTests = totalTests + 1;
expectedStart = [2, 2];

if isequal(start, expectedStart)
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 2 - Starting Position: [%d, %d] (Expected: [%d, %d])\n', ...
        start(1), start(2), expectedStart(1), expectedStart(2));
else
    fprintf('[FAIL] Test 2 - Starting Position: [%d, %d] (Expected: [%d, %d])\n', ...
        start(1), start(2), expectedStart(1), expectedStart(2));
end

%% Test 3: Destination Position
totalTests = totalTests + 1;
expectedGoal = [19, 19];

if isequal(goal, expectedGoal)
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 3 - Destination Position: [%d, %d] (Expected: [%d, %d])\n', ...
        goal(1), goal(2), expectedGoal(1), expectedGoal(2));
else
    fprintf('[FAIL] Test 3 - Destination Position: [%d, %d] (Expected: [%d, %d])\n', ...
        goal(1), goal(2), expectedGoal(1), expectedGoal(2));
end

%% Test 4: Start and Destination Unblocked
totalTests = totalTests + 1;
startVal = map(start(1), start(2));
goalVal  = map(goal(1), goal(2));

if startVal == 0 && goalVal == 0
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 4 - Start & Goal Free: startVal=%d, goalVal=%d (Both 0)\n', ...
        startVal, goalVal);
else
    fprintf('[FAIL] Test 4 - Start & Goal Free: startVal=%d, goalVal=%d (Expected 0)\n', ...
        startVal, goalVal);
end

%% Test 5: Obstacle Count and Binary Value Verification
totalTests = totalTests + 1;
allBinary = all(map(:) == 0 | map(:) == 1);
obstacleCount = sum(map(:) == 1);
totalCells = numel(map);
obstaclePercent = (obstacleCount / totalCells) * 100;

if allBinary && obstacleCount > 0 && obstacleCount < totalCells
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 5 - Obstacle Count: %d / %d cells (%.1f%% terrain coverage, all binary 0/1)\n', ...
        obstacleCount, totalCells, obstaclePercent);
else
    fprintf('[FAIL] Test 5 - Obstacle Count verification failed.\n');
end

%% Test 6: Reproducibility Verification
totalTests = totalTests + 1;
[map2, start2, goal2] = environment();

if isequal(map, map2) && isequal(start, start2) && isequal(goal, goal2)
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 6 - Reproducibility: Identical environment generated across calls.\n');
else
    fprintf('[FAIL] Test 6 - Reproducibility: Environment mismatch across calls.\n');
end

%% Display ASCII Representation of Planetary Terrain
fprintf('\n----------------------------------------------------\n');
fprintf('  ASCII TERRAIN MAP PREVIEW (20x20)\n');
fprintf('  Legend: . = Free Terrain, # = Obstacle, S = Start, G = Goal\n');
fprintf('----------------------------------------------------\n   ');

for c = 1:cols
    fprintf('%2d', c);
end
fprintf('\n');

for r = 1:rows
    fprintf('%2d ', r);
    for c = 1:cols
        if r == start(1) && c == start(2)
            fprintf(' S');
        elseif r == goal(1) && c == goal(2)
            fprintf(' G');
        elseif map(r, c) == 1
            fprintf(' #');
        else
            fprintf(' .');
        end
    end
    fprintf('\n');
end
fprintf('----------------------------------------------------\n');

%% Overall Summary
fprintf('\nTest Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: ALL TESTS COMPLETED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED. PLEASE REVIEW LOGS.\n');
end
fprintf('====================================================\n');
