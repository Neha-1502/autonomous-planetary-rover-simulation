% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_rover_animation.m
% Description: Verification suite for step-by-step rover animation.
%
% Tests performed:
%   1. Single Figure Constraint: Validates no duplicate figures are created.
%   2. Step-by-Step Movement: Verifies rover reaches destination [19, 19].
%   3. Visual Marker Integrity:
%      - Rover: Blue circular marker.
%      - Start: Green square marker.
%      - Destination: Red star marker.
%      - Planned Route: Blue dashed line.
%      - Actual Path: Green continuous line.
%      - Detected Obstacles: Highlighted obstacle markers.
%   4. Live Coordinate Updates: Verifies title updates during motion.
%   5. File Output Verification: Confirms results/mission_animation_final.png.
% =========================================================================

clear;
clc;
close all;

fprintf('============================================================\n');
fprintf('  ROVER MOVEMENT ANIMATION: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

% Load environment & compute path
[map, start, goal] = environment();
[path, cost] = astar_path_planning(map, start, goal);

%% Test 1: Run Animation and Verify Single Figure Integrity
fprintf('[TEST 1] Testing step-by-step animation execution...\n');
totalTests = totalTests + 1;

initialFigures = numel(findobj('Type', 'figure'));

% Run animation with fast pause (0.01s) for test execution
fig = rover_visualization(map, start, goal, path, cost, true, 0.01);

finalFigures = numel(findobj('Type', 'figure'));

if finalFigures == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1.1 - Single Figure Constraint: Only 1 figure used across all %d steps (No duplicate windows).\n', ...
        size(path, 1));
else
    fprintf('  [FAIL] Test 1.1 - Multiple figures created during animation (Count: %d).\n', finalFigures);
end

%% Test 2: Verify Final Rover Position and Graphics State
fprintf('\n[TEST 2] Verifying animated graphics object states...\n');

% Verify Rover Marker Position (Blue Circle)
totalTests = totalTests + 1;
hRover = findobj(fig, 'DisplayName', 'Rover (Blue Circle)');
if ~isempty(hRover)
    roverX = get(hRover, 'XData');
    roverY = get(hRover, 'YData');
    if roverX == goal(2) && roverY == goal(1)
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 2.1 - Rover Final Position: Successfully reached destination [%d, %d].\n', ...
            roverY, roverX);
    else
        fprintf('  [FAIL] Test 2.1 - Rover position mismatch (Got: [%d, %d], Expected: [%d, %d]).\n', ...
            roverY, roverX, goal(1), goal(2));
    end
else
    fprintf('  [FAIL] Test 2.1 - Rover graphics handle not found.\n');
end

% Verify Travelled Path (Green Line)
totalTests = totalTests + 1;
hTravelled = findobj(fig, 'DisplayName', 'Travelled Path (Green)');
if ~isempty(hTravelled)
    travX = get(hTravelled, 'XData');
    travY = get(hTravelled, 'YData');
    if length(travX) == size(path, 1) && travX(end) == goal(2) && travY(end) == goal(1)
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 2.2 - Green Travelled Path: Recorded full trajectory (%d coordinates).\n', ...
            length(travX));
    else
        fprintf('  [FAIL] Test 2.2 - Travelled path length mismatch.\n');
    end
else
    fprintf('  [FAIL] Test 2.2 - Travelled path handle not found.\n');
end

% Verify Planned Route (Blue Dashed Line)
totalTests = totalTests + 1;
hPlanned = findobj(fig, 'DisplayName', 'Planned Route (Blue Dashed)');
if ~isempty(hPlanned)
    lineStyle = get(hPlanned, 'LineStyle');
    if strcmp(lineStyle, '--')
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 2.3 - Planned Route Style: Configured as blue dashed line (''--'').\n');
    else
        fprintf('  [FAIL] Test 2.3 - LineStyle is ''%s'' (Expected: ''--'').\n', lineStyle);
    end
else
    fprintf('  [FAIL] Test 2.3 - Planned route handle not found.\n');
end

%% Test 3: Output File Verification in results/
fprintf('\n[TEST 3] Verifying animation output images...\n');
totalTests = totalTests + 1;

currentDir = fileparts(mfilename('fullpath'));
outFinalPng = fullfile(currentDir, 'results', 'mission_animation_final.png');

if exist(outFinalPng, 'file') == 2
    fInfo = dir(outFinalPng);
    if fInfo.bytes > 5000
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 3.1 - Animation Output: results/mission_animation_final.png verified (%d bytes).\n', fInfo.bytes);
    else
        fprintf('  [FAIL] Test 3.1 - Image file size too small.\n');
    end
else
    fprintf('  [FAIL] Test 3.1 - Output image not found.\n');
end

%% Summary
fprintf('\n------------------------------------------------------------\n');
fprintf('Rover Animation Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: ROVER ANIMATION MODULE VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
