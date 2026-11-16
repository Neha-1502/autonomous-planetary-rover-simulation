% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_mission_controller.m
% Description: Comprehensive verification of the autonomous mission controller.
%
% Tests performed:
%   1. Nominal Mission (Static 20x20 planetary environment):
%      Validates end-to-end traversal from [2, 2] to [19, 19].
%   2. Dynamic Mission (Hazard event mid-transit):
%      Validates hazard sensing, emergency halt, real-time replanning,
%      and arrival at destination.
%   3. Unreachable Destination Mission (Safety abort):
%      Validates safe shutdown when no valid route to destination exists.
% =========================================================================

clear;
clc;
close all;

fprintf('============================================================\n');
fprintf('  AUTONOMOUS MISSION CONTROLLER: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

% Load environment
[map, start, goal] = environment();

%% =========================================================================
%% Scenario 1: Nominal Mission in Static Mars Terrain
%% =========================================================================
fprintf('[SCENARIO 1] Testing Nominal Mission (Static Planetary Environment)...\n');
totalTests = totalTests + 1;

[finalPos1, info1, state1] = mission_controller(map, start, goal, [], 2.5, false);

if info1.success && isequal(finalPos1, goal) && info1.totalStepsTaken > 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] Scenario 1 - Nominal Mission: Reached destination [%d, %d] in %d steps (Replans: %d).\n', ...
        finalPos1(1), finalPos1(2), info1.totalStepsTaken, info1.replanCount);
    fprintf('         Status: %s\n\n', info1.status);
else
    fprintf('  [FAIL] Scenario 1 - Nominal mission failed to reach destination.\n\n');
end

%% =========================================================================
%% Scenario 2: Dynamic Mission with Mid-Transit Obstacle & Replanning
%% =========================================================================
fprintf('[SCENARIO 2] Testing Dynamic Mission (Mid-Transit Hazard Injection)...\n');
totalTests = totalTests + 1;

% Dynamic event: At step 4, rockfall occurs at [2, 8]
dynamicObstacles = [4, 2, 8];

[finalPos2, info2, state2] = mission_controller(map, start, goal, dynamicObstacles, 2.5, false);

% Verification: Reached goal, replanned at least once, and did NOT collide with [2, 8]
hitBlockedCell = any(info2.pathHistory(:, 1) == 2 & info2.pathHistory(:, 2) == 8);

if info2.success && isequal(finalPos2, goal) && info2.replanCount >= 1 && ~hitBlockedCell
    passedTests = passedTests + 1;
    fprintf('  [PASS] Scenario 2 - Dynamic Replanning: Reached [%d, %d] in %d steps after %d replan(s).\n', ...
        finalPos2(1), finalPos2(2), info2.totalStepsTaken, info2.replanCount);
    fprintf('         Safety Check: Dynamic obstacle [2, 8] was strictly avoided.\n');
    fprintf('         Status: %s\n\n', info2.status);
else
    fprintf('  [FAIL] Scenario 2 - Dynamic mission failed.\n\n');
end

%% =========================================================================
%% Scenario 3: Safety Abort on Unreachable Destination
%% =========================================================================
fprintf('[SCENARIO 3] Testing Safety Abort (Fully Blocked Destination)...\n');
totalTests = totalTests + 1;

blockedMap = map;
% Seal off all 4-connected neighbors of goal [19, 19]
blockedMap(18, 19) = 1; % Up
blockedMap(20, 19) = 1; % Down
blockedMap(19, 18) = 1; % Left
blockedMap(19, 20) = 1; % Right

[finalPos3, info3, state3] = mission_controller(blockedMap, start, goal, [], 2.5, false);

if ~info3.success && ~isequal(finalPos3, goal)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Scenario 3 - Unreachable Abort: Controller safely aborted mission without crash.\n');
    fprintf('         Status: %s\n\n', info3.status);
else
    fprintf('  [FAIL] Scenario 3 - Expected mission abort on blocked destination.\n\n');
end

%% =========================================================================
%% Summary
%% =========================================================================
fprintf('------------------------------------------------------------\n');
fprintf('Mission Controller Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: AUTONOMOUS MISSION CONTROLLER VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
