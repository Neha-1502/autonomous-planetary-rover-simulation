% =========================================================================
% Script: test_calculate_metrics.m
% Description: Verifies calculate_metrics.m and the battery logic in
%              mission_controller.m.
%
% Part A: hand-built missionInfo structs; expected values are worked out by
%         hand from the definitions documented in calculate_metrics.m.
% Part B: the real mission controller; expected values are derived from the
%         controller's own outputs or from A* on the same map. No
%         performance numbers are hard-coded from a guess.
% =========================================================================
clear; clc; close all;

fprintf('============================================================\n');
fprintf('  MISSION METRICS: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests = 0; passedTests = 0; tol = 1e-9;
mk = @(hist, ok) struct('pathHistory', hist, 'success', ok, ...
                        'replanCount', 0, 'detectedObstacles', []);

%% ------------------------------------------------------------------------
%% PART A: calculation checks on hand-built data
%% ------------------------------------------------------------------------
fprintf('[PART A] Metric calculations on hand-built missions\n');

% A1. 4 straight moves, default battery (100 units, 1 unit/move)
totalTests = totalTests + 1;
hist = [2 2; 2 3; 2 4; 2 5; 2 6];
m = calculate_metrics(mk(hist, false));
if abs(m.totalDistance - 4) < tol && m.movements == 4 && ...
   abs(m.batteryConsumed - 4) < tol && abs(m.batteryRemaining - 96) < tol && ...
   abs(m.batteryRemainingPercent - 96) < tol && isnan(m.executionTimeSec)
    passedTests = passedTests + 1;
    fprintf('  [PASS] A1 - Distance 4, moves 4, consumed 4, remaining 96 (96%%), time NaN.\n');
else
    fprintf('  [FAIL] A1 - Basic distance/battery values wrong.\n');
end

% A2. Euclidean distance: a single jump of (3,4) has length 5
totalTests = totalTests + 1;
m = calculate_metrics(mk([1 1; 4 5], false));
if abs(m.totalDistance - 5) < tol && m.movements == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] A2 - Euclidean segment length 5 for a (3,4) displacement.\n');
else
    fprintf('  [FAIL] A2 - Distance calculation wrong (got %.4f).\n', m.totalDistance);
end

% A3. Custom battery: capacity 50, 2.5 units/move, 4 moves -> 10 used, 80%
totalTests = totalTests + 1;
m = calculate_metrics(mk(hist, false), [], struct('capacity', 50, 'energyPerMove', 2.5));
if abs(m.batteryConsumed - 10) < tol && abs(m.batteryRemaining - 40) < tol && ...
   abs(m.batteryRemainingPercent - 80) < tol
    passedTests = passedTests + 1;
    fprintf('  [PASS] A3 - Custom battery: consumed 10, remaining 40 (80%%).\n');
else
    fprintf('  [FAIL] A3 - Custom battery calculation wrong.\n');
end

% A4. Battery clamp: 120 moves on a 100-unit battery cannot go negative
totalTests = totalTests + 1;
longHist = [(1:121)', ones(121, 1)];
m = calculate_metrics(mk(longHist, false));
if m.batteryRemaining == 0 && m.batteryConsumed == 100 && ...
   m.batteryRemainingPercent == 0 && m.batteryDepleted
    passedTests = passedTests + 1;
    fprintf('  [PASS] A4 - Battery clamped at 0 (consumed 100, never negative).\n');
else
    fprintf('  [FAIL] A4 - Battery went below zero or was not flagged depleted.\n');
end

% A5. Counts and time pass-through
totalTests = totalTests + 1;
info = mk(hist, false);
info.detectedObstacles = [3 3; 3 4; 3 3];   % 2 distinct cells
info.replanCount = 2;
info.newObstacleCount = 1;
m1 = calculate_metrics(info, 1.25);
info.executionTimeSec = 0.5;
m2 = calculate_metrics(info);
if m1.obstaclesDetected == 2 && m1.pathRecalculations == 2 && ...
   m1.newObstaclesDiscovered == 1 && abs(m1.executionTimeSec - 1.25) < tol && ...
   abs(m2.executionTimeSec - 0.5) < tol
    passedTests = passedTests + 1;
    fprintf('  [PASS] A5 - Distinct obstacles 2, replans 2, new 1, time arg/field used.\n');
else
    fprintf('  [FAIL] A5 - Counts or execution time wrong.\n');
end

% A6. Completion: goal reached -> 100
totalTests = totalTests + 1;
m = calculate_metrics(mk([1 1; 1 2], true));
if m.completionPercent == 100
    passedTests = passedTests + 1;
    fprintf('  [PASS] A6 - Completion 100%% when goal reached.\n');
else
    fprintf('  [FAIL] A6 - Completion should be 100 on success.\n');
end

% A7. Completion partial: 5x5 empty map, start [1,1], goal [5,5] (dStart=8),
%     rover stopped at [3,3] (dFinal=4) -> 50%
totalTests = totalTests + 1;
info = mk([1 1; 1 2; 1 3; 2 3; 3 3], false);
info.finalMap = zeros(5, 5);
info.startPos = [1 1]; info.goalPos = [5 5]; info.finalPos = [3 3];
m = calculate_metrics(info);
if abs(m.completionPercent - 50) < tol
    passedTests = passedTests + 1;
    fprintf('  [PASS] A7 - Partial completion = 50%% (dStart 8, dFinal 4).\n');
else
    fprintf('  [FAIL] A7 - Partial completion wrong (got %.4f).\n', m.completionPercent);
end

% A8. Completion: no movement -> 0
totalTests = totalTests + 1;
m = calculate_metrics(mk([1 1], false));
if m.completionPercent == 0 && m.movements == 0 && m.totalDistance == 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] A8 - No movement: completion 0, distance 0, moves 0.\n');
else
    fprintf('  [FAIL] A8 - No-movement case wrong.\n');
end

% A9. Rover ended farther from goal than it began -> clamped to 0
totalTests = totalTests + 1;
info = mk([3 3; 3 2; 3 1], false);
info.finalMap = zeros(5, 5);
info.startPos = [3 3]; info.goalPos = [3 5]; info.finalPos = [3 1];
m = calculate_metrics(info);
if m.completionPercent == 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] A9 - Moving away from goal clamps completion at 0.\n');
else
    fprintf('  [FAIL] A9 - Expected 0, got %.4f.\n', m.completionPercent);
end

% A10. Goal unreachable on final map -> NaN with explanation (not a guess)
totalTests = totalTests + 1;
info = mk([1 1; 1 2; 2 2], false);
fm = zeros(5, 5); fm(4, 5) = 1; fm(5, 4) = 1;
info.finalMap = fm;
info.startPos = [1 1]; info.goalPos = [5 5]; info.finalPos = [2 2];
m = calculate_metrics(info);
if isnan(m.completionPercent) && strncmp(m.completionMethod, 'undefined', 9)
    passedTests = passedTests + 1;
    fprintf('  [PASS] A10 - Unreachable goal: completion NaN (%s).\n', m.completionMethod);
else
    fprintf('  [FAIL] A10 - Expected NaN completion for unreachable goal.\n');
end

% A11. Invalid inputs raise errors
totalTests = totalTests + 1;
err1 = false; err2 = false;
try, calculate_metrics(5); catch, err1 = true; end
try, calculate_metrics(mk(hist, false), [], struct('capacity', -1)); catch, err2 = true; end
if err1 && err2
    passedTests = passedTests + 1;
    fprintf('  [PASS] A11 - Non-struct input and negative capacity rejected.\n');
else
    fprintf('  [FAIL] A11 - Invalid input was not rejected.\n');
end

% A12. Defaults / partial overrides
totalTests = totalTests + 1;
p1 = calculate_metrics('defaults');
p2 = calculate_metrics('defaults', struct('capacity', 20));
if p1.capacity == 100 && p1.energyPerMove == 1 && p2.capacity == 20 && p2.energyPerMove == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] A12 - Defaults 100/1; partial override keeps other default.\n');
else
    fprintf('  [FAIL] A12 - Default battery parameters wrong.\n');
end

%% ------------------------------------------------------------------------
%% PART B: integration with mission_controller
%% ------------------------------------------------------------------------
fprintf('\n[PART B] Integration with the mission controller\n');
[map, start, goal] = environment();
[~, baseCost] = astar_path_planning(map, start, goal);   % optimal cost, static map

% B1. Nominal mission
totalTests = totalTests + 1;
[~, info] = mission_controller(map, start, goal, [], 2.5, false);
m = calculate_metrics(info);
if info.success && m.movements == info.totalStepsTaken && ...
   m.batteryConsumed == info.batteryUsed && m.batteryRemaining == info.batteryRemaining && ...
   abs(m.batteryRemainingPercent - 100 * (100 - info.totalStepsTaken) / 100) < tol && ...
   m.completionPercent == 100 && m.pathRecalculations == 0 && ...
   abs(m.totalDistance - m.movements) < tol && ...
   isfinite(m.executionTimeSec) && m.executionTimeSec >= 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] B1 - Nominal: %d moves, battery %.1f%% left, completion 100%%, time %.3f s.\n', ...
        m.movements, m.batteryRemainingPercent, m.executionTimeSec);
else
    fprintf('  [FAIL] B1 - Nominal mission metrics inconsistent.\n');
end

% B2. Dynamic hazard mission
totalTests = totalTests + 1;
[~, info] = mission_controller(map, start, goal, [4 2 8], 2.5, false);
m = calculate_metrics(info);
if info.success && m.pathRecalculations >= 1 && m.newObstaclesDiscovered >= 1 && ...
   m.obstaclesDetected > 0 && m.movements >= baseCost && ...
   m.batteryConsumed == m.movements * m.energyPerMove
    passedTests = passedTests + 1;
    fprintf('  [PASS] B2 - Dynamic hazard: %d replan(s), %d new obstacle(s), %d moves (static optimum %d).\n', ...
        m.pathRecalculations, m.newObstaclesDiscovered, m.movements, baseCost);
else
    fprintf('  [FAIL] B2 - Dynamic mission metrics wrong.\n');
end

% B3. Battery runs out mid-mission (capacity 10, 1 unit/move)
totalTests = totalTests + 1;
[finalPos, info] = mission_controller(map, start, goal, [], 2.5, false, ...
                                      struct('capacity', 10, 'energyPerMove', 1));
m = calculate_metrics(info, [], info.batteryParams);
expectedPct = 100 * (1 - (baseCost - 10) / baseCost);   % rover follows an optimal path
if ~info.success && info.totalStepsTaken == 10 && info.batteryRemaining == 0 && ...
   ~isempty(strfind(info.status, 'Battery')) && ~isequal(finalPos, goal) && ...
   m.batteryRemaining == 0 && m.batteryDepleted && abs(m.completionPercent - expectedPct) < 1e-6
    passedTests = passedTests + 1;
    fprintf('  [PASS] B3 - Stopped at battery 0 after 10 moves; completion %.2f%%.\n', m.completionPercent);
else
    fprintf('  [FAIL] B3 - Battery depletion not handled correctly (status: %s).\n', info.status);
end

% B4. Battery can never go negative: 3 units/move on 10 units -> 3 moves, 1 left
totalTests = totalTests + 1;
[~, info] = mission_controller(map, start, goal, [], 2.5, false, ...
                               struct('capacity', 10, 'energyPerMove', 3));
if ~info.success && info.totalStepsTaken == 3 && abs(info.batteryRemaining - 1) < tol && ...
   info.batteryRemaining >= 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] B4 - Move not started when battery < cost (3 moves, 1.0 unit left).\n');
else
    fprintf('  [FAIL] B4 - Expected 3 moves and 1 unit left (got %d moves, %.2f left).\n', ...
        info.totalStepsTaken, info.batteryRemaining);
end

% B5. Battery exactly sufficient: reaching the goal at 0 is still a success
totalTests = totalTests + 1;
[~, info] = mission_controller(map, start, goal, [], 2.5, false, ...
                               struct('capacity', baseCost, 'energyPerMove', 1));
m = calculate_metrics(info, [], info.batteryParams);
if info.success && info.batteryRemaining == 0 && m.batteryDepleted && ...
   m.missionSuccess && m.completionPercent == 100
    passedTests = passedTests + 1;
    fprintf('  [PASS] B5 - Goal reached with exactly 0 battery counts as success.\n');
else
    fprintf('  [FAIL] B5 - Exact-battery mission handled incorrectly.\n');
end

%% Summary
fprintf('\n------------------------------------------------------------\n');
fprintf('Metrics Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: MISSION METRICS VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');