% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_astar_path_planning.m
% Description: Verification suite for custom A* path planning implementation.
%
% Tests performed:
%   1. Obstacle-Free Environment (5x5 grid, optimal Manhattan trajectory)
%   2. Planetary Environment with Obstacles (20x20 grid, hazard avoidance)
%   3. Destination with No Valid Route (Fully enclosed goal)
%   4. Blocked Endpoints (Start or Goal positioned on obstacle)
% =========================================================================

clear;
clc;

fprintf('============================================================\n');
fprintf('  A* PATH PLANNING ALGORITHM: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

%% =========================================================================
%% Test Case 1: Obstacle-Free Environment (5x5 grid)
%% =========================================================================
fprintf('[TEST 1] Testing Obstacle-Free Environment (5x5 Grid)...\n');
totalTests = totalTests + 1;

emptyMap = zeros(5, 5);
start1   = [1, 1];
goal1    = [5, 5];

[path1, cost1] = astar_path_planning(emptyMap, start1, goal1);
expectedCost1 = abs(start1(1) - goal1(1)) + abs(start1(2) - goal1(2)); % 8

if ~isempty(path1) && cost1 == expectedCost1 && ...
   isequal(path1(1, :), start1) && isequal(path1(end, :), goal1)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1 - Obstacle-Free: Optimal path found with cost = %d (Expected: %d).\n', ...
        cost1, expectedCost1);
    fprintf('         Waypoints (%d steps):\n         ', size(path1, 1));
    for i = 1:size(path1, 1)
        fprintf('[%d, %d] ', path1(i, 1), path1(i, 2));
        if i < size(path1, 1), fprintf('-> '); end
    end
    fprintf('\n\n');
else
    fprintf('  [FAIL] Test 1 - Obstacle-free pathfinding failed.\n\n');
end

%% =========================================================================
%% Test Case 2: Planetary Environment with Obstacles (20x20 Grid)
%% =========================================================================
fprintf('[TEST 2] Testing Planetary Environment with Obstacles (20x20 Grid)...\n');
totalTests = totalTests + 1;

[map20, start20, goal20] = environment();
[path2, cost2] = astar_path_planning(map20, start20, goal20);

% Check collision-free property along entire trajectory
hasCollision = false;
if ~isempty(path2)
    for p = 1:size(path2, 1)
        if map20(path2(p, 1), path2(p, 2)) == 1
            hasCollision = true;
            break;
        end
    end
end

if ~isempty(path2) && ~hasCollision && ...
   isequal(path2(1, :), start20) && isequal(path2(end, :), goal20)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2 - Obstacle Avoidance: Valid route found! Cost = %d steps (%d waypoints).\n', ...
        cost2, size(path2, 1));
    fprintf('         Collision check: 0 obstacle collisions detected along trajectory.\n');
    fprintf('         Start waypoint: [%d, %d] | Goal waypoint: [%d, %d]\n\n', ...
        path2(1, 1), path2(1, 2), path2(end, 1), path2(end, 2));

    % Render ASCII Map with Planned Trajectory
    fprintf('------------------------------------------------------------\n');
    fprintf('  ASCII MAP: A* PLANNED TRAJECTORY (''*'' = Path, ''#'' = Hazard)\n');
    fprintf('------------------------------------------------------------\n   ');
    for c = 1:size(map20, 2), fprintf('%2d', c); end
    fprintf('\n');

    pathMask = false(size(map20));
    for k = 1:size(path2, 1)
        pathMask(path2(k, 1), path2(k, 2)) = true;
    end

    for r = 1:size(map20, 1)
        fprintf('%2d ', r);
        for c = 1:size(map20, 2)
            if r == start20(1) && c == start20(2)
                fprintf(' S');
            elseif r == goal20(1) && c == goal20(2)
                fprintf(' G');
            elseif pathMask(r, c)
                fprintf(' *');
            elseif map20(r, c) == 1
                fprintf(' #');
            else
                fprintf(' .');
            end
        end
        fprintf('\n');
    end
    fprintf('------------------------------------------------------------\n\n');
else
    fprintf('  [FAIL] Test 2 - Planetary obstacle pathfinding failed.\n\n');
end

%% =========================================================================
%% Test Case 3: Destination with No Valid Route (Fully Enclosed Goal)
%% =========================================================================
fprintf('[TEST 3] Testing Destination with No Valid Route (Blocked Goal)...\n');
totalTests = totalTests + 1;

blockedMap = zeros(6, 6);
start3 = [1, 1];
goal3  = [6, 6];

% Build an impassable wall sealing off goal at [6, 6]
blockedMap(5, 5) = 1;
blockedMap(5, 6) = 1;
blockedMap(6, 5) = 1;

[path3, cost3] = astar_path_planning(blockedMap, start3, goal3);

if isempty(path3) && isinf(cost3)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3 - Unreachable Goal: Correctly returned empty matrix [] and Inf cost.\n\n');
else
    fprintf('  [FAIL] Test 3 - Failed to reject unreachable destination.\n\n');
end

%% =========================================================================
%% Test Case 4: Blocked Start and Destination Edge Cases
%% =========================================================================
fprintf('[TEST 4] Testing Blocked Endpoint Rejection...\n');
totalTests = totalTests + 1;

mapWithBlockedStart = zeros(5, 5);
mapWithBlockedStart(1, 1) = 1; % Start cell is an obstacle
[path4a, cost4a] = astar_path_planning(mapWithBlockedStart, [1, 1], [5, 5]);

mapWithBlockedGoal = zeros(5, 5);
mapWithBlockedGoal(5, 5) = 1;  % Goal cell is an obstacle
[path4b, cost4b] = astar_path_planning(mapWithBlockedGoal, [1, 1], [5, 5]);

if isempty(path4a) && isempty(path4b)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 4 - Blocked Endpoints: Correctly rejected blocked start and blocked goal.\n\n');
else
    fprintf('  [FAIL] Test 4 - Blocked endpoint handling failed.\n\n');
end

%% =========================================================================
%% Explanation: Cell Selection Mechanism in A*
%% =========================================================================
fprintf('============================================================\n');
fprintf('  HOW A* SELECTS THE NEXT CELL (MATLAB IMPLEMENTATION)\n');
fprintf('============================================================\n');
fprintf('1. Evaluation Function:\n');
fprintf('   For every cell n, f(n) = g(n) + h(n)\n');
fprintf('   - g(n): Actual shortest distance traveled from start to n.\n');
fprintf('   - h(n): Admissible Manhattan distance heuristic: |r - goal_r| + |c - goal_c|.\n');
fprintf('   - f(n): Estimated total cost of path passing through n.\n\n');
fprintf('2. Selection Rule:\n');
fprintf('   At each iteration, the algorithm scans openSet (cells awaiting evaluation)\n');
fprintf('   and selects the cell (currR, currC) with minimum fScore.\n\n');
fprintf('3. Tie-Breaking Strategy:\n');
fprintf('   If multiple candidate cells share the identical minimum fScore,\n');
fprintf('   the cell with the lower hScore (closest to the destination) is chosen.\n');
fprintf('   This guides the rover directly toward the goal and avoids wasted node expansions.\n\n');

%% Summary
fprintf('------------------------------------------------------------\n');
fprintf('Test Suite Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: A* PATH PLANNING MODULE VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
