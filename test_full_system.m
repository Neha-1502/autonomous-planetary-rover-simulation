% =========================================================================
% Script: test_full_system.m
% Description: Complete scenario testing of the Autonomous Planetary Rover
%              Simulation. Every case executes the real modules
%              (environment, astar_path_planning, rover_movement,
%              obstacle_detection, mission_controller, calculate_metrics),
%              records EXPECTED behaviour and ACTUAL behaviour, and reports
%              PASS only if every individual check holds.
%
% Scenarios:
%   S1 Obstacle-free environment        S5 No valid route
%   S2 Multiple static obstacles        S6 Rover reaches destination
%   S3 New obstacle during movement     S7 Battery depletion
%   S4 Destination blocked              S8 Moving outside the map
%
% Saved to results/full_test/ (next to this file):
%   full_test_report.txt, full_test_results.csv, full_test_results.mat,
%   and one map image per mission case (e.g. S3a.png).
%
% Expected step counts come from independent calls to the A* planner or
% from hand arithmetic (Manhattan distance), never from a previous run.
% A* prints "[A* Warning]" lines for blocked/outside endpoints: that is
% expected output in the S4, S5 and S8 cases.
% =========================================================================
clear; clc; close all;

outDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'full_test');
if ~exist(outDir, 'dir'), mkdir(outDir); end

fprintf('============================================================\n');
fprintf('  AUTONOMOUS PLANETARY ROVER: FULL SYSTEM TEST\n');
fprintf('============================================================\n\n');

R = [];
R = [R, runSafely(@() case_S1(outDir),  'S1')];
R = [R, runSafely(@() case_S2(outDir),  'S2')];
R = [R, runSafely(@() case_S3(outDir),  'S3')];
R = [R, runSafely(@() case_S4(outDir),  'S4')];
R = [R, runSafely(@() case_S5(outDir),  'S5')];
R = [R, runSafely(@() case_S6(outDir),  'S6')];
R = [R, runSafely(@() case_S7(outDir),  'S7')];
R = [R, runSafely(@() case_S8(outDir),  'S8')];

fprintf('\n\n');
printReport(1, R);

fid = fopen(fullfile(outDir, 'full_test_report.txt'), 'w');
if fid > 0, printReport(fid, R); fclose(fid); end
writeResultsCsv(fullfile(outDir, 'full_test_results.csv'), R);
save(fullfile(outDir, 'full_test_results.mat'), 'R');
fprintf('Results saved in: %s\n', outDir);

% =========================================================================
% S1: Obstacle-free environment
% =========================================================================
function r = case_S1(outDir)
    map = zeros(20); start = [2, 2]; goal = [19, 19];
    [~, info] = mission_controller(map, start, goal, [], 2.5, false);
    m = calculate_metrics(info);
    manhattan = sum(abs(goal - start));
    checks = {
        'mission succeeds',                 info.success
        'final position equals goal',       isequal(info.finalPos, goal)
        'moves equal Manhattan distance',   info.totalStepsTaken == manhattan
        'no path recalculations',           info.replanCount == 0
        'no collisions',                    ~collides(info)
        'exactly one cell per move',        singleSteps(info.pathHistory)
        'battery % = 100 - moves',          abs(m.batteryRemainingPercent - (100 - manhattan)) < 1e-9
        };
    r = makeResult('S1', 'S1 Obstacle-free environment', 'Empty 20x20 map, [2,2] -> [19,19]', ...
        'Reaches [19,19] in 34 moves (Manhattan optimum), 0 replans, no collisions, 66% battery left.', ...
        actualText(info, m), checks, saveCaseMap(outDir, 'S1', info, start, goal));
end

% =========================================================================
% S2: Multiple static obstacles
% =========================================================================
function r = case_S2(outDir)
    [map, start, goal] = environment();
    nObs = sum(map(:) == 1);
    [~, optCost] = astar_path_planning(map, start, goal);
    [~, info] = mission_controller(map, start, goal, [], 2.5, false);
    m = calculate_metrics(info);
    checks = {
        'map has several obstacles',        nObs > 1
        'mission succeeds',                 info.success
        'moves equal independent A* cost',  info.totalStepsTaken == optCost
        'no path recalculations (static)',  info.replanCount == 0
        'no collisions',                    ~collides(info)
        'exactly one cell per move',        singleSteps(info.pathHistory)
        'obstacles were sensed en route',   m.obstaclesDetected > 0
        };
    r = makeResult('S2', 'S2 Multiple static obstacles', ...
        sprintf('Default Mars map (%d obstacle cells)', nObs), ...
        'Reaches the goal along the A* optimum, avoiding every obstacle, 0 replans, hazards sensed on the way.', ...
        actualText(info, m), checks, saveCaseMap(outDir, 'S2', info, start, goal));
end

% =========================================================================
% S3: Newly introduced obstacle during movement
% =========================================================================
function r = case_S3(outDir)
    [map, start, goal] = environment();
    [basePath, ~] = astar_path_planning(map, start, goal);

    % 3a: rockfall on the route after 4 moves
    [~, infoA] = mission_controller(map, start, goal, [4, 2, 8], 2.5, false);
    mA = calculate_metrics(infoA);
    checksA = {
        'mission succeeds',                 infoA.success
        'at least one replan',              infoA.replanCount >= 1
        'new obstacle was discovered',      infoA.newObstacleCount >= 1
        'hazard cell [2,8] never entered',  ~visited(infoA, [2, 8])
        'hazard present in final map',      infoA.finalMap(2, 8) == 1
        'no collisions',                    ~collides(infoA)
        'exactly one cell per move',        singleSteps(infoA.pathHistory)
        };
    rA = makeResult('S3a', 'S3 New obstacle during movement', 'Rockfall at [2,8] after 4 moves', ...
        'Rover senses it, halts, replans (>=1), detours, reaches goal, never enters [2,8].', ...
        actualText(infoA, mA), checksA, saveCaseMap(outDir, 'S3a', infoA, start, goal));

    % 3b: obstacle appears two cells ahead on the route (inside sensor range)
    cB = basePath(3, :);
    [~, infoB] = mission_controller(map, start, goal, [0, cB(1), cB(2)], 2.5, false);
    mB = calculate_metrics(infoB);
    checksB = {
        'mission succeeds',                 infoB.success
        'at least one replan',              infoB.replanCount >= 1
        'hazard cell never entered',        ~visited(infoB, cB)
        'no collisions',                    ~collides(infoB)
        };
    rB = makeResult('S3b', 'S3 New obstacle during movement', ...
        sprintf('Obstacle appears at [%d,%d], two cells ahead, at step 0', cB(1), cB(2)), ...
        'Detected immediately (inside 2.5 range); replan; goal reached; hazard never entered.', ...
        actualText(infoB, mB), checksB, saveCaseMap(outDir, 'S3b', infoB, start, goal));

    % 3c: obstacle appears far ahead on the route (outside sensor range)
    cC = basePath(13, :);
    [~, infoC] = mission_controller(map, start, goal, [0, cC(1), cC(2)], 2.5, false);
    mC = calculate_metrics(infoC);
    checksC = {
        'mission succeeds',                 infoC.success
        'at least one replan (when sensed)', infoC.replanCount >= 1
        'hazard cell never entered',        ~visited(infoC, cC)
        'no collisions',                    ~collides(infoC)
        };
    rC = makeResult('S3c', 'S3 New obstacle during movement', ...
        sprintf('Obstacle appears at [%d,%d], far ahead (outside sensor range)', cC(1), cC(2)), ...
        'Unknown to the rover until within range; then replan; goal reached; hazard never entered.', ...
        actualText(infoC, mC), checksC, saveCaseMap(outDir, 'S3c', infoC, start, goal));

    r = [rA, rB, rC];
end

% =========================================================================
% S4: Destination blocked
% =========================================================================
function r = case_S4(outDir)
    [map, start, goal] = environment();

    % 4a: goal cell is an obstacle from the beginning
    mapA = map; mapA(goal(1), goal(2)) = 1;
    [~, infoA] = mission_controller(mapA, start, goal, [], 2.5, false);
    mA = calculate_metrics(infoA);
    checksA = {
        'mission does not succeed',         ~infoA.success
        'rover does not move',              infoA.totalStepsTaken == 0
        'rover stays at start',             isequal(infoA.finalPos, start)
        'status reports no initial route',  has(infoA.status, 'No valid initial route')
        'battery untouched',                infoA.batteryRemaining == infoA.batteryParams.capacity
        };
    rA = makeResult('S4a', 'S4 Destination blocked', 'Goal [19,19] is an obstacle from the start', ...
        'Mission fails safely at start: no movement, "No valid initial route", battery untouched.', ...
        actualText(infoA, mA), checksA, saveCaseMap(outDir, 'S4a', infoA, start, goal));

    % 4b: goal becomes blocked mid-mission
    [~, infoB] = mission_controller(map, start, goal, [6, goal(1), goal(2)], 2.5, false);
    mB = calculate_metrics(infoB);
    checksB = {
        'mission does not succeed',         ~infoB.success
        'rover never enters goal cell',     ~visited(infoB, goal)
        'final position is not the goal',   ~isequal(infoB.finalPos, goal)
        'replan was attempted',             infoB.replanCount >= 1
        'status reports unreachable',       has(infoB.status, 'unreachable')
        'battery not negative',             infoB.batteryRemaining >= 0
        'no collisions',                    ~collides(infoB)
        };
    rB = makeResult('S4b', 'S4 Destination blocked', 'Goal [19,19] becomes an obstacle after 6 moves', ...
        'Rover travels on the old route, senses the blocked goal when in range, halts, replan fails, mission aborts "unreachable" short of the goal.', ...
        actualText(infoB, mB), checksB, saveCaseMap(outDir, 'S4b', infoB, start, goal));

    r = [rA, rB];
end

% =========================================================================
% S5: No valid route
% =========================================================================
function r = case_S5(outDir)
    start = [2, 2]; goal = [19, 19];

    % 5a: full wall across the map
    wall = zeros(20); wall(:, 10) = 1;
    [pW, cW] = astar_path_planning(wall, start, goal);
    [~, infoA] = mission_controller(wall, start, goal, [], 2.5, false);
    mA = calculate_metrics(infoA);
    checksA = {
        'A* returns empty path and Inf cost', isempty(pW) && isinf(cW)
        'mission does not succeed',         ~infoA.success
        'rover does not move',              infoA.totalStepsTaken == 0
        'status reports no initial route',  has(infoA.status, 'No valid initial route')
        };
    rA = makeResult('S5a', 'S5 No valid route', 'Full obstacle wall in column 10', ...
        'A* returns [] / Inf; mission aborts at start with "No valid initial route".', ...
        actualText(infoA, mA), checksA, saveCaseMap(outDir, 'S5a', infoA, start, goal));

    % 5b: goal enclosed by obstacles on its four neighbours
    [mapE, ~, ~] = environment();
    mapE(18, 19) = 1; mapE(20, 19) = 1; mapE(19, 18) = 1; mapE(19, 20) = 1;
    [pE, cE] = astar_path_planning(mapE, start, goal);
    [~, infoB] = mission_controller(mapE, start, goal, [], 2.5, false);
    mB = calculate_metrics(infoB);
    checksB = {
        'A* returns empty path and Inf cost', isempty(pE) && isinf(cE)
        'mission does not succeed',         ~infoB.success
        'rover does not move',              infoB.totalStepsTaken == 0
        'status reports no initial route',  has(infoB.status, 'No valid initial route')
        };
    rB = makeResult('S5b', 'S5 No valid route', 'Goal enclosed on all four sides', ...
        'A* returns [] / Inf; mission aborts at start.', ...
        actualText(infoB, mB), checksB, saveCaseMap(outDir, 'S5b', infoB, start, goal));

    % 5c: goal gets sealed during the mission
    [mapD, ~, ~] = environment();
    sealed = [18 19; 20 19; 19 18; 19 20];
    ev = [repmat(3, 4, 1), sealed];
    [~, infoC] = mission_controller(mapD, start, goal, ev, 2.5, false);
    mC = calculate_metrics(infoC);
    enteredSealed = false;
    for k = 1:4, enteredSealed = enteredSealed || visited(infoC, sealed(k, :)); end
    checksC = {
        'mission does not succeed',         ~infoC.success
        'final position is not the goal',   ~isequal(infoC.finalPos, goal)
        'replan was attempted',             infoC.replanCount >= 1
        'status reports unreachable',       has(infoC.status, 'unreachable')
        'rover never enters sealed cells',  ~enteredSealed
        'no collisions',                    ~collides(infoC)
        'battery not negative',             infoC.batteryRemaining >= 0
        };
    rC = makeResult('S5c', 'S5 No valid route', 'All four neighbours of the goal become obstacles after 3 moves', ...
        'Rover keeps replanning on what it knows, then a replan fails once it has sensed the seal: abort "unreachable", no collision.', ...
        actualText(infoC, mC), checksC, saveCaseMap(outDir, 'S5c', infoC, start, goal));

    r = [rA, rB, rC];
end

% =========================================================================
% S6: Rover reaches destination
% =========================================================================
function r = case_S6(outDir)
    [map, start, goal] = environment();
    [~, info] = mission_controller(map, start, goal, [], 2.5, false);
    m = calculate_metrics(info);
    checksA = {
        'mission succeeds',                 info.success
        'final position equals goal',       isequal(info.finalPos, goal)
        'status starts with SUCCESS',       strncmp(info.status, 'SUCCESS', 7)
        'history ends at goal',             isequal(info.pathHistory(end, :), goal)
        'history starts at start',          isequal(info.pathHistory(1, :), start)
        'moves = history rows - 1',         info.totalStepsTaken == size(info.pathHistory, 1) - 1
        'completion is 100%',               m.completionPercent == 100
        'battery used = moves x cost',      abs(info.batteryUsed - info.totalStepsTaken * info.batteryParams.energyPerMove) < 1e-9
        };
    rA = makeResult('S6a', 'S6 Rover reaches destination', 'Default Mars map, [2,2] -> [19,19]', ...
        'SUCCESS status, rover on [19,19], history start..goal consistent, completion 100%, battery accounting exact.', ...
        actualText(info, m), checksA, saveCaseMap(outDir, 'S6a', info, start, goal));

    % 6b: already at the destination
    [~, infoB] = mission_controller(map, start, start, [], 2.5, false);
    mB = calculate_metrics(infoB);
    checksB = {
        'mission succeeds',                 infoB.success
        'zero moves',                       infoB.totalStepsTaken == 0
        'no replans',                       infoB.replanCount == 0
        'completion is 100%',               mB.completionPercent == 100
        'battery untouched',                infoB.batteryRemaining == infoB.batteryParams.capacity
        };
    rB = makeResult('S6b', 'S6 Rover reaches destination', 'Start equals goal ([2,2])', ...
        'Immediate success, 0 moves, 0 replans, battery untouched.', ...
        actualText(infoB, mB), checksB, '');

    r = [rA, rB];
end

% =========================================================================
% S7: Battery depletion
% =========================================================================
function r = case_S7(outDir)
    [map, start, goal] = environment();
    [~, optCost] = astar_path_planning(map, start, goal);

    % 7a: 10-unit battery, 1 unit per move
    [~, infoA] = mission_controller(map, start, goal, [], 2.5, false, struct('capacity', 10, 'energyPerMove', 1));
    mA = calculate_metrics(infoA);
    checksA = {
        'mission does not succeed',         ~infoA.success
        'exactly 10 moves',                 infoA.totalStepsTaken == 10
        'battery is exactly 0',             infoA.batteryRemaining == 0
        'status reports battery',           has(infoA.status, 'Battery')
        'metrics flag depletion',           mA.batteryDepleted
        'completion = 100*10/optimal cost', abs(mA.completionPercent - 100 * 10 / optCost) < 1e-6
        'no collisions',                    ~collides(infoA)
        };
    rA = makeResult('S7a', 'S7 Battery depletion', 'Capacity 10 units, 1 unit/move', ...
        'Stops after exactly 10 moves, battery 0 (never negative), status "Battery depleted", partial completion.', ...
        actualText(infoA, mA), checksA, saveCaseMap(outDir, 'S7a', infoA, start, goal));

    % 7b: one unit short of the optimal route
    [~, infoB] = mission_controller(map, start, goal, [], 2.5, false, struct('capacity', optCost - 1, 'energyPerMove', 1));
    mB = calculate_metrics(infoB);
    checksB = {
        'mission does not succeed',         ~infoB.success
        'moves = capacity (one short)',     infoB.totalStepsTaken == optCost - 1
        'battery is exactly 0',             infoB.batteryRemaining == 0
        'final position is not the goal',   ~isequal(infoB.finalPos, goal)
        'status reports battery',           has(infoB.status, 'Battery')
        };
    rB = makeResult('S7b', 'S7 Battery depletion', 'Capacity = optimal cost - 1', ...
        'Rover stops one cell short of the goal with battery 0.', ...
        actualText(infoB, mB), checksB, saveCaseMap(outDir, 'S7b', infoB, start, goal));

    % 7c: exactly enough battery
    [~, infoC] = mission_controller(map, start, goal, [], 2.5, false, struct('capacity', optCost, 'energyPerMove', 1));
    mC = calculate_metrics(infoC);
    checksC = {
        'mission succeeds',                 infoC.success
        'battery is exactly 0',             infoC.batteryRemaining == 0
        'metrics flag depletion',           mC.batteryDepleted
        'reaching goal at 0 is SUCCESS',    strncmp(infoC.status, 'SUCCESS', 7)
        };
    rC = makeResult('S7c', 'S7 Battery depletion', 'Capacity = optimal cost (exact)', ...
        'Goal reached with exactly 0 battery; counted as success.', ...
        actualText(infoC, mC), checksC, saveCaseMap(outDir, 'S7c', infoC, start, goal));

    % 7d: battery never negative when cost does not divide capacity
    [~, infoD] = mission_controller(map, start, goal, [], 2.5, false, struct('capacity', 10, 'energyPerMove', 3));
    mD = calculate_metrics(infoD);
    checksD = {
        'mission does not succeed',         ~infoD.success
        '3 moves only',                     infoD.totalStepsTaken == 3
        'remaining battery is 1',           abs(infoD.batteryRemaining - 1) < 1e-9
        'battery not negative',             infoD.batteryRemaining >= 0
        'metrics agree',                    abs(mD.batteryRemaining - 1) < 1e-9
        };
    rD = makeResult('S7d', 'S7 Battery depletion', 'Capacity 10, 3 units/move', ...
        'A move the battery cannot pay for is never started: 3 moves, 1 unit left, not negative.', ...
        actualText(infoD, mD), checksD, saveCaseMap(outDir, 'S7d', infoD, start, goal));

    % 7e: depletion combined with a replan
    [~, infoE] = mission_controller(map, start, goal, [4, 2, 8], 2.5, false, struct('capacity', 20, 'energyPerMove', 1));
    mE = calculate_metrics(infoE);
    checksE = {
        'mission does not succeed',         ~infoE.success
        'exactly 20 moves',                 infoE.totalStepsTaken == 20
        'battery is exactly 0',             infoE.batteryRemaining == 0
        'replan happened before depletion', infoE.replanCount >= 1
        'hazard cell never entered',        ~visited(infoE, [2, 8])
        'no collisions',                    ~collides(infoE)
        };
    rE = makeResult('S7e', 'S7 Battery depletion', 'Capacity 20 with rockfall at [2,8] after 4 moves', ...
        'Replans around the rockfall, then stops at 20 moves with battery 0.', ...
        actualText(infoE, mE), checksE, saveCaseMap(outDir, 'S7e', infoE, start, goal));

    r = [rA, rB, rC, rD, rE];
end

% =========================================================================
% S8: Rover attempting to move outside the map
% =========================================================================
function r = case_S8(outDir)
    map20 = zeros(20);

    % 8a: rover_movement rejects every off-grid step
    tests = {[1 1], 'up'; [1 1], 'left'; [20 20], 'down'; [20 20], 'right'; ...
             [1 20], 'up'; [1 20], 'right'; [20 1], 'down'; [20 1], 'left'; ...
             [1 10], 'up'; [10 1], 'left'; [10 20], 'right'; [20 10], 'down'};
    allRejected = true; allUnchanged = true; allMsg = true;
    for k = 1:size(tests, 1)
        rv = rover_movement(tests{k, 1});
        [rv2, ok, msg] = rover_movement(rv, tests{k, 2}, map20);
        allRejected  = allRejected && ~ok;
        allUnchanged = allUnchanged && isequal(rv2.position, tests{k, 1}) && ...
                       size(rv2.history, 1) == 1 && rv2.stepCount == 0;
        allMsg       = allMsg && has(msg, 'Outside');
    end
    checksA = {
        'every off-grid move rejected',     allRejected
        'position, history, count unchanged', allUnchanged
        'message reports "Outside" grid',   allMsg
        };
    rA = makeResult('S8a', 'S8 Moving outside the map', ...
        sprintf('%d off-grid movement attempts from corners and edges', size(tests, 1)), ...
        'Each attempt returns success=false, position/history/step count unchanged, message says Outside.', ...
        sprintf('rejected=%d, unchanged=%d, outside-message=%d', allRejected, allUnchanged, allMsg), checksA, '');

    % 8b: A* refuses start/goal outside the grid
    bad = {[0 5], [10 10]; [21 5], [10 10]; [5 0], [10 10]; [5 21], [10 10]; ...
           [10 10], [0 5]; [10 10], [21 5]; [10 10], [5 0]; [10 10], [5 21]};
    allEmpty = true;
    for k = 1:size(bad, 1)
        [p, c] = astar_path_planning(map20, bad{k, 1}, bad{k, 2});
        allEmpty = allEmpty && isempty(p) && isinf(c);
    end
    checksB = {'A* returns [] and Inf for every outside endpoint', allEmpty};
    rB = makeResult('S8b', 'S8 Moving outside the map', sprintf('%d A* queries with start or goal outside the grid', size(bad, 1)), ...
        'Every query returns empty path and Inf cost (no crash).', ...
        sprintf('all empty/Inf=%d', allEmpty), checksB, '');

    % 8c: a full mission along the grid border stays inside the map
    [~, infoC] = mission_controller(map20, [1, 1], [20, 20], [], 2.5, false);
    mC = calculate_metrics(infoC);
    h = infoC.pathHistory;
    inside = all(h(:) >= 1 & h(:) <= 20);
    checksC = {
        'mission succeeds',                 infoC.success
        'every visited cell inside 1..20',  inside
        'moves equal Manhattan distance 38', infoC.totalStepsTaken == 38
        'exactly one cell per move',        singleSteps(h)
        };
    rC = makeResult('S8c', 'S8 Moving outside the map', 'Empty map, corner [1,1] -> corner [20,20]', ...
        'Reaches [20,20] in 38 moves; every visited cell stays within rows/cols 1..20.', ...
        actualText(infoC, mC), checksC, saveCaseMap(outDir, 'S8c', infoC, [1, 1], [20, 20]));

    % 8d: destination outside the map
    [mapE, startE, ~] = environment();
    [~, infoD] = mission_controller(mapE, startE, [25, 25], [], 2.5, false);
    mD = calculate_metrics(infoD);
    checksD = {
        'no crash and mission does not succeed', ~infoD.success
        'rover does not move',              infoD.totalStepsTaken == 0
        'status reports no initial route',  has(infoD.status, 'No valid initial route')
        };
    rD = makeResult('S8d', 'S8 Moving outside the map', 'Destination [25,25] is outside the 20x20 map', ...
        'Safe failure at start: no movement, "No valid initial route", no exception.', ...
        actualText(infoD, mD), checksD, '');

    % 8e: sensor at all four corners does not index outside the map
    sensorOk = true;
    corners = [1 1; 1 20; 20 1; 20 20];
    for k = 1:4
        try
            obstacle_detection(mapE, corners(k, :), 3.0);
        catch
            sensorOk = false;
        end
    end
    checksE = {'proximity sensor runs at all four corners without error', sensorOk};
    rE = makeResult('S8e', 'S8 Moving outside the map', 'Proximity sensor (R=3.0) at the four map corners', ...
        'No out-of-bounds indexing error.', sprintf('no error at corners=%d', sensorOk), checksE, '');

    % 8f: dynamic hazards placed outside the map are ignored
    [~, optCost] = astar_path_planning(mapE, startE, [19, 19]);
    [~, infoF] = mission_controller(mapE, startE, [19, 19], [2, 25, 25; 2, 0, 3], 2.5, false);
    mF = calculate_metrics(infoF);
    ignoredLogged = any(cellfun(@(s) has(s, 'ignored'), infoF.executionLog));
    checksF = {
        'mission succeeds',                 infoF.success
        'moves equal the unaffected optimum', infoF.totalStepsTaken == optCost
        'no replans',                       infoF.replanCount == 0
        'events logged as ignored',         ignoredLogged
        };
    rF = makeResult('S8f', 'S8 Moving outside the map', 'Dynamic hazards at [25,25] and [0,3] (outside the map)', ...
        'Events are ignored and logged; the mission is unaffected.', ...
        actualText(infoF, mF), checksF, '');

    r = [rA, rB, rC, rD, rE, rF];
end

% =========================================================================
% Helpers
% =========================================================================
function r = makeResult(id, scenario, caseName, expected, actual, checks, imageFile)
    failed = {};
    for k = 1:size(checks, 1)
        v = checks{k, 2};
        ok = (islogical(v) || isnumeric(v)) && ~isempty(v) && all(v(:));
        if ~ok, failed{end+1} = checks{k, 1}; end %#ok<AGROW>
    end
    r.id       = id;
    r.group    = regexp(id, '^S\d', 'match', 'once');
    r.scenario = scenario;
    r.caseName = caseName;
    r.expected = expected;
    r.actual   = actual;
    r.nChecks  = size(checks, 1);
    r.failed   = strjoin(failed, '; ');
    if isempty(failed), r.verdict = 'PASS'; else, r.verdict = 'FAIL'; end
    r.image    = imageFile;
end

function r = runSafely(fn, id)
    try
        r = fn();
    catch ME
        r = makeResult(id, [id ' (crashed)'], 'Test case crashed', 'Executes without an uncaught error', ...
                       ['ERROR: ' ME.message], {'no uncaught error', false}, '');
    end
end

function tf = has(str, sub)
    tf = ~isempty(strfind(str, sub)); %#ok<STREMP>
end

function tf = collides(info)
    h = info.pathHistory; M = info.finalMap;
    tf = any(M(sub2ind(size(M), h(:, 1), h(:, 2))) == 1);
end

function tf = visited(info, cell)
    tf = any(all(info.pathHistory == repmat(cell, size(info.pathHistory, 1), 1), 2));
end

function tf = singleSteps(h)
    tf = size(h, 1) < 2 || all(sum(abs(diff(h, 1, 1)), 2) == 1);
end

function s = actualText(info, m)
    if isnan(m.completionPercent), comp = 'n/a'; else, comp = sprintf('%.1f%%', m.completionPercent); end
    s = sprintf('success=%d, moves=%d, replans=%d, final=[%d,%d], battery=%.1f%%, completion=%s, status="%s"', ...
        info.success, info.totalStepsTaken, info.replanCount, info.finalPos(1), info.finalPos(2), ...
        m.batteryRemainingPercent, comp, info.status);
end

function f = saveCaseMap(outDir, id, info, start, goal)
    f = '';
    try
        fig = figure('Visible', 'off', 'Color', 'w');
        ax = axes('Parent', fig);
        M = info.finalMap;
        imagesc(ax, M, [0, 1]); hold(ax, 'on');
        colormap(ax, [0.82, 0.70, 0.55; 0.25, 0.25, 0.25]);
        h = info.pathHistory;
        plot(ax, h(:, 2), h(:, 1), '-', 'Color', [0.08, 0.64, 0.29], 'LineWidth', 2.5);
        plot(ax, start(2), start(1), 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.13, 0.77, 0.37], 'MarkerEdgeColor', 'k');
        if all(goal >= 1) && goal(1) <= size(M, 1) && goal(2) <= size(M, 2)
            plot(ax, goal(2), goal(1), 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.94, 0.27, 0.27], 'MarkerEdgeColor', 'k');
        end
        plot(ax, info.finalPos(2), info.finalPos(1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.15, 0.45, 0.95], 'MarkerEdgeColor', 'k');
        set(ax, 'YDir', 'reverse');
        axis(ax, 'equal'); xlim(ax, [0.5, size(M, 2) + 0.5]); ylim(ax, [0.5, size(M, 1) + 0.5]);
        title(ax, sprintf('%s | success=%d | moves=%d | replans=%d', id, info.success, info.totalStepsTaken, info.replanCount));
        f = fullfile(outDir, [id '.png']);
        saveas(fig, f);
        delete(fig);
    catch
        f = '';
        if exist('fig', 'var') && ishghandle(fig), delete(fig); end
    end
end

function printReport(fid, R)
    fprintf(fid, '============================================================\n');
    fprintf(fid, '  FULL SYSTEM TEST REPORT\n');
    fprintf(fid, '============================================================\n');
    for k = 1:numel(R)
        fprintf(fid, '\n[%s] %s -- %s\n', R(k).verdict, R(k).id, R(k).caseName);
        fprintf(fid, '   Scenario : %s\n', R(k).scenario);
        fprintf(fid, '   Expected : %s\n', R(k).expected);
        fprintf(fid, '   Actual   : %s\n', R(k).actual);
        if ~isempty(R(k).failed)
            fprintf(fid, '   FAILED CHECKS: %s\n', R(k).failed);
        end
    end

    fprintf(fid, '\n------------------------------------------------------------\n');
    fprintf(fid, '  SUMMARY BY SCENARIO (a scenario passes only if all its cases pass)\n');
    fprintf(fid, '------------------------------------------------------------\n');
    groups = unique({R.group}, 'stable');
    nPassGroups = 0;
    for g = 1:numel(groups)
        idx = strcmp({R.group}, groups{g});
        allPass = all(strcmp({R(idx).verdict}, 'PASS'));
        nPassGroups = nPassGroups + allPass;
        titleStr = R(find(idx, 1)).scenario;
        if allPass, v = 'PASS'; else, v = 'FAIL'; end
        fprintf(fid, '  %-4s %-45s %d/%d cases  %s\n', groups{g}, titleStr, ...
            sum(strcmp({R(idx).verdict}, 'PASS')), sum(idx), v);
    end
    nPass = sum(strcmp({R.verdict}, 'PASS'));
    fprintf(fid, '\nCases passed    : %d / %d\n', nPass, numel(R));
    fprintf(fid, 'Scenarios passed: %d / %d\n', nPassGroups, numel(groups));
    if nPass == numel(R)
        fprintf(fid, 'STATUS: ALL EXECUTED CASES PASSED.\n');
    else
        fprintf(fid, 'STATUS: %d CASE(S) FAILED - see FAILED CHECKS above.\n', numel(R) - nPass);
    end
    fprintf(fid, '============================================================\n');
end

function writeResultsCsv(filePath, R)
    fid = fopen(filePath, 'w');
    if fid < 0, warning('Could not write %s', filePath); return; end
    q = @(t) ['"' strrep(t, '"', '""') '"'];
    fprintf(fid, 'id,scenario,case,verdict,checks,failed_checks,expected,actual,image\n');
    for k = 1:numel(R)
        fprintf(fid, '%s,%s,%s,%s,%d,%s,%s,%s,%s\n', R(k).id, q(R(k).scenario), q(R(k).caseName), ...
            R(k).verdict, R(k).nChecks, q(R(k).failed), q(R(k).expected), q(R(k).actual), q(R(k).image));
    end
    fclose(fid);
end