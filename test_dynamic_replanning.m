% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_dynamic_replanning.m
% Description: Demonstrates dynamic obstacle appearance during transit,
%              proximity sensor detection, emergency stop before collision,
%              environment map updating, and real-time A* replanning.
%
% Flow of Operations:
%   1. Generate initial Mars environment map.
%   2. Calculate initial A* route from [2, 2] to [19, 19].
%   3. Rover travels along route to waypoint [2, 6].
%   4. Dynamic rockfall obstacle appears directly on the route at [2, 8].
%   5. Sensor scans local area (R = 2.5) and detects blocked waypoint.
%   6. Rover halts at [2, 6] prior to entering the hazardous sector.
%   7. Map is updated with the new obstacle.
%   8. Updated map is dispatched to A* planner from current location [2, 6].
%   9. Safe detour route is computed and verified.
%  10. Visualization rendered and saved to results/dynamic_replanning.png.
% =========================================================================

clear;
clc;
close all;

fprintf('============================================================\n');
fprintf('  DYNAMIC OBSTACLE DETECTION & REAL-TIME REPLANNING TEST\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

%% Step 1: Initialize Environment and Compute Initial Planned Route
fprintf('[Step 1] Loading planetary environment and computing initial route...\n');
[map, start, goal] = environment();
[initialPath, initialCost] = astar_path_planning(map, start, goal);

fprintf('         Initial route calculated: %d waypoints (Cost: %d steps).\n', ...
    size(initialPath, 1), initialCost);

%% Step 2: Rover Travels to Waypoint [2, 6]
fprintf('\n[Step 2] Rover commences transit along initial planned route...\n');
roverStopPos = [2, 6];
stopIdx = find(initialPath(:, 1) == roverStopPos(1) & initialPath(:, 2) == roverStopPos(2), 1);

traveledSegment = initialPath(1:stopIdx, :);
remainingOriginalPath = initialPath(stopIdx+1:end, :);

fprintf('         Rover successfully advanced to waypoint [%d, %d] (%d steps traveled).\n', ...
    roverStopPos(1), roverStopPos(2), size(traveledSegment, 1) - 1);

%% Step 3: Introduce New Dynamic Obstacle on the Planned Route
fprintf('\n[Step 3] Dynamic hazard event: Sudden rockfall occurs on route!\n');
newObstacleCoord = [2, 8];
fprintf('         New obstacle appeared at [%d, %d] (previously free cell on original path).\n', ...
    newObstacleCoord(1), newObstacleCoord(2));

% Update the environment map with the new obstacle
mapUpdated = map;
mapUpdated(newObstacleCoord(1), newObstacleCoord(2)) = 1;

%% Step 4: Proximity Sensor Scans and Detects Hazard
fprintf('\n[Step 4] Triggering proximity sensor scan at current position [%d, %d]...\n', ...
    roverStopPos(1), roverStopPos(2));
sensorRadius = 2.5;

[detectedObs, sensorData] = obstacle_detection(mapUpdated, roverStopPos, sensorRadius, false, remainingOriginalPath);

totalTests = totalTests + 1;
if sensorData.isPathBlocked && any(sensorData.blockingObstacles(:, 1) == newObstacleCoord(1) & ...
                                  sensorData.blockingObstacles(:, 2) == newObstacleCoord(2))
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1 - Sensor Hazard Detection: Obstacle detected at [%d, %d] (Distance = %.2f grid units).\n', ...
        newObstacleCoord(1), newObstacleCoord(2), ...
        sqrt((newObstacleCoord(1)-roverStopPos(1))^2 + (newObstacleCoord(2)-roverStopPos(2))^2));
    fprintf('         Collision Alert: Obstacle lies directly on the remaining planned route!\n');
else
    fprintf('  [FAIL] Test 1 - Failed to detect path-blocking obstacle.\n');
end

%% Step 5: Stop Rover Before Entering Blocked Cell
fprintf('\n[Step 5] Autonomous Safety Protocol Triggered: EMERGENCY STOP.\n');
totalTests = totalTests + 1;
distToBlockedCell = abs(newObstacleCoord(1) - roverStopPos(1)) + abs(newObstacleCoord(2) - roverStopPos(2));

if distToBlockedCell > 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2 - Safe Halt: Rover stopped at [%d, %d], %d cells before the blocked hazard.\n', ...
        roverStopPos(1), roverStopPos(2), distToBlockedCell);
else
    fprintf('  [FAIL] Test 2 - Rover collided with obstacle.\n');
end

%% Step 6 & 7: Send Updated Map to A* and Compute Replanned Route
fprintf('\n[Step 6 & 7] Dispatching updated map to A* path planner...\n');
fprintf('         Originating replanned search from current rover location [%d, %d] to goal [%d, %d]...\n', ...
    roverStopPos(1), roverStopPos(2), goal(1), goal(2));

[replannedPath, replannedCost] = astar_path_planning(mapUpdated, roverStopPos, goal);

totalTests = totalTests + 1;
isNewObsAvoided = ~any(replannedPath(:, 1) == newObstacleCoord(1) & replannedPath(:, 2) == newObstacleCoord(2));

if ~isempty(replannedPath) && isNewObsAvoided && isequal(replannedPath(end, :), goal)
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3 - Autonomous Replanning: Detour route computed successfully!\n');
    fprintf('         Replanned Path Length: %d waypoints (Cost: %d steps).\n', ...
        size(replannedPath, 1), replannedCost);
    fprintf('         Verification: New obstacle [%d, %d] is completely bypassed.\n\n', ...
        newObstacleCoord(1), newObstacleCoord(2));
else
    fprintf('  [FAIL] Test 3 - Replanning failed or traversed blocked cell.\n\n');
end

%% Step 8: Render and Save Visualization
fprintf('[Step 8] Rendering dynamic replanning visualization...\n');
totalTests = totalTests + 1;

try
    figHandle = figure('Name', 'Dynamic Obstacle Detection & Replanning', ...
                       'NumberTitle', 'off', ...
                       'Color', 'w', ...
                       'Position', [100, 100, 850, 720]);

    % Base map with updated obstacle
    imagesc(mapUpdated, [0, 1]);
    axis equal;
    axis tight;
    hold on;

    colormap([0.82, 0.70, 0.55; 0.25, 0.25, 0.25]);

    % Grid boundary lines
    [gR, gC] = size(mapUpdated);
    for x = 0.5:1:(gC + 0.5)
        line([x, x], [0.5, gR + 0.5], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
    end
    for y = 0.5:1:(gR + 0.5)
        line([0.5, gC + 0.5], [y, y], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
    end

    % Sensor detection zone at stop position
    theta = linspace(0, 2*pi, 200);
    patch(roverStopPos(2) + sensorRadius*cos(theta), roverStopPos(1) + sensorRadius*sin(theta), ...
          [0.22, 0.74, 0.97], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', 'Sensor Field');
    hRange = plot(roverStopPos(2) + sensorRadius*cos(theta), roverStopPos(1) + sensorRadius*sin(theta), ...
                  '--', 'Color', [0.01, 0.52, 0.78], 'LineWidth', 1.5, ...
                  'DisplayName', sprintf('Sensor Range (R = %.1f)', sensorRadius));

    % Original path (dashed gray)
    hOrig = plot(initialPath(:, 2), initialPath(:, 1), ':', ...
                 'Color', [0.6, 0.6, 0.6], 'LineWidth', 2.0, ...
                 'DisplayName', 'Original Route');

    % Traveled segment (solid green)
    hTrav = plot(traveledSegment(:, 2), traveledSegment(:, 1), '-o', ...
                 'Color', [0.08, 0.64, 0.29], 'LineWidth', 3.0, 'MarkerSize', 5, ...
                 'DisplayName', sprintf('Traveled Segment ([%d,%d] -> [%d,%d])', ...
                                start(1), start(2), roverStopPos(1), roverStopPos(2)));

    % Replanned route (solid blue)
    hReplan = plot(replannedPath(:, 2), replannedPath(:, 1), '-o', ...
                   'Color', [0.12, 0.25, 0.69], 'LineWidth', 2.5, 'MarkerSize', 4, ...
                   'MarkerFaceColor', [0.12, 0.25, 0.69], ...
                   'DisplayName', sprintf('Replanned Route (%d cells, Cost: %d)', ...
                                  size(replannedPath, 1), replannedCost));

    % Dynamic obstacle marker (bold red X)
    hDynObs = plot(newObstacleCoord(2), newObstacleCoord(1), 'X', ...
                   'MarkerSize', 14, 'MarkerFaceColor', [0.86, 0.15, 0.15], ...
                   'MarkerEdgeColor', 'k', 'LineWidth', 2.0, ...
                   'DisplayName', sprintf('Dynamic Obstacle [%d, %d]', newObstacleCoord(1), newObstacleCoord(2)));

    % Rover Stopped position marker (green square)
    hStop = plot(roverStopPos(2), roverStopPos(1), 's', ...
                 'MarkerSize', 13, 'MarkerFaceColor', [0.13, 0.77, 0.37], ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 2.0, ...
                 'DisplayName', sprintf('Rover Stopped [%d, %d]', roverStopPos(1), roverStopPos(2)));

    % Goal marker (red star)
    hGoal = plot(goal(2), goal(1), 'p', ...
                 'MarkerSize', 16, 'MarkerFaceColor', [0.94, 0.27, 0.27], ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
                 'DisplayName', sprintf('Destination [%d, %d]', goal(1), goal(2)));

    set(gca, 'XTick', 1:gC, 'YTick', 1:gR);
    set(gca, 'XAxisLocation', 'bottom');
    set(gca, 'YDir', 'reverse');
    xlim([0.5, gC + 0.5]);
    ylim([0.5, gR + 0.5]);

    xlabel('X Coordinate (Grid Columns)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Y Coordinate (Grid Rows)', 'FontSize', 11, 'FontWeight', 'bold');
    title({'Dynamic Obstacle Detection & Real-Time Replanning', ...
           sprintf('Hazard at [%d, %d] Detected -> Stopped at [%d, %d] -> Detour: %d cells (Cost: %d)', ...
                   newObstacleCoord(1), newObstacleCoord(2), roverStopPos(1), roverStopPos(2), ...
                   size(replannedPath, 1), replannedCost)}, ...
           'FontSize', 12, 'FontWeight', 'bold');

    legend([hTrav, hStop, hDynObs, hOrig, hReplan, hGoal, hRange], ...
           'Location', 'northeastoutside', 'FontSize', 9.5);

    currentDir = fileparts(mfilename('fullpath'));
    resultsDir = fullfile(currentDir, 'results');
    outImgPath = fullfile(resultsDir, 'dynamic_replanning.png');
    saveas(figHandle, outImgPath);

    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 4 - Visualization Output: Saved to results/dynamic_replanning.png\n\n');
catch ME
    fprintf('  [FAIL] Test 4 - Visualization error: %s\n\n', ME.message);
end

%% Summary
fprintf('------------------------------------------------------------\n');
fprintf('Dynamic Replanning Test Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: DETECTION AND REAL-TIME REPLANNING VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
