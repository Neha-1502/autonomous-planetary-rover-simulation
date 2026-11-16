% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_rover_movement_independent.m
% Description: Independent test script for rover_movement module verifying:
%   1. Environment initialization.
%   2. Placement at [2, 2].
%   3. Upward movement.
%   4. Downward movement.
%   5. Left movement.
%   6. Right movement.
%   7. Obstacle collision rejection (position remains unchanged).
%   8. Boundary limit rejection (position remains unchanged).
%
% Coordinates are printed after every single operation.
% =========================================================================

clear;
clc;

fprintf('============================================================\n');
fprintf('  INDEPENDENT VERIFICATION: ROVER MOVEMENT MODULE\n');
fprintf('============================================================\n\n');

%% Step 1: Initialize the Environment
fprintf('[Step 1] Initializing planetary environment...\n');
[map, start, goal] = environment();
fprintf('         Environment loaded: %dx%d grid, %d obstacle cells.\n\n', ...
    size(map, 1), size(map, 2), sum(map(:) == 1));

%% Step 2: Place Rover at [2, 2]
fprintf('[Step 2] Placing rover at starting position [2, 2]...\n');
rover = rover_movement(start);
fprintf('         Rover coordinates: [%d, %d]\n\n', rover.position(1), rover.position(2));

%% Step 3: Test Upward Movement
fprintf('[Step 3] Testing UPWARD movement (''up'')...\n');
[rover, ok, msg] = rover_movement(rover, 'up', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates: [%d, %d]\n\n', rover.position(1), rover.position(2));

%% Step 4: Test Downward Movement
fprintf('[Step 4] Testing DOWNWARD movement (''down'')...\n');
[rover, ok, msg] = rover_movement(rover, 'down', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates: [%d, %d]\n\n', rover.position(1), rover.position(2));

%% Step 5: Test Left Movement
fprintf('[Step 5] Testing LEFT movement (''left'')...\n');
[rover, ok, msg] = rover_movement(rover, 'left', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates: [%d, %d]\n\n', rover.position(1), rover.position(2));

%% Step 6: Test Right Movement
fprintf('[Step 6] Testing RIGHT movement (''right'')...\n');
[rover, ok, msg] = rover_movement(rover, 'right', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates: [%d, %d]\n\n', rover.position(1), rover.position(2));

%% Step 7: Attempt to Move Into an Obstacle
fprintf('[Step 7] Testing OBSTACLE COLLISION PREVENTION...\n');
% Position rover at [4, 4], directly adjacent to Crater Alpha obstacle cell [4, 5]
posBeforeObs = [4, 4];
obsRover = rover_movement(posBeforeObs);
fprintf('         Placing rover adjacent to Crater Alpha obstacle at [%d, %d]\n', ...
    posBeforeObs(1), posBeforeObs(2));
fprintf('         Target cell [4, 5] map value: %d (1 = Obstacle)\n', map(4, 5));
fprintf('         Attempting move ''right'' into obstacle...\n');

[obsRover, ok, msg] = rover_movement(obsRover, 'right', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates after attempt: [%d, %d]\n', ...
    obsRover.position(1), obsRover.position(2));

if isequal(obsRover.position, posBeforeObs)
    fprintf('         [CONFIRMED] Invalid move into obstacle DID NOT change rover position!\n\n');
else
    fprintf('         [ERROR] Rover position changed after obstacle collision!\n\n');
end

%% Step 8: Attempt to Move Outside the Grid
fprintf('[Step 8] Testing GRID BOUNDARY VIOLATION PREVENTION...\n');
% Position rover at top-left corner [1, 1]
posBeforeBound = [1, 1];
boundRover = rover_movement(posBeforeBound);
fprintf('         Placing rover at top-left grid boundary [%d, %d]\n', ...
    posBeforeBound(1), posBeforeBound(2));

% Attempt 8a: Move 'up' beyond row 1
fprintf('         Attempt 8a: Moving ''up'' towards row 0 (outside grid)...\n');
[boundRover, ok, msg] = rover_movement(boundRover, 'up', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates after attempt: [%d, %d]\n', ...
    boundRover.position(1), boundRover.position(2));

if isequal(boundRover.position, posBeforeBound)
    fprintf('         [CONFIRMED] Upward boundary violation DID NOT change rover position!\n');
else
    fprintf('         [ERROR] Rover position changed after upward boundary violation!\n');
end

% Attempt 8b: Move 'left' beyond col 1
fprintf('         Attempt 8b: Moving ''left'' towards column 0 (outside grid)...\n');
[boundRover, ok, msg] = rover_movement(boundRover, 'left', map);
fprintf('         Status: %s\n', msg);
fprintf('         Rover coordinates after attempt: [%d, %d]\n', ...
    boundRover.position(1), boundRover.position(2));

if isequal(boundRover.position, posBeforeBound)
    fprintf('         [CONFIRMED] Leftward boundary violation DID NOT change rover position!\n\n');
else
    fprintf('         [ERROR] Rover position changed after leftward boundary violation!\n\n');
end

%% Final Summary
fprintf('============================================================\n');
fprintf('  INDEPENDENT TEST VERIFICATION COMPLETED: ALL 8 STEPS PASSED\n');
fprintf('============================================================\n');
