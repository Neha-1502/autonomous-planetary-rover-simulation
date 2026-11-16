% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_rover_movement.m
% Description: Verification suite for rover kinematics, 4-direction movement,
%              boundary collision avoidance, obstacle collision avoidance,
%              and trajectory history tracking.
% =========================================================================

clear;
clc;

fprintf('====================================================\n');
fprintf('  ROVER MOVEMENT MODULE: VERIFICATION TEST SUITE\n');
fprintf('====================================================\n\n');

% Load standard 20x20 planetary environment
[map, startPos, ~] = environment();

totalTests  = 0;
passedTests = 0;

%% -------------------------------------------------------------------------
%% Group 1: Initialization Tests
%% -------------------------------------------------------------------------
fprintf('[GROUP 1] Testing State Initialization...\n');

totalTests = totalTests + 1;
rover = rover_movement(startPos);

if isequal(rover.position, [2, 2]) && ...
   isequal(rover.history, [2, 2]) && ...
   rover.stepCount == 0
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1.1 - Initial State: Pos=[2,2], History length=1, Steps=0\n');
else
    fprintf('  [FAIL] Test 1.1 - Initial State incorrect.\n');
end

%% -------------------------------------------------------------------------
%% Group 2: Four-Directional Movement Tests ('right', 'down', 'left', 'up')
%% -------------------------------------------------------------------------
fprintf('\n[GROUP 2] Testing 4-Direction Movement...\n');

% Move 1: Right -> Expected [2, 3]
totalTests = totalTests + 1;
[rover, ok, msg] = rover_movement(rover, 'right', map);
if ok && isequal(rover.position, [2, 3]) && rover.stepCount == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.1 - Move RIGHT: Moved [2,2] -> [2,3] (Steps=%d)\n', rover.stepCount);
else
    fprintf('  [FAIL] Test 2.1 - Move RIGHT failed. Msg: %s\n', msg);
end

% Move 2: Down -> Expected [3, 3]
totalTests = totalTests + 1;
[rover, ok, msg] = rover_movement(rover, 'down', map);
if ok && isequal(rover.position, [3, 3]) && rover.stepCount == 2
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.2 - Move DOWN:  Moved [2,3] -> [3,3] (Steps=%d)\n', rover.stepCount);
else
    fprintf('  [FAIL] Test 2.2 - Move DOWN failed. Msg: %s\n', msg);
end

% Move 3: Left -> Expected [3, 2]
totalTests = totalTests + 1;
[rover, ok, msg] = rover_movement(rover, 'left', map);
if ok && isequal(rover.position, [3, 2]) && rover.stepCount == 3
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.3 - Move LEFT:  Moved [3,3] -> [3,2] (Steps=%d)\n', rover.stepCount);
else
    fprintf('  [FAIL] Test 2.3 - Move LEFT failed. Msg: %s\n', msg);
end

% Move 4: Up -> Expected [2, 2]
totalTests = totalTests + 1;
[rover, ok, msg] = rover_movement(rover, 'up', map);
if ok && isequal(rover.position, [2, 2]) && rover.stepCount == 4
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.4 - Move UP:    Moved [3,2] -> [2,2] (Steps=%d)\n', rover.stepCount);
else
    fprintf('  [FAIL] Test 2.4 - Move UP failed. Msg: %s\n', msg);
end

% History verification after 4 sequential moves (should have 5 entries)
totalTests = totalTests + 1;
expectedHistory = [2, 2; 2, 3; 3, 3; 3, 2; 2, 2];
if isequal(rover.history, expectedHistory) && size(rover.history, 1) == 5
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.5 - History Tracking: Complete 5-coordinate trajectory matches expected path.\n');
else
    fprintf('  [FAIL] Test 2.5 - History tracking mismatch.\n');
end

%% -------------------------------------------------------------------------
%% Group 3: Boundary Condition Tests (Grid limits: 1 to 20)
%% -------------------------------------------------------------------------
fprintf('\n[GROUP 3] Testing Grid Boundary Collision Prevention...\n');

% Boundary 1: Top Edge - Attempt to move 'up' from [1, 1]
totalTests = totalTests + 1;
edgeRoverTop = rover_movement([1, 1]);
[edgeRoverTop, ok, msg] = rover_movement(edgeRoverTop, 'up', map);
if ~ok && isequal(edgeRoverTop.position, [1, 1]) && size(edgeRoverTop.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.1 - Top Boundary: Moving ''up'' from [1,1] correctly blocked.\n');
else
    fprintf('  [FAIL] Test 3.1 - Top boundary check failed.\n');
end

% Boundary 2: Left Edge - Attempt to move 'left' from [1, 1]
totalTests = totalTests + 1;
edgeRoverLeft = rover_movement([1, 1]);
[edgeRoverLeft, ok, msg] = rover_movement(edgeRoverLeft, 'left', map);
if ~ok && isequal(edgeRoverLeft.position, [1, 1]) && size(edgeRoverLeft.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.2 - Left Boundary: Moving ''left'' from [1,1] correctly blocked.\n');
else
    fprintf('  [FAIL] Test 3.2 - Left boundary check failed.\n');
end

% Boundary 3: Bottom Edge - Attempt to move 'down' from [20, 20]
totalTests = totalTests + 1;
edgeRoverBottom = rover_movement([20, 20]);
[edgeRoverBottom, ok, msg] = rover_movement(edgeRoverBottom, 'down', map);
if ~ok && isequal(edgeRoverBottom.position, [20, 20]) && size(edgeRoverBottom.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.3 - Bottom Boundary: Moving ''down'' from [20,20] correctly blocked.\n');
else
    fprintf('  [FAIL] Test 3.3 - Bottom boundary check failed.\n');
end

% Boundary 4: Right Edge - Attempt to move 'right' from [20, 20]
totalTests = totalTests + 1;
edgeRoverRight = rover_movement([20, 20]);
[edgeRoverRight, ok, msg] = rover_movement(edgeRoverRight, 'right', map);
if ~ok && isequal(edgeRoverRight.position, [20, 20]) && size(edgeRoverRight.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.4 - Right Boundary: Moving ''right'' from [20,20] correctly blocked.\n');
else
    fprintf('  [FAIL] Test 3.4 - Right boundary check failed.\n');
end

%% -------------------------------------------------------------------------
%% Group 4: Obstacle Collision Tests
%% -------------------------------------------------------------------------
fprintf('\n[GROUP 4] Testing Obstacle Collision Prevention...\n');

% Collision 1: Move 'right' from [4, 4] into Crater Alpha obstacle cell [4, 5]
totalTests = totalTests + 1;
obsRover1 = rover_movement([4, 4]);
[obsRover1, ok, msg] = rover_movement(obsRover1, 'right', map);
if ~ok && isequal(obsRover1.position, [4, 4]) && size(obsRover1.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 4.1 - Crater Collision: Move [4,4]->[4,5] blocked (map(4,5)=%d).\n', map(4, 5));
else
    fprintf('  [FAIL] Test 4.1 - Crater obstacle collision check failed.\n');
end

% Collision 2: Move 'down' from [8, 2] into Boulder Cluster obstacle cell [9, 2]
totalTests = totalTests + 1;
obsRover2 = rover_movement([8, 2]);
[obsRover2, ok, msg] = rover_movement(obsRover2, 'down', map);
if ~ok && isequal(obsRover2.position, [8, 2]) && size(obsRover2.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 4.2 - Boulder Collision: Move [8,2]->[9,2] blocked (map(9,2)=%d).\n', map(9, 2));
else
    fprintf('  [FAIL] Test 4.2 - Boulder obstacle collision check failed.\n');
end

% Collision 3: Move 'right' from [3, 12] into Rock Ridge obstacle cell [3, 13]
totalTests = totalTests + 1;
obsRover3 = rover_movement([3, 12]);
[obsRover3, ok, msg] = rover_movement(obsRover3, 'right', map);
if ~ok && isequal(obsRover3.position, [3, 12]) && size(obsRover3.history, 1) == 1
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 4.3 - Rock Ridge Collision: Move [3,12]->[3,13] blocked (map(3,13)=%d).\n', map(3, 13));
else
    fprintf('  [FAIL] Test 4.3 - Rock ridge obstacle collision check failed.\n');
end

%% -------------------------------------------------------------------------
%% Group 5: Invalid Direction Handling
%% -------------------------------------------------------------------------
fprintf('\n[GROUP 5] Testing Invalid Movement Directions...\n');

totalTests = totalTests + 1;
invRover = rover_movement([2, 2]);
[invRover, ok, msg] = rover_movement(invRover, 'diagonal', map);
if ~ok && isequal(invRover.position, [2, 2])
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 5.1 - Invalid Direction: Rejected ''diagonal'' command gracefully.\n');
else
    fprintf('  [FAIL] Test 5.1 - Invalid direction handling failed.\n');
end

%% -------------------------------------------------------------------------
%% Summary
%% -------------------------------------------------------------------------
fprintf('\n----------------------------------------------------\n');
fprintf('Test Suite Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: ALL ROVER MOVEMENT TESTS COMPLETED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('====================================================\n');
