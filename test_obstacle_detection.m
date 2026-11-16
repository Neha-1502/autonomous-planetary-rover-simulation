% =========================================================================
% Project: Autonomous Planetary Rover Simulation Using MATLAB
% Script: test_obstacle_detection.m
% Description: Verification suite for simulated mathematical proximity sensor.
%
% Tests performed:
%   1. Configurable Sensing Radius (Radius = 2.0 vs 3.0).
%   2. Distinction Between Nearby and Distant Hazards.
%   3. Boundary Positions Handling (Top-left [1, 1], Bottom-right [20, 20]).
%   4. Sensor Detection Area Visualization and File Generation.
% =========================================================================

clear;
clc;
close all;

fprintf('============================================================\n');
fprintf('  PROXIMITY SENSOR MODULE: VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

totalTests  = 0;
passedTests = 0;

% Load standard 20x20 planetary environment
[map, start, goal] = environment();

%% =========================================================================
%% Test 1: Configurable Sensing Radius & Detection Accuracy
%% =========================================================================
fprintf('[TEST 1] Testing Obstacle Detection with Configurable Radius...\n');
totalTests = totalTests + 1;

roverPos1 = [4, 4]; % Positioned near Crater Alpha
radius1 = 3.0;
[detected1, data1] = obstacle_detection(map, roverPos1, radius1);

if data1.totalDetected == 10 && size(detected1, 1) == 10
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1.1 - Detection at [%d, %d] (R = %.1f): Detected %d obstacle cells.\n', ...
        roverPos1(1), roverPos1(2), radius1, data1.totalDetected);
else
    fprintf('  [FAIL] Test 1.1 - Detection count mismatch (Got: %d, Expected: 10).\n', data1.totalDetected);
end

% Verify smaller radius
totalTests = totalTests + 1;
radiusSmall = 1.5;
[detectedSmall, dataSmall] = obstacle_detection(map, roverPos1, radiusSmall);

if dataSmall.totalDetected == 3 && size(detectedSmall, 1) == 3
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 1.2 - Scaled Radius (R = %.1f): Detected %d cells (properly bounded).\n\n', ...
        radiusSmall, dataSmall.totalDetected);
else
    fprintf('  [FAIL] Test 1.2 - Scaled radius test failed.\n\n');
end

%% =========================================================================
%% Test 2: Distinction Between Nearby and Distant Obstacles
%% =========================================================================
fprintf('[TEST 2] Testing Classification (Nearby vs Distant Hazards)...\n');
totalTests = totalTests + 1;

% For radius 3.0, nearThreshold = 1.5
% Expected: 3 nearby hazards (distances <= 1.5), 7 distant hazards (1.5 < dist <= 3.0)
hasValidClassification = (data1.numNearby == 3) && (data1.numDistant == 7) && ...
    all(data1.distances(1:data1.numNearby) <= data1.nearThreshold);

if hasValidClassification
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 2.1 - Alert Zones: %d Nearby Hazards (R <= %.1f), %d Distant Hazards (%.1f < R <= %.1f).\n', ...
        data1.numNearby, data1.nearThreshold, data1.numDistant, data1.nearThreshold, radius1);
    fprintf('         Nearby coordinates:\n         ');
    for i = 1:size(data1.nearbyObstacles, 1)
        fprintf('[%d, %d] ', data1.nearbyObstacles(i, 1), data1.nearbyObstacles(i, 2));
    end
    fprintf('\n\n');
else
    fprintf('  [FAIL] Test 2.1 - Classification into nearby and distant hazards failed.\n\n');
end

%% =========================================================================
%% Test 3: Grid Boundary Positions Handling
%% =========================================================================
fprintf('[TEST 3] Testing Sensor Operation at Grid Boundaries...\n');

% Boundary Case A: Top-left corner [1, 1]
totalTests = totalTests + 1;
try
    [detBound1, dataBound1] = obstacle_detection(map, [1, 1], 3.0);
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.1 - Top-Left Boundary [1, 1]: Executed safely without out-of-bounds errors (Detected: %d).\n', ...
        dataBound1.totalDetected);
catch ME
    fprintf('  [FAIL] Test 3.1 - Boundary error at [1, 1]: %s\n', ME.message);
end

% Boundary Case B: Bottom-right corner [20, 20]
totalTests = totalTests + 1;
try
    [detBound2, dataBound2] = obstacle_detection(map, [20, 20], 3.0);
    passedTests = passedTests + 1;
    fprintf('  [PASS] Test 3.2 - Bottom-Right Boundary [20, 20]: Executed safely (Detected: %d).\n', ...
        dataBound2.totalDetected);
catch ME
    fprintf('  [FAIL] Test 3.2 - Boundary error at [20, 20]: %s\n', ME.message);
end

% Boundary Case C: Northern edge [1, 14] adjacent to Rock Ridge
totalTests = totalTests + 1;
try
    [detRidge, dataRidge] = obstacle_detection(map, [1, 14], 3.0);
    if dataRidge.totalDetected > 0
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 3.3 - North Ridge Boundary [1, 14]: Successfully sensed %d ridge obstacles across boundary.\n\n', ...
            dataRidge.totalDetected);
    else
        fprintf('  [FAIL] Test 3.3 - Expected ridge detection at [1, 14].\n\n');
    end
catch ME
    fprintf('  [FAIL] Test 3.3 - Boundary error at [1, 14]: %s\n\n', ME.message);
end

%% =========================================================================
%% Test 4: Sensor Detection Area Visualization
%% =========================================================================
fprintf('[TEST 4] Testing Sensor Area Visualization...\n');
totalTests = totalTests + 1;

try
    fig = obstacle_detection(map, roverPos1, radius1, true);
    if isgraphics(fig, 'figure')
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 4.1 - Visualization Render: Generated figure with range circles and hazard markers.\n');
    else
        fprintf('  [FAIL] Test 4.1 - Figure handle invalid.\n');
    end
catch ME
    fprintf('  [FAIL] Test 4.1 - Visualization error: %s\n', ME.message);
end

% Check saved file
totalTests = totalTests + 1;
currentDir = fileparts(mfilename('fullpath'));
imagePath = fullfile(currentDir, 'results', 'sensor_detection.png');

if exist(imagePath, 'file') == 2
    fInfo = dir(imagePath);
    if fInfo.bytes > 5000
        passedTests = passedTests + 1;
        fprintf('  [PASS] Test 4.2 - Output Image: results/sensor_detection.png verified (%d bytes).\n', fInfo.bytes);
    else
        fprintf('  [FAIL] Test 4.2 - Image file size too small.\n');
    end
else
    fprintf('  [FAIL] Test 4.2 - results/sensor_detection.png not found.\n');
end

%% =========================================================================
%% Summary
%% =========================================================================
fprintf('\n------------------------------------------------------------\n');
fprintf('Sensor Test Suite Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: PROXIMITY SENSOR MODULE VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');
