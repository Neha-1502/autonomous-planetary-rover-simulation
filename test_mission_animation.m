% =========================================================================
% Script: test_mission_animation.m
% Description: End-to-end test: mission_controller (with a dynamic rockfall
%              and replanning) -> rover_visualization animation.
%
% Tests:
%   1. Mission succeeds and replans at least once.
%   2. Animation uses exactly one figure.
%   3. Rover marker ends on the goal; travelled line equals pathHistory.
%   4. Dynamic hazard appears on the map and is never traversed.
%   5. Planned route is dashed; detected obstacles are shown.
%   6. Output images exist in results/.
% =========================================================================
clear; clc; close all;

fprintf('============================================================\n');
fprintf('  MISSION ANIMATION: END-TO-END VERIFICATION\n');
fprintf('============================================================\n\n');

totalTests = 0; passedTests = 0;

[map, start, goal] = environment();
dynamicEvents = [4, 2, 8];
[~, info] = mission_controller(map, start, goal, dynamicEvents, 2.5, false);
[plan, cost] = astar_path_planning(map, start, goal);

%% Test 1
totalTests = totalTests + 1;
if info.success && info.replanCount >= 1
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 1 - Mission: reached goal in %d steps, %d replan(s).\n', ...
        info.totalStepsTaken, info.replanCount);
else
    fprintf('[FAIL] Test 1 - Mission did not succeed with replanning.\n');
end

%% Animate
opts.travelled     = info.pathHistory;
opts.dynamicEvents = dynamicEvents;
opts.sensorRadius  = 2.5;
fig = rover_visualization(map, start, goal, plan, cost, true, 0.01, opts);

%% Test 2
totalTests = totalTests + 1;
nFigs = numel(findobj('Type', 'figure'));
if nFigs == 1
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 2 - Single figure used for all %d frames.\n', size(info.pathHistory, 1));
else
    fprintf('[FAIL] Test 2 - %d figures exist (expected 1).\n', nFigs);
end

%% Test 3
totalTests = totalTests + 1;
hRover = findobj(fig, 'DisplayName', 'Rover (Blue Circle)');
hTrav  = findobj(fig, 'DisplayName', 'Travelled Path (Green)');
tx = get(hTrav, 'XData'); ty = get(hTrav, 'YData');
if get(hRover, 'XData') == goal(2) && get(hRover, 'YData') == goal(1) && ...
   isequal([ty(:), tx(:)], info.pathHistory)
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 3 - Rover on goal; green path matches %d-cell mission history.\n', numel(tx));
else
    fprintf('[FAIL] Test 3 - Rover/travelled path mismatch.\n');
end

%% Test 4
totalTests = totalTests + 1;
hMapImg = findobj(fig, 'Type', 'image');
shown = get(hMapImg, 'CData');
hitHazard = any(info.pathHistory(:,1) == 2 & info.pathHistory(:,2) == 8);
if shown(2, 8) == 1 && ~hitHazard
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 4 - Rockfall [2, 8] drawn on map and never traversed.\n');
else
    fprintf('[FAIL] Test 4 - Dynamic hazard not displayed or was traversed.\n');
end

%% Test 5
totalTests = totalTests + 1;
hPlan = findobj(fig, 'DisplayName', 'Planned Route (Blue Dashed)');
hDet  = findobj(fig, 'DisplayName', 'Detected Obstacles');
if strcmp(get(hPlan, 'LineStyle'), '--') && ~isempty(get(hDet, 'XData')) && ~all(isnan(get(hDet, 'XData')))
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 5 - Dashed planned route and detected-obstacle markers present.\n');
else
    fprintf('[FAIL] Test 5 - Planned route style or detected markers missing.\n');
end

%% Test 6
totalTests = totalTests + 1;
d = fileparts(mfilename('fullpath'));
f = fullfile(d, 'results', 'mission_animation_final.png');
fi = dir(f);
if exist(f, 'file') == 2 && ~isempty(fi) && fi(1).bytes > 5000
    passedTests = passedTests + 1;
    fprintf('[PASS] Test 6 - results/mission_animation_final.png saved.\n');
else
    fprintf('[FAIL] Test 6 - Output image missing or too small.\n');
end

fprintf('\n------------------------------------------------------------\n');
fprintf('Mission Animation Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: MISSION ANIMATION VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n');