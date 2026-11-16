% =========================================================================
% Script: test_main_rover.m
% Description: End-to-end integration test of main_rover.m (two-panel UI),
%              exercising environment, A*, rover_movement, obstacle_detection,
%              mission_controller, calculate_metrics, Start/Reset, image and
%              CSV output.
%
% Note: the window is shown ('Visible','on') so that the saved map image is
% a real screen capture. For a headless run change the Visible value below
% to 'off' (image capture may then be blank on some MATLAB versions).
% =========================================================================
clear; clc; close all;

fprintf('============================================================\n');
fprintf('  MAIN ROVER SIMULATION: INTEGRATION TEST\n');
fprintf('============================================================\n\n');

totalTests = 0; passedTests = 0;
vis = 'on';
outDir = fullfile(tempdir, 'rover_main_test_output');
if exist(outDir, 'dir'), rmdir(outDir, 's'); end

closeWin = @(h) delete(h(ishghandle(h)));   % delete directly (avoids close() on custom CloseRequestFcn)
getStr  = @(fig, tag) get(findobj(fig, 'Tag', tag), 'String');
csvVal  = @(txt, name) str2double(regexp(txt, [name ',([^,\n]*),'], 'tokens', 'once'));

%% T1: full mission through the UI (rockfall at [2,8] -> replanning)
totalTests = totalTests + 1;
res = main_rover('AutoStart', true, 'PauseTime', 0, 'Visible', vis, 'OutputDir', outDir);
fig = res.figure;
info = res.missionInfo; m = res.metrics;
nFig = numel(findobj('Type', 'figure'));
nPanels = numel(findobj(fig, 'Type', 'uipanel'));
if info.success && m.completionPercent == 100 && m.pathRecalculations >= 1 && ...
   nFig == 1 && nPanels == 2
    passedTests = passedTests + 1;
    fprintf('[PASS] T1 - Mission via UI: success, %d moves, %d replan(s); 1 window, 2 panels.\n', ...
        m.movements, m.pathRecalculations);
else
    fprintf('[FAIL] T1 - Mission/UI structure (success=%d, figs=%d, panels=%d).\n', ...
        info.success, nFig, nPanels);
end

%% T2: right-panel values equal the computed metrics
totalTests = totalTests + 1;
okCoords = strcmp(getStr(fig, 'val_coords'), '[19, 19]');
okBat    = strcmp(getStr(fig, 'val_battery'), sprintf('%.1f %%', m.batteryRemainingPercent));
okDist   = strcmp(getStr(fig, 'val_distance'), sprintf('%.1f cells', m.totalDistance));
okObs    = strcmp(getStr(fig, 'val_obstacles'), sprintf('%d', m.obstaclesDetected));
okRep    = strcmp(getStr(fig, 'val_replans'), sprintf('%d', m.pathRecalculations));
okComp   = strcmp(getStr(fig, 'val_completion'), '100.0 %');
st = getStr(fig, 'txt_status');
okStat   = strncmp(st, 'MISSION COMPLETE', 16);
if okCoords && okBat && okDist && okObs && okRep && okComp && okStat
    passedTests = passedTests + 1;
    fprintf('[PASS] T2 - Telemetry panel matches metrics (battery %s, status "%s").\n', ...
        getStr(fig, 'val_battery'), st);
else
    fprintf('[FAIL] T2 - Telemetry mismatch [coords %d bat %d dist %d obs %d rep %d comp %d status %d].\n', ...
        okCoords, okBat, okDist, okObs, okRep, okComp, okStat);
end

%% T3: saved map image and metrics CSV
totalTests = totalTests + 1;
okFiles = exist(res.files.map, 'file') == 2 && exist(res.files.dashboard, 'file') == 2 && ...
          exist(res.files.csv, 'file') == 2;
okSize = false; okCsv = false;
if okFiles
    fMap = dir(res.files.map); fDash = dir(res.files.dashboard);
    okSize = fMap.bytes > 1000 && fDash.bytes > 1000;
    txt = fileread(res.files.csv);
    okCsv = strncmp(txt, 'metric,value,unit', 17) && ...
            csvVal(txt, 'movements') == m.movements && ...
            csvVal(txt, 'mission_success') == 1 && ...
            csvVal(txt, 'path_recalculations') == m.pathRecalculations && ...
            abs(csvVal(txt, 'battery_remaining_percent') - m.batteryRemainingPercent) < 1e-4 && ...
            abs(csvVal(txt, 'mission_completion_percent') - 100) < 1e-9;
end
if okFiles && okSize && okCsv
    passedTests = passedTests + 1;
    fprintf('[PASS] T3 - final_map.png, mission_dashboard.png and mission_metrics.csv saved; CSV values match.\n');
else
    fprintf('[FAIL] T3 - Output files (exist %d, size %d, csv %d).\n', okFiles, okSize, okCsv);
end

%% T4: Reset button returns the simulation to its idle state
totalTests = totalTests + 1;
btn = findobj(fig, 'Tag', 'btn_reset');
cb = get(btn, 'Callback'); cb(btn, []);
hTrav = findobj(fig, 'DisplayName', 'Travelled Route');
if strcmp(getStr(fig, 'val_coords'), '[2, 2]') && strcmp(getStr(fig, 'val_battery'), '100.0 %') && ...
   strcmp(getStr(fig, 'val_completion'), '0.0 %') && strcmp(getStr(fig, 'val_replans'), '0') && ...
   ~isempty(strfind(getStr(fig, 'txt_status'), 'READY')) && numel(get(hTrav, 'XData')) == 1
    passedTests = passedTests + 1;
    fprintf('[PASS] T4 - Reset: rover [2, 2], battery 100%%, completion 0%%, travelled path cleared.\n');
else
    fprintf('[FAIL] T4 - Reset did not restore the idle state.\n');
end

%% T5: Start button reruns the mission after a reset (deterministic)
totalTests = totalTests + 1;
set(findobj(fig, 'Tag', 'btn_start'), 'Enable', 'on');
sb = findobj(fig, 'Tag', 'btn_start');
cb = get(sb, 'Callback'); cb(sb, []);
if strcmp(getStr(fig, 'val_coords'), '[19, 19]') && ...
   strcmp(getStr(fig, 'val_moves'), sprintf('%d', m.movements)) && ...
   strncmp(getStr(fig, 'txt_status'), 'MISSION COMPLETE', 16) && numel(findobj('Type', 'figure')) == 1
    passedTests = passedTests + 1;
    fprintf('[PASS] T5 - Restart after Reset reproduces the mission (%d moves), still one window.\n', m.movements);
else
    fprintf('[FAIL] T5 - Restart after reset failed.\n');
end
closeWin(fig);

%% T6: battery failure shown as failure, saved in CSV
totalTests = totalTests + 1;
res2 = main_rover('AutoStart', true, 'PauseTime', 0, 'Visible', vis, 'OutputDir', outDir, ...
                  'Battery', struct('capacity', 10, 'energyPerMove', 1));
f2 = res2.figure;
txt = fileread(res2.files.csv);
if ~res2.missionInfo.success && strncmp(getStr(f2, 'txt_status'), 'MISSION STOPPED', 15) && ...
   strcmp(getStr(f2, 'val_battery'), '0.0 %') && csvVal(txt, 'mission_success') == 0 && ...
   csvVal(txt, 'battery_remaining') == 0 && csvVal(txt, 'movements') == 10
    passedTests = passedTests + 1;
    fprintf('[PASS] T6 - Battery failure: "%s", CSV records 10 moves, battery 0.\n', getStr(f2, 'txt_status'));
else
    fprintf('[FAIL] T6 - Battery-depleted mission not reported/saved correctly.\n');
end
closeWin(f2);

%% T7: static mission with no dynamic events -> no replans
totalTests = totalTests + 1;
res3 = main_rover('AutoStart', true, 'PauseTime', 0, 'Visible', vis, 'OutputDir', outDir, ...
                  'DynamicEvents', []);
if res3.missionInfo.success && res3.metrics.pathRecalculations == 0 && ...
   res3.metrics.newObstaclesDiscovered == 0
    passedTests = passedTests + 1;
    fprintf('[PASS] T7 - No hazards: success with 0 replans, 0 new obstacles.\n');
else
    fprintf('[FAIL] T7 - Static mission metrics wrong.\n');
end
closeWin(res3.figure);

%% T8: controller step callback can abort a mission (used by Reset / close)
totalTests = totalTests + 1;
[map, start, goal] = environment();
[pos8, info8] = mission_controller(map, start, goal, [], 2.5, false, [], @(s) s.stepsTaken < 5);
if ~info8.success && info8.totalStepsTaken == 5 && ...
   strcmp(info8.status, 'STOPPED: Mission aborted by operator.') && ~isequal(pos8, goal)
    passedTests = passedTests + 1;
    fprintf('[PASS] T8 - Callback abort: mission stopped after 5 moves with operator-abort status.\n');
else
    fprintf('[FAIL] T8 - Callback abort not honoured (status: %s).\n', info8.status);
end

%% T9: bad option names are rejected
totalTests = totalTests + 1;
gotId = '';
try
    main_rover('NotAnOption', 1);
catch ME
    gotId = ME.identifier;
end
if strcmp(gotId, 'main_rover:badArgs')
    passedTests = passedTests + 1;
    fprintf('[PASS] T9 - Unknown option rejected with main_rover:badArgs.\n');
else
    fprintf('[FAIL] T9 - Unknown option not rejected.\n');
end

%% Summary
fprintf('\n------------------------------------------------------------\n');
fprintf('Main Rover Integration Summary: %d / %d tests PASSED.\n', passedTests, totalTests);
if passedTests == totalTests
    fprintf('STATUS: MAIN ROVER SIMULATION VERIFIED SUCCESSFULLY.\n');
else
    fprintf('STATUS: SOME TESTS FAILED.\n');
end
fprintf('Output files from this test: %s\n', outDir);
fprintf('============================================================\n');