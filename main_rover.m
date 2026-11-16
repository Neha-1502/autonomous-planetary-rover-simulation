function varargout = main_rover(varargin)
% MAIN_ROVER Integrated autonomous planetary rover simulation (two-panel UI).
%
% Run with:   main_rover
%
% LEFT PANEL  : Mars terrain, obstacles, rover, start, goal, planned route
%               (blue dashed), travelled route (green), sensor detections
%               (red triangles) and dynamic hazards (black X).
% RIGHT PANEL : rover coordinates, movements, battery %, distance travelled,
%               obstacles detected, path recalculations, mission completion
%               %, mission status, last event, and the Start / Reset buttons.
%
% Modules used:
%   environment.m          terrain, start, goal
%   astar_path_planning.m  route preview (and replanning inside the controller)
%   mission_controller.m   mission loop (rover_movement + obstacle_detection
%                          + astar_path_planning), streams live state here
%   calculate_metrics.m    all telemetry numbers and the battery model
%
% At the end of every mission the program saves (in results/ by default):
%   final_map.png          left panel (terrain + routes)
%   mission_dashboard.png  the whole window
%   mission_metrics.csv    mission metrics (metric,value,unit)
%
% Optional name/value arguments:
%   'AutoStart'     false  start the mission immediately and block until done
%   'PauseTime'     0.08   seconds between animation frames (0 = fastest)
%   'SensorRadius'  2.5    proximity sensor radius in grid units
%   'DynamicEvents' [4 2 8] Kx3 [step,row,col] hazards appearing mid-mission
%   'Battery'       []     struct(capacity, energyPerMove); see calculate_metrics
%   'OutputDir'     ''     output folder (default: results/ next to this file)
%   'Visible'       'on'   'off' hides the window (for automated tests)
%   'Verbose'       false  print controller telemetry to the Command Window
%
% Optional output: results struct (figure, missionInfo, metrics, files).
% Meaningful mainly with 'AutoStart', true, because buttons run later.
%
% Base MATLAB only (figure, uipanel, uicontrol, axes, plot, set, drawnow).

    opts = parseOptions(varargin);

    % ---------------------------------------------------------------------
    % State shared with the nested callback functions (declared here so
    % nested functions share them instead of creating private copies)
    % ---------------------------------------------------------------------
    [map0, start, goal] = environment();
    running = false;  abortFlag = false;  resetPending = false;
    missionInfo = [];  metrics = [];
    savedFiles = struct('map', '', 'dashboard', '', 'csv', '');
    hMap = [];  hPlanned = [];  hTravelled = [];  hDetected = [];
    hDynamic = [];  hStart = [];  hGoal = [];  hRover = [];  hTitle = [];

    % ---------------------------------------------------------------------
    % Window and the two panels
    % ---------------------------------------------------------------------
    fig = figure('Name', 'Autonomous Planetary Rover Simulation', ...
                 'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
                 'Color', [0.94, 0.94, 0.94], 'Position', [80, 60, 1180, 680], ...
                 'Visible', opts.Visible, 'CloseRequestFcn', @onClose);

    leftPanel  = uipanel('Parent', fig, 'Title', 'Mars Terrain', ...
                         'FontSize', 11, 'FontWeight', 'bold', ...
                         'Units', 'normalized', 'Position', [0.01, 0.02, 0.62, 0.96]);
    rightPanel = uipanel('Parent', fig, 'Title', 'Mission Telemetry', ...
                         'FontSize', 11, 'FontWeight', 'bold', ...
                         'Units', 'normalized', 'Position', [0.64, 0.02, 0.35, 0.96]);
    panelBg = get(rightPanel, 'BackgroundColor');

    ax = axes('Parent', leftPanel, 'Units', 'normalized', 'Position', [0.08, 0.08, 0.66, 0.86]);

    % ---- Right panel: telemetry rows -------------------------------------
    rowLabels = {'Rover coordinates', 'Movements', 'Battery', 'Distance travelled', ...
                 'Obstacles detected', 'Path recalculations', 'Mission completion'};
    rowTags   = {'val_coords', 'val_moves', 'val_battery', 'val_distance', ...
                 'val_obstacles', 'val_replans', 'val_completion'};
    valueText = gobjects(numel(rowLabels), 1);
    for k = 1:numel(rowLabels)
        y = 0.90 - 0.072 * (k - 1);
        uicontrol('Parent', rightPanel, 'Style', 'text', 'Units', 'normalized', ...
                  'Position', [0.05, y, 0.50, 0.05], 'String', rowLabels{k}, ...
                  'HorizontalAlignment', 'left', 'FontSize', 10, 'BackgroundColor', panelBg);
        valueText(k) = uicontrol('Parent', rightPanel, 'Style', 'text', 'Units', 'normalized', ...
                  'Position', [0.55, y, 0.42, 0.05], 'String', '-', 'Tag', rowTags{k}, ...
                  'HorizontalAlignment', 'right', 'FontSize', 11, 'FontWeight', 'bold', ...
                  'BackgroundColor', panelBg);
    end

    uicontrol('Parent', rightPanel, 'Style', 'text', 'Units', 'normalized', ...
              'Position', [0.05, 0.39, 0.90, 0.04], 'String', 'Mission status', ...
              'HorizontalAlignment', 'left', 'FontSize', 10, 'BackgroundColor', panelBg);
    statusText = uicontrol('Parent', rightPanel, 'Style', 'text', 'Units', 'normalized', ...
              'Position', [0.05, 0.25, 0.90, 0.13], 'Tag', 'txt_status', ...
              'String', '', 'HorizontalAlignment', 'left', 'FontSize', 11, ...
              'FontWeight', 'bold', 'BackgroundColor', panelBg);
    eventText = uicontrol('Parent', rightPanel, 'Style', 'text', 'Units', 'normalized', ...
              'Position', [0.05, 0.15, 0.90, 0.09], 'Tag', 'txt_event', ...
              'String', '', 'HorizontalAlignment', 'left', 'FontSize', 8, ...
              'ForegroundColor', [0.35, 0.35, 0.35], 'BackgroundColor', panelBg);

    startBtn = uicontrol('Parent', rightPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
              'Position', [0.05, 0.03, 0.42, 0.09], 'String', 'Start Mission', ...
              'FontSize', 11, 'FontWeight', 'bold', 'Tag', 'btn_start', ...
              'Callback', @startMission);
    uicontrol('Parent', rightPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
              'Position', [0.53, 0.03, 0.42, 0.09], 'String', 'Reset', ...
              'FontSize', 11, 'FontWeight', 'bold', 'Tag', 'btn_reset', ...
              'Callback', @resetSimulation);

    % ---------------------------------------------------------------------
    % Initial (idle) state, then optional auto-start
    % ---------------------------------------------------------------------
    doReset();
    if opts.AutoStart
        startMission();
    end

    if nargout > 0
        varargout{1} = struct('figure', fig, 'missionInfo', missionInfo, ...
                              'metrics', metrics, 'files', savedFiles);
    end

    % =====================================================================
    % Nested functions (callbacks)
    % =====================================================================

    % ---- Requirement 1: start mission ------------------------------------
    function startMission(~, ~)
        if running || ~ishandle(fig)
            return;
        end
        running = true;  abortFlag = false;  resetPending = false;
        set(startBtn, 'Enable', 'off');

        [map0, start, goal] = environment();     % fresh terrain every run
        drawScene();
        missionInfo = [];  metrics = [];
        set(statusText, 'String', 'RUNNING ...', 'ForegroundColor', [0.10, 0.30, 0.75]);
        set(eventText, 'String', 'Mission started.');
        drawnow;

        try
            [~, missionInfo] = mission_controller(map0, start, goal, ...
                opts.DynamicEvents, opts.SensorRadius, opts.Verbose, ...
                opts.Battery, @onStep);
        catch ME
            running = false;
            if ishandle(fig)
                set(statusText, 'String', ['ERROR: ' ME.message], 'ForegroundColor', [0.75, 0.10, 0.10]);
                set(startBtn, 'Enable', 'on');
            end
            rethrow(ME);
        end
        running = false;

        if ~ishandle(fig)
            return;                               % window closed during the run
        end
        if resetPending
            doReset();                            % Reset was pressed during the run
            return;
        end
        finishMission();
    end

    % ---- Requirement 2: reset simulation ---------------------------------
    function resetSimulation(~, ~)
        if running
            abortFlag = true;                     % mission stops at next step,
            resetPending = true;                  % then startMission() resets
            set(eventText, 'String', 'Reset requested - stopping mission ...');
            return;
        end
        doReset();
    end

    function doReset()
        [map0, start, goal] = environment();
        missionInfo = [];  metrics = [];
        drawScene();
        idle = struct('pathHistory', start, 'success', false, 'replanCount', 0, ...
                      'detectedObstacles', [], 'startPos', start, 'goalPos', goal, ...
                      'finalPos', start, 'finalMap', map0);
        showTelemetry(start, calculate_metrics(idle, [], opts.Battery));
        set(statusText, 'String', 'READY - press "Start Mission"', 'ForegroundColor', [0.20, 0.20, 0.20]);
        set(eventText, 'String', '');
        set(startBtn, 'Enable', 'on');
        drawnow;
    end

    % ---- Requirement 3: live update from the mission controller ----------
    function keepGoing = onStep(snap)
        keepGoing = ishandle(fig) && ~abortFlag;
        if ~keepGoing
            return;
        end

        % Left panel
        set(hMap, 'CData', snap.trueMap);
        [dr, dc] = ind2sub(size(map0), find(snap.trueMap ~= map0));
        set(hDynamic, 'XData', dc, 'YData', dr);
        if ~isempty(snap.detectedObstacles)
            set(hDetected, 'XData', snap.detectedObstacles(:, 2), ...
                           'YData', snap.detectedObstacles(:, 1));
        end
        set(hTravelled, 'XData', snap.history(:, 2), 'YData', snap.history(:, 1));
        if ~isempty(snap.currentPath)
            set(hPlanned, 'XData', snap.currentPath(:, 2), 'YData', snap.currentPath(:, 1));
        end
        set(hRover, 'XData', snap.position(2), 'YData', snap.position(1));
        set(hTitle, 'String', sprintf('Step %d | Rover [%d, %d] | Destination [%d, %d]', ...
                  snap.stepsTaken, snap.position(1), snap.position(2), goal(1), goal(2)));

        % Right panel: metrics computed by calculate_metrics from live state
        live = struct('pathHistory', snap.history, ...
                      'success', isequal(snap.position, goal), ...
                      'replanCount', snap.replanCount, ...
                      'detectedObstacles', snap.detectedObstacles, ...
                      'newObstacleCount', snap.newObstacleCount, ...
                      'startPos', start, 'goalPos', goal, ...
                      'finalPos', snap.position, 'finalMap', snap.trueMap);
        showTelemetry(snap.position, calculate_metrics(live, [], snap.batteryParams));
        set(eventText, 'String', snap.message);

        drawnow;
        if opts.PauseTime > 0
            pause(opts.PauseTime);
        end
    end

    % ---- Requirement 4: completion / failure display ---------------------
    function finishMission()
        metrics = calculate_metrics(missionInfo, [], missionInfo.batteryParams);
        showTelemetry(missionInfo.finalPos, metrics);

        detail = regexprep(missionInfo.status, '^(SUCCESS|FAILED|STOPPED):\s*', '');
        if missionInfo.success
            msg = ['MISSION COMPLETE: ' detail];
            col = [0.05, 0.50, 0.15];
        elseif strncmp(missionInfo.status, 'STOPPED', 7)
            msg = ['MISSION STOPPED: ' detail];
            col = [0.75, 0.10, 0.10];
        else
            msg = ['MISSION FAILED: ' detail];
            col = [0.75, 0.10, 0.10];
        end
        set(statusText, 'String', msg, 'ForegroundColor', col);
        set(hTitle, 'String', msg, 'Color', col);

        saveOutputs();                            % requirements 5 and 6
        set(startBtn, 'Enable', 'on');
        drawnow;
    end

    % ---- Requirements 5 and 6: save map image and metrics CSV ------------
    function saveOutputs()
        outDir = opts.OutputDir;
        if ~exist(outDir, 'dir')
            mkdir(outDir);
        end
        mapFile  = fullfile(outDir, 'final_map.png');
        dashFile = fullfile(outDir, 'mission_dashboard.png');
        csvFile  = fullfile(outDir, 'mission_metrics.csv');

        drawnow;
        % saveas/print do not support figures containing UI components, so
        % images are captured from the screen (getframe + imwrite).
        okMap  = captureToPng(fig, getpixelposition(leftPanel), mapFile);   % left panel only
        okDash = captureToPng(fig, [], dashFile);                           % whole window

        if writeMetricsCsv(csvFile, metrics, missionInfo)
            savedFiles.csv = csvFile;
        end
        if okMap,  savedFiles.map = mapFile;       end
        if okDash, savedFiles.dashboard = dashFile; end

        set(eventText, 'String', sprintf('Saved: %s, %s, %s', ...
            'final_map.png', 'mission_dashboard.png', 'mission_metrics.csv'));
        if okMap,  fprintf('Saved final map      : %s\n', mapFile);  end
        if okDash, fprintf('Saved dashboard      : %s\n', dashFile); end
        fprintf('Saved metrics (CSV)  : %s\n', csvFile);
    end

    % ---- Left panel scene (terrain, markers, route preview) --------------
    function drawScene()
        delete(findobj(leftPanel, 'Type', 'legend'));
        cla(ax);
        [rows, cols] = size(map0);

        hMap = imagesc(ax, map0, [0, 1]);
        hold(ax, 'on');
        colormap(ax, [0.82, 0.70, 0.55; 0.25, 0.25, 0.25]);   % free / obstacle
        for gx = 0.5:1:(cols + 0.5)
            line('Parent', ax, 'XData', [gx, gx], 'YData', [0.5, rows + 0.5], ...
                 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
        end
        for gy = 0.5:1:(rows + 0.5)
            line('Parent', ax, 'XData', [0.5, cols + 0.5], 'YData', [gy, gy], ...
                 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
        end

        % Route preview from A* on the initial terrain
        [p0, ~] = astar_path_planning(map0, start, goal);
        if isempty(p0), p0 = [NaN, NaN]; end

        hPlanned = plot(ax, p0(:, 2), p0(:, 1), '--', 'Color', [0.12, 0.25, 0.69], ...
                        'LineWidth', 2.0, 'DisplayName', 'Planned Route');
        hTravelled = plot(ax, start(2), start(1), '-', 'Color', [0.08, 0.64, 0.29], ...
                        'LineWidth', 2.8, 'DisplayName', 'Travelled Route');
        hDetected = plot(ax, NaN, NaN, '^', 'MarkerSize', 9, ...
                        'MarkerFaceColor', [0.94, 0.27, 0.27], 'MarkerEdgeColor', 'k', ...
                        'DisplayName', 'Detected Obstacles');
        hDynamic = plot(ax, NaN, NaN, 'x', 'MarkerSize', 14, 'Color', 'k', ...
                        'LineWidth', 2.5, 'DisplayName', 'Dynamic Hazard');
        hStart = plot(ax, start(2), start(1), 's', 'MarkerSize', 13, ...
                        'MarkerFaceColor', [0.13, 0.77, 0.37], 'MarkerEdgeColor', 'k', ...
                        'LineWidth', 1.5, 'DisplayName', 'Start');
        hGoal = plot(ax, goal(2), goal(1), 'p', 'MarkerSize', 16, ...
                        'MarkerFaceColor', [0.94, 0.27, 0.27], 'MarkerEdgeColor', 'k', ...
                        'LineWidth', 1.5, 'DisplayName', 'Destination');
        hRover = plot(ax, start(2), start(1), 'o', 'MarkerSize', 12, ...
                        'MarkerFaceColor', [0.15, 0.45, 0.95], 'MarkerEdgeColor', 'k', ...
                        'LineWidth', 1.5, 'DisplayName', 'Rover');

        axis(ax, 'equal');
        set(ax, 'XTick', 1:cols, 'YTick', 1:rows, 'YDir', 'reverse', 'FontSize', 7);
        xlim(ax, [0.5, cols + 0.5]);
        ylim(ax, [0.5, rows + 0.5]);
        xlabel(ax, 'X Coordinate (Grid Columns)', 'FontSize', 9);
        ylabel(ax, 'Y Coordinate (Grid Rows)', 'FontSize', 9);
        hTitle = title(ax, sprintf('Step 0 | Rover [%d, %d] | Destination [%d, %d]', ...
                                   start(1), start(2), goal(1), goal(2)), 'FontSize', 10);
        legend(ax, [hRover, hStart, hGoal, hPlanned, hTravelled, hDetected, hDynamic], ...
               'Location', 'eastoutside', 'FontSize', 8);
    end

    % ---- Right panel values ---------------------------------------------
    function showTelemetry(pos, m)
        set(valueText(1), 'String', sprintf('[%d, %d]', pos(1), pos(2)));
        set(valueText(2), 'String', sprintf('%d', m.movements));
        set(valueText(3), 'String', sprintf('%.1f %%', m.batteryRemainingPercent), ...
                          'ForegroundColor', batteryColor(m.batteryRemainingPercent));
        set(valueText(4), 'String', sprintf('%.1f cells', m.totalDistance));
        set(valueText(5), 'String', sprintf('%d', m.obstaclesDetected));
        set(valueText(6), 'String', sprintf('%d', m.pathRecalculations));
        if isnan(m.completionPercent)
            set(valueText(7), 'String', 'n/a');
        else
            set(valueText(7), 'String', sprintf('%.1f %%', m.completionPercent));
        end
    end

    function onClose(~, ~)
        abortFlag = true;                         % lets a running mission stop cleanly
        if ishghandle(fig)
            delete(fig);
        end
    end
end

% =========================================================================
% Local helper functions
% =========================================================================

function opts = parseOptions(args)
    opts.AutoStart     = false;
    opts.PauseTime     = 0.08;
    opts.SensorRadius  = 2.5;
    opts.DynamicEvents = [4, 2, 8];   % rockfall at [2, 8] after 4 moves
    opts.Battery       = [];
    opts.OutputDir     = fullfile(fileparts(mfilename('fullpath')), 'results');
    opts.Visible       = 'on';
    opts.Verbose       = false;

    if mod(numel(args), 2) ~= 0
        error('main_rover:badArgs', 'Options must be name/value pairs.');
    end
    names = fieldnames(opts);
    for i = 1:2:numel(args)
        idx = find(strcmpi(names, args{i}), 1);
        if isempty(idx)
            error('main_rover:badArgs', 'Unknown option "%s".', num2str(args{i}));
        end
        opts.(names{idx}) = args{i + 1};
    end
    if isempty(opts.OutputDir)
        opts.OutputDir = fullfile(fileparts(mfilename('fullpath')), 'results');
    end
end

function ok = captureToPng(fig, rect, filePath)
% Save a screen capture of the figure (or of a pixel rectangle) as PNG.
    ok = false;
    try
        drawnow;
        if isempty(rect)
            frame = getframe(fig);
        else
            frame = getframe(fig, rect);
        end
        imwrite(frame.cdata, filePath);
        ok = true;
        return;
    catch
    end
    try                                   % R2020b+: whole window with UI components
        exportapp(fig, filePath);
        ok = true;
    catch ME
        warning('main_rover:image', 'Could not save %s (%s).', filePath, ME.message);
    end
end

function col = batteryColor(pct)
    if pct > 50
        col = [0.05, 0.50, 0.15];     % green
    elseif pct > 20
        col = [0.85, 0.50, 0.00];     % orange
    else
        col = [0.75, 0.10, 0.10];     % red
    end
end

function ok = writeMetricsCsv(filePath, m, info)
% Write mission metrics as CSV rows: metric,value,unit
    ok = false;
    fid = fopen(filePath, 'w');
    if fid < 0
        warning('main_rover:csv', 'Could not open %s for writing.', filePath);
        return;
    end
    closer = onCleanup(@() fclose(fid));   %#ok<NASGU>

    num = @(x) sprintf('%.6g', x);
    txt = @(t) ['"' strrep(t, '"', '""') '"'];
    row = @(name, value, unit) fprintf(fid, '%s,%s,%s\n', name, value, unit);

    fprintf(fid, 'metric,value,unit\n');
    row('mission_status',            txt(info.status),              '');
    row('mission_success',           num(double(info.success)),     'boolean');
    row('start_row',                 num(info.startPos(1)),         'grid');
    row('start_col',                 num(info.startPos(2)),         'grid');
    row('goal_row',                  num(info.goalPos(1)),          'grid');
    row('goal_col',                  num(info.goalPos(2)),          'grid');
    row('final_row',                 num(info.finalPos(1)),         'grid');
    row('final_col',                 num(info.finalPos(2)),         'grid');
    row('movements',                 num(m.movements),              'moves');
    row('total_distance',            num(m.totalDistance),          'cells');
    row('obstacles_detected',        num(m.obstaclesDetected),      'cells');
    row('new_obstacles_discovered',  num(m.newObstaclesDiscovered), 'cells');
    row('path_recalculations',       num(m.pathRecalculations),     'count');
    row('battery_capacity',          num(m.batteryCapacity),        'energy units');
    row('energy_per_move',           num(m.energyPerMove),          'energy units');
    row('battery_consumed',          num(m.batteryConsumed),        'energy units');
    row('battery_remaining',         num(m.batteryRemaining),       'energy units');
    row('battery_remaining_percent', num(m.batteryRemainingPercent), 'percent');
    row('mission_completion_percent', num(m.completionPercent),     'percent');
    row('completion_method',         txt(m.completionMethod),       '');
    row('execution_time',            num(m.executionTimeSec),       'seconds (computation)');
    ok = true;
end