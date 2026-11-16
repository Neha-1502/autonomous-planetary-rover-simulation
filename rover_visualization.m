function figHandle = rover_visualization(map, start, goal, plannedPath, cost, animate, pauseTime, opts)
% ROVER_VISUALIZATION Real-time visualization and animation of the rover.
%
% Usage:
%   figHandle = rover_visualization()                          % Full animated demo
%   figHandle = rover_visualization(map, start, goal)          % Static map + A* route
%   figHandle = rover_visualization(map, start, goal, path)    % Static map + given route
%   figHandle = rover_visualization(map, start, goal, path, cost, true, 0.05)
%   figHandle = rover_visualization(map, start, goal, path, cost, true, 0.05, opts)
%
% Optional opts struct (for animating a real mission, e.g. with replanning):
%   opts.travelled     - Nx2 actual rover history (missionInfo.pathHistory).
%                        The rover follows this instead of plannedPath, and
%                        plannedPath is kept as the ORIGINAL planned route.
%   opts.dynamicEvents - Kx3 [stepNumber, row, col] hazards that appear
%                        mid-mission (same convention as mission_controller).
%   opts.sensorRadius  - Sensor radius for the detected-obstacle markers (2.5).
%
% Visual elements:
%   Terrain          : light brown regolith, dark grey craters/rocks
%   Rover            : blue filled circle
%   Start            : green filled square
%   Destination      : red filled star
%   Planned route    : blue dashed line
%   Travelled path   : green solid line
%   Detected hazards : red triangles (proximity sensor detections)
%   Dynamic hazards  : black X (appear at the step they materialize)
%
% Animation: one cell per frame, a single figure whose graphics handles are
% updated with set() followed by drawnow. No new figure is created per move.
% Base MATLAB only.

    % ---------------------------------------------------------------------
    % 1. Inputs and defaults
    % ---------------------------------------------------------------------
    if nargin < 1 || isempty(map) || nargin < 2 || isempty(start) || nargin < 3 || isempty(goal)
        [map, start, goal] = environment();
    end

    if nargin < 4 || isempty(plannedPath)
        [plannedPath, cost] = astar_path_planning(map, start, goal);
    elseif nargin < 5 || isempty(cost)
        cost = size(plannedPath, 1) - 1;
    end

    if nargin < 6 || isempty(animate)
        animate = (nargin == 0);      % animate by default only for the no-argument demo
    end

    if nargin < 7 || isempty(pauseTime)
        pauseTime = 0.06;             % seconds per frame
    end

    if nargin < 8 || isempty(opts)
        opts = struct();
    end
    travelled     = getOpt(opts, 'travelled', []);
    dynamicEvents = getOpt(opts, 'dynamicEvents', []);
    sensorRadius  = getOpt(opts, 'sensorRadius', 2.5);

    [gridRows, gridCols] = size(map);
    hasPath = ~isempty(plannedPath) && ~isinf(cost);

    % Sequence of cells the rover visits during the animation
    if ~isempty(travelled)
        seq = travelled;
    elseif hasPath
        seq = plannedPath;
    else
        seq = [];
    end
    keepFullPlan = ~isempty(travelled);   % real mission: show original plan unchanged

    % ---------------------------------------------------------------------
    % 2. Single figure and static scene
    % ---------------------------------------------------------------------
    figHandle = figure('Name', 'Planetary Rover Simulation - Animation', ...
                       'NumberTitle', 'off', 'Color', 'w', ...
                       'Position', [100, 100, 850, 720]);

    displayMap = map;                                   % terrain currently drawn
    hMap = imagesc(displayMap, [0, 1]);
    axis equal; axis tight; hold on;
    colormap([0.82, 0.70, 0.55; 0.25, 0.25, 0.25]);     % free / obstacle

    for x = 0.5:1:(gridCols + 0.5)
        line([x, x], [0.5, gridRows + 0.5], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
    end
    for y = 0.5:1:(gridRows + 0.5)
        line([0.5, gridCols + 0.5], [y, y], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
    end

    % ---------------------------------------------------------------------
    % 3. Graphics handles (updated later with set())
    % ---------------------------------------------------------------------
    if hasPath
        hPlannedRoute = plot(plannedPath(:, 2), plannedPath(:, 1), '--', ...
            'Color', [0.12, 0.25, 0.69], 'LineWidth', 2.0, ...
            'DisplayName', 'Planned Route (Blue Dashed)');
    else
        hPlannedRoute = plot(nan, nan, '--', 'Color', [0.12, 0.25, 0.69], ...
            'LineWidth', 2.0, 'DisplayName', 'Planned Route (Blue Dashed)');
    end

    hTravelledPath = plot(start(2), start(1), '-', ...
        'Color', [0.08, 0.64, 0.29], 'LineWidth', 2.8, ...
        'DisplayName', 'Travelled Path (Green)');

    hDetectedObs = plot(nan, nan, '^', ...
        'MarkerSize', 10, 'MarkerFaceColor', [0.94, 0.27, 0.27], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
        'DisplayName', 'Detected Obstacles');

    hDynObs = plot(nan, nan, 'x', ...
        'MarkerSize', 14, 'Color', 'k', 'LineWidth', 2.5, ...
        'DisplayName', 'Dynamic Hazard');

    hStart = plot(start(2), start(1), 's', ...
        'MarkerSize', 13, 'MarkerFaceColor', [0.13, 0.77, 0.37], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Start [%d, %d]', start(1), start(2)));

    hGoal = plot(goal(2), goal(1), 'p', ...
        'MarkerSize', 16, 'MarkerFaceColor', [0.94, 0.27, 0.27], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Destination [%d, %d]', goal(1), goal(2)));

    hRover = plot(start(2), start(1), 'o', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.15, 0.45, 0.95], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', 'Rover (Blue Circle)');

    set(gca, 'XTick', 1:gridCols, 'YTick', 1:gridRows);
    set(gca, 'XAxisLocation', 'bottom');
    set(gca, 'YDir', 'reverse');
    xlim([0.5, gridCols + 0.5]);
    ylim([0.5, gridRows + 0.5]);
    xlabel('X Coordinate (Grid Columns)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Y Coordinate (Grid Rows)', 'FontSize', 11, 'FontWeight', 'bold');

    hTitle = title(sprintf('Mars Rover Animation | Step: 0 | Rover Pos: [%d, %d] | Destination: [%d, %d]', ...
                           start(1), start(2), goal(1), goal(2)), ...
                   'FontSize', 12, 'FontWeight', 'bold');

    legend([hRover, hStart, hGoal, hPlannedRoute, hTravelledPath, hDetectedObs, hDynObs], ...
           'Location', 'northeastoutside', 'FontSize', 9.5);

    % ---------------------------------------------------------------------
    % 4. Frame loop: ONE CELL PER FRAME, same figure every time
    % ---------------------------------------------------------------------
    allDetected = [];
    dynMarkers  = zeros(0, 2);
    eventFired  = false(size(dynamicEvents, 1), 1);
    runLoop     = ~isempty(seq) && (animate || ~isempty(travelled));
    totalMoves  = size(seq, 1) - 1;
    currCoord   = start;

    if runLoop
        for frame = 1:size(seq, 1)
            currCoord  = seq(frame, :);
            stepsTaken = frame - 1;

            % Hazards materialize in the world before the rover's next move
            for e = 1:size(dynamicEvents, 1)
                if ~eventFired(e) && stepsTaken >= dynamicEvents(e, 1)
                    eventFired(e) = true;
                    er = dynamicEvents(e, 2);
                    ec = dynamicEvents(e, 3);
                    if er >= 1 && er <= gridRows && ec >= 1 && ec <= gridCols ...
                            && ~isequal([er, ec], currCoord)
                        displayMap(er, ec) = 1;
                        dynMarkers(end+1, :) = [er, ec]; %#ok<AGROW>
                    end
                end
            end
            if ~isempty(dynMarkers)
                set(hMap, 'CData', displayMap);
                set(hDynObs, 'XData', dynMarkers(:, 2), 'YData', dynMarkers(:, 1));
            end

            % Proximity sensor at the current cell
            [newObs, ~] = obstacle_detection(displayMap, currCoord, sensorRadius, false);
            if ~isempty(newObs)
                allDetected = unique([allDetected; newObs], 'rows');
                set(hDetectedObs, 'XData', allDetected(:, 2), 'YData', allDetected(:, 1));
            end

            % Rover and travelled path
            set(hRover, 'XData', currCoord(2), 'YData', currCoord(1));
            set(hTravelledPath, 'XData', seq(1:frame, 2), 'YData', seq(1:frame, 1));

            % Planned route: shrinks as it is consumed, unless a real mission
            % (replanning) is shown, where the original plan stays visible
            if hasPath && ~keepFullPlan
                set(hPlannedRoute, 'XData', plannedPath(frame:end, 2), ...
                                   'YData', plannedPath(frame:end, 1));
            end

            % Live coordinates
            set(hTitle, 'String', sprintf( ...
                'Mars Rover Animation | Step: %d/%d | Rover Pos: [%d, %d] | Destination: [%d, %d] | Hazards Sensed: %d', ...
                stepsTaken, totalMoves, currCoord(1), currCoord(2), goal(1), goal(2), size(allDetected, 1)));

            if animate
                drawnow;
                pause(pauseTime);
            end
        end

        % Final status
        if isequal(currCoord, goal)
            set(hTitle, 'String', sprintf('MISSION COMPLETE! Destination [%d, %d] Reached | Steps: %d', ...
                                          goal(1), goal(2), totalMoves));
        else
            set(hTitle, 'String', sprintf('MISSION STOPPED at [%d, %d] | Destination [%d, %d] not reached | Steps: %d', ...
                                          currCoord(1), currCoord(2), goal(1), goal(2), totalMoves));
        end
        drawnow;
    end

    % ---------------------------------------------------------------------
    % 5. Save output images
    % ---------------------------------------------------------------------
    currentDir = fileparts(mfilename('fullpath'));
    resultsDir = fullfile(currentDir, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end
    saveas(figHandle, fullfile(resultsDir, 'planned_route.png'));
    saveas(figHandle, fullfile(resultsDir, 'mission_animation_final.png'));
    saveas(figHandle, fullfile(resultsDir, 'mission_map.png'));

end

% -------------------------------------------------------------------------
% Helper: read an optional field from the opts struct
% -------------------------------------------------------------------------
function value = getOpt(opts, name, defaultValue)
    if isstruct(opts) && isfield(opts, name) && ~isempty(opts.(name))
        value = opts.(name);
    else
        value = defaultValue;
    end
end