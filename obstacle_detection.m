function [detectedObstacles, sensorData, figHandle] = obstacle_detection(map, roverPos, sensorRadius, visualize, plannedPath)
% OBSTACLE_DETECTION Simulates an on-board omnidirectional proximity sensor.
%
% This is a mathematical proximity sensor simulation (Euclidean distance
% field) that models range-based obstacle sensing, alert classification,
% trajectory collision prediction, and detection zone visualization.
% Base MATLAB only.
%
% Usage:
%   detectedObstacles = obstacle_detection(map, roverPos, sensorRadius)
%   [detectedObstacles, sensorData] = obstacle_detection(map, roverPos, sensorRadius)
%   [detectedObstacles, sensorData] = obstacle_detection(map, roverPos, sensorRadius, false, plannedPath)
%   [detectedObstacles, sensorData, figHandle] = obstacle_detection(map, roverPos, sensorRadius, true, plannedPath)
%
% Inputs:
%   map          - 20x20 binary matrix (0 = free terrain, 1 = obstacle)
%   roverPos     - 1x2 coordinate vector [row, col] representing rover location
%   sensorRadius - Configurable scalar sensing radius in grid units (default: 3.0)
%   visualize    - Optional logical flag (default: false); renders detection area
%   plannedPath  - Optional Px2 matrix of remaining waypoints. If provided,
%                  checks whether any detected obstacle blocks the planned route.
%
% Outputs:
%   detectedObstacles - Kx2 matrix of detected obstacle coordinates [row, col]
%   sensorData        - Struct containing detailed sensor telemetry:
%                         .detectedCoords    - Kx2 matrix of detected coordinates
%                         .distances         - Kx1 vector of Euclidean distances
%                         .nearbyObstacles   - Coordinates within near-hazard zone
%                         .distantObstacles  - Coordinates within distant-warning zone
%                         .nearThreshold     - Distance boundary (sensorRadius / 2)
%                         .totalDetected     - Total obstacle count within range
%                         .numNearby         - Count of critical nearby obstacles
%                         .numDistant        - Count of distant warning obstacles
%                         .isPathBlocked     - Logical true if an obstacle lies on plannedPath
%                         .blockingObstacles - Coordinates of obstacles blocking the route
%   figHandle         - Handle to visualization figure (if visualize is true)

    % 1. Input parsing & defaults
    if nargin < 1 || isempty(map)
        [map, ~, ~] = environment();
    end

    if nargin < 2 || isempty(roverPos)
        roverPos = [2, 2];
    end

    if nargin < 3 || isempty(sensorRadius)
        sensorRadius = 3.0;
    end

    if nargin < 4 || isempty(visualize)
        visualize = (nargout >= 3);
    end

    if nargin < 5
        plannedPath = [];
    end

    figHandle = [];
    [gridRows, gridCols] = size(map);

    % Define classification threshold:
    % Distance <= nearThreshold: Critical / Immediate Hazard (Near Zone)
    % nearThreshold < Distance <= sensorRadius: Advisory / Warning Hazard (Distant Zone)
    nearThreshold = sensorRadius / 2.0;

    rRover = roverPos(1);
    cRover = roverPos(2);

    % =====================================================================
    % 2. Safe Bounding Box & Boundary Clamping
    % =====================================================================
    % Prevent indexing outside matrix boundaries [1..gridRows, 1..gridCols]
    rMin = max(1, floor(rRover - sensorRadius));
    rMax = min(gridRows, ceil(rRover + sensorRadius));
    cMin = max(1, floor(cRover - sensorRadius));
    cMax = min(gridCols, ceil(cRover + sensorRadius));

    % Pre-allocate storage
    detectedList = [];
    distList     = [];
    nearbyList   = [];
    distantList  = [];

    % =====================================================================
    % 3. Mathematical Proximity Sensing (Euclidean Range Evaluation)
    % =====================================================================
    for r = rMin:rMax
        for c = cMin:cMax
            % Euclidean distance from rover position to candidate grid cell
            dist = sqrt((r - rRover)^2 + (c - cRover)^2);

            % Obstacle detection within sensor radius
            if dist <= sensorRadius && map(r, c) == 1
                detectedList = [detectedList; r, c];
                distList     = [distList; dist];

                % Distinguish between nearby and distant obstacles
                if dist <= nearThreshold
                    nearbyList = [nearbyList; r, c];
                else
                    distantList = [distantList; r, c];
                end
            end
        end
    end

    % Check if any detected obstacle intersects the planned route
    blockingList = [];
    if ~isempty(plannedPath) && ~isempty(detectedList)
        for i = 1:size(detectedList, 1)
            obs = detectedList(i, :);
            for p = 1:size(plannedPath, 1)
                if isequal(obs, plannedPath(p, :))
                    blockingList = [blockingList; obs];
                    break;
                end
            end
        end
    end

    % Assign outputs
    detectedObstacles = detectedList;

    sensorData.detectedCoords    = detectedList;
    sensorData.distances         = distList;
    sensorData.nearbyObstacles   = nearbyList;
    sensorData.distantObstacles  = distantList;
    sensorData.nearThreshold     = nearThreshold;
    sensorData.sensorRadius      = sensorRadius;
    sensorData.totalDetected     = size(detectedList, 1);
    sensorData.numNearby         = size(nearbyList, 1);
    sensorData.numDistant        = size(distantList, 1);
    sensorData.isPathBlocked     = ~isempty(blockingList);
    sensorData.blockingObstacles = blockingList;

    % =====================================================================
    % 4. Sensor Detection Area Visualization
    % =====================================================================
    if visualize
        figHandle = figure('Name', 'Proximity Sensor Detection Area', ...
                           'NumberTitle', 'off', ...
                           'Color', 'w', ...
                           'Position', [100, 100, 800, 680]);

        % Display base map
        imagesc(map, [0, 1]);
        axis equal;
        axis tight;
        hold on;

        % Apply terrain colormap: 0 = Light brown, 1 = Dark grey
        terrainColormap = [
            0.82, 0.70, 0.55;  % 0: Free planetary terrain (light brown)
            0.25, 0.25, 0.25   % 1: Obstacles (craters, boulders, rock ridges)
        ];
        colormap(terrainColormap);

        % Cell boundary grid lines
        for x = 0.5:1:(gridCols + 0.5)
            line([x, x], [0.5, gridRows + 0.5], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
        end
        for y = 0.5:1:(gridRows + 0.5)
            line([0.5, gridCols + 0.5], [y, y], 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.5);
        end

        % Draw Sensor Field Geometry
        theta = linspace(0, 2*pi, 200);

        % Outer detection circle (Sensor Range)
        outerX = cRover + sensorRadius * cos(theta);
        outerY = rRover + sensorRadius * sin(theta);
        patch(outerX, outerY, [0.22, 0.74, 0.97], ...
              'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Sensor Field');
        hRange = plot(outerX, outerY, '--', 'Color', [0.01, 0.52, 0.78], ...
                      'LineWidth', 1.8, 'DisplayName', sprintf('Sensor Limit (R = %.1f)', sensorRadius));

        % Inner detection circle (Near-Hazard Threshold)
        innerX = cRover + nearThreshold * cos(theta);
        innerY = rRover + nearThreshold * sin(theta);
        hNearZone = plot(innerX, innerY, ':', 'Color', [0.96, 0.62, 0.04], ...
                         'LineWidth', 1.8, 'DisplayName', sprintf('Near Zone (R <= %.1f)', nearThreshold));

        % Rover Marker (Green Square)
        hRover = plot(cRover, rRover, 's', ...
            'MarkerSize', 14, ...
            'MarkerFaceColor', [0.13, 0.77, 0.37], ...
            'MarkerEdgeColor', 'k', ...
            'LineWidth', 1.5, ...
            'DisplayName', sprintf('Rover [%d, %d]', rRover, cRover));

        % Detected Nearby Hazards (Red Triangles)
        if ~isempty(nearbyList)
            hNearObs = plot(nearbyList(:, 2), nearbyList(:, 1), '^', ...
                'MarkerSize', 11, ...
                'MarkerFaceColor', [0.94, 0.27, 0.27], ...
                'MarkerEdgeColor', 'k', ...
                'LineWidth', 1.2, ...
                'DisplayName', sprintf('Nearby Hazards (%d)', size(nearbyList, 1)));
        else
            hNearObs = plot(nan, nan, '^', 'MarkerFaceColor', [0.94, 0.27, 0.27], ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Nearby Hazards (0)');
        end

        % Detected Distant Hazards (Amber Circles)
        if ~isempty(distantList)
            hDistObs = plot(distantList(:, 2), distantList(:, 1), 'o', ...
                'MarkerSize', 9, ...
                'MarkerFaceColor', [0.96, 0.62, 0.04], ...
                'MarkerEdgeColor', 'k', ...
                'LineWidth', 1.2, ...
                'DisplayName', sprintf('Distant Hazards (%d)', size(distantList, 1)));
        else
            hDistObs = plot(nan, nan, 'o', 'MarkerFaceColor', [0.96, 0.62, 0.04], ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Distant Hazards (0)');
        end

        % Proxy handles for colormap terrain in legend
        hFree = plot(nan, nan, 's', 'MarkerSize', 10, ...
            'MarkerFaceColor', [0.82, 0.70, 0.55], 'MarkerEdgeColor', [0.5, 0.5, 0.5], ...
            'LineWidth', 1, 'DisplayName', 'Free Terrain');
        hObs = plot(nan, nan, 's', 'MarkerSize', 10, ...
            'MarkerFaceColor', [0.25, 0.25, 0.25], 'MarkerEdgeColor', 'k', ...
            'LineWidth', 1, 'DisplayName', 'Obstacles (Craters / Rocks)');

        % Axis limits and labels
        set(gca, 'XTick', 1:gridCols, 'YTick', 1:gridRows);
        set(gca, 'XAxisLocation', 'bottom');
        set(gca, 'YDir', 'reverse');
        xlim([0.5, gridCols + 0.5]);
        ylim([0.5, gridRows + 0.5]);

        xlabel('X Coordinate (Grid Columns)', 'FontSize', 11, 'FontWeight', 'bold');
        ylabel('Y Coordinate (Grid Rows)', 'FontSize', 11, 'FontWeight', 'bold');

        title({'Mars Rover Proximity Sensor Simulation', ...
               sprintf('Rover at [%d, %d] | Total Detected: %d (Nearby: %d, Distant: %d)', ...
                       rRover, cRover, sensorData.totalDetected, sensorData.numNearby, sensorData.numDistant)}, ...
               'FontSize', 12, 'FontWeight', 'bold');

        legend([hFree, hObs, hRover, hRange, hNearZone, hNearObs, hDistObs], ...
               'Location', 'northeastoutside', 'FontSize', 10);

        % Save visualization in results/
        currentDir = fileparts(mfilename('fullpath'));
        resultsDir = fullfile(currentDir, 'results');
        if ~exist(resultsDir, 'dir')
            mkdir(resultsDir);
        end
        saveas(figHandle, fullfile(resultsDir, 'sensor_detection.png'));
        fprintf('Sensor visualization successfully saved to results/sensor_detection.png\n');
    end

end
