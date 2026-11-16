function [finalPos, missionInfo, roverState] = mission_controller(map, start, goal, dynamicEvents, sensorRadius, verbose, batteryParams, stepCallback)
% MISSION_CONTROLLER Autonomous planetary rover mission execution.
%
% Integrates:
%   environment.m        - default terrain, start and goal
%   rover_movement.m     - one-cell-at-a-time kinematics and history
%   obstacle_detection.m - range-based proximity sensor
%   astar_path_planning.m- route planning and replanning
%
% Usage:
%   [finalPos, missionInfo, roverState] = mission_controller()
%   [finalPos, missionInfo, roverState] = mission_controller(map, start, goal)
%   [finalPos, missionInfo, roverState] = mission_controller(map, start, goal, dynamicEvents, sensorRadius, verbose)
%   [finalPos, missionInfo, roverState] = mission_controller(..., verbose, batteryParams)
%   [finalPos, missionInfo, roverState] = mission_controller(..., batteryParams, stepCallback)
%
% Inputs (all optional; [] selects the default):
%   map           - binary matrix (0 = free, 1 = obstacle). Default: environment()
%   start         - [row, col]. Default: [2, 2] (or environment() start)
%   goal          - [row, col]. Default: [19, 19] (or environment() goal)
%   dynamicEvents - Kx3 matrix [stepNumber, row, col]. An obstacle appears
%                   at [row, col] once stepNumber moves have been executed.
%                   The rover is NOT told about it: it must sense it.
%   sensorRadius  - sensing radius in grid units. Default: 2.5
%   verbose       - print telemetry. Default: true
%   batteryParams - struct with optional fields capacity (default 100) and
%                   energyPerMove (default 1); see calculate_metrics.m for
%                   the battery model. Each valid move costs energyPerMove.
%                   The mission stops when the remaining battery is less
%                   than one move's cost, so it never goes negative.
%   stepCallback  - Optional function handle keepGoing = f(snapshot), called
%                   before the first move, once per control-loop pass (after
%                   sensing/replanning, before the move) and once at the end.
%                   Returning false aborts the mission ('STOPPED: Mission
%                   aborted by operator.'). snapshot fields: phase, position,
%                   history, stepsTaken, replanCount, detectedObstacles,
%                   newObstacleCount, battery, batteryParams, currentPath,
%                   trueMap, message. Used by main_rover.m for live updates.
%
% Outputs:
%   finalPos    - [row, col] where the rover stopped
%   missionInfo - struct: success, status, startPos, goalPos, finalPos,
%                 totalStepsTaken, pathHistory, replanCount,
%                 detectedObstacles, executionLog, finalMap (true terrain),
%                 knownMap (what the rover believed at the end),
%                 batteryRemaining, batteryUsed, batteryParams,
%                 newObstacleCount (obstacles first learned by sensing),
%                 executionTimeSec (wall-clock computation time)
%   roverState  - final struct from rover_movement.m
%
% Map model:
%   trueMap  - ground truth. Dynamic obstacles are added here. Used by the
%              sensor and by rover_movement (physical collision).
%   knownMap - the rover's belief. Starts equal to the supplied map and is
%              updated only with obstacles the sensor detects. A* plans on it.

    % ---------------------------------------------------------------------
    % 1. Inputs and initialization
    % ---------------------------------------------------------------------
    if nargin < 1 || isempty(map)
        [map, defaultStart, defaultGoal] = environment();
    else
        defaultStart = [2, 2];
        defaultGoal  = [19, 19];
    end
    if nargin < 2 || isempty(start),         start = defaultStart; end
    if nargin < 3 || isempty(goal),          goal  = defaultGoal;  end
    if nargin < 4,                           dynamicEvents = [];   end
    if nargin < 5 || isempty(sensorRadius),  sensorRadius = 2.5;   end
    if nargin < 6 || isempty(verbose),       verbose = true;       end
    if nargin < 7,                           batteryParams = [];   end
    if nargin < 8,                           stepCallback  = [];   end

    t0 = tic;                                            % execution-time clock
    batteryParams = calculate_metrics('defaults', batteryParams);
    battery = batteryParams.capacity;                    % remaining energy units
    newObstacleCount = 0;

    if ~isempty(dynamicEvents) && size(dynamicEvents, 2) ~= 3
        error('mission_controller:badEvents', ...
              'dynamicEvents must be a Kx3 matrix of [stepNumber, row, col].');
    end

    [gridRows, gridCols] = size(map);
    trueMap  = map;
    knownMap = map;

    logEntries = {};
    logEntries{end+1} = sprintf('Mission initialized: Start=[%d, %d], Goal=[%d, %d], SensorRadius=%.1f', ...
                                start(1), start(2), goal(1), goal(2), sensorRadius);

    if verbose
        fprintf('============================================================\n');
        fprintf('  AUTONOMOUS PLANETARY ROVER MISSION CONTROLLER\n');
        fprintf('============================================================\n');
        fprintf('Start Position : [%d, %d]\n', start(1), start(2));
        fprintf('Destination    : [%d, %d]\n', goal(1), goal(2));
        fprintf('Sensor Radius  : %.1f grid units\n\n', sensorRadius);
    end

    % Requirement 1: initialize the rover
    roverState           = rover_movement(start);
    replanCount          = 0;
    allDetectedObstacles = [];
    eventFired           = false(size(dynamicEvents, 1), 1);

    % ---------------------------------------------------------------------
    % 2. Initial route (requirement 3)
    % ---------------------------------------------------------------------
    if verbose, fprintf('[Phase 1] Computing initial route using A*...\n'); end
    [currentPath, pathCost] = astar_path_planning(knownMap, roverState.position, goal);

    if isempty(currentPath)
        statusMsg = 'FAILED: No valid initial route exists to destination.';
        logEntries{end+1} = statusMsg;
        if verbose, fprintf('  [ERROR] %s\n\n', statusMsg); end

        finalPos    = roverState.position;
        extra = struct('batteryRemaining', battery, 'batteryUsed', 0, ...
                       'batteryParams', batteryParams, 'newObstacleCount', 0, ...
                       'executionTimeSec', toc(t0));
        missionInfo = buildMissionInfo(false, statusMsg, start, goal, finalPos, 0, ...
                                       roverState.history, 0, [], logEntries, trueMap, knownMap, extra);
        return;
    end

    logEntries{end+1} = sprintf('Initial route: %d waypoints (cost %d).', size(currentPath, 1), pathCost);
    if verbose
        fprintf('  Initial path found: %d waypoints (Cost: %d steps).\n\n', size(currentPath, 1), pathCost);
        fprintf('[Phase 2] Commencing autonomous transit...\n');
    end

    % ---------------------------------------------------------------------
    % 3. Navigation loop (requirements 4-9)
    % ---------------------------------------------------------------------
    maxSteps      = 300;          % ceiling on executed moves
    maxIterations = 5 * maxSteps; % ceiling on loop passes (guards against stalls)
    iterations    = 0;
    stepsTaken    = 0;
    failedMoves   = 0;            % consecutive rejected moves
    abortReason   = '';

    if ~notifyStep(stepCallback, 'planned', roverState, stepsTaken, replanCount, ...
                   allDetectedObstacles, newObstacleCount, battery, batteryParams, ...
                   currentPath, trueMap, logEntries)
        abortReason = 'STOPPED: Mission aborted by operator.';
    end

    while isempty(abortReason) && ~isequal(roverState.position, goal) && stepsTaken < maxSteps
        iterations = iterations + 1;
        if iterations > maxIterations
            abortReason = 'STOPPED: Iteration limit exceeded (no progress).';
            break;
        end

        % Battery gate: never start a move the battery cannot pay for
        if battery < batteryParams.energyPerMove - 1e-12
            abortReason = sprintf('STOPPED: Battery depleted (%.2f units left) at [%d, %d].', ...
                                  battery, roverState.position(1), roverState.position(2));
            logEntries{end+1} = abortReason;
            if verbose, fprintf('  [BATTERY] %s\n', abortReason); end
            break;
        end

        currPos = roverState.position;

        % --- Dynamic hazards appear in the TRUE terrain only -------------
        for e = 1:size(dynamicEvents, 1)
            if eventFired(e) || stepsTaken < dynamicEvents(e, 1)
                continue;
            end
            eventFired(e) = true;
            r = dynamicEvents(e, 2);
            c = dynamicEvents(e, 3);

            if r < 1 || r > gridRows || c < 1 || c > gridCols
                msg = sprintf('[Step %d] Dynamic event at [%d, %d] ignored (outside grid).', stepsTaken, r, c);
            elseif isequal([r, c], currPos)
                msg = sprintf('[Step %d] Dynamic event at [%d, %d] ignored (rover occupies cell).', stepsTaken, r, c);
            else
                trueMap(r, c) = 1;
                msg = sprintf('[Step %d] Dynamic obstacle materialized at [%d, %d]!', stepsTaken, r, c);
            end
            logEntries{end+1} = msg;
            if verbose, fprintf('  >> %s\n', msg); end
        end

        % --- Requirement 5: sense for new obstacles ---------------------
        if size(currentPath, 1) > 1
            remainingPath = currentPath(2:end, :);
        else
            remainingPath = [];
        end

        [detectedObs, sensorData] = obstacle_detection(trueMap, currPos, sensorRadius, false, remainingPath);

        newlyKnown = zeros(0, 2);
        for k = 1:size(detectedObs, 1)
            if knownMap(detectedObs(k, 1), detectedObs(k, 2)) == 0
                newlyKnown(end+1, :) = detectedObs(k, :); %#ok<AGROW>
                knownMap(detectedObs(k, 1), detectedObs(k, 2)) = 1;
            end
        end
        newObstacleCount = newObstacleCount + size(newlyKnown, 1);
        if ~isempty(detectedObs)
            allDetectedObstacles = unique([allDetectedObstacles; detectedObs], 'rows');
        end
        for k = 1:size(newlyKnown, 1)
            msg = sprintf('[Step %d] New obstacle sensed at [%d, %d].', stepsTaken, newlyKnown(k, 1), newlyKnown(k, 2));
            logEntries{end+1} = msg; %#ok<AGROW>
            if verbose, fprintf('  [SENSOR] %s\n', msg); end
        end

        % --- Requirement 6: is the planned movement unsafe? -------------
        pathBlocked = sensorData.isPathBlocked;
        if ~isempty(remainingPath)
            idx = sub2ind(size(knownMap), remainingPath(:, 1), remainingPath(:, 2));
            pathBlocked = pathBlocked || any(knownMap(idx) == 1);
        end

        % --- Requirement 7: halt and recalculate ------------------------
        if pathBlocked
            replanCount = replanCount + 1;
            msg = sprintf('[Step %d] EMERGENCY HALT at [%d, %d]: hazard on route. Recalculating...', ...
                          stepsTaken, currPos(1), currPos(2));
            logEntries{end+1} = msg;
            if verbose, fprintf('  [HALT] %s\n', msg); end

            [newPath, newCost] = astar_path_planning(knownMap, currPos, goal);

            % Requirement 9: stop if no valid route exists
            if isempty(newPath)
                abortReason = sprintf('FAILED: Destination unreachable after replan at [%d, %d].', ...
                                      currPos(1), currPos(2));
                logEntries{end+1} = abortReason;
                if verbose, fprintf('  [FAILURE] %s\n', abortReason); end
                break;
            end

            currentPath = newPath;
            msg = sprintf('  Replan #%d successful: %d waypoints (Cost: %d steps).', ...
                          replanCount, size(currentPath, 1), newCost);
            logEntries{end+1} = msg;
            if verbose, fprintf('%s\n', msg); end
        end

        % A route with fewer than 2 nodes cannot be followed: replan once
        if size(currentPath, 1) < 2
            [currentPath, ~] = astar_path_planning(knownMap, currPos, goal);
            if isempty(currentPath) || size(currentPath, 1) < 2
                abortReason = 'FAILED: No usable route to destination.';
                logEntries{end+1} = abortReason;
                break;
            end
        end

        % Live update (GUI): state after sensing/replanning, before the move
        if ~notifyStep(stepCallback, 'step', roverState, stepsTaken, replanCount, ...
                       allDetectedObstacles, newObstacleCount, battery, batteryParams, ...
                       currentPath, trueMap, logEntries)
            abortReason = 'STOPPED: Mission aborted by operator.';
            logEntries{end+1} = abortReason;
            break;
        end

        % --- Requirement 4: take one step along the route ---------------
        delta = currentPath(2, :) - currPos;
        if     isequal(delta, [-1,  0]), dirStr = 'up';
        elseif isequal(delta, [ 1,  0]), dirStr = 'down';
        elseif isequal(delta, [ 0, -1]), dirStr = 'left';
        elseif isequal(delta, [ 0,  1]), dirStr = 'right';
        else,                            dirStr = '';
        end

        % Collision is checked against the TRUE terrain
        [roverState, moveSuccess, moveMsg] = rover_movement(roverState, dirStr, trueMap);

        if moveSuccess
            failedMoves = 0;
            stepsTaken  = stepsTaken + 1;
            battery     = max(0, battery - batteryParams.energyPerMove);
            currentPath = currentPath(2:end, :);
            logEntries{end+1} = sprintf('Step %d: Moved %s -> [%d, %d]', ...
                                        stepsTaken, dirStr, roverState.position(1), roverState.position(2));
        else
            % Rover physically refused the move: treat the target as an
            % obstacle, then replan from the current position.
            failedMoves = failedMoves + 1;
            tgt = currentPath(2, :);
            if tgt(1) >= 1 && tgt(1) <= gridRows && tgt(2) >= 1 && tgt(2) <= gridCols
                knownMap(tgt(1), tgt(2)) = 1;
            end
            msg = sprintf('[Step %d] Movement refused (%s). Replanning.', stepsTaken, moveMsg);
            logEntries{end+1} = msg;
            if verbose, fprintf('  [WARN] %s\n', msg); end

            if failedMoves >= 3
                abortReason = 'FAILED: Repeated movement failures.';
                logEntries{end+1} = abortReason;
                break;
            end

            replanCount = replanCount + 1;
            [currentPath, ~] = astar_path_planning(knownMap, roverState.position, goal);
            if isempty(currentPath)
                abortReason = 'FAILED: Destination unreachable. Mission aborted.';
                logEntries{end+1} = abortReason;
                break;
            end
        end
    end

    notifyStep(stepCallback, 'final', roverState, stepsTaken, replanCount, ...
               allDetectedObstacles, newObstacleCount, battery, batteryParams, ...
               currentPath, trueMap, logEntries);

    % ---------------------------------------------------------------------
    % 4. Final assessment (requirement 10)
    % ---------------------------------------------------------------------
    finalPos = roverState.position;
    missionSuccess = isequal(finalPos, goal);

    if missionSuccess
        statusMsg = 'SUCCESS: Mission accomplished. Destination reached.';
    elseif ~isempty(abortReason)
        statusMsg = abortReason;
    elseif stepsTaken >= maxSteps
        statusMsg = 'STOPPED: Maximum step limit exceeded without reaching destination.';
    else
        statusMsg = 'FAILED: Destination is unreachable. Mission aborted.';
    end

    logEntries{end+1} = sprintf('Mission terminated: %s (Total steps: %d, Replans: %d)', ...
                                statusMsg, stepsTaken, replanCount);

    if verbose
        fprintf('\n============================================================\n');
        fprintf('  MISSION TERMINATION SUMMARY\n');
        fprintf('============================================================\n');
        fprintf('Status            : %s\n', statusMsg);
        fprintf('Final Position    : [%d, %d]\n', finalPos(1), finalPos(2));
        fprintf('Target Destination: [%d, %d]\n', goal(1), goal(2));
        fprintf('Total Steps Taken : %d\n', stepsTaken);
        fprintf('Replan Count      : %d\n', replanCount);
        fprintf('Total Hazards Sensed: %d\n', size(allDetectedObstacles, 1));
        fprintf('Battery Remaining : %.2f / %.2f units\n', battery, batteryParams.capacity);
        fprintf('============================================================\n\n');
    end

    extra = struct('batteryRemaining', battery, ...
                   'batteryUsed', batteryParams.capacity - battery, ...
                   'batteryParams', batteryParams, 'newObstacleCount', newObstacleCount, ...
                   'executionTimeSec', toc(t0));
    missionInfo = buildMissionInfo(missionSuccess, statusMsg, start, goal, finalPos, ...
                                   stepsTaken, roverState.history, replanCount, ...
                                   allDetectedObstacles, logEntries, trueMap, knownMap, extra);
end

% =========================================================================
% Helper: telemetry struct builder
% =========================================================================
function info = buildMissionInfo(success, status, startPos, goalPos, finalPos, ...
                                 steps, history, replans, detectedObs, log, trueMap, knownMap, extra)
    info.success           = success;
    info.status            = status;
    info.startPos          = startPos;
    info.goalPos           = goalPos;
    info.finalPos          = finalPos;
    info.totalStepsTaken   = steps;
    info.pathHistory       = history;
    info.replanCount       = replans;
    info.detectedObstacles = detectedObs;
    info.executionLog      = log;
    info.finalMap          = trueMap;
    info.knownMap          = knownMap;
    names = fieldnames(extra);
    for i = 1:numel(names)
        info.(names{i}) = extra.(names{i});
    end
end

% =========================================================================
% Helper: send a state snapshot to the optional step callback
% =========================================================================
function keepGoing = notifyStep(cb, phase, roverState, stepsTaken, replanCount, ...
                                detected, newObstacleCount, battery, batteryParams, ...
                                currentPath, trueMap, logEntries)
    keepGoing = true;
    if isempty(cb)
        return;
    end
    snap.phase             = phase;
    snap.position          = roverState.position;
    snap.history           = roverState.history;
    snap.stepsTaken        = stepsTaken;
    snap.replanCount       = replanCount;
    snap.detectedObstacles = detected;
    snap.newObstacleCount  = newObstacleCount;
    snap.battery           = battery;
    snap.batteryParams     = batteryParams;
    snap.currentPath       = currentPath;
    snap.trueMap           = trueMap;
    if isempty(logEntries), snap.message = ''; else, snap.message = logEntries{end}; end
    keepGoing = logical(cb(snap));
end