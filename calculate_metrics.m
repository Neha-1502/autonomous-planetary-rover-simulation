function metrics = calculate_metrics(missionInfo, timeElapsed, batteryParams)
% CALCULATE_METRICS Mission performance indicators for the rover simulation.
%
% Usage:
%   metrics = calculate_metrics(missionInfo)
%   metrics = calculate_metrics(missionInfo, timeElapsed)
%   metrics = calculate_metrics(missionInfo, timeElapsed, batteryParams)
%   calculate_metrics(missionInfo)            % no output: prints a report
%   params  = calculate_metrics('defaults')   % default battery parameters
%   params  = calculate_metrics('defaults', overrides)  % defaults + overrides
%
% Inputs:
%   missionInfo   - struct returned by mission_controller.m. Fields used:
%                   pathHistory, success, startPos, goalPos, finalPos,
%                   detectedObstacles, replanCount, and (optional)
%                   finalMap, newObstacleCount, executionTimeSec, status.
%   timeElapsed   - Optional execution time in seconds. If empty/omitted,
%                   missionInfo.executionTimeSec is used when present,
%                   otherwise the time is reported as NaN (not available).
%   batteryParams - Optional struct overriding the battery model. If omitted,
%                   missionInfo.batteryParams (set by mission_controller) is
%                   used when present, otherwise the defaults.
%
% -------------------------------------------------------------------------
% BATTERY MODEL (deliberately simple; NOT a physical rover power model)
% -------------------------------------------------------------------------
%   capacity      = 100 energy units (default)
%   energyPerMove = 1 energy unit per VALID one-cell movement (default)
%
%   consumed  = min(capacity, movements * energyPerMove)
%   remaining = capacity - consumed          (never negative)
%   percent   = 100 * remaining / capacity
%
%   Only successful movements use energy. Sensing, planning, replanning
%   and halted time cost nothing. The mission controller stops the rover
%   when the remaining energy is less than one movement's cost, so the
%   battery can never go below zero. Both numbers are model parameters,
%   not measurements of any real vehicle.
%
% -------------------------------------------------------------------------
% DEFINITIONS
% -------------------------------------------------------------------------
%   totalDistance     Sum of Euclidean lengths between consecutive cells of
%                     pathHistory, in grid units (1 per 4-connected move).
%   movements         Valid moves executed = size(pathHistory,1) - 1.
%   obstaclesDetected Number of DISTINCT obstacle cells the proximity
%                     sensor saw within range at any time (this includes
%                     known static obstacles that came within range).
%   newObstaclesDiscovered
%                     Obstacles the rover did not know about before
%                     sensing them (i.e. dynamic hazards). NaN if the
%                     missionInfo has no newObstacleCount field.
%   pathRecalculations missionInfo.replanCount.
%   completionPercent 100 when the goal was reached. Otherwise
%                       100 * (1 - dFinal / dStart)
%                     where dStart / dFinal are A* shortest-path lengths
%                     to the goal from the start / final position on
%                     missionInfo.finalMap, clamped to [0, 100]. If the
%                     rover never moved it is 0. If the goal is not
%                     reachable on finalMap (or no map is available) the
%                     value is NaN and completionMethod says why.
%   executionTimeSec  Wall-clock computation time of the simulation, not
%                     simulated rover travel time.
%
% Base MATLAB only.

    % ---- 'defaults' mode: return battery parameters ---------------------
    if (ischar(missionInfo) || isstring(missionInfo)) && strcmpi(char(missionInfo), 'defaults')
        if nargin < 2, timeElapsed = []; end
        metrics = mergeBatteryParams(timeElapsed);
        return;
    end

    if ~isstruct(missionInfo) || ~isfield(missionInfo, 'pathHistory')
        error('calculate_metrics:badInput', ...
              'First argument must be the missionInfo struct from mission_controller.');
    end

    if nargin < 2, timeElapsed = []; end
    if nargin < 3, batteryParams = []; end
    % Default to the battery model the mission was actually run with
    if isempty(batteryParams) && isfield(missionInfo, 'batteryParams')
        batteryParams = missionInfo.batteryParams;
    end
    params = mergeBatteryParams(batteryParams);

    history = missionInfo.pathHistory;

    % ---- 1 & 7. Distance and movements ----------------------------------
    movements = max(0, size(history, 1) - 1);
    if size(history, 1) >= 2
        totalDistance = sum(sqrt(sum(diff(history, 1, 1).^2, 2)));
    else
        totalDistance = 0;
    end

    % ---- 2 & 3. Obstacles and recalculations -----------------------------
    obstaclesDetected = 0;
    if isfield(missionInfo, 'detectedObstacles') && ~isempty(missionInfo.detectedObstacles)
        obstaclesDetected = size(unique(missionInfo.detectedObstacles, 'rows'), 1);
    end

    newObstacles = NaN;
    if isfield(missionInfo, 'newObstacleCount')
        newObstacles = missionInfo.newObstacleCount;
    end

    recalcs = 0;
    if isfield(missionInfo, 'replanCount')
        recalcs = missionInfo.replanCount;
    end

    % ---- 4 & 5. Battery ----------------------------------------------------
    consumed  = min(params.capacity, movements * params.energyPerMove);
    remaining = max(0, params.capacity - consumed);
    remainingPercent = 100 * remaining / params.capacity;

    % ---- 6. Mission completion ---------------------------------------------
    success = isfield(missionInfo, 'success') && missionInfo.success;
    [completion, method] = missionCompletion(missionInfo, success, movements);

    % ---- 8. Execution time --------------------------------------------------
    if ~isempty(timeElapsed)
        execTime = timeElapsed;
    elseif isfield(missionInfo, 'executionTimeSec')
        execTime = missionInfo.executionTimeSec;
    else
        execTime = NaN;
    end

    % ---- Assemble output ------------------------------------------------------
    metrics = struct();
    metrics.totalDistance          = totalDistance;
    metrics.movements              = movements;
    metrics.obstaclesDetected      = obstaclesDetected;
    metrics.newObstaclesDiscovered = newObstacles;
    metrics.pathRecalculations     = recalcs;
    metrics.batteryCapacity        = params.capacity;
    metrics.energyPerMove          = params.energyPerMove;
    metrics.batteryConsumed        = consumed;
    metrics.batteryRemaining       = remaining;
    metrics.batteryRemainingPercent = remainingPercent;
    metrics.batteryDepleted        = remaining <= 1e-12;
    metrics.completionPercent      = completion;
    metrics.completionMethod       = method;
    metrics.executionTimeSec       = execTime;
    metrics.missionSuccess         = success;
    if isfield(missionInfo, 'status'),   metrics.status   = missionInfo.status;   end
    if isfield(missionInfo, 'startPos'), metrics.startPos = missionInfo.startPos; end
    if isfield(missionInfo, 'goalPos'),  metrics.goalPos  = missionInfo.goalPos;  end
    if isfield(missionInfo, 'finalPos'), metrics.finalPos = missionInfo.finalPos; end

    if nargout == 0
        printReport(metrics);
    end
end

% =========================================================================
% Battery parameter defaults + validation (single source of truth, also
% used by mission_controller.m via calculate_metrics('defaults', ...)).
% =========================================================================
function params = mergeBatteryParams(overrides)
    params.capacity      = 100;   % energy units
    params.energyPerMove = 1;     % energy units per valid movement

    if ~isempty(overrides)
        if ~isstruct(overrides)
            error('calculate_metrics:badParams', 'batteryParams must be a struct.');
        end
        names = fieldnames(params);
        for i = 1:numel(names)
            if isfield(overrides, names{i}) && ~isempty(overrides.(names{i}))
                params.(names{i}) = overrides.(names{i});
            end
        end
    end

    if ~(isscalar(params.capacity) && params.capacity > 0) || ...
       ~(isscalar(params.energyPerMove) && params.energyPerMove > 0)
        error('calculate_metrics:badParams', ...
              'capacity and energyPerMove must be positive scalars.');
    end
end

% =========================================================================
% Mission completion percentage (see DEFINITIONS above)
% =========================================================================
function [pct, method] = missionCompletion(info, success, movements)
    if success
        pct = 100;
        method = 'goal reached';
        return;
    end
    if movements == 0
        pct = 0;
        method = 'no movement';
        return;
    end
    if ~isfield(info, 'finalMap') || isempty(info.finalMap) || ...
       ~isfield(info, 'startPos') || ~isfield(info, 'goalPos') || ~isfield(info, 'finalPos')
        pct = NaN;
        method = 'undefined: map or positions not available';
        return;
    end

    map = info.finalMap;
    goal = info.goalPos;
    if map(goal(1), goal(2)) == 1
        pct = NaN;
        method = 'undefined: goal blocked on final map';
        return;
    end

    [~, dStart] = astar_path_planning(map, info.startPos, goal);
    [~, dFinal] = astar_path_planning(map, info.finalPos, goal);

    if isinf(dStart) || isinf(dFinal)
        pct = NaN;
        method = 'undefined: goal unreachable on final map';
    elseif dStart == 0
        pct = 100;
        method = 'shortest-path distance on final map';
    else
        pct = min(100, max(0, 100 * (1 - dFinal / dStart)));
        method = 'shortest-path distance on final map';
    end
end

% =========================================================================
% Printed report (only when called with no output argument)
% =========================================================================
function printReport(m)
    fprintf('============================================================\n');
    fprintf('  MISSION PERFORMANCE METRICS\n');
    fprintf('============================================================\n');
    fprintf('Total distance travelled : %.2f grid units\n', m.totalDistance);
    fprintf('Movements                : %d\n', m.movements);
    fprintf('Obstacles detected       : %d distinct cells\n', m.obstaclesDetected);
    if ~isnan(m.newObstaclesDiscovered)
        fprintf('New obstacles discovered : %d\n', m.newObstaclesDiscovered);
    end
    fprintf('Path recalculations      : %d\n', m.pathRecalculations);
    fprintf('Battery consumed         : %.2f / %.2f units\n', m.batteryConsumed, m.batteryCapacity);
    fprintf('Battery remaining        : %.2f units (%.1f%%)\n', m.batteryRemaining, m.batteryRemainingPercent);
    if isnan(m.completionPercent)
        fprintf('Mission completion       : n/a (%s)\n', m.completionMethod);
    else
        fprintf('Mission completion       : %.1f%% (%s)\n', m.completionPercent, m.completionMethod);
    end
    if isnan(m.executionTimeSec)
        fprintf('Execution time           : not available\n');
    else
        fprintf('Execution time           : %.3f s (computation time)\n', m.executionTimeSec);
    end
    fprintf('============================================================\n');
end