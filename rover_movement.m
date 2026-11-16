function [roverState, success, msg] = rover_movement(roverState, direction, map)
% ROVER_MOVEMENT Simulates discrete kinematics and navigation steps for the rover.
%
% Usage:
%   1. Initialization:
%      roverState = rover_movement(startPos);
%      (Initializes rover state struct at startPos, e.g. [2, 2])
%
%   2. Step Movement:
%      [roverState, success, msg] = rover_movement(roverState, direction, map);
%
% Inputs:
%   roverState - Struct containing:
%                  .position  - [row, col] current coordinates (1x2 vector)
%                  .history   - Nx2 matrix of all visited coordinates
%                  .stepCount - Total number of valid moves executed
%                (If passed as a 1x2 numeric vector, it is converted to struct)
%   direction  - Movement direction: 'up', 'down', 'left', or 'right'
%   map        - 20x20 binary obstacle grid (0 = free terrain, 1 = obstacle)
%                (Optional; defaults to environment() map if omitted)
%
% Outputs:
%   roverState - Updated rover state struct.
%   success    - Logical flag: true (1) if move succeeded, false (0) if blocked.
%   msg        - Descriptive status string explaining the result.
%
% Constraints:
%   - Uses simple base MATLAB functions only.
%   - Moves strictly one cell at a time in 4 cardinal directions.
%   - Blocks movements outside grid boundaries [1..rows, 1..cols].
%   - Blocks movements into obstacle cells (map == 1).
%   - Updates history and position only upon valid move.

    % =====================================================================
    % Mode 1: Rover State Initialization
    % =====================================================================
    if nargin < 2
        if nargin < 1 || isempty(roverState)
            startPos = [2, 2];
        elseif isnumeric(roverState) && numel(roverState) == 2
            startPos = [roverState(1), roverState(2)];
        elseif isstruct(roverState) && isfield(roverState, 'position')
            startPos = roverState.position;
        else
            error('Invalid initial position format. Provide a 1x2 coordinate vector [row, col].');
        end

        stateStruct.position  = [startPos(1), startPos(2)];
        stateStruct.history   = [startPos(1), startPos(2)];
        stateStruct.stepCount = 0;

        roverState = stateStruct;
        success    = true;
        msg        = sprintf('Rover initialized at position [%d, %d].', startPos(1), startPos(2));
        return;
    end

    % Normalize input roverState if supplied as bare coordinate vector [row, col]
    if isnumeric(roverState) && numel(roverState) == 2
        initPos = [roverState(1), roverState(2)];
        roverState = struct('position', initPos, 'history', initPos, 'stepCount', 0);
    end

    % Default to environment() map if not provided
    if nargin < 3 || isempty(map)
        [map, ~, ~] = environment();
    end

    [gridRows, gridCols] = size(map);

    % =====================================================================
    % Step 1: Decode 4-Direction Movement (Single-cell displacements)
    % =====================================================================
    if isstring(direction) || ischar(direction)
        dirStr = lower(char(direction));
    else
        dirStr = '';
    end

    switch dirStr
        case 'up'
            delta = [-1,  0];   % Move one cell north (decrease row index)
        case 'down'
            delta = [ 1,  0];   % Move one cell south (increase row index)
        case 'left'
            delta = [ 0, -1];   % Move one cell west (decrease column index)
        case 'right'
            delta = [ 0,  1];   % Move one cell east (increase column index)
        otherwise
            success = false;
            msg = sprintf('Invalid direction "%s". Supported directions: ''up'', ''down'', ''left'', ''right''.', ...
                          dirStr);
            return;
    end

    % Calculate candidate coordinates
    currentPos   = roverState.position;
    candidatePos = currentPos + delta;
    candRow      = candidatePos(1);
    candCol      = candidatePos(2);

    % =====================================================================
    % Step 2: Prevent Movement Outside Grid (Boundary Check)
    % =====================================================================
    if candRow < 1 || candRow > gridRows || candCol < 1 || candCol > gridCols
        success = false;
        msg = sprintf('BLOCKED: Cannot move %s to [%d, %d]. Outside %dx%d grid limits.', ...
                      dirStr, candRow, candCol, gridRows, gridCols);
        return;
    end

    % =====================================================================
    % Step 3: Prevent Movement into Obstacles (Collision Check)
    % =====================================================================
    if map(candRow, candCol) == 1
        success = false;
        msg = sprintf('BLOCKED: Obstacle detected at [%d, %d]. Movement aborted.', ...
                      candRow, candCol);
        return;
    end

    % =====================================================================
    % Step 4: Valid Movement Execution (Update Position & History)
    % =====================================================================
    roverState.position  = candidatePos;
    roverState.history   = [roverState.history; candidatePos];
    roverState.stepCount = roverState.stepCount + 1;
    success = true;
    msg = sprintf('SUCCESS: Moved %s to [%d, %d]. Total steps: %d.', ...
                  dirStr, candRow, candCol, roverState.stepCount);

end
