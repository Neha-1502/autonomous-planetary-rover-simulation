function [path, cost] = astar_path_planning(map, start, goal)
% ASTAR_PATH_PLANNING Custom A* pathfinding algorithm built from scratch.
%
% Implements A* search on a 2D grid using only basic MATLAB matrices,
% logical arrays, and loops. No toolboxes or built-in graph functions used.
%
% Inputs:
%   map   - 2D binary matrix (0 = free terrain, 1 = obstacle)
%   start - 1x2 coordinate vector [row, col] for starting position
%   goal  - 1x2 coordinate vector [row, col] for destination
%
% Outputs:
%   path  - Px2 matrix of [row, col] waypoints from start to goal.
%           Returns [] (empty matrix) if no valid path exists.
%   cost  - Total path cost (number of steps). Returns Inf if unreachable.
%
% Algorithm Properties:
%   - Movement: 4-directional (Up, Down, Left, Right). Step cost = 1.
%   - Heuristic: Manhattan distance h(n) = |r - goal_r| + |c - goal_c|.
%   - Open Set: Tracked via logical matrix openSet(rows, cols).
%   - Closed Set: Tracked via logical matrix closedSet(rows, cols).
%   - Cost Scores:
%       gScore(r, c): Exact shortest path cost from start to (r, c).
%       hScore(r, c): Admissible Manhattan heuristic to goal.
%       fScore(r, c): Estimated total cost f(n) = g(n) + h(n).
%   - Parent tracking: parentR(r, c) and parentC(r, c) matrices.

    % Default return values for failure
    path = [];
    cost = Inf;

    [gridRows, gridCols] = size(map);

    % =====================================================================
    % 1. Input Validation & Blocked Endpoint Checks
    % =====================================================================
    % Check boundary limits for start and goal
    if start(1) < 1 || start(1) > gridRows || start(2) < 1 || start(2) > gridCols || ...
       goal(1) < 1  || goal(1) > gridRows  || goal(2) < 1  || goal(2) > gridCols
        fprintf('[A* Warning] Start or goal coordinates are outside grid boundaries.\n');
        return;
    end

    % Check if starting cell is blocked
    if map(start(1), start(2)) == 1
        fprintf('[A* Warning] Starting position [%d, %d] is an obstacle.\n', start(1), start(2));
        return;
    end

    % Check if destination cell is blocked
    if map(goal(1), goal(2)) == 1
        fprintf('[A* Warning] Destination position [%d, %d] is an obstacle.\n', goal(1), goal(2));
        return;
    end

    % Trivial case: Start is already at Goal
    if isequal(start, goal)
        path = start;
        cost = 0;
        return;
    end

    % =====================================================================
    % 2. Initialize Data Structures (Basic Matrices)
    % =====================================================================
    % gScore: Shortest distance from start to each cell (initialized to Inf)
    gScore = inf(gridRows, gridCols);

    % fScore: Estimated total cost f = g + h (initialized to Inf)
    fScore = inf(gridRows, gridCols);

    % openSet: Boolean mask of nodes discovered and awaiting evaluation
    openSet = false(gridRows, gridCols);

    % closedSet: Boolean mask of nodes already expanded and finalized
    closedSet = false(gridRows, gridCols);

    % Parent pointers for path reconstruction
    parentR = zeros(gridRows, gridCols);
    parentC = zeros(gridRows, gridCols);

    % 4-directional movements: [delta_row, delta_col]
    % Up, Down, Left, Right
    directions = [
        -1,  0;  % Up
         1,  0;  % Down
         0, -1;  % Left
         0,  1   % Right
    ];

    % Initialize start node
    sRow = start(1);
    sCol = start(2);
    gRow = goal(1);
    gCol = goal(2);

    gScore(sRow, sCol) = 0;
    % Manhattan Distance heuristic: |r - goal_r| + |c - goal_c|
    hStart = abs(sRow - gRow) + abs(sCol - gCol);
    fScore(sRow, sCol) = hStart;
    openSet(sRow, sCol) = true;

    % =====================================================================
    % 3. Main A* Search Loop
    % =====================================================================
    goalReached = false;

    while any(openSet(:))
        % -----------------------------------------------------------------
        % Node Selection:
        % Pick the node in openSet with the lowest fScore.
        % Tie-breaking: If fScores are equal, choose the node with lower hScore.
        % -----------------------------------------------------------------
        minF = Inf;
        minH = Inf;
        currR = 0;
        currC = 0;

        for r = 1:gridRows
            for c = 1:gridCols
                if openSet(r, c)
                    hVal = abs(r - gRow) + abs(c - gCol);
                    if fScore(r, c) < minF || (fScore(r, c) == minF && hVal < minH)
                        minF = fScore(r, c);
                        minH = hVal;
                        currR = r;
                        currC = c;
                    end
                end
            end
        end

        % Check if destination is reached
        if currR == gRow && currC == gCol
            goalReached = true;
            break;
        end

        % Move current node from openSet to closedSet
        openSet(currR, currC) = false;
        closedSet(currR, currC) = true;

        % -----------------------------------------------------------------
        % Neighbor Expansion (4 Directions)
        % -----------------------------------------------------------------
        for d = 1:size(directions, 1)
            nbrR = currR + directions(d, 1);
            nbrC = currC + directions(d, 2);

            % Boundary Check
            if nbrR < 1 || nbrR > gridRows || nbrC < 1 || nbrC > gridCols
                continue;
            end

            % Obstacle Avoidance: Skip obstacle cells
            if map(nbrR, nbrC) == 1
                continue;
            end

            % Skip already evaluated cells
            if closedSet(nbrR, nbrC)
                continue;
            end

            % Tentative gScore: Current cost + unit step cost (1)
            tentativeG = gScore(currR, currC) + 1;

            % If newly discovered or found a cheaper path to this neighbor
            if tentativeG < gScore(nbrR, nbrC)
                % Record parent pointers
                parentR(nbrR, nbrC) = currR;
                parentC(nbrR, nbrC) = currC;

                % Update scores
                gScore(nbrR, nbrC) = tentativeG;
                hScore = abs(nbrR - gRow) + abs(nbrC - gCol); % Manhattan distance
                fScore(nbrR, nbrC) = tentativeG + hScore;

                % Add neighbor to openSet
                openSet(nbrR, nbrC) = true;
            end
        end
    end

    % =====================================================================
    % 4. Path Reconstruction (Backtracking from Goal to Start)
    % =====================================================================
    if ~goalReached
        % No valid route exists
        path = [];
        cost = Inf;
        return;
    end

    % Backtrack using parent pointers
    reconstructedPath = [];
    curr = [gRow, gCol];

    while ~isempty(curr) && curr(1) ~= 0 && curr(2) ~= 0
        reconstructedPath = [curr; reconstructedPath]; % Prepend current node
        if curr(1) == sRow && curr(2) == sCol
            break;
        end
        pR = parentR(curr(1), curr(2));
        pC = parentC(curr(1), curr(2));
        curr = [pR, pC];
    end

    path = reconstructedPath;
    cost = size(path, 1) - 1;

end
