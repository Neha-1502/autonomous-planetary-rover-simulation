function [map, start, goal] = environment()
% ENVIRONMENT Generates a 20x20 planetary terrain map with obstacles.
%
% Outputs:
%   map   - 20x20 numeric matrix (0 = free terrain, 1 = obstacle)
%   start - 1x2 coordinate vector [row, col] for rover starting point ([2, 2])
%   goal  - 1x2 coordinate vector [row, col] for destination point ([19, 19])
%
% Constraints:
%   - Uses base MATLAB only (no external toolboxes or dependencies).
%   - Reproducible obstacle placement using fixed RNG seed.
%   - Start and destination coordinates are guaranteed unblocked.

    % 1. Set fixed random generator seed for reproducibility
    rng(42, 'twister');

    % 2. Initialize 20x20 grid with zeros (0 = free terrain)
    gridRows = 20;
    gridCols = 20;
    map = zeros(gridRows, gridCols);

    % 3. Define rover start and goal positions
    start = [2, 2];
    goal  = [19, 19];

    % 4. Add crater-like obstacles (circular impact depressions)
    % Craters defined as [center_row, center_col, radius]
    craters = [
        6,   6, 2.4;   % Crater Alpha (North-West basin)
       11,  14, 2.6;   % Crater Beta  (Central-East basin)
       15,   7, 2.2    % Crater Gamma (South-West basin)
    ];

    [X, Y] = meshgrid(1:gridCols, 1:gridRows);
    for i = 1:size(craters, 1)
        cRow = craters(i, 1);
        cCol = craters(i, 2);
        cRad = craters(i, 3);
        craterMask = ((Y - cRow).^2 + (X - cCol).^2) <= (cRad^2);
        map(craterMask) = 1;
    end

    % 5. Add rock formations (ridges and boulder clusters)
    % Rock Ridge 1: Elongated diagonal formation in North-East
    rockRidge1 = [
        3, 13; 4, 13; 4, 14; 5, 14; 5, 15; 6, 15; 7, 16
    ];

    % Rock Ridge 2: Transverse formation in Central-South
    rockRidge2 = [
        14, 11; 14, 12; 15, 12; 15, 13; 16, 13
    ];

    % Boulder Clusters: Clustered rock fields
    boulderClusters = [
        9,  2;  9,  3; 10,  3; 10,  4;  % West boulder field
       17, 14; 17, 15; 18, 15; 18, 16   % South-East boulder field
    ];

    % Combine all rock formation coordinates
    rockCoordinates = [rockRidge1; rockRidge2; boulderClusters];
    for k = 1:size(rockCoordinates, 1)
        r = rockCoordinates(k, 1);
        c = rockCoordinates(k, 2);
        map(r, c) = 1;
    end

    % 6. Ensure start and goal coordinates are explicitly clear (0)
    map(start(1), start(2)) = 0;
    map(goal(1), goal(2))   = 0;

end
