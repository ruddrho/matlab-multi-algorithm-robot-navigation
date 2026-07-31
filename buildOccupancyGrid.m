function occupancyGrid = buildOccupancyGrid(environment,cfg)
%BUILDOCCUPANCYGRID Build a safety-inflated binary occupancy grid.
%
% Every grid cell is checked with the same physical collision function
% used by the simulator. The robot radius is enlarged by the configured
% path-planning margin.

resolution = cfg.path.gridResolution;
numberOfColumns = ceil(environment.worldWidth*resolution);
numberOfRows = ceil(environment.worldHeight*resolution);

occupied = false(numberOfRows,numberOfColumns);

inflatedConfig = cfg;
inflatedConfig.robot.radius = ...
    cfg.robot.radius+cfg.path.inflationMargin;

for row = 1:numberOfRows
    y = (row-0.5)/resolution;

    for column = 1:numberOfColumns
        x = (column-0.5)/resolution;

        occupied(row,column) = collisionCheck( ...
            [x;y;0],environment,inflatedConfig);
    end
end

occupancyGrid.occupied = occupied;
occupancyGrid.resolution = resolution;
occupancyGrid.numberOfRows = numberOfRows;
occupancyGrid.numberOfColumns = numberOfColumns;
occupancyGrid.worldWidth = environment.worldWidth;
occupancyGrid.worldHeight = environment.worldHeight;
end
