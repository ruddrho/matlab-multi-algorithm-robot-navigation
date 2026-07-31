function [path,planningInfo,searchHistory] = planPathDijkstra( ...
    occupancyGrid,startPoint,goalPoint)
%PLANPATHDIJKSTRA Toolbox-free 8-connected Dijkstra planner.
%
% The optional third output records the node-expansion order for the
% synchronized comparison animation.

occupied = occupancyGrid.occupied;
numberOfRows = occupancyGrid.numberOfRows;
numberOfColumns = occupancyGrid.numberOfColumns;
resolution = occupancyGrid.resolution;

[startRow,startColumn] = worldToGridLocal(startPoint,occupancyGrid);
[goalRow,goalColumn] = worldToGridLocal(goalPoint,occupancyGrid);

[startRow,startColumn] = nearestFreeCellLocal(startRow,startColumn,occupied);
[goalRow,goalColumn] = nearestFreeCellLocal(goalRow,goalColumn,occupied);

startIndex = sub2ind([numberOfRows numberOfColumns],startRow,startColumn);
goalIndex = sub2ind([numberOfRows numberOfColumns],goalRow,goalColumn);

totalCells = numberOfRows*numberOfColumns;
gCost = inf(totalCells,1);
parent = zeros(totalCells,1);
openSet = false(totalCells,1);
closedSet = false(totalCells,1);

gCost(startIndex) = 0;
openSet(startIndex) = true;

moves = [ ...
    -1 -1 sqrt(2);
    -1  0 1;
    -1  1 sqrt(2);
     0 -1 1;
     0  1 1;
     1 -1 sqrt(2);
     1  0 1;
     1  1 sqrt(2)];

expandedGrid = zeros(totalCells,2);
parentGrid = nan(totalCells,2);
expandedNodes = 0;
found = false;

while any(openSet)
    openIndices = find(openSet);
    [~,minimumLocation] = min(gCost(openIndices));
    currentIndex = openIndices(minimumLocation);

    openSet(currentIndex) = false;

    if closedSet(currentIndex)
        continue;
    end

    closedSet(currentIndex) = true;
    expandedNodes = expandedNodes+1;

    [currentRow,currentColumn] = ind2sub( ...
        [numberOfRows numberOfColumns],currentIndex);

    expandedGrid(expandedNodes,:) = [currentRow currentColumn];

    if parent(currentIndex) ~= 0
        [parentRow,parentColumn] = ind2sub( ...
            [numberOfRows numberOfColumns],parent(currentIndex));
        parentGrid(expandedNodes,:) = [parentRow parentColumn];
    end

    if currentIndex == goalIndex
        found = true;
        break;
    end

    for moveIndex = 1:size(moves,1)
        rowOffset = moves(moveIndex,1);
        columnOffset = moves(moveIndex,2);

        nextRow = currentRow+rowOffset;
        nextColumn = currentColumn+columnOffset;

        if nextRow < 1 || nextRow > numberOfRows || ...
                nextColumn < 1 || nextColumn > numberOfColumns
            continue;
        end

        if occupied(nextRow,nextColumn)
            continue;
        end

        if rowOffset ~= 0 && columnOffset ~= 0
            if occupied(currentRow,nextColumn) || ...
                    occupied(nextRow,currentColumn)
                continue;
            end
        end

        nextIndex = sub2ind([numberOfRows numberOfColumns], ...
            nextRow,nextColumn);

        if closedSet(nextIndex)
            continue;
        end

        tentativeCost = gCost(currentIndex)+moves(moveIndex,3);

        if tentativeCost < gCost(nextIndex)
            parent(nextIndex) = currentIndex;
            gCost(nextIndex) = tentativeCost;
            openSet(nextIndex) = true;
        end
    end
end

expandedGrid = expandedGrid(1:expandedNodes,:);
parentGrid = parentGrid(1:expandedNodes,:);

searchHistory.type = 'grid';
searchHistory.expandedWorld = [ ...
    (expandedGrid(:,2)-0.5)/resolution, ...
    (expandedGrid(:,1)-0.5)/resolution];
searchHistory.parentWorld = nan(expandedNodes,2);
validParents = ~isnan(parentGrid(:,1));
searchHistory.parentWorld(validParents,:) = [ ...
    (parentGrid(validParents,2)-0.5)/resolution, ...
    (parentGrid(validParents,1)-0.5)/resolution];
searchHistory.count = expandedNodes;

if ~found
    path = zeros(0,2);
    planningInfo.success = false;
    planningInfo.expandedNodes = expandedNodes;
    planningInfo.pathLength = inf;
    return;
end

indexPath = goalIndex;
currentIndex = goalIndex;

while currentIndex ~= startIndex
    currentIndex = parent(currentIndex);
    if currentIndex == 0
        path = zeros(0,2);
        planningInfo.success = false;
        planningInfo.expandedNodes = expandedNodes;
        planningInfo.pathLength = inf;
        return;
    end
    indexPath(end+1,1) = currentIndex; %#ok<AGROW>
end

indexPath = flipud(indexPath);
path = zeros(numel(indexPath),2);

for index = 1:numel(indexPath)
    [row,column] = ind2sub([numberOfRows numberOfColumns],indexPath(index));
    path(index,:) = [(column-0.5)/resolution,(row-0.5)/resolution];
end

path(1,:) = startPoint;
path(end,:) = goalPoint;

planningInfo.success = true;
planningInfo.expandedNodes = expandedNodes;
planningInfo.pathLength = sum(sqrt(sum(diff(path,1,1).^2,2)));
end

function [row,column] = worldToGridLocal(point,occupancyGrid)
column = round(point(1)*occupancyGrid.resolution+0.5);
row = round(point(2)*occupancyGrid.resolution+0.5);
column = min(max(column,1),occupancyGrid.numberOfColumns);
row = min(max(row,1),occupancyGrid.numberOfRows);
end

function [freeRow,freeColumn] = nearestFreeCellLocal(startRow,startColumn,occupied)
[numberOfRows,numberOfColumns] = size(occupied);
if ~occupied(startRow,startColumn)
    freeRow = startRow;
    freeColumn = startColumn;
    return;
end
maximumRadius = max(numberOfRows,numberOfColumns);
for radius = 1:maximumRadius
    rowMinimum = max(1,startRow-radius);
    rowMaximum = min(numberOfRows,startRow+radius);
    columnMinimum = max(1,startColumn-radius);
    columnMaximum = min(numberOfColumns,startColumn+radius);
    bestDistance = inf;
    freeRow = [];
    freeColumn = [];
    for row = rowMinimum:rowMaximum
        for column = columnMinimum:columnMaximum
            if occupied(row,column)
                continue;
            end
            distance = hypot(double(row-startRow),double(column-startColumn));
            if distance < bestDistance
                bestDistance = distance;
                freeRow = row;
                freeColumn = column;
            end
        end
    end
    if ~isempty(freeRow)
        return;
    end
end
error('No free occupancy-grid cell could be found.');
end
