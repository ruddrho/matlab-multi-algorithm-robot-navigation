function [path,planningInfo,treeHistory] = planPathRRT( ...
    occupancyGrid,startPoint,goalPoint,cfg)
%PLANPATHRRT Basic Rapidly-exploring Random Tree with history output.

maximumIterations = cfg.compare.rrt.maximumIterations;
stepSize = cfg.compare.rrt.stepSize;
goalBias = cfg.compare.rrt.goalBias;
goalThreshold = cfg.compare.rrt.goalThreshold;

nodes = startPoint;
parents = 0;
found = false;
goalNodeIndex = 1;

for iteration = 1:maximumIterations
    if rand < goalBias
        samplePoint = goalPoint;
    else
        samplePoint = sampleFreePointLocal(occupancyGrid);
    end

    distances = sqrt(sum((nodes-samplePoint).^2,2));
    [~,nearestIndex] = min(distances);
    newPoint = steerLocal(nodes(nearestIndex,:),samplePoint,stepSize);

    if ~lineIsFreeLocal(nodes(nearestIndex,:),newPoint,occupancyGrid)
        continue;
    end

    nodes(end+1,:) = newPoint; %#ok<AGROW>
    parents(end+1,1) = nearestIndex; %#ok<AGROW>
    newIndex = size(nodes,1);

    if norm(newPoint-goalPoint) <= goalThreshold && ...
            lineIsFreeLocal(newPoint,goalPoint,occupancyGrid)
        nodes(end+1,:) = goalPoint; %#ok<AGROW>
        parents(end+1,1) = newIndex; %#ok<AGROW>
        goalNodeIndex = size(nodes,1);
        found = true;
        break;
    end
end

treeHistory.type = 'tree';
treeHistory.nodes = nodes;
treeHistory.parents = parents;
treeHistory.count = size(nodes,1);

if ~found
    path = zeros(0,2);
    planningInfo.success = false;
    planningInfo.nodeCount = size(nodes,1);
    planningInfo.pathLength = inf;
    return;
end

path = backtrackPathLocal(nodes,parents,goalNodeIndex);
planningInfo.success = true;
planningInfo.nodeCount = size(nodes,1);
planningInfo.pathLength = sum(sqrt(sum(diff(path,1,1).^2,2)));
end

function samplePoint = sampleFreePointLocal(occupancyGrid)
while true
    samplePoint = [rand*occupancyGrid.worldWidth,rand*occupancyGrid.worldHeight];
    if pointIsFreeLocal(samplePoint,occupancyGrid)
        return;
    end
end
end

function point = steerLocal(fromPoint,toPoint,stepSize)
delta = toPoint-fromPoint;
distance = norm(delta);
if distance <= stepSize
    point = toPoint;
else
    point = fromPoint + stepSize*delta/max(distance,eps);
end
end

function path = backtrackPathLocal(nodes,parents,goalIndex)
path = nodes(goalIndex,:);
currentIndex = goalIndex;
while parents(currentIndex) ~= 0
    currentIndex = parents(currentIndex);
    path(end+1,:) = nodes(currentIndex,:); %#ok<AGROW>
end
path = flipud(path);
end

function free = pointIsFreeLocal(point,occupancyGrid)
column = round(point(1)*occupancyGrid.resolution+0.5);
row = round(point(2)*occupancyGrid.resolution+0.5);
column = min(max(column,1),occupancyGrid.numberOfColumns);
row = min(max(row,1),occupancyGrid.numberOfRows);
free = ~occupancyGrid.occupied(row,column);
end

function free = lineIsFreeLocal(pointA,pointB,occupancyGrid)
distance = norm(pointB-pointA);
sampleCount = max(2,ceil(distance*occupancyGrid.resolution*3));
free = true;
for sampleIndex = 0:sampleCount
    ratio = sampleIndex/sampleCount;
    point = pointA+ratio*(pointB-pointA);
    if ~pointIsFreeLocal(point,occupancyGrid)
        free = false;
        return;
    end
end
end
