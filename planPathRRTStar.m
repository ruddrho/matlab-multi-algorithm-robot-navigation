function [path,planningInfo,treeHistory] = planPathRRTStar( ...
    occupancyGrid,startPoint,goalPoint,cfg)
%PLANPATHRRTSTAR RRT* planner with rewiring and animation history.

maximumIterations = cfg.compare.rrtStar.maximumIterations;
stepSize = cfg.compare.rrtStar.stepSize;
goalBias = cfg.compare.rrtStar.goalBias;
goalThreshold = cfg.compare.rrtStar.goalThreshold;
rewireRadius = cfg.compare.rrtStar.rewireRadius;

nodes = startPoint;
parents = 0;
costs = 0;
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

    neighbourIndices = find(sqrt(sum((nodes-newPoint).^2,2)) <= rewireRadius);

    bestParent = nearestIndex;
    bestCost = costs(nearestIndex)+norm(newPoint-nodes(nearestIndex,:));

    for index = 1:numel(neighbourIndices)
        candidateIndex = neighbourIndices(index);
        if ~lineIsFreeLocal(nodes(candidateIndex,:),newPoint,occupancyGrid)
            continue;
        end
        candidateCost = costs(candidateIndex)+norm(newPoint-nodes(candidateIndex,:));
        if candidateCost < bestCost
            bestParent = candidateIndex;
            bestCost = candidateCost;
        end
    end

    nodes(end+1,:) = newPoint; %#ok<AGROW>
    parents(end+1,1) = bestParent; %#ok<AGROW>
    costs(end+1,1) = bestCost; %#ok<AGROW>
    newIndex = size(nodes,1);

    for index = 1:numel(neighbourIndices)
        candidateIndex = neighbourIndices(index);
        if candidateIndex == bestParent
            continue;
        end
        if ~lineIsFreeLocal(newPoint,nodes(candidateIndex,:),occupancyGrid)
            continue;
        end
        rewiredCost = bestCost+norm(newPoint-nodes(candidateIndex,:));
        if rewiredCost < costs(candidateIndex)
            parents(candidateIndex) = newIndex;
            costs(candidateIndex) = rewiredCost;
        end
    end

    if norm(newPoint-goalPoint) <= goalThreshold && ...
            lineIsFreeLocal(newPoint,goalPoint,occupancyGrid)
        nodes(end+1,:) = goalPoint; %#ok<AGROW>
        parents(end+1,1) = newIndex; %#ok<AGROW>
        costs(end+1,1) = bestCost+norm(newPoint-goalPoint); %#ok<AGROW>
        goalNodeIndex = size(nodes,1);
        found = true;
        break;
    end
end

if ~found
    [found,goalNodeIndex,nodes,parents,costs] = ...
        attemptGoalConnectionLocal(nodes,parents,costs,goalPoint,occupancyGrid);
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

function [found,goalNodeIndex,nodes,parents,costs] = ...
    attemptGoalConnectionLocal(nodes,parents,costs,goalPoint,occupancyGrid)

distances = sqrt(sum((nodes-goalPoint).^2,2));
[~,order] = sort(distances,'ascend');
found = false;
goalNodeIndex = 1;

for index = 1:min(15,numel(order))
    nodeIndex = order(index);
    if lineIsFreeLocal(nodes(nodeIndex,:),goalPoint,occupancyGrid)
        nodes(end+1,:) = goalPoint; %#ok<AGROW>
        parents(end+1,1) = nodeIndex; %#ok<AGROW>
        costs(end+1,1) = costs(nodeIndex)+norm(goalPoint-nodes(nodeIndex,:)); %#ok<AGROW>
        goalNodeIndex = size(nodes,1);
        found = true;
        return;
    end
end
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
    point = fromPoint+stepSize*delta/max(distance,eps);
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
