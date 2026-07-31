function [path,planningInfo,roadmapHistory] = planPathPRM( ...
    occupancyGrid,startPoint,goalPoint,cfg)
%PLANPATHPRM Probabilistic Roadmap with roadmap history output.

sampleCount = cfg.compare.prm.sampleCount;
connectionCount = cfg.compare.prm.connectionCount;
connectionRadius = cfg.compare.prm.connectionRadius;

nodes = zeros(sampleCount+2,2);
nodes(1,:) = startPoint;
nodes(2,:) = goalPoint;

index = 3;
while index <= sampleCount+2
    candidate = [rand*occupancyGrid.worldWidth,rand*occupancyGrid.worldHeight];
    if pointIsFreeLocal(candidate,occupancyGrid)
        nodes(index,:) = candidate;
        index = index+1;
    end
end

numberOfNodes = size(nodes,1);
adjacency = inf(numberOfNodes,numberOfNodes);
for i = 1:numberOfNodes
    adjacency(i,i) = 0;
end

edgePairs = zeros(0,2);

for nodeIndex = 1:numberOfNodes
    distances = sqrt(sum((nodes-nodes(nodeIndex,:)).^2,2));
    [~,order] = sort(distances,'ascend');

    connected = 0;
    for neighbourOrder = 2:numberOfNodes
        neighbourIndex = order(neighbourOrder);
        distance = distances(neighbourIndex);

        if distance > connectionRadius
            continue;
        end

        if lineIsFreeLocal(nodes(nodeIndex,:),nodes(neighbourIndex,:),occupancyGrid)
            edgeIsNew = isinf(adjacency(nodeIndex,neighbourIndex));

            adjacency(nodeIndex,neighbourIndex) = distance;
            adjacency(neighbourIndex,nodeIndex) = distance;

            if edgeIsNew
                edgePairs(end+1,:) = sort([nodeIndex neighbourIndex]); %#ok<AGROW>
            end

            connected = connected+1;
        end

        if connected >= connectionCount
            break;
        end
    end
end

[startToGoalPath,success] = dijkstraGraphLocal(adjacency,1,2);

roadmapHistory.type = 'roadmap';
roadmapHistory.nodes = nodes;
roadmapHistory.edges = edgePairs;
roadmapHistory.count = size(edgePairs,1);

if ~success
    path = zeros(0,2);
    planningInfo.success = false;
    planningInfo.nodeCount = numberOfNodes;
    planningInfo.pathLength = inf;
    return;
end

path = nodes(startToGoalPath,:);
planningInfo.success = true;
planningInfo.nodeCount = numberOfNodes;
planningInfo.pathLength = sum(sqrt(sum(diff(path,1,1).^2,2)));
end

function [pathIndices,success] = dijkstraGraphLocal(adjacency,startIndex,goalIndex)
numberOfNodes = size(adjacency,1);
distances = inf(numberOfNodes,1);
parents = zeros(numberOfNodes,1);
visited = false(numberOfNodes,1);

distances(startIndex) = 0;

for iteration = 1:numberOfNodes
    candidateDistances = distances;
    candidateDistances(visited) = inf;
    [minimumDistance,currentIndex] = min(candidateDistances);

    if isinf(minimumDistance)
        break;
    end

    visited(currentIndex) = true;

    if currentIndex == goalIndex
        break;
    end

    neighbours = find(isfinite(adjacency(currentIndex,:)) & ~visited');

    for n = 1:numel(neighbours)
        neighbourIndex = neighbours(n);
        tentativeDistance = distances(currentIndex)+adjacency(currentIndex,neighbourIndex);
        if tentativeDistance < distances(neighbourIndex)
            distances(neighbourIndex) = tentativeDistance;
            parents(neighbourIndex) = currentIndex;
        end
    end
end

if ~visited(goalIndex)
    pathIndices = zeros(0,1);
    success = false;
    return;
end

pathIndices = goalIndex;
currentIndex = goalIndex;
while currentIndex ~= startIndex
    currentIndex = parents(currentIndex);
    if currentIndex == 0
        pathIndices = zeros(0,1);
        success = false;
        return;
    end
    pathIndices(end+1,1) = currentIndex; %#ok<AGROW>
end
pathIndices = flipud(pathIndices);
success = true;
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
