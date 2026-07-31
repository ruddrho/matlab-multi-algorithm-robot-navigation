function processedPath = postProcessPath( ...
    rawPath,occupancyGrid,cfg)
%POSTPROCESSPATH Simplify and uniformly resample an A* path.
%
% Greedy line-of-sight simplification removes unnecessary grid turns.
% Uniform resampling then gives the look-ahead follower continuous,
% closely spaced target points. No spline is used, so obstacle clearance
% remains consistent with the occupancy grid.

if size(rawPath,1) <= 2
    simplifiedPath = rawPath;
else
    simplifiedPath = rawPath(1,:);
    anchorIndex = 1;

    while anchorIndex < size(rawPath,1)
        candidateIndex = size(rawPath,1);

        while candidateIndex > anchorIndex+1
            if lineIsFreeLocal( ...
                    rawPath(anchorIndex,:), ...
                    rawPath(candidateIndex,:), ...
                    occupancyGrid)
                break;
            end

            candidateIndex = candidateIndex-1;
        end

        simplifiedPath(end+1,:) = ...
            rawPath(candidateIndex,:); %#ok<AGROW>

        anchorIndex = candidateIndex;
    end
end

segmentLengths = sqrt(sum( ...
    diff(simplifiedPath,1,1).^2,2));

arcLength = [0;cumsum(segmentLengths)];

if arcLength(end) <= cfg.path.resampleSpacing
    processedPath = simplifiedPath;
    return;
end

queryArcLength = ...
    (0:cfg.path.resampleSpacing:arcLength(end))';

if queryArcLength(end) < arcLength(end)
    queryArcLength(end+1,1) = arcLength(end);
end

processedPath = [ ...
    interp1(arcLength,simplifiedPath(:,1), ...
    queryArcLength,'linear'), ...
    interp1(arcLength,simplifiedPath(:,2), ...
    queryArcLength,'linear')];

processedPath(1,:) = rawPath(1,:);
processedPath(end,:) = rawPath(end,:);
end

function free = lineIsFreeLocal( ...
    pointA,pointB,occupancyGrid)

distance = norm(pointB-pointA);
sampleCount = max(2,ceil( ...
    distance*occupancyGrid.resolution*3));

free = true;

for sampleIndex = 0:sampleCount
    ratio = sampleIndex/sampleCount;
    point = pointA+ratio*(pointB-pointA);

    column = round( ...
        point(1)*occupancyGrid.resolution+0.5);
    row = round( ...
        point(2)*occupancyGrid.resolution+0.5);

    column = min(max(column,1), ...
        occupancyGrid.numberOfColumns);
    row = min(max(row,1), ...
        occupancyGrid.numberOfRows);

    if occupancyGrid.occupied(row,column)
        free = false;
        return;
    end
end
end
