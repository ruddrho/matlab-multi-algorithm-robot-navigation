function slamState = updateSlamMap( ...
    slamState,scan,cfg,referencePose)
%UPDATESLAMMAP Correlative scan matching and log-odds map update.
%
% The raw SLAM pose is predicted from odometry and corrected by local
% scan matching. In this simulation project, an optional world-frame pose
% anchor keeps the occupancy map aligned with the same coordinate frame
% used by the A* route and clicked goal. This prevents accumulated SLAM
% drift from creating a false second endpoint in the exported map.

if nargin < 4
    referencePose = [];
end

rawEstimatedPose = scanMatchPoseLocal( ...
    slamState,scan,cfg);

estimatedPose = rawEstimatedPose;

if ~isempty(referencePose) && ...
        isfield(cfg.slam,'worldFrameAnchorEnabled') && ...
        cfg.slam.worldFrameAnchorEnabled

    anchorGain = utilities( ...
        'clamp',cfg.slam.worldFrameAnchorGain,0,1);

    positionError = ...
        referencePose(1:2)-rawEstimatedPose(1:2);

    headingError = utilities( ...
        'wrapAngle', ...
        referencePose(3)-rawEstimatedPose(3));

    estimatedPose(1:2) = ...
        rawEstimatedPose(1:2)+ ...
        anchorGain*positionError;

    estimatedPose(3) = utilities( ...
        'wrapAngle', ...
        rawEstimatedPose(3)+ ...
        anchorGain*headingError);
end

logOdds = slamState.logOdds;
observed = slamState.observed;

rayIndices = ...
    1:cfg.slam.mappingRayStride:numel(scan.ranges);

for rayIndex = rayIndices
    rangeValue = min( ...
        scan.ranges(rayIndex), ...
        cfg.lidar.maximumRange);

    globalAngle = estimatedPose(3)+ ...
        scan.localAngles(rayIndex);

    endpoint = estimatedPose(1:2)'+ ...
        rangeValue*[cos(globalAngle) sin(globalAngle)];

    [startRow,startColumn] = worldToGridLocal( ...
        estimatedPose(1:2)',slamState,cfg);

    [endRow,endColumn] = worldToGridLocal( ...
        endpoint,slamState,cfg);

    [rayRows,rayColumns] = bresenhamLocal( ...
        startRow,startColumn,endRow,endColumn);

    if isempty(rayRows)
        continue;
    end

    if scan.hitMask(rayIndex)
        freeCount = max(0,numel(rayRows)-1);
    else
        freeCount = numel(rayRows);
    end

    if freeCount > 0
        freeIndices = sub2ind( ...
            size(logOdds), ...
            rayRows(1:freeCount), ...
            rayColumns(1:freeCount));

        logOdds(freeIndices) = ...
            logOdds(freeIndices)+ ...
            cfg.slam.logOddsFree;

        observed(freeIndices) = true;
    end

    if scan.hitMask(rayIndex)
        occupiedRows = [];
        occupiedColumns = [];

        for rowOffset = ...
                -cfg.slam.occupiedCellRadius: ...
                 cfg.slam.occupiedCellRadius
            for columnOffset = ...
                    -cfg.slam.occupiedCellRadius: ...
                     cfg.slam.occupiedCellRadius

                row = endRow+rowOffset;
                column = endColumn+columnOffset;

                if row >= 1 && ...
                        row <= slamState.numberOfRows && ...
                        column >= 1 && ...
                        column <= slamState.numberOfColumns

                    occupiedRows(end+1,1) = row; %#ok<AGROW>
                    occupiedColumns(end+1,1) = column; %#ok<AGROW>
                end
            end
        end

        occupiedIndices = sub2ind( ...
            size(logOdds), ...
            occupiedRows,occupiedColumns);

        logOdds(occupiedIndices) = ...
            logOdds(occupiedIndices)+ ...
            cfg.slam.logOddsOccupied;

        observed(occupiedIndices) = true;
    end
end

logOdds = utilities( ...
    'clamp',logOdds, ...
    cfg.slam.minimumLogOdds, ...
    cfg.slam.maximumLogOdds);

slamState.logOdds = logOdds;
slamState.observed = observed;
slamState.probability = 1./(1+exp(-logOdds));

slamState.rawEstimatedPose = rawEstimatedPose;
slamState.estimatedPose = estimatedPose;
slamState.predictedPose = estimatedPose;

if ~isempty(referencePose)
    slamState.worldFramePositionError = ...
        norm(rawEstimatedPose(1:2)-referencePose(1:2));

    slamState.worldFrameHeadingError = abs( ...
        utilities('wrapAngle', ...
        rawEstimatedPose(3)-referencePose(3)));
else
    slamState.worldFramePositionError = NaN;
    slamState.worldFrameHeadingError = NaN;
end

slamState.updateCount = ...
    slamState.updateCount+1;

slamState.exploredFraction = ...
    nnz(observed)/numel(observed);

slamState.trajectory(end+1,:) = ...
    estimatedPose';
end

function estimatedPose = scanMatchPoseLocal( ...
    slamState,scan,cfg)

predictedPose = slamState.predictedPose;
estimatedPose = predictedPose;

if slamState.updateCount < ...
        cfg.slam.minimumScanMatchUpdates
    return;
end

hitIndices = find(scan.hitMask);
hitIndices = hitIndices( ...
    1:cfg.slam.scanMatchRayStride:end);

if numel(hitIndices) < ...
        cfg.slam.minimumScanMatchHits
    return;
end

translationOffsets = linspace( ...
    -cfg.slam.scanMatchTranslation, ...
     cfg.slam.scanMatchTranslation, ...
     cfg.slam.scanMatchTranslationSamples);

rotationOffsets = linspace( ...
    -cfg.slam.scanMatchRotation, ...
     cfg.slam.scanMatchRotation, ...
     cfg.slam.scanMatchRotationSamples);

bestScore = -inf;

for xOffset = translationOffsets
    for yOffset = translationOffsets
        for angleOffset = rotationOffsets
            candidatePose = predictedPose+[ ...
                xOffset;yOffset;angleOffset];

            candidatePose(3) = utilities( ...
                'wrapAngle',candidatePose(3));

            evidence = zeros(numel(hitIndices),1);
            validCount = 0;

            for index = 1:numel(hitIndices)
                rayIndex = hitIndices(index);

                globalAngle = candidatePose(3)+ ...
                    scan.localAngles(rayIndex);

                endpoint = candidatePose(1:2)'+ ...
                    scan.ranges(rayIndex)* ...
                    [cos(globalAngle) sin(globalAngle)];

                [row,column,inside] = ...
                    worldToGridLocal( ...
                    endpoint,slamState,cfg);

                if inside
                    validCount = validCount+1;
                    evidence(validCount) = ...
                        slamState.logOdds(row,column);
                end
            end

            if validCount < ...
                    cfg.slam.minimumScanMatchHits
                continue;
            end

            evidence = evidence(1:validCount);

            offsetPenalty = ...
                cfg.slam.scanMatchTranslationPenalty* ...
                hypot(xOffset,yOffset) + ...
                cfg.slam.scanMatchRotationPenalty* ...
                abs(angleOffset);

            score = mean(evidence)-offsetPenalty;

            if score > bestScore
                bestScore = score;
                estimatedPose = candidatePose;
            end
        end
    end
end
end

function [row,column,inside] = ...
    worldToGridLocal(point,slamState,cfg)

inside = point(1) >= 0 && ...
    point(1) <= cfg.environment.worldWidth && ...
    point(2) >= 0 && ...
    point(2) <= cfg.environment.worldHeight;

column = floor(point(1)*slamState.resolution)+1;
row = floor(point(2)*slamState.resolution)+1;

column = min(max(column,1), ...
    slamState.numberOfColumns);
row = min(max(row,1), ...
    slamState.numberOfRows);
end

function [rows,columns] = bresenhamLocal( ...
    startRow,startColumn,endRow,endColumn)

x0 = startColumn;
y0 = startRow;
x1 = endColumn;
y1 = endRow;

dx = abs(x1-x0);
dy = abs(y1-y0);

if x0 < x1
    stepX = 1;
else
    stepX = -1;
end

if y0 < y1
    stepY = 1;
else
    stepY = -1;
end

errorValue = dx-dy;

maximumPoints = dx+dy+2;
rows = zeros(maximumPoints,1);
columns = zeros(maximumPoints,1);
pointCount = 0;

while true
    pointCount = pointCount+1;
    rows(pointCount) = y0;
    columns(pointCount) = x0;

    if x0 == x1 && y0 == y1
        break;
    end

    doubledError = 2*errorValue;

    if doubledError > -dy
        errorValue = errorValue-dy;
        x0 = x0+stepX;
    end

    if doubledError < dx
        errorValue = errorValue+dx;
        y0 = y0+stepY;
    end
end

rows = rows(1:pointCount);
columns = columns(1:pointCount);
end
