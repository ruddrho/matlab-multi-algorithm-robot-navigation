function [localTarget,newProgress,pathInfo] = ...
    pathFollower(pose,path,progress,cfg,currentVelocity)
%PATHFOLLOWER Accurate continuous polyline projection and look-ahead.
%
% Unlike nearest-sample tracking, this function projects the robot onto
% actual path segments. Progress is stored as travelled arc length in
% metres, so it remains continuous and never jumps between discrete path
% samples. The look-ahead distance changes with speed and cross-track
% error to improve both high-speed stability and corner accuracy.

if nargin < 5
    currentVelocity = 0;
end

robotPosition = pose(1:2)';
numberOfPoints = size(path,1);

if numberOfPoints < 2
    localTarget = path(1,:);
    newProgress = 0;

    pathInfo.crossTrackError = norm(robotPosition-localTarget);
    pathInfo.signedCrossTrackError = 0;
    pathInfo.remainingDistance = pathInfo.crossTrackError;
    pathInfo.progressFraction = 1;
    pathInfo.speedLimit = cfg.path.minimumCruiseSpeed;
    pathInfo.targetIndex = 1;
    pathInfo.nearestIndex = 1;
    pathInfo.headingChange = 0;
    pathInfo.projectedPoint = localTarget;
    pathInfo.pathTangent = [cos(pose(3)) sin(pose(3))];
    pathInfo.lookAheadDistance = 0;
    pathInfo.progressArcLength = 0;
    return;
end

segmentVectors = diff(path,1,1);
segmentLengths = sqrt(sum(segmentVectors.^2,2));
segmentLengths = max(segmentLengths,eps);
cumulativeLength = [0;cumsum(segmentLengths)];
totalLength = cumulativeLength(end);

progress = utilities('clamp',progress,0,totalLength);

currentSegment = find(cumulativeLength <= progress,1,'last');

if isempty(currentSegment)
    currentSegment = 1;
end

currentSegment = min(currentSegment,numberOfPoints-1);

searchStart = max( ...
    1,currentSegment-cfg.path.projectionBacktrackSegments);

maximumSearchArc = min( ...
    totalLength,progress+cfg.path.projectionSearchDistance);

searchEnd = find( ...
    cumulativeLength(1:end-1) <= maximumSearchArc, ...
    1,'last');

if isempty(searchEnd)
    searchEnd = numberOfPoints-1;
end

searchEnd = min(searchEnd,numberOfPoints-1);

bestDistance = inf;
bestArcLength = progress;
bestProjection = path(currentSegment,:);
bestSegment = currentSegment;
bestParameter = 0;

for segmentIndex = searchStart:searchEnd
    pointA = path(segmentIndex,:);
    segmentVector = segmentVectors(segmentIndex,:);
    squaredLength = dot(segmentVector,segmentVector);

    parameter = dot( ...
        robotPosition-pointA,segmentVector) / ...
        max(squaredLength,eps);

    parameter = utilities('clamp',parameter,0,1);

    projection = pointA+parameter*segmentVector;
    projectionArc = cumulativeLength(segmentIndex)+ ...
        parameter*segmentLengths(segmentIndex);

    if projectionArc < ...
            progress-cfg.path.maximumBackwardProjection
        continue;
    end

    distance = norm(robotPosition-projection);

    % A small backward-progress penalty prevents switching to a nearby
    % parallel segment behind the robot.
    backwardPenalty = cfg.path.backwardProjectionPenalty* ...
        max(0,progress-projectionArc);

    score = distance+backwardPenalty;

    if score < bestDistance
        bestDistance = score;
        bestArcLength = projectionArc;
        bestProjection = projection;
        bestSegment = segmentIndex;
        bestParameter = parameter;
    end
end

newProgress = max(progress,bestArcLength);

% Recompute projection at monotonic progress when the closest geometric
% point was slightly behind the already achieved progress.
[projectedPoint,projectionSegment,projectionParameter] = ...
    interpolateAtArcLocal( ...
    path,segmentLengths,cumulativeLength,newProgress);

pathTangent = segmentVectors(projectionSegment,:) / ...
    segmentLengths(projectionSegment);

normalVector = [-pathTangent(2) pathTangent(1)];

signedCrossTrackError = dot( ...
    robotPosition-projectedPoint,normalVector);

crossTrackError = abs(signedCrossTrackError);

%% Dynamic look-ahead
lookAheadDistance = ...
    cfg.path.lookAheadMinimum + ...
    cfg.path.lookAheadSpeedGain*abs(currentVelocity) - ...
    cfg.path.lookAheadErrorGain*crossTrackError;

lookAheadDistance = utilities( ...
    'clamp',lookAheadDistance, ...
    cfg.path.lookAheadMinimum, ...
    cfg.path.lookAheadMaximum);

targetArcLength = min( ...
    totalLength,newProgress+lookAheadDistance);

[geometricTarget,targetSegment,~] = ...
    interpolateAtArcLocal( ...
    path,segmentLengths,cumulativeLength,targetArcLength);

% Cross-track correction rotates the desired target direction toward the
% path centreline. The correction is bounded so the robot still turns
% smoothly and does not oscillate around the route.
targetVector = geometricTarget-robotPosition;
targetDistance = max(norm(targetVector),eps);
targetHeadingRaw = atan2(targetVector(2),targetVector(1));

correctionAngle = atan2( ...
    cfg.path.crossTrackCorrectionGain*signedCrossTrackError, ...
    max(abs(currentVelocity),0.35));

correctionAngle = utilities( ...
    'clamp',correctionAngle, ...
    -cfg.path.maximumCorrectionAngle, ...
     cfg.path.maximumCorrectionAngle);

% IMPORTANT SIGN:
% A positive signed error means the robot is on the left side of the
% directed route and must steer right. A negative error means it is on
% the right side and must steer left. Therefore the correction is
% SUBTRACTED from the raw target heading.
correctedHeading = targetHeadingRaw-correctionAngle;

localTarget = robotPosition+ ...
    targetDistance*[cos(correctedHeading) sin(correctedHeading)];

%% Curvature preview and speed limit
previewArcLength = min( ...
    totalLength,targetArcLength+cfg.path.cornerPreviewDistance);

[~,previewSegment,~] = ...
    interpolateAtArcLocal( ...
    path,segmentLengths,cumulativeLength,previewArcLength);

targetTangent = segmentVectors(targetSegment,:) / ...
    segmentLengths(targetSegment);

previewTangent = segmentVectors(previewSegment,:) / ...
    segmentLengths(previewSegment);

targetHeading = atan2(targetTangent(2),targetTangent(1));
previewHeading = atan2(previewTangent(2),previewTangent(1));

headingChange = abs(utilities( ...
    'wrapAngle',previewHeading-targetHeading));

cornerFactor = max( ...
    cfg.path.minimumCornerSpeedFactor, ...
    1-cfg.path.cornerSlowdownGain*headingChange/pi);

errorFactor = max( ...
    cfg.path.minimumErrorSpeedFactor, ...
    1-cfg.path.errorSlowdownGain* ...
    crossTrackError/max(cfg.path.trackingSigma,eps));

speedLimit = cfg.robot.maximumLinearVelocity* ...
    min(cornerFactor,errorFactor);

speedLimit = max( ...
    cfg.path.minimumCruiseSpeed,speedLimit);

remainingDistance = max(0,totalLength-newProgress);

if remainingDistance < cfg.path.finalSlowdownDistance
    speedLimit = min( ...
        speedLimit, ...
        max(cfg.path.minimumCruiseSpeed, ...
        cfg.path.finalSlowdownGain*remainingDistance));
end

pathInfo.crossTrackError = crossTrackError;
pathInfo.signedCrossTrackError = signedCrossTrackError;
pathInfo.remainingDistance = remainingDistance;
pathInfo.progressFraction = newProgress/max(totalLength,eps);
pathInfo.speedLimit = speedLimit;
pathInfo.targetIndex = targetSegment;
pathInfo.nearestIndex = projectionSegment;
pathInfo.headingChange = headingChange;
pathInfo.projectedPoint = projectedPoint;
pathInfo.pathTangent = pathTangent;
pathInfo.lookAheadDistance = lookAheadDistance;
pathInfo.progressArcLength = newProgress;
pathInfo.projectionParameter = projectionParameter;
pathInfo.originalBestSegment = bestSegment;
pathInfo.originalBestParameter = bestParameter;
pathInfo.geometricTarget = geometricTarget;
pathInfo.correctedTarget = localTarget;
pathInfo.correctionAngle = correctionAngle;

referenceStartArc = max( ...
    0,newProgress-cfg.path.referenceWindowBehind);
referenceEndArc = min( ...
    totalLength,newProgress+cfg.path.referenceWindowAhead);

referenceArc = linspace( ...
    referenceStartArc,referenceEndArc, ...
    cfg.path.trajectorySampleCount)';

referencePath = zeros(numel(referenceArc),2);

for referenceIndex = 1:numel(referenceArc)
    referencePath(referenceIndex,:) = interpolateAtArcLocal( ...
        path,segmentLengths,cumulativeLength, ...
        referenceArc(referenceIndex));
end

pathInfo.referencePath = referencePath;
end

function [point,segmentIndex,parameter] = ...
    interpolateAtArcLocal( ...
    path,segmentLengths,cumulativeLength,arcLength)

numberOfPoints = size(path,1);
totalLength = cumulativeLength(end);
arcLength = utilities('clamp',arcLength,0,totalLength);

segmentIndex = find( ...
    cumulativeLength(1:end-1) <= arcLength, ...
    1,'last');

if isempty(segmentIndex)
    segmentIndex = 1;
end

segmentIndex = min(segmentIndex,numberOfPoints-1);

parameter = (arcLength-cumulativeLength(segmentIndex)) / ...
    max(segmentLengths(segmentIndex),eps);

parameter = utilities('clamp',parameter,0,1);

point = path(segmentIndex,:)+ ...
    parameter*(path(segmentIndex+1,:)-path(segmentIndex,:));
end
