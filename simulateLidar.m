function scan = simulateLidar( ...
    pose,environment,localAngles,cfg)
%SIMULATELIDAR Exact 360-degree LiDAR ray casting.
%
% Rectangle, wall, and polygon intersections use exact ray-to-segment
% calculations. Cylindrical obstacles use exact ray-to-circle
% intersections.

numberOfRays = numel(localAngles);

ranges = cfg.lidar.maximumRange* ...
    ones(1,numberOfRays);

hitMask = false(1,numberOfRays);
hitPoints = zeros(numberOfRays,2);

rayOrigin = pose(1:2)';

for rayIndex = 1:numberOfRays
    globalAngle = pose(3)+localAngles(rayIndex);
    rayDirection = [cos(globalAngle) sin(globalAngle)];

    nearestDistance = cfg.lidar.maximumRange;
    hitDetected = false;

    %% Line segments: walls, rectangles, irregular polygons
    for segmentIndex = 1:size(environment.segments,1)
        segment = environment.segments(segmentIndex,:);

        segmentStart = segment(1:2);
        segmentVector = ...
            segment(3:4)-segment(1:2);

        denominator = cross2D( ...
            rayDirection,segmentVector);

        if abs(denominator) < 1e-12
            continue;
        end

        originDifference = ...
            segmentStart-rayOrigin;

        rayDistance = cross2D( ...
            originDifference,segmentVector) / ...
            denominator;

        segmentParameter = cross2D( ...
            originDifference,rayDirection) / ...
            denominator;

        if rayDistance >= 0 && ...
                segmentParameter >= 0 && ...
                segmentParameter <= 1 && ...
                rayDistance < nearestDistance
            nearestDistance = rayDistance;
            hitDetected = true;
        end
    end

    %% Cylindrical obstacles
    for circleIndex = 1:size(environment.circles,1)
        circle = environment.circles(circleIndex,:);
        circleCenter = circle(1:2);
        circleRadius = circle(3);

        offset = rayOrigin-circleCenter;

        quadraticB = 2*dot( ...
            rayDirection,offset);

        quadraticC = dot(offset,offset)- ...
            circleRadius^2;

        discriminant = quadraticB^2-4*quadraticC;

        if discriminant < 0
            continue;
        end

        root1 = (-quadraticB- ...
            sqrt(discriminant))/2;
        root2 = (-quadraticB+ ...
            sqrt(discriminant))/2;

        positiveRoots = [root1 root2];
        positiveRoots = positiveRoots( ...
            positiveRoots >= 0);

        if isempty(positiveRoots)
            continue;
        end

        rayDistance = min(positiveRoots);

        if rayDistance < nearestDistance
            nearestDistance = rayDistance;
            hitDetected = true;
        end
    end

    nearestDistance = min( ...
        nearestDistance,cfg.lidar.maximumRange);

    ranges(rayIndex) = nearestDistance;
    hitMask(rayIndex) = hitDetected && ...
        nearestDistance < ...
        cfg.lidar.maximumRange-1e-9;

    hitPoints(rayIndex,:) = ...
        rayOrigin+nearestDistance*rayDirection;
end

scan.localAngles = localAngles;
scan.ranges = ranges;
scan.hitMask = hitMask;
scan.hitPoints = hitPoints;
scan.globalAngles = pose(3)+localAngles;
end

function value = cross2D(vectorA,vectorB)
value = vectorA(1)*vectorB(2)- ...
    vectorA(2)*vectorB(1);
end
