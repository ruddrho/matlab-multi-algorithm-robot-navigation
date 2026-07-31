function environment = createEnvironment(cfg)
%CREATEENVIRONMENT Create a realistic indoor search-and-rescue world.
%
% The environment contains:
% - boundary walls,
% - warehouse-style rectangular obstacles,
% - cylindrical obstacles,
% - irregular polygon obstacles,
% - narrow but navigable corridors,
% - additional randomly placed non-overlapping obstacles.
%
% No navigation path is generated or stored.

environment.worldWidth = cfg.environment.worldWidth;
environment.worldHeight = cfg.environment.worldHeight;

%% Fixed rectangular warehouse obstacles: [x y width height]
environment.rectangles = [ ...
    4.20  1.20  1.20  4.20;
    4.20  9.60  1.20  4.80;
    8.50  4.30  1.30  4.20;
    8.50 11.60  1.30  3.00;
   12.80  1.30  1.20  4.70;
   12.80  9.20  1.20  5.20;
   17.10  4.00  1.25  4.10;
   17.10 11.30  1.25  3.30];

%% Fixed cylindrical obstacles: [centerX centerY radius]
environment.circles = [ ...
    2.90  8.20 0.65;
    6.70  2.60 0.58;
    6.80 12.90 0.70;
   11.20  7.40 0.72;
   15.50  7.10 0.62;
   19.80  2.80 0.62;
   19.40 12.80 0.72];

%% Irregular polygon obstacles
environment.polygons = cell(3,1);

environment.polygons{1} = [ ...
    6.20 7.10;
    7.15 6.45;
    7.85 7.20;
    7.45 8.20;
    6.35 8.10];

environment.polygons{2} = [ ...
   10.60 12.40;
   11.65 11.80;
   12.35 12.55;
   12.15 13.65;
   11.05 13.80;
   10.35 13.10];

environment.polygons{3} = [ ...
   15.00 2.20;
   16.00 1.75;
   16.70 2.55;
   16.35 3.55;
   15.25 3.45;
   14.65 2.85];

%% Add random non-overlapping obstacles
randomRectangles = zeros(0,4);
randomCircles = zeros(0,3);

maximumAttempts = 400;
attempt = 0;

while size(randomRectangles,1) < ...
        cfg.environment.randomRectangleCount && ...
        attempt < maximumAttempts
    attempt = attempt+1;

    widthValue = ...
        cfg.environment.randomRectangleSize(1)+ ...
        rand*diff(cfg.environment.randomRectangleSize);

    heightValue = ...
        cfg.environment.randomRectangleSize(1)+ ...
        rand*diff(cfg.environment.randomRectangleSize);

    candidate = [ ...
        1.0+rand*(environment.worldWidth-widthValue-2.0), ...
        1.0+rand*(environment.worldHeight-heightValue-2.0), ...
        widthValue,heightValue];

    if obstacleCandidateIsValid( ...
            candidate,'rectangle',environment, ...
            randomRectangles,randomCircles,cfg)
        randomRectangles(end+1,:) = candidate; %#ok<AGROW>
    end
end

attempt = 0;

while size(randomCircles,1) < ...
        cfg.environment.randomCircleCount && ...
        attempt < maximumAttempts
    attempt = attempt+1;

    radiusValue = ...
        cfg.environment.randomCircleRadius(1)+ ...
        rand*diff(cfg.environment.randomCircleRadius);

    candidate = [ ...
        1.0+radiusValue+ ...
        rand*(environment.worldWidth-2*(1.0+radiusValue)), ...
        1.0+radiusValue+ ...
        rand*(environment.worldHeight-2*(1.0+radiusValue)), ...
        radiusValue];

    if obstacleCandidateIsValid( ...
            candidate,'circle',environment, ...
            randomRectangles,randomCircles,cfg)
        randomCircles(end+1,:) = candidate; %#ok<AGROW>
    end
end

environment.rectangles = [ ...
    environment.rectangles;randomRectangles];

environment.circles = [ ...
    environment.circles;randomCircles];

%% Precompute all line segments for exact LiDAR ray intersection
segments = [ ...
    0 0 environment.worldWidth 0;
    environment.worldWidth 0 ...
        environment.worldWidth environment.worldHeight;
    environment.worldWidth environment.worldHeight ...
        0 environment.worldHeight;
    0 environment.worldHeight 0 0];

for index = 1:size(environment.rectangles,1)
    rectangleData = environment.rectangles(index,:);
    x1 = rectangleData(1);
    y1 = rectangleData(2);
    x2 = x1+rectangleData(3);
    y2 = y1+rectangleData(4);

    segments = [segments; ...
        x1 y1 x2 y1;
        x2 y1 x2 y2;
        x2 y2 x1 y2;
        x1 y2 x1 y1]; %#ok<AGROW>
end

for index = 1:numel(environment.polygons)
    polygon = environment.polygons{index};
    nextPolygon = [polygon(2:end,:);polygon(1,:)];

    segments = [segments; ...
        polygon(:,1) polygon(:,2) ...
        nextPolygon(:,1) nextPolygon(:,2)]; %#ok<AGROW>
end

environment.segments = segments;
end

function valid = obstacleCandidateIsValid( ...
    candidate,candidateType,environment, ...
    randomRectangles,randomCircles,cfg)

margin = cfg.environment.minimumObstacleSeparation;
startPoint = cfg.robot.startPose(1:2)';

if strcmpi(candidateType,'rectangle')
    candidateCenter = [ ...
        candidate(1)+candidate(3)/2, ...
        candidate(2)+candidate(4)/2];

    candidateRadius = ...
        0.5*hypot(candidate(3),candidate(4));
else
    candidateCenter = candidate(1:2);
    candidateRadius = candidate(3);
end

if norm(candidateCenter-startPoint) < ...
        candidateRadius+cfg.environment.startClearance
    valid = false;
    return;
end

for index = 1:size(environment.rectangles,1)
    obstacle = environment.rectangles(index,:);
    obstacleCenter = [ ...
        obstacle(1)+obstacle(3)/2, ...
        obstacle(2)+obstacle(4)/2];
    obstacleRadius = ...
        0.5*hypot(obstacle(3),obstacle(4));

    if norm(candidateCenter-obstacleCenter) < ...
            candidateRadius+obstacleRadius+margin
        valid = false;
        return;
    end
end

for index = 1:size(environment.circles,1)
    obstacle = environment.circles(index,:);

    if norm(candidateCenter-obstacle(1:2)) < ...
            candidateRadius+obstacle(3)+margin
        valid = false;
        return;
    end
end

for index = 1:numel(environment.polygons)
    polygon = environment.polygons{index};
    polygonCenter = mean(polygon,1);
    polygonRadius = max(sqrt(sum( ...
        (polygon-polygonCenter).^2,2)));

    if norm(candidateCenter-polygonCenter) < ...
            candidateRadius+polygonRadius+margin
        valid = false;
        return;
    end
end

for index = 1:size(randomRectangles,1)
    obstacle = randomRectangles(index,:);
    obstacleCenter = [ ...
        obstacle(1)+obstacle(3)/2, ...
        obstacle(2)+obstacle(4)/2];
    obstacleRadius = ...
        0.5*hypot(obstacle(3),obstacle(4));

    if norm(candidateCenter-obstacleCenter) < ...
            candidateRadius+obstacleRadius+margin
        valid = false;
        return;
    end
end

for index = 1:size(randomCircles,1)
    obstacle = randomCircles(index,:);

    if norm(candidateCenter-obstacle(1:2)) < ...
            candidateRadius+obstacle(3)+margin
        valid = false;
        return;
    end
end

valid = true;
end
