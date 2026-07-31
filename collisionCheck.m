function collision = collisionCheck( ...
    pose,environment,cfg)
%COLLISIONCHECK Circular robot footprint against all physical obstacles.

robotCenter = pose(1:2)';
robotRadius = cfg.robot.radius;

%% World boundary
if robotCenter(1)-robotRadius <= 0 || ...
        robotCenter(1)+robotRadius >= ...
        environment.worldWidth || ...
        robotCenter(2)-robotRadius <= 0 || ...
        robotCenter(2)+robotRadius >= ...
        environment.worldHeight
    collision = true;
    return;
end

%% Rectangles
for index = 1:size(environment.rectangles,1)
    rectangleData = environment.rectangles(index,:);

    closestX = min(max( ...
        robotCenter(1),rectangleData(1)), ...
        rectangleData(1)+rectangleData(3));

    closestY = min(max( ...
        robotCenter(2),rectangleData(2)), ...
        rectangleData(2)+rectangleData(4));

    if hypot( ...
            robotCenter(1)-closestX, ...
            robotCenter(2)-closestY) <= ...
            robotRadius
        collision = true;
        return;
    end
end

%% Cylinders
for index = 1:size(environment.circles,1)
    circle = environment.circles(index,:);

    if norm(robotCenter-circle(1:2)) <= ...
            robotRadius+circle(3)
        collision = true;
        return;
    end
end

%% Irregular polygons
for index = 1:numel(environment.polygons)
    polygon = environment.polygons{index};

    [inside,onBoundary] = inpolygon( ...
        robotCenter(1),robotCenter(2), ...
        polygon(:,1),polygon(:,2));

    if inside || onBoundary
        collision = true;
        return;
    end

    nextPolygon = [polygon(2:end,:);polygon(1,:)];

    for edgeIndex = 1:size(polygon,1)
        edgeDistance = utilities( ...
            'pointSegmentDistance', ...
            robotCenter, ...
            polygon(edgeIndex,:), ...
            nextPolygon(edgeIndex,:));

        if edgeDistance <= robotRadius
            collision = true;
            return;
        end
    end
end

collision = false;
end
