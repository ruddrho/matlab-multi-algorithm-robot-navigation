function nextPose = robotKinematics( ...
    pose,command,timeStep)
%ROBOTKINEMATICS Exact differential-drive/unicycle pose integration.
%
% command(1) = linear velocity v
% command(2) = angular velocity omega
%
% The exact constant-command integration avoids numerical side-slipping
% and produces natural circular arcs during turns.

linearVelocity = command(1);
angularVelocity = command(2);
heading = pose(3);

if abs(angularVelocity) < 1e-9
    nextPose = [ ...
        pose(1)+linearVelocity*cos(heading)*timeStep;
        pose(2)+linearVelocity*sin(heading)*timeStep;
        heading];
else
    nextHeading = heading+ ...
        angularVelocity*timeStep;

    turningRadius = ...
        linearVelocity/angularVelocity;

    nextPose = [ ...
        pose(1)+turningRadius*( ...
        sin(nextHeading)-sin(heading));
        pose(2)-turningRadius*( ...
        cos(nextHeading)-cos(heading));
        nextHeading];
end

nextPose(3) = utilities( ...
    'wrapAngle',nextPose(3));
end
