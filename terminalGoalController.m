function [command,predictedTrajectory,debug,active] = ...
    terminalGoalController(pose,currentCommand,goal,scan,cfg)
%TERMINALGOALCONTROLLER Fast, smooth final approach to the clicked goal.
%
% This controller does not use a path, waypoint, or global planner. It is
% activated only when:
%   1) the goal is inside the configured terminal radius, and
%   2) the current LiDAR scan confirms a collision-free direct corridor.
%
% It removes slow DWA creeping near the destination while preserving
% acceleration limits, heading alignment, and obstacle safety.

distanceToGoal = norm(pose(1:2)'-goal);
goalHeading = atan2(goal(2)-pose(2),goal(1)-pose(1));
headingError = utilities('wrapAngle',goalHeading-pose(3));
localGoalBearing = headingError;

% Check a LiDAR corridor wide enough for the circular robot body.
corridorHalfAngle = atan2( ...
    cfg.robot.radius+cfg.goal.terminalCorridorBuffer, ...
    max(distanceToGoal,0.10));

angularDifference = abs(utilities( ...
    'wrapAngle',scan.localAngles-localGoalBearing));

corridorMask = angularDifference <= corridorHalfAngle;

if ~any(corridorMask)
    [~,nearestRay] = min(angularDifference);
    corridorMask(nearestRay) = true;
end

corridorRange = min(scan.ranges(corridorMask));

directCorridorClear = corridorRange > ...
    distanceToGoal+cfg.goal.terminalCorridorBuffer;

active = distanceToGoal <= cfg.goal.terminalModeRadius && ...
    directCorridorClear;

if ~active
    command = currentCommand;
    predictedTrajectory = pose';
    debug = defaultDebugLocal(scan,false);
    return;
end

%% Smooth heading alignment
targetAngularVelocity = ...
    cfg.goal.terminalHeadingGain*headingError;

targetAngularVelocity = utilities( ...
    'clamp',targetAngularVelocity, ...
    -cfg.robot.maximumAngularVelocity, ...
     cfg.robot.maximumAngularVelocity);

maximumAngularChange = ...
    cfg.robot.maximumAngularAcceleration* ...
    cfg.simulation.timeStep;

angularVelocity = utilities( ...
    'clamp',targetAngularVelocity, ...
    currentCommand(2)-maximumAngularChange, ...
    currentCommand(2)+maximumAngularChange);

%% Fast but controlled final translation
remainingDistance = max( ...
    distanceToGoal-0.45*cfg.goal.tolerance,0);

targetLinearVelocity = min( ...
    cfg.goal.terminalMaximumSpeed, ...
    cfg.goal.terminalDistanceGain*remainingDistance);

if distanceToGoal > cfg.goal.tolerance
    targetLinearVelocity = max( ...
        targetLinearVelocity, ...
        cfg.goal.terminalMinimumSpeed);
else
    targetLinearVelocity = 0;
end

% Rotate first when the goal is significantly off the robot's heading.
if abs(headingError) >= cfg.goal.terminalRotateFirstAngle
    targetLinearVelocity = 0;
else
    headingFactor = max(0.15, ...
        1-abs(headingError)/ ...
        cfg.goal.terminalHeadingSlowdownAngle);

    targetLinearVelocity = ...
        targetLinearVelocity*headingFactor;
end

maximumAccelerationChange = ...
    cfg.robot.maximumLinearAcceleration* ...
    cfg.simulation.timeStep;

maximumDecelerationChange = ...
    cfg.robot.maximumLinearDeceleration* ...
    cfg.simulation.timeStep;

if targetLinearVelocity >= currentCommand(1)
    linearVelocity = min( ...
        targetLinearVelocity, ...
        currentCommand(1)+maximumAccelerationChange);
else
    linearVelocity = max( ...
        targetLinearVelocity, ...
        currentCommand(1)-maximumDecelerationChange);
end

linearVelocity = utilities( ...
    'clamp',linearVelocity, ...
    cfg.robot.minimumLinearVelocity, ...
    cfg.goal.terminalMaximumSpeed);

command = [linearVelocity;angularVelocity];

%% Short local prediction for display and safety information
predictionStep = cfg.dwa.predictionTimeStep;
predictionCount = max(1,ceil( ...
    cfg.goal.terminalPredictionTime/predictionStep));

predictedTrajectory = zeros(predictionCount+1,3);
predictedTrajectory(1,:) = pose';

predictedPose = pose;

for index = 1:predictionCount
    predictedPose = robotKinematics( ...
        predictedPose,command,predictionStep);

    predictedTrajectory(index+1,:) = predictedPose';
end

debug = defaultDebugLocal(scan,true);
debug.bestScore = 1;
debug.validCandidateCount = 1;
debug.progressScore = 1;
debug.minimumClearance = min(scan.ranges);
debug.terminalMode = true;
end

function debug = defaultDebugLocal(scan,terminalMode)
%DEFAULTDEBUGLOCAL Provide the same debug interface used by DWA.

frontMask = abs(scan.localAngles) < 35*pi/180;
leftMask = scan.localAngles > 20*pi/180 & ...
    scan.localAngles < 150*pi/180;
rightMask = scan.localAngles < -20*pi/180 & ...
    scan.localAngles > -150*pi/180;

debug.bestScore = 0;
debug.validCandidateCount = 0;
debug.minimumClearance = min(scan.ranges);
debug.progressScore = 0;
debug.openingFront = mean(scan.ranges(frontMask));
debug.openingLeft = mean(scan.ranges(leftMask));
debug.openingRight = mean(scan.ranges(rightMask));
debug.recoveryActive = false;
debug.terminalMode = terminalMode;
end
