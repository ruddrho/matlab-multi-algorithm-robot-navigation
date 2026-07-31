function slamState = initializeSlamMap(cfg)
%INITIALIZESLAMMAP Initialize a live log-odds occupancy map.
%
% The state contains a noisy odometry prediction, a lightweight
% correlative scan-matching correction, and a LiDAR inverse sensor model.

resolution = cfg.slam.resolution;
numberOfColumns = ceil(cfg.environment.worldWidth*resolution);
numberOfRows = ceil(cfg.environment.worldHeight*resolution);

slamState.resolution = resolution;
slamState.numberOfColumns = numberOfColumns;
slamState.numberOfRows = numberOfRows;
slamState.logOdds = zeros(numberOfRows,numberOfColumns);
slamState.observed = false(numberOfRows,numberOfColumns);
slamState.probability = 0.5*ones(numberOfRows,numberOfColumns);

slamState.estimatedPose = cfg.robot.startPose(:);
slamState.predictedPose = cfg.robot.startPose(:);
slamState.trajectory = cfg.robot.startPose(:)';
slamState.updateCount = 0;
slamState.exploredFraction = 0;
slamState.lastScanMatchScore = 0;
slamState.rawEstimatedPose = cfg.robot.startPose(:);
slamState.worldFramePositionError = 0;
slamState.worldFrameHeadingError = 0;
end
