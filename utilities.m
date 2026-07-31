function varargout = utilities(action,varargin)
%UTILITIES Shared configuration and compatibility helpers.

switch lower(action)
    case 'defaultconfig'
        varargout{1} = defaultConfigLocal();

    case 'wrapangle'
        angle = varargin{1};
        varargout{1} = mod(angle+pi,2*pi)-pi;

    case 'clamp'
        value = varargin{1};
        lowerLimit = varargin{2};
        upperLimit = varargin{3};
        varargout{1} = min(max( ...
            value,lowerLimit),upperLimit);

    case 'pointsegmentdistance'
        point = varargin{1};
        segmentStart = varargin{2};
        segmentEnd = varargin{3};

        segmentVector = ...
            segmentEnd-segmentStart;

        denominator = dot( ...
            segmentVector,segmentVector);

        if denominator <= eps
            varargout{1} = ...
                norm(point-segmentStart);
        else
            parameter = dot( ...
                point-segmentStart, ...
                segmentVector)/denominator;

            parameter = min(max(parameter,0),1);

            closestPoint = ...
                segmentStart+ ...
                parameter*segmentVector;

            varargout{1} = ...
                norm(point-closestPoint);
        end

    case 'capturefigurergb'
        figureHandle = varargin{1};
        temporaryImage = varargin{2};

        if isempty(figureHandle) || ...
                ~ishandle(figureHandle) || ...
                ~strcmpi(get(figureHandle,'Type'),'figure')
            error('A valid figure handle is required.');
        end

        set(0,'CurrentFigure',figureHandle);
        drawnow;
        set(figureHandle,'PaperPositionMode','auto');

        if exist(temporaryImage,'file')
            delete(temporaryImage);
        end

        print(temporaryImage,'-dpng','-r90');
        rgbImage = imread(temporaryImage);

        if ndims(rgbImage) == 2
            rgbImage = repmat(rgbImage,[1 1 3]);
        elseif size(rgbImage,3) > 3
            rgbImage = rgbImage(:,:,1:3);
        end

        if ~isa(rgbImage,'uint8')
            if isfloat(rgbImage)
                maximumValue = max(rgbImage(:));

                if maximumValue <= 1
                    rgbImage = uint8(round( ...
                        255*min(max(rgbImage,0),1)));
                else
                    rgbImage = uint8(min(max( ...
                        round(rgbImage),0),255));
                end
            else
                rgbImage = uint8(rgbImage);
            end
        end

        varargout{1} = rgbImage;

    case 'fitframe'
        inputImage = varargin{1};
        targetSize = varargin{2};

        targetHeight = targetSize(1);
        targetWidth = targetSize(2);

        outputImage = uint8( ...
            255*ones(targetHeight, ...
            targetWidth,3));

        copyHeight = min( ...
            targetHeight,size(inputImage,1));

        copyWidth = min( ...
            targetWidth,size(inputImage,2));

        sourceRowStart = floor( ...
            (size(inputImage,1)-copyHeight)/2)+1;

        sourceColumnStart = floor( ...
            (size(inputImage,2)-copyWidth)/2)+1;

        targetRowStart = floor( ...
            (targetHeight-copyHeight)/2)+1;

        targetColumnStart = floor( ...
            (targetWidth-copyWidth)/2)+1;

        outputImage( ...
            targetRowStart: ...
            targetRowStart+copyHeight-1, ...
            targetColumnStart: ...
            targetColumnStart+copyWidth-1,:) = ...
            inputImage( ...
            sourceRowStart: ...
            sourceRowStart+copyHeight-1, ...
            sourceColumnStart: ...
            sourceColumnStart+copyWidth-1,:);

        varargout{1} = outputImage;

    case 'safelast'
        vector = varargin{1};
        defaultValue = varargin{2};

        if isempty(vector)
            varargout{1} = defaultValue;
        else
            varargout{1} = vector(end);
        end

    case 'safemean'
        vector = varargin{1};
        defaultValue = varargin{2};

        if isempty(vector)
            varargout{1} = defaultValue;
        else
            varargout{1} = mean(vector);
        end

    case 'safemax'
        vector = varargin{1};
        defaultValue = varargin{2};

        if isempty(vector)
            varargout{1} = defaultValue;
        else
            varargout{1} = max(vector);
        end

    case 'safemin'
        vector = varargin{1};
        defaultValue = varargin{2};

        if isempty(vector)
            varargout{1} = defaultValue;
        else
            varargout{1} = min(vector);
        end

    case 'saferms'
        vector = varargin{1};
        defaultValue = varargin{2};

        if isempty(vector)
            varargout{1} = defaultValue;
        else
            varargout{1} = ...
                sqrt(mean(vector.^2));
        end

    otherwise
        error('Unknown utilities action: %s',action);
end
end

function cfg = defaultConfigLocal()
%DEFAULTCONFIGLOCAL Complete project configuration.

%% Environment
cfg.environment.worldWidth = 22.0;
cfg.environment.worldHeight = 16.0;
cfg.environment.randomSeed = 37;
cfg.environment.randomRectangleCount = 2;
cfg.environment.randomCircleCount = 2;
cfg.environment.randomRectangleSize = [0.65 1.05];
cfg.environment.randomCircleRadius = [0.30 0.48];
cfg.environment.minimumObstacleSeparation = 0.55;
cfg.environment.startClearance = 2.0;

%% Robot geometry and differential-drive limits
cfg.robot.startPose = [1.50 1.50 0.20];
cfg.robot.radius = 0.48;
cfg.robot.wheelBase = 0.72;
cfg.robot.wheelRadius = 0.11;
cfg.robot.wheelLength = 0.34;
cfg.robot.wheelWidth = 0.12;
cfg.robot.headingArrowLength = 0.72;

cfg.robot.minimumLinearVelocity = 0.0;
cfg.robot.maximumLinearVelocity = 5.70;  % 2x faster than previous project
cfg.robot.maximumAngularVelocity = 3.70;
cfg.robot.maximumLinearAcceleration = 3.90;
cfg.robot.maximumLinearDeceleration = 5.10;
cfg.robot.maximumAngularAcceleration = 5.20;

%% Simulation
cfg.simulation.timeStep = 0.08;
cfg.simulation.maximumTime = 180.0;

%% LiDAR
cfg.lidar.numberOfRays = 181;
cfg.lidar.maximumRange = 8.00;

%% DWA sampling and prediction
cfg.dwa.linearVelocitySamples = 10;
cfg.dwa.angularVelocitySamples = 21;
cfg.dwa.predictionTime = 1.40;
cfg.dwa.predictionTimeStep = 0.10;
cfg.dwa.reactionTime = 0.20;
cfg.dwa.lowSpeedSafetyThreshold = 0.20;
cfg.dwa.stoppingScoreScale = 1.10;

cfg.dwa.weights.heading = 0.27;
cfg.dwa.weights.progress = 0.32;
cfg.dwa.weights.clearance = 0.22;
cfg.dwa.weights.velocity = 0.10;
cfg.dwa.weights.smoothness = 0.06;
cfg.dwa.weights.steering = 0.04;
cfg.dwa.weights.braking = 0.08;
cfg.dwa.weights.goalProximity = 0.05;
cfg.dwa.weights.recovery = 0.00;


%% Hybrid global path planning and continuous path following
cfg.path.gridResolution = 8;          % occupancy cells per metre
cfg.path.inflationMargin = 0.12;      % extra safety around robot body
cfg.path.resampleSpacing = 0.08;   % denser reference line      % path sample spacing (m)
cfg.path.searchWindow = 160;          % nearest-point search window
cfg.path.minimumCruiseSpeed = 0.90;
cfg.path.maximumCrossTrackError = 1.60;
cfg.path.replanTimeout = 5.0;         % seconds without path progress
cfg.path.progressIndexThreshold = 0.65; % arc-length progress (m)
cfg.path.minimumReplanInterval = 1.5;
cfg.path.cornerSlowdownGain = 0.72;
cfg.path.showGlobalPath = true;
cfg.path.lookAheadMinimum = 0.45;
cfg.path.lookAheadMaximum = 1.25;
cfg.path.lookAheadSpeedGain = 0.18;
cfg.path.lookAheadErrorGain = 1.10;
cfg.path.projectionSearchDistance = 9.0;
cfg.path.projectionBacktrackSegments = 4;
cfg.path.maximumBackwardProjection = 0.35;
cfg.path.backwardProjectionPenalty = 0.25;
cfg.path.cornerPreviewDistance = 2.0;
cfg.path.minimumCornerSpeedFactor = 0.30;
cfg.path.minimumErrorSpeedFactor = 0.20;
cfg.path.errorSlowdownGain = 1.05;
cfg.path.finalSlowdownDistance = 3.5;
cfg.path.finalSlowdownGain = 0.75;
cfg.path.trackingWeight = 0.95;
cfg.path.trackingSigma = 0.24;
cfg.path.guidanceActive = false;
cfg.path.referencePoint = [0 0];
cfg.path.referenceTangent = [1 0];
cfg.path.headingTrackingWeight = 0.34;
cfg.path.progressTrackingWeight = 0.24;
cfg.path.crossTrackCorrectionGain = 1.10;
cfg.path.maximumCorrectionAngle = 30*pi/180;
cfg.path.trajectorySampleCount = 25;
cfg.path.referenceWindowBehind = 0.30;
cfg.path.referenceWindowAhead = 4.50;
cfg.path.maximumTrackingSpeedError = 0.75;
cfg.path.routeLockEnabled = true;
cfg.path.maximumPredictionDeviation = 0.38;
cfg.path.routeLockRecoveryMargin = 0.12;
cfg.path.maximumFinalDeviationGrowth = 0.03;
cfg.path.currentCrossTrackError = 0;


%% Multi-algorithm planning comparison
cfg.compare.enabled = true;
cfg.compare.rrt.maximumIterations = 5000;
cfg.compare.rrt.stepSize = 0.70;
cfg.compare.rrt.goalBias = 0.18;
cfg.compare.rrt.goalThreshold = 0.95;
cfg.compare.rrtStar.maximumIterations = 6500;
cfg.compare.rrtStar.stepSize = 0.65;
cfg.compare.rrtStar.goalBias = 0.20;
cfg.compare.rrtStar.goalThreshold = 0.90;
cfg.compare.rrtStar.rewireRadius = 1.40;
cfg.compare.prm.sampleCount = 450;
cfg.compare.prm.connectionCount = 14;
cfg.compare.prm.connectionRadius = 3.2;

cfg.compare.animation.enabled = true;
cfg.compare.animation.recordVideo = true;
cfg.compare.animation.frameRate = 24;
cfg.compare.animation.explorationFrames = 72;
cfg.compare.animation.routeRevealFrames = 28;
cfg.compare.animation.finalHoldSeconds = 1.0;


%% Simultaneous autonomous robots for planner comparison
cfg.compare.navigation.enabled = true;
cfg.compare.navigation.recordVideo = true;
cfg.compare.navigation.frameRate = 15;
cfg.compare.navigation.videoCaptureStride = 2;
cfg.compare.navigation.maximumTime = 150.0;
cfg.compare.navigation.finalHoldSeconds = 1.5;
cfg.compare.navigation.lidarDisplayStride = 6;

% The same DWA settings are used by all five comparison robots. Sampling
% is moderately reduced so five independent controllers can run together.
cfg.compare.navigation.linearVelocitySamples = 6;
cfg.compare.navigation.angularVelocitySamples = 13;
cfg.compare.navigation.predictionTime = 1.20;
cfg.compare.navigation.predictionTimeStep = 0.12;

%% Goal approach
cfg.goal.tolerance = 0.30;
cfg.goal.slowingRadius = 1.35;
cfg.goal.approachGain = 4.50;
cfg.goal.terminalDistanceWeight = 0.30;
cfg.goal.terminalSpeedWeight = 0.12;

% Fast final docking settings. This controller is enabled only when the
% final direct corridor is confirmed free by the current LiDAR scan.
cfg.goal.terminalModeRadius = 1.30;
cfg.goal.terminalMaximumSpeed = 2.40;
cfg.goal.terminalMinimumSpeed = 0.48;
cfg.goal.terminalDistanceGain = 6.90;
cfg.goal.terminalHeadingGain = 2.60;
cfg.goal.terminalRotateFirstAngle = 38*pi/180;
cfg.goal.terminalHeadingSlowdownAngle = 62*pi/180;
cfg.goal.terminalCorridorBuffer = 0.08;
cfg.goal.terminalPredictionTime = 1.10;

%% Safety
cfg.safety.minimumGeometricClearance = 0.08;
cfg.safety.clearanceBuffer = 0.20;
cfg.safety.emergencyTurnRate = 2.20;

%% Local-minimum recovery — still purely local and LiDAR based
cfg.recovery.memorySamples = 70;
cfg.recovery.minimumSamples = 50;
cfg.recovery.minimumProgress = 0.18;
cfg.recovery.maximumDuration = 4.0;
cfg.recovery.minimumTurnRate = 0.70;
cfg.recovery.maximumRecoveryVelocity = 0.40;
cfg.recovery.exitProgressScore = 0.63;

cfg.recovery.weights.heading = 0.08;
cfg.recovery.weights.progress = 0.10;
cfg.recovery.weights.clearance = 0.33;
cfg.recovery.weights.velocity = 0.02;
cfg.recovery.weights.smoothness = 0.08;
cfg.recovery.weights.steering = 0.02;
cfg.recovery.weights.braking = 0.12;
cfg.recovery.weights.goalProximity = 0.05;
cfg.recovery.weights.recovery = 0.20;

%% SLAM-live occupancy mapping
cfg.slam.resolution = 8;
cfg.slam.logOddsOccupied = 0.90;
cfg.slam.logOddsFree = -0.34;
cfg.slam.minimumLogOdds = -4.0;
cfg.slam.maximumLogOdds = 4.0;
cfg.slam.mappingRayStride = 1;
cfg.slam.occupiedCellRadius = 1;

cfg.slam.minimumScanMatchUpdates = 5;
cfg.slam.minimumScanMatchHits = 8;
cfg.slam.scanMatchRayStride = 4;
cfg.slam.scanMatchTranslation = 0.10;
cfg.slam.scanMatchRotation = 2.0*pi/180;
cfg.slam.scanMatchTranslationSamples = 3;
cfg.slam.scanMatchRotationSamples = 3;
cfg.slam.scanMatchTranslationPenalty = 0.55;
cfg.slam.scanMatchRotationPenalty = 0.35;

cfg.slam.linearOdometryNoiseStd = 0.010;
cfg.slam.angularOdometryNoiseStd = 0.008;
cfg.slam.worldFrameAnchorEnabled = true;
cfg.slam.worldFrameAnchorGain = 1.0;

%% Visualization
cfg.visualization.updateStride = 1;
cfg.visualization.trajectoryDotStride = 3;
cfg.visualization.showLocalPredictionDots = false;

cfg.visualization.robotBodyColor = [0.90 0.16 0.20];
cfg.visualization.wheelColor = [0.05 0.05 0.06];
cfg.visualization.headingColor = [0.05 0.25 0.90];
cfg.visualization.lidarRayColor = [0.18 0.70 0.46];
cfg.visualization.lidarHitColor = [0.90 0.12 0.12];
cfg.visualization.trajectoryDotColor = [0.15 0.35 0.85];
cfg.visualization.predictionDotColor = [0.70 0.20 0.75];

%% Output
cfg.output.resultsFolder = fullfile(pwd,'results');
end
