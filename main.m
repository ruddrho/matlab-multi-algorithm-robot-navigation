%% main.m
% FIVE-ALGORITHM AUTONOMOUS ROBOT NAVIGATION COMPARISON
%
% Algorithms:
%   1. A*
%   2. Dijkstra
%   3. RRT
%   4. RRT*
%   5. PRM
%
% One clicked goal is synchronized across all planners. Every planner
% uses the same start pose, environment, inflated occupancy grid, robot
% model, LiDAR, DWA controller, path follower, safety limits, and SLAM
% configuration.
%
% Generated outputs:
%   results/planning_algorithms_comparison_animation.avi
%   results/planning_algorithms_animation_final.png
%   results/planning_algorithm_comparison.png
%   results/planning_algorithm_comparison_metrics.csv
%   results/multi_algorithm_robot_navigation.avi
%   results/multi_algorithm_robot_navigation_final.png
%   results/multi_algorithm_robot_navigation_metrics.csv
%   results/slam_live_occupancy_map.png
%   results/navigation_results.mat
%   results/navigation_results.csv
%   results/multi_algorithm_project_results.mat

clear functions;
clear;
clc;
close all;

projectFolder = fileparts(mfilename('fullpath'));

if isempty(projectFolder)
    projectFolder = pwd;
end

addpath(projectFolder);
cd(projectFolder);

cfg = utilities('defaultConfig');

if ~exist(cfg.output.resultsFolder,'dir')
    mkdir(cfg.output.resultsFolder);
end

rng(cfg.environment.randomSeed,'twister');

%% Shared environment and synchronized goal
environment = createEnvironment(cfg);

selectionFigure = figure( ...
    'Name','Select Shared Goal for All Five Algorithms', ...
    'Color','w', ...
    'Position',[70 60 1200 780], ...
    'NumberTitle','off');

selectionAxes = axes( ...
    'Parent',selectionFigure, ...
    'Position',[0.07 0.09 0.88 0.84]);

plotEnvironment(selectionAxes,environment,cfg);

goal = goalSelection( ...
    selectionFigure,selectionAxes,environment,cfg);

startPoint = cfg.robot.startPose(1:2);

%% Shared safety-inflated occupancy grid
occupancyGrid = buildOccupancyGrid(environment,cfg);

%% A* route used as the first synchronized planner result
astarTimer = tic;

[astarRawPath,astarInfo,astarSearchHistory] = ...
    planPathAStar( ...
    occupancyGrid,startPoint,goal);

astarPlanningTime = toc(astarTimer);

if ~astarInfo.success || isempty(astarRawPath)
    error([ ...
        'A* could not find a collision-free route to the selected goal. ' ...
        'Please run the project again and select another free goal.']);
end

%% Plan, animate, and navigate all five algorithms
comparisonResults = comparePlanningAlgorithms( ...
    environment,occupancyGrid,startPoint,goal, ...
    astarRawPath,astarInfo,astarPlanningTime, ...
    astarSearchHistory,cfg);

%% Save only current-project data
save(fullfile(cfg.output.resultsFolder, ...
    'multi_algorithm_project_results.mat'), ...
    'cfg','environment','occupancyGrid', ...
    'startPoint','goal','comparisonResults');

fprintf('\n===== FIVE-ALGORITHM PROJECT COMPLETE =====\n');
fprintf('Shared start: (%.2f, %.2f) m\n', ...
    startPoint(1),startPoint(2));
fprintf('Shared goal:  (%.2f, %.2f) m\n', ...
    goal(1),goal(2));
fprintf('Results folder: %s\n',cfg.output.resultsFolder);
