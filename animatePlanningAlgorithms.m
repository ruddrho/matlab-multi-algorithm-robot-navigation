function animatePlanningAlgorithms( ...
    environment,startPoint,goalPoint,comparisonResults,cfg)
%ANIMATEPLANNINGALGORITHMS Five simultaneous planner animations.
%
% Layout follows the uploaded reference video:
% - Dijkstra: upper-left
% - A*: upper-right
% - RRT: middle-left
% - RRT*: middle-right
% - PRM: full-width bottom panel
%
% Blue/cyan graphics show exploration or graph growth. Bright green shows
% the final path. All panels share the same start, goal, and environment.

if ~cfg.compare.animation.enabled
    return;
end

animationFigure = figure( ...
    'Color',[0 0 0], ...
    'Position',[40 20 1120 960], ...
    'Name','A*, Dijkstra, RRT, RRT*, PRM Animation', ...
    'NumberTitle','off');

headerAxes = axes( ...
    'Parent',animationFigure, ...
    'Position',[0.03 0.925 0.94 0.065], ...
    'Color',[0 0 0], ...
    'XLim',[0 1],'YLim',[0 1], ...
    'XTick',[],'YTick',[], ...
    'Visible','off');

text(headerAxes,0.5,0.58, ...
    'A*, Dijkstra, RRT, RRT*, PRM — Synchronized Animation', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'Color','w', ...
    'FontWeight','bold', ...
    'FontSize',17, ...
    'Interpreter','none');

text(headerAxes,0.5,0.12, ...
    'Same environment | Same start | Same clicked goal', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'Color',[0.75 0.85 1.00], ...
    'FontSize',10, ...
    'Interpreter','none');

panelPositions = [ ...
    0.055 0.635 0.415 0.260; ... % Dijkstra
    0.530 0.635 0.415 0.260; ... % A*
    0.055 0.345 0.415 0.260; ... % RRT
    0.530 0.345 0.415 0.260; ... % RRT*
    0.160 0.055 0.680 0.250];    % PRM

panelOrder = [2 1 3 4 5];
panelTitles = {'Dijkstra Pathfinding','A* Pathfinding', ...
    'RRT Pathfinding','RRT* Pathfinding','PRM Pathfinding'};

handles = repmat(struct( ...
    'axes',[], ...
    'searchLine',[], ...
    'searchPoints',[], ...
    'routeLine',[], ...
    'statusText',[]),5,1);

for panelIndex = 1:5
    resultIndex = panelOrder(panelIndex);
    handles(panelIndex).axes = axes( ...
        'Parent',animationFigure, ...
        'Position',panelPositions(panelIndex,:), ...
        'Color',[0.025 0.025 0.065]);

    drawDarkEnvironmentLocal( ...
        handles(panelIndex).axes,environment,startPoint,goalPoint);

    title(handles(panelIndex).axes,panelTitles{panelIndex}, ...
        'Color','w','FontWeight','bold','FontSize',10, ...
        'Interpreter','none');

    handles(panelIndex).searchLine = plot( ...
        handles(panelIndex).axes,nan,nan, ...
        'Color',[0.08 0.42 1.00], ...
        'LineWidth',0.45);

    handles(panelIndex).searchPoints = plot( ...
        handles(panelIndex).axes,nan,nan,'.', ...
        'Color',[0.10 0.72 1.00], ...
        'MarkerSize',4);

    handles(panelIndex).routeLine = plot( ...
        handles(panelIndex).axes,nan,nan, ...
        'Color',[0.10 1.00 0.34], ...
        'LineWidth',2.4);

    handles(panelIndex).statusText = text( ...
        handles(panelIndex).axes,0.02,0.98, ...
        sprintf('%s: preparing...',comparisonResults(resultIndex).name), ...
        'Units','normalized', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','top', ...
        'Color',[0.90 0.95 1.00], ...
        'FontSize',7.5, ...
        'FontWeight','bold', ...
        'Interpreter','none');
end

% Prevent axes toolbars from appearing in animation/video frames.
hideAxesToolbars(animationFigure);

videoObject = [];
temporaryImage = '';
targetVideoSize = [];

if cfg.compare.animation.recordVideo
    videoObject = VideoWriter( ...
        fullfile(cfg.output.resultsFolder, ...
        'planning_algorithms_comparison_animation.avi'), ...
        'Motion JPEG AVI');
    videoObject.FrameRate = cfg.compare.animation.frameRate;
    open(videoObject);
    temporaryImage = [tempname '.png'];
end

try
    explorationFrames = cfg.compare.animation.explorationFrames;

    for frameIndex = 1:explorationFrames
        progress = frameIndex/explorationFrames;

        for panelIndex = 1:5
            resultIndex = panelOrder(panelIndex);
            result = comparisonResults(resultIndex);

            updateSearchGraphicLocal( ...
                handles(panelIndex),result.animationHistory,progress);

            set(handles(panelIndex).statusText,'String',sprintf( ...
                '%s search: %3.0f%%',result.name,100*progress));
        end

        drawnow;

        if cfg.compare.animation.recordVideo
            [targetVideoSize,videoObject] = captureFrameLocal( ...
                animationFigure,temporaryImage,targetVideoSize,videoObject);
        end
    end

    routeFrames = cfg.compare.animation.routeRevealFrames;

    for frameIndex = 1:routeFrames
        progress = frameIndex/routeFrames;

        for panelIndex = 1:5
            resultIndex = panelOrder(panelIndex);
            result = comparisonResults(resultIndex);

            if result.success && ~isempty(result.displayPath)
                routePointCount = max(2,round( ...
                    progress*size(result.displayPath,1)));
                routePointCount = min(routePointCount,size(result.displayPath,1));

                set(handles(panelIndex).routeLine, ...
                    'XData',result.displayPath(1:routePointCount,1), ...
                    'YData',result.displayPath(1:routePointCount,2));

                set(handles(panelIndex).statusText,'String',sprintf( ...
                    '%s route: %3.0f%%',result.name,100*progress));
            else
                set(handles(panelIndex).statusText, ...
                    'String',[result.name ': no path found'], ...
                    'Color',[1.00 0.35 0.35]);
            end
        end

        drawnow;

        if cfg.compare.animation.recordVideo
            [targetVideoSize,videoObject] = captureFrameLocal( ...
                animationFigure,temporaryImage,targetVideoSize,videoObject);
        end
    end

    for panelIndex = 1:5
        resultIndex = panelOrder(panelIndex);
        result = comparisonResults(resultIndex);

        if result.success
            set(handles(panelIndex).statusText, ...
                'String',sprintf( ...
                '%s | %.3f s | %.2f m | %d nodes', ...
                result.name,result.planningTime, ...
                result.pathLength,result.searchNodes), ...
                'Color',[0.65 1.00 0.72]);
        end
    end

    holdFrames = max(1,round( ...
        cfg.compare.animation.finalHoldSeconds* ...
        cfg.compare.animation.frameRate));

    for frameIndex = 1:holdFrames
        drawnow;
        if cfg.compare.animation.recordVideo
            [targetVideoSize,videoObject] = captureFrameLocal( ...
                animationFigure,temporaryImage,targetVideoSize,videoObject);
        else
            pause(1/cfg.compare.animation.frameRate);
        end
    end

    hideAxesToolbars(animationFigure);

    print(animationFigure,fullfile( ...
        cfg.output.resultsFolder, ...
        'planning_algorithms_animation_final.png'), ...
        '-dpng','-r180');

    if cfg.compare.animation.recordVideo
        close(videoObject);
    end
catch errorInformation
    if cfg.compare.animation.recordVideo
        try
            close(videoObject);
        catch
        end
    end

    if ~isempty(temporaryImage) && exist(temporaryImage,'file')
        delete(temporaryImage);
    end

    rethrow(errorInformation);
end

if ~isempty(temporaryImage) && exist(temporaryImage,'file')
    delete(temporaryImage);
end
end

function drawDarkEnvironmentLocal(axesHandle,environment,startPoint,goalPoint)
cla(axesHandle);
hold(axesHandle,'on');
axis(axesHandle,'equal');
axis(axesHandle,[0 environment.worldWidth 0 environment.worldHeight]);
box(axesHandle,'on');
grid(axesHandle,'on');

set(axesHandle, ...
    'Color',[0.025 0.025 0.065], ...
    'XColor',[0.68 0.72 0.80], ...
    'YColor',[0.68 0.72 0.80], ...
    'GridColor',[0.22 0.25 0.35], ...
    'GridAlpha',0.25, ...
    'FontSize',7);

rectangle(axesHandle, ...
    'Position',[0 0 environment.worldWidth environment.worldHeight], ...
    'EdgeColor',[1.00 0.22 0.18], ...
    'LineWidth',2.0);

for index = 1:size(environment.rectangles,1)
    rectangle(axesHandle, ...
        'Position',environment.rectangles(index,:), ...
        'FaceColor',[0.55 0.08 0.07], ...
        'EdgeColor',[1.00 0.25 0.20], ...
        'LineWidth',1.0);
end

for index = 1:size(environment.circles,1)
    circle = environment.circles(index,:);
    rectangle(axesHandle, ...
        'Position',[circle(1)-circle(3),circle(2)-circle(3), ...
        2*circle(3),2*circle(3)], ...
        'Curvature',[1 1], ...
        'FaceColor',[0.55 0.08 0.07], ...
        'EdgeColor',[1.00 0.25 0.20], ...
        'LineWidth',1.0);
end

for index = 1:numel(environment.polygons)
    polygon = environment.polygons{index};
    patch(axesHandle,polygon(:,1),polygon(:,2), ...
        [0.55 0.08 0.07], ...
        'EdgeColor',[1.00 0.25 0.20], ...
        'LineWidth',1.0);
end

plot(axesHandle,startPoint(1),startPoint(2), ...
    'o','Color',[1.00 1.00 0.00], ...
    'MarkerFaceColor',[0.85 1.00 0.00], ...
    'MarkerSize',6,'LineWidth',1.1);

plot(axesHandle,goalPoint(1),goalPoint(2), ...
    'p','Color',[1.00 0.20 0.95], ...
    'MarkerFaceColor',[1.00 0.20 0.95], ...
    'MarkerSize',9,'LineWidth',1.1);
end

function updateSearchGraphicLocal(handles,history,progress)
if isempty(history) || ~isstruct(history) || ~isfield(history,'type')
    return;
end

switch lower(history.type)
    case 'grid'
        if isempty(history.expandedWorld)
            return;
        end

        count = max(1,round(progress*size(history.expandedWorld,1)));
        count = min(count,size(history.expandedWorld,1));

        set(handles.searchPoints, ...
            'XData',history.expandedWorld(1:count,1), ...
            'YData',history.expandedWorld(1:count,2));

        % Sparse parent links give the search wave additional motion while
        % keeping the panel readable.
        sparseIndices = 1:4:count;
        validMask = all(isfinite(history.parentWorld(sparseIndices,:)),2);
        sparseIndices = sparseIndices(validMask);

        edgeX = nan(1,3*numel(sparseIndices));
        edgeY = nan(1,3*numel(sparseIndices));

        for index = 1:numel(sparseIndices)
            nodeIndex = sparseIndices(index);
            dataIndex = 3*(index-1)+1;
            edgeX(dataIndex:dataIndex+2) = [ ...
                history.parentWorld(nodeIndex,1), ...
                history.expandedWorld(nodeIndex,1),nan];
            edgeY(dataIndex:dataIndex+2) = [ ...
                history.parentWorld(nodeIndex,2), ...
                history.expandedWorld(nodeIndex,2),nan];
        end

        set(handles.searchLine,'XData',edgeX,'YData',edgeY);

    case 'tree'
        if isempty(history.nodes) || size(history.nodes,1) < 2
            return;
        end

        count = max(2,round(progress*size(history.nodes,1)));
        count = min(count,size(history.nodes,1));
        [edgeX,edgeY] = treeLineDataLocal( ...
            history.nodes,history.parents,count);

        set(handles.searchLine,'XData',edgeX,'YData',edgeY);
        set(handles.searchPoints, ...
            'XData',history.nodes(1:count,1), ...
            'YData',history.nodes(1:count,2));

    case 'roadmap'
        if isempty(history.edges)
            return;
        end

        count = max(1,round(progress*size(history.edges,1)));
        count = min(count,size(history.edges,1));
        [edgeX,edgeY] = roadmapLineDataLocal( ...
            history.nodes,history.edges,count);

        set(handles.searchLine,'XData',edgeX,'YData',edgeY);

        nodeCount = max(2,round(progress*size(history.nodes,1)));
        nodeCount = min(nodeCount,size(history.nodes,1));
        set(handles.searchPoints, ...
            'XData',history.nodes(1:nodeCount,1), ...
            'YData',history.nodes(1:nodeCount,2));
end
end

function [edgeX,edgeY] = treeLineDataLocal(nodes,parents,count)
edgeCount = max(0,count-1);
edgeX = nan(1,3*edgeCount);
edgeY = nan(1,3*edgeCount);
writeIndex = 0;

for nodeIndex = 2:count
    parentIndex = parents(nodeIndex);
    if parentIndex < 1 || parentIndex > count || ...
            parentIndex > size(nodes,1) || parentIndex == nodeIndex
        continue;
    end

    writeIndex = writeIndex+1;
    dataIndex = 3*(writeIndex-1)+1;
    edgeX(dataIndex:dataIndex+2) = [ ...
        nodes(parentIndex,1),nodes(nodeIndex,1),nan];
    edgeY(dataIndex:dataIndex+2) = [ ...
        nodes(parentIndex,2),nodes(nodeIndex,2),nan];
end

edgeX = edgeX(1:3*writeIndex);
edgeY = edgeY(1:3*writeIndex);
end

function [edgeX,edgeY] = roadmapLineDataLocal(nodes,edges,count)
edgeX = nan(1,3*count);
edgeY = nan(1,3*count);

for edgeIndex = 1:count
    nodeA = edges(edgeIndex,1);
    nodeB = edges(edgeIndex,2);
    dataIndex = 3*(edgeIndex-1)+1;
    edgeX(dataIndex:dataIndex+2) = [ ...
        nodes(nodeA,1),nodes(nodeB,1),nan];
    edgeY(dataIndex:dataIndex+2) = [ ...
        nodes(nodeA,2),nodes(nodeB,2),nan];
end
end

function [targetVideoSize,videoObject] = captureFrameLocal( ...
    figureHandle,temporaryImage,targetVideoSize,videoObject)

hideAxesToolbars(figureHandle);

rgbFrame = utilities( ...
    'captureFigureRGB',figureHandle,temporaryImage);

if isempty(targetVideoSize)
    frameHeight = size(rgbFrame,1)-mod(size(rgbFrame,1),2);
    frameWidth = size(rgbFrame,2)-mod(size(rgbFrame,2),2);
    targetVideoSize = [frameHeight frameWidth];
end

rgbFrame = utilities('fitFrame',rgbFrame,targetVideoSize);
writeVideo(videoObject,rgbFrame);
end
