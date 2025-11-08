function RRTMultiUAV()
% RRTMultiUAV
% Visualizes RRT pathfinding for multiple UAVs with proper legend and animation.
%
% Blue = Start point
% Red  = Goal point
% Colored lines = RRT tree expansion
% Thick line = Final path
% Moving circles = UAVs following paths

clc; clear; close all;
rng(1); % For reproducibility

%% === Parameters ===
mapBounds = [0 100 0 100];
numDrones = 2;        % Adjust this to number of UAVs
stepSize = 3.5;
maxIters = 2500;
goalBias = 0.1;
goalRadius = 5;
interpStep = 0.01;     % Smooth path interpolation step

% === Drone Speed UI ===
answer = inputdlg({'Please enter drone speed! (1 = slowest, 100 = fastest):'}, ...
    'Drone Speed Control', [15 80], {'20'});
if isempty(answer)
    pauseTime = 0.02; % default if cancelled
else
    userSpeed = str2double(answer{1});
    userSpeed = max(1, min(25, userSpeed)); % clamp range
    pauseTime = 0.1 / userSpeed;  % higher speed → smaller pause
end
fprintf('Drone animation speed set to %.3f seconds per step.\n', pauseTime);

% Obstacles [x y radius]
obstacles = [30 40 8;
             60 60 10;
             40 80 6;
             75 25 8;
             20 70 6];

% Start and goal positions (you can edit freely)
starts = [10 10;
          15 85];
goals  = [90 90;
          80 95];

colors = lines(numDrones);

%% === Figure Setup ===
fig = figure('Name','RRT Multi-UAV Pathfinding','Color','w','Position',[100 100 1000 700]);
axis(mapBounds); axis equal; hold on; grid on;
xlabel('X'); ylabel('Y');
title('RRT Pathfinding for Multiple UAVs');
set(gca,'FontSize',11);

% Obstacles
theta = linspace(0,2*pi,64);
for i=1:size(obstacles,1)
    patch(obstacles(i,1) + obstacles(i,3)*cos(theta), ...
          obstacles(i,2) + obstacles(i,3)*sin(theta), ...
          [0.7 0.7 0.7],'EdgeColor','k','FaceAlpha',0.9);
end

% Start & Goal markers
startHandles = gobjects(numDrones,1);
goalHandles = gobjects(numDrones,1);
for i=1:numDrones
    startHandles(i) = plot(starts(i,1), starts(i,2), 'o', ...
        'MarkerFaceColor','b','MarkerEdgeColor','k','MarkerSize',6);
    goalHandles(i) = plot(goals(i,1), goals(i,2), 'o', ...
        'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',8);
    text(starts(i,1)+2, starts(i,2), sprintf('Drone %d Start',i), ...
        'Color','b','FontSize',10);
    text(goals(i,1)+2, goals(i,2), sprintf('Drone %d Goal',i), ...
        'Color','r','FontSize',10);
end

lgd = legend([startHandles; goalHandles], ...
    [arrayfun(@(i) sprintf('Drone %d Start',i),1:numDrones,'UniformOutput',false), ...
     arrayfun(@(i) sprintf('Drone %d Goal',i),1:numDrones,'UniformOutput',false)], ...
    'Location','eastoutside');
lgd.Title.String = 'Legend';
lgd.Box = 'on';

%% === RRT Structures ===
nodes = cell(numDrones,1);
parent = cell(numDrones,1);
found = false(numDrones,1);
foundNodeIndex = zeros(numDrones,1);

for i=1:numDrones
    nodes{i} = starts(i,:);
    parent{i} = 0;
end

isCollision = @(p1,p2) segmentCollisionWithCircles(p1,p2,obstacles);
sampleRandom = @() [mapBounds(1)+(mapBounds(2)-mapBounds(1))*rand, ...
                    mapBounds(3)+(mapBounds(4)-mapBounds(3))*rand];

fprintf('Growing RRT Trees...\n');
%% === Grow Trees ===
iter = 0;
while (~all(found)) && iter < maxIters
    iter = iter + 1;
    for d = 1:numDrones
        if found(d), continue; end

        % Sampling
        if rand < goalBias
            qRand = goals(d,:);
        else
            qRand = sampleRandom();
        end

        % Nearest node
        currNodes = nodes{d};
        [~, idxNearest] = min(sum((currNodes - qRand).^2,2));
        qNearest = currNodes(idxNearest,:);

        % Step toward
        dir = qRand - qNearest;
        dist = norm(dir);
        if dist == 0, continue; end
        qNew = qNearest + (stepSize/dist)*dir;

        % Check for collision
        if ~isCollision(qNearest, qNew)
            nodes{d}(end+1,:) = qNew;
            parent{d}(end+1) = idxNearest;
            plot([qNearest(1) qNew(1)], [qNearest(2) qNew(2)], '-', ...
                'Color', colors(d,:), 'LineWidth', 0.8);
            drawnow limitrate;

            % Check goal reach
            if norm(qNew - goals(d,:)) <= goalRadius
                found(d) = true;
                foundNodeIndex(d) = size(nodes{d},1);
                fprintf('Drone %d found path in %d iterations.\n', d, iter);
                plot([qNew(1) goals(d,1)], [qNew(2) goals(d,2)], '--', ...
                    'Color', colors(d,:), 'LineWidth', 1.5);
                nodes{d}(end+1,:) = goals(d,:);
                parent{d}(end+1) = foundNodeIndex(d);
                foundNodeIndex(d) = size(nodes{d},1);
            end
        end
    end
end

%% === Reconstruct Paths ===
paths = cell(numDrones,1);
for d=1:numDrones
    if found(d)
        idx = foundNodeIndex(d);
        path = nodes{d}(idx,:);
        while parent{d}(idx) ~= 0
            idx = parent{d}(idx);
            path = [nodes{d}(idx,:); path]; %#ok<AGROW>
        end
        paths{d} = path;
        plot(path(:,1), path(:,2), '-', 'Color', colors(d,:), 'LineWidth', 2.5);
    else
        paths{d} = [];
    end
end
drawnow;

%% === Animate Drones ===
fprintf('Animating UAVs along paths...\n');

% Interpolate for smooth motion
interpPaths = cell(numDrones,1);
maxLen = 0;
for d=1:numDrones
    P = paths{d};
    if isempty(P), continue; end
    pts = P(1,:);
    for k=2:size(P,1)
        seg = P(k,:) - P(k-1,:);
        segLen = norm(seg);
        nSteps = max(ceil(segLen / interpStep),1);
        t = linspace(0,1,nSteps+1)';
        newPts = (1-t)*P(k-1,:) + t*P(k,:);
        pts = [pts; newPts(2:end,:)]; %#ok<AGROW>
    end
    interpPaths{d} = pts;
    maxLen = max(maxLen, size(pts,1));
end

% Initialize UAV graphics
droneHandles = gobjects(numDrones,1);
for d=1:numDrones
    if ~isempty(interpPaths{d})
        droneHandles(d) = plot(interpPaths{d}(1,1), interpPaths{d}(1,2), 'o', ...
            'MarkerFaceColor', colors(d,:), 'MarkerEdgeColor','k', 'MarkerSize', 8);
    end
end
drawnow;

% Animate all drones together
for t = 1:maxLen
    for d=1:numDrones
        if isempty(interpPaths{d}), continue; end
        idx = min(t, size(interpPaths{d},1));
        set(droneHandles(d), 'XData', interpPaths{d}(idx,1), ...
                              'YData', interpPaths{d}(idx,2));
    end
    drawnow limitrate;
    %pause(pauseTime);  % <- Drone speed control
end

fprintf('All drones have reached end point. Simulation Complete!\n');

end

%% === Helper Collision Functions ===
%%
function hit = segmentCollisionWithCircles(p1,p2, circles)
hit = false;
for i=1:size(circles,1)
    if segmentCircleIntersect(p1,p2,circles(i,1:2),circles(i,3))
        hit = true;
        return;
    end
end
end

function tf = segmentCircleIntersect(a,b,center,r)
ab = b - a; ac = center - a;
ab2 = dot(ab,ab);
if ab2 == 0
    d = norm(ac);
else
    t = max(0,min(1, dot(ac,ab)/ab2));
    proj = a + t*ab;
    d = norm(proj - center);
end
tf = (d <= r);
end