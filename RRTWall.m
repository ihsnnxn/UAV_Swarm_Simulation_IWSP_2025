function RRTWall()
% RRT_PathPlanning_Complex
% Demonstrates Path Planning via RRT (Rapidly-Exploring Random Tree)
% with complex obstacles and walls.
%
% Blue  = Start point
% Red   = Goal point
% Gray  = RRT tree expansion
% Green = Final path
% Magenta circle = UAV moving along path
%
% Figure 6.13: Path Planning via RRT (Rapidly-Exploring Random Tree)

clc; clear; close all;
rng(2); % For reproducibility

%% === Parameters ===
mapBounds = [0 100 0 100];
stepSize = 10.0;
maxIters = 4000;
goalBias = 0.1;
goalRadius = 5;

% Start and Goal positions
start = [10 10];
goal  = [90 85];

%% === Create Obstacles ===
% Format: [x y width height]
rectObstacles = [
    20 20 15 10;
    45 10 10 25;
    65 25 15 10;
    25 50 20 10;
    60 55 10 20;
    40 75 10 10;
    80 60 10 25
];

% Circular obstacles [x y r]
circObstacles = [
    30 35 5;
    75 40 5;
    50 60 4;
];

%% === Draw Map ===
fig = figure('Name','Figure 6.13: Path Planning via RRT',...
    'Color','w','Position',[100 100 1000 700]);
hold on; axis(mapBounds); axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Path Planning via RRT (Rapidly-Exploring Random Tree)');
set(gca,'FontSize',11);

% Draw walls (boundaries)
plot([0 100 100 0 0],[0 0 100 100 0],'k-','LineWidth',2);

% Draw rectangular obstacles
for i = 1:size(rectObstacles,1)
    rectangle('Position',rectObstacles(i,:), ...
              'FaceColor',[0.6 0.6 0.6],'EdgeColor','k');
end

% Draw circular obstacles
theta = linspace(0,2*pi,64);
for i=1:size(circObstacles,1)
    fill(circObstacles(i,1)+circObstacles(i,3)*cos(theta), ...
         circObstacles(i,2)+circObstacles(i,3)*sin(theta), ...
         [0.6 0.6 0.6],'EdgeColor','k');
end

% Mark start & goal
plot(start(1), start(2), 'o', 'MarkerFaceColor','b','MarkerEdgeColor','k','MarkerSize',8);
plot(goal(1), goal(2), 'o', 'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',8);
legend({'Walls','Rectangular Obstacles','Circular Obstacles','Start','Goal'}, ...
    'Location','eastoutside','Box','on');

%% === Define Helper Functions ===
isCollision = @(p1,p2) checkCollision(p1,p2,rectObstacles,circObstacles);
sampleRandom = @() [mapBounds(1)+(mapBounds(2)-mapBounds(1))*rand, ...
                    mapBounds(3)+(mapBounds(4)-mapBounds(3))*rand];

%% === Initialize Tree ===
nodes = start;
parent = 0;
found = false;
fprintf('Growing RRT Tree...\n');

%% === RRT Expansion ===
for iter = 1:maxIters
    % Random sample (biased toward goal sometimes)
    if rand < goalBias
        qRand = goal;
    else
        qRand = sampleRandom();
    end

    % Nearest node
    [~, idxNearest] = min(sum((nodes - qRand).^2,2));
    qNearest = nodes(idxNearest,:);
    dir = qRand - qNearest;
    dist = norm(dir);
    if dist == 0, continue; end

    % Extend step
    qNew = qNearest + (stepSize/dist)*dir;

    % Check collisions
    if ~isCollision(qNearest,qNew)
        nodes(end+1,:) = qNew;
        parent(end+1) = idxNearest;

        % Draw tree edge
        plot([qNearest(1) qNew(1)], [qNearest(2) qNew(2)], '-', ...
            'Color',[0.7 0.7 0.7],'LineWidth',0.8);
        drawnow limitrate;

        % Check goal
        if norm(qNew - goal) <= goalRadius
            fprintf('Goal reached in %d iterations!\n',iter);
            nodes(end+1,:) = goal;
            parent(end+1) = size(nodes,1)-1;
            found = true;
            break;
        end
    end
end

%% === Reconstruct Path ===
if found
    path = goal;
    idx = size(nodes,1);
    while parent(idx) ~= 0
        idx = parent(idx);
        path = [nodes(idx,:); path]; %#ok<AGROW>
    end
    plot(path(:,1), path(:,2), '-g', 'LineWidth',2.5);
else
    warning('Goal not reached.');
    return;
end

%% === Animate UAV ===
interpStep = 0.5;
interpPts = path(1,:);
for k=2:size(path,1)
    seg = path(k,:) - path(k-1,:);
    segLen = norm(seg);
    nSteps = max(ceil(segLen/interpStep),1);
    t = linspace(0,1,nSteps+1)';
    newPts = (1-t)*path(k-1,:) + t*path(k,:);
    interpPts = [interpPts; newPts(2:end,:)]; %#ok<AGROW>
end

uav = plot(interpPts(1,1), interpPts(1,2), 'o', ...
           'MarkerFaceColor','m','MarkerEdgeColor','k','MarkerSize',10);

fprintf('Animating UAV along final RRT path...\n');
for i = 1:size(interpPts,1)
    set(uav,'XData',interpPts(i,1),'YData',interpPts(i,2));
    drawnow limitrate;
    pause(0.18); % Adjust for speed
end
fprintf('Path Planning via RRT complete.\n');
end

%% === Collision Checking Helper ===
%%
function hit = checkCollision(p1,p2,rectObs,circObs)
hit = false;
% Rectangular obstacles
for i=1:size(rectObs,1)
    rect = rectObs(i,:);
    if segmentIntersectsRect(p1,p2,rect)
        hit = true; return;
    end
end
% Circular obstacles
for i=1:size(circObs,1)
    if segmentCircleIntersect(p1,p2,circObs(i,1:2),circObs(i,3))
        hit = true; return;
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

function hit = segmentIntersectsRect(p1,p2,rect)
x = [rect(1), rect(1)+rect(3)];
y = [rect(2), rect(2)+rect(4)];
edges = [x(1) y(1) x(2) y(1);
         x(2) y(1) x(2) y(2);
         x(2) y(2) x(1) y(2);
         x(1) y(2) x(1) y(1)];
hit = false;
for i=1:4
    if linesIntersect(p1,p2,edges(i,1:2),edges(i,3:4))
        hit = true; return;
    end
end
end

function tf = linesIntersect(p1,p2,p3,p4)
den = (p4(2)-p3(2))*(p2(1)-p1(1)) - (p4(1)-p3(1))*(p2(2)-p1(2));
if den == 0, tf = false; return; end
ua = ((p4(1)-p3(1))*(p1(2)-p3(2)) - (p4(2)-p3(2))*(p1(1)-p3(1)))/den;
ub = ((p2(1)-p1(1))*(p1(2)-p3(2)) - (p2(2)-p1(2))*(p1(1)-p3(1)))/den;
tf = (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
end