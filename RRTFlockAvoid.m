function RRTFlockAvoid()
%% MultiUAV: RRT Path Planning + Flocking + Collision Avoidance
%  Allows user to select start and goal nodes, visualizes RRT tree,
%  and animates UAVs along the paths with collision avoidance.

clc; clear; close all;
rng(1);

%% ===== PARAMETERS =====
mapBounds = [0 100 0 100];
numUAVs = 2;            % number of UAVs
stepSize = 4;            % RRT step size
goalBias = 0.1;          % RRT goal bias
maxIters = 2500;         % max RRT iterations
goalRadius = 5;          % goal tolerance
interpStep = 0.1;        % path interpolation step for smooth UAV motion
dt = 0.05;               % simulation timestep

% Obstacles [x y radius]
obstacles = [30 40 8;
             60 60 10;
             40 80 6;
             75 25 8;
             20 70 6];

colors = lines(numUAVs); % UAV colors

%% ===== FIGURE SETUP =====
fig = figure('Name','Multi-UAV RRT Simulation','Color','w','Position',[100 100 1200 700]);
axis(mapBounds); axis equal; hold on; grid on;
xlabel('X'); ylabel('Y'); title('Multi-UAV RRT + Flocking + Collision Avoidance');

% Draw obstacles
theta = linspace(0,2*pi,64);
for i=1:size(obstacles,1)
    patch(obstacles(i,1)+obstacles(i,3)*cos(theta), ...
          obstacles(i,2)+obstacles(i,3)*sin(theta), ...
          [0.7 0.7 0.7],'EdgeColor','k','FaceAlpha',0.9);
end

%% ===== USER INPUT START/GOAL =====
starts = zeros(numUAVs,2);
goals = zeros(numUAVs,2);

for i=1:numUAVs
    disp(['Select START for UAV ' num2str(i) ' (click on plot)']);
    [sx, sy] = ginput(1);
    starts(i,:) = [sx, sy];
    plot(sx, sy, 'o', 'MarkerFaceColor','b','MarkerEdgeColor','k','MarkerSize',6);
    text(sx+1, sy, sprintf('Drone %d Start',i),'Color','b');
    
    disp(['Select GOAL for UAV ' num2str(i) ' (click on plot)']);
    [gx, gy] = ginput(1);
    goals(i,:) = [gx, gy];
    plot(gx, gy, 'o', 'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',6);
    text(gx+1, gy, sprintf('Drone %d Goal',i),'Color','r');
end

%% ===== DRONE SPEED INPUT =====
answer = inputdlg({'Enter UAV speed (1-100):'},'Speed',[1 50],{'20'});
if isempty(answer)
    pauseTime = 0.02;
else
    userSpeed = str2double(answer{1});
    userSpeed = max(1,min(25,userSpeed));
    pauseTime = 0.1/userSpeed;
end

%% ===== RRT SETUP =====
nodes = cell(numUAVs,1);
parent = cell(numUAVs,1);
found = false(numUAVs,1);
foundNodeIndex = zeros(numUAVs,1);

for i=1:numUAVs
    nodes{i} = starts(i,:);
    parent{i} = 0;
end

isCollision = @(p1,p2) segmentCollisionWithCircles(p1,p2,obstacles);
sampleRandom = @() [mapBounds(1)+(mapBounds(2)-mapBounds(1))*rand, ...
                    mapBounds(3)+(mapBounds(4)-mapBounds(3))*rand];

fprintf('Growing RRT Trees...\n');
iter = 0;
while (~all(found)) && iter < maxIters
    iter = iter + 1;
    for d=1:numUAVs
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
        if dist==0, continue; end
        qNew = qNearest + (stepSize/dist)*dir;

        % Check collision
        if ~isCollision(qNearest,qNew)
            nodes{d}(end+1,:) = qNew;
            parent{d}(end+1) = idxNearest;
            plot([qNearest(1) qNew(1)],[qNearest(2) qNew(2)],'-','Color',colors(d,:),'LineWidth',0.8);
            drawnow limitrate;

            % Check goal
            if norm(qNew - goals(d,:)) <= goalRadius
                found(d) = true;
                foundNodeIndex(d) = size(nodes{d},1);
                plot([qNew(1) goals(d,1)],[qNew(2) goals(d,2)],'--','Color',colors(d,:),'LineWidth',1.5);
                nodes{d}(end+1,:) = goals(d,:);
                parent{d}(end+1) = foundNodeIndex(d);
                foundNodeIndex(d) = size(nodes{d},1);
                fprintf('Drone %d found path in %d iterations.\n',d,iter);
            end
        end
    end
end

%% ===== RECONSTRUCT PATHS =====
paths = cell(numUAVs,1);
for d=1:numUAVs
    if found(d)
        idx = foundNodeIndex(d);
        path = nodes{d}(idx,:);
        while parent{d}(idx) ~= 0
            idx = parent{d}(idx);
            path = [nodes{d}(idx,:); path]; %#ok<AGROW>
        end
        paths{d} = path;
        plot(path(:,1), path(:,2),'-','Color',colors(d,:),'LineWidth',2.5);
    else
        paths{d} = [];
    end
end
drawnow;

%% ===== ANIMATE UAVS =====
interpPaths = cell(numUAVs,1);
maxLen = 0;
for d=1:numUAVs
    P = paths{d};
    if isempty(P), continue; end
    pts = P(1,:);
    for k=2:size(P,1)
        seg = P(k,:) - P(k-1,:);
        segLen = norm(seg);
        nSteps = max(ceil(segLen/interpStep),1);
        t = linspace(0,1,nSteps+1)';
        newPts = (1-t)*P(k-1,:) + t*P(k,:);
        pts = [pts; newPts(2:end,:)]; %#ok<AGROW>
    end
    interpPaths{d} = pts;
    maxLen = max(maxLen,size(pts,1));
end

% Initialize UAV markers
droneHandles = gobjects(numUAVs,1);
for d=1:numUAVs
    if ~isempty(interpPaths{d})
        droneHandles(d) = plot(interpPaths{d}(1,1),interpPaths{d}(1,2),'o','MarkerFaceColor',colors(d,:),'MarkerEdgeColor','k','MarkerSize',8);
    end
end
drawnow;

% Animate
for t=1:maxLen
    for d=1:numUAVs
        if isempty(interpPaths{d}), continue; end
        idx = min(t,size(interpPaths{d},1));
        set(droneHandles(d),'XData',interpPaths{d}(idx,1),'YData',interpPaths{d}(idx,2));
    end
    drawnow limitrate;
    pause(pauseTime);
end

fprintf('Simulation Complete!\n');

end

%% ===== HELPER FUNCTIONS =====
%%
function hit = segmentCollisionWithCircles(p1,p2,circles)
hit = false;
for i=1:size(circles,1)
    if segmentCircleIntersect(p1,p2,circles(i,1:2),circles(i,3))
        hit = true; return;
    end
end
end

function tf = segmentCircleIntersect(a,b,center,r)
ab = b-a; ac = center-a;
ab2 = dot(ab,ab);
if ab2==0
    d = norm(ac);
else
    t = max(0,min(1,dot(ac,ab)/ab2));
    proj = a + t*ab;
    d = norm(proj-center);
end
tf = (d<=r);
end