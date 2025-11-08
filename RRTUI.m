function RRTUI()
% RRT_UI_Simulator
% Interactive RRT path planning visualization with adjustable parameters.
%
% Features:
% - Sliders for Step Size, Goal Bias, Max Iterations
% - "Generate New Map" button
% - Runtime and iteration count display
% - Multiple obstacles and walls

clc; clear; close all;

%% === Figure & UI Setup ===
fig = figure('Name','RRT Path Planning Simulator',...
    'Color','w','Position',[100 100 1100 700]);

% Axes for map
ax = axes('Parent',fig,'Units','normalized','Position',[0.05 0.1 0.65 0.85]);
axis(ax,[0 100 0 100]); axis equal; grid on; hold on;
xlabel('X'); ylabel('Y'); title('RRT Path Planning');
set(gca,'FontSize',11);

% UI Panel
uip = uipanel('Parent',fig,'Units','normalized','Position',[0.75 0.1 0.22 0.85],...
    'Title','Controls','FontSize',11,'BackgroundColor',[0.95 0.95 0.95]);

% Step Size
uicontrol(uip,'Style','text','String','Step Size (1–10)',...
    'Units','normalized','Position',[0.1 0.9 0.8 0.05],'BackgroundColor',[0.95 0.95 0.95]);
stepSlider = uicontrol(uip,'Style','slider','Min',1,'Max',10,'Value',3.5,...
    'Units','normalized','Position',[0.1 0.85 0.8 0.05]);

% Goal Bias
uicontrol(uip,'Style','text','String','Goal Bias (0–0.5)',...
    'Units','normalized','Position',[0.1 0.77 0.8 0.05],'BackgroundColor',[0.95 0.95 0.95]);
biasSlider = uicontrol(uip,'Style','slider','Min',0,'Max',0.5,'Value',0.1,...
    'Units','normalized','Position',[0.1 0.72 0.8 0.05]);

% Max Iterations
uicontrol(uip,'Style','text','String','Max Iterations (1000–5000)',...
    'Units','normalized','Position',[0.1 0.64 0.8 0.05],'BackgroundColor',[0.95 0.95 0.95]);
iterSlider = uicontrol(uip,'Style','slider','Min',1000,'Max',5000,'Value',2500,...
    'Units','normalized','Position',[0.1 0.59 0.8 0.05]);

% Run Button
runBtn = uicontrol(uip,'Style','pushbutton','String','Run RRT Simulation',...
    'FontSize',10,'BackgroundColor',[0.6 0.9 0.6],'Units','normalized',...
    'Position',[0.1 0.48 0.8 0.07],'Callback',@runSimulation);

% Generate New Map Button
regenBtn = uicontrol(uip,'Style','pushbutton','String','Generate New Map',...
    'FontSize',10,'BackgroundColor',[0.7 0.8 1],'Units','normalized',...
    'Position',[0.1 0.39 0.8 0.07],'Callback',@(~,~)drawMap());

% Info Text
infoText = uicontrol(uip,'Style','text','String','Runtime info here...',...
    'Units','normalized','Position',[0.05 0.2 0.9 0.15],...
    'BackgroundColor',[0.95 0.95 0.95],'FontSize',10,'HorizontalAlignment','left');

% Initialize map
drawMap();

%% === Callback Function: Run Simulation ===
    function runSimulation(~,~)
        cla(ax); drawMap();

        stepSize = get(stepSlider,'Value');
        goalBias = get(biasSlider,'Value');
        maxIters = round(get(iterSlider,'Value'));

        fprintf('Running RRT Simulation with Step=%.2f, Bias=%.2f, MaxIters=%d\n',...
            stepSize, goalBias, maxIters);

        % Map and obstacles
        mapBounds = [0 100 0 100];
        obstacles = evalin('base','obstacles'); %#ok<EVLDIR>

        % Start and goal
        startPos = [10 10];
        goalPos  = [90 90];

        % Draw start/goal
        plot(startPos(1),startPos(2),'bo','MarkerFaceColor','b','MarkerSize',8);
        plot(goalPos(1),goalPos(2),'ro','MarkerFaceColor','r','MarkerSize',8);
        text(startPos(1)+2,startPos(2),'Start','Color','b','FontSize',10);
        text(goalPos(1)+2,goalPos(2),'Goal','Color','r','FontSize',10);

        % Collision function
        isCollision = @(p1,p2) segmentCollisionWithCircles(p1,p2,obstacles);
        sampleRandom = @() [mapBounds(1)+(mapBounds(2)-mapBounds(1))*rand,...
                            mapBounds(3)+(mapBounds(4)-mapBounds(3))*rand];

        % RRT init
        nodes = startPos;
        parent = 0;
        found = false;
        tic;
        for iter = 1:maxIters
            if rand < goalBias
                qRand = goalPos;
            else
                qRand = sampleRandom();
            end

            % Find nearest
            [~, idxNearest] = min(sum((nodes - qRand).^2,2));
            qNearest = nodes(idxNearest,:);
            dir = qRand - qNearest;
            dist = norm(dir);
            if dist == 0, continue; end
            qNew = qNearest + (stepSize/dist)*dir;

            % Check collision
            if ~isCollision(qNearest, qNew)
                nodes(end+1,:) = qNew; %#ok<AGROW>
                parent(end+1) = idxNearest;
                plot([qNearest(1) qNew(1)], [qNearest(2) qNew(2)], ...
                    '-', 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
                drawnow limitrate;
                if norm(qNew - goalPos) < 5
                    found = true; break;
                end
            end
        end
        elapsed = toc;

        % Reconstruct path
        if found
            idx = size(nodes,1);
            path = nodes(idx,:);
            while parent(idx) ~= 0
                idx = parent(idx);
                path = [nodes(idx,:); path]; %#ok<AGROW>
            end
            plot(path(:,1),path(:,2),'-r','LineWidth',2.5);
            % Animate drone
            for k = 1:size(path,1)
                plot(path(k,1),path(k,2),'ko','MarkerFaceColor','g','MarkerSize',8);
                drawnow; pause(0.02);
            end
            msg = sprintf('✅ Path found in %d iterations (%.2f s)',iter,elapsed);
        else
            msg = sprintf('⚠️ No path found after %d iterations (%.2f s)',maxIters,elapsed);
        end
        disp(msg);
        set(infoText,'String',msg);
    end

%% === Map Generator Function ===
    function drawMap()
        axes(ax); cla; hold on;
        axis(ax,[0 100 0 100]); axis equal; grid on;
        xlabel('X'); ylabel('Y'); title('RRT Path Planning');
        set(gca,'FontSize',11);
        % Create random obstacles and walls
        walls = [0 100 0 0; 0 0 0 100; 100 100 0 100; 0 100 100 100]; %#ok<NASGU>
        obstacles = [30 40 8;
                     60 60 10;
                     40 80 6;
                     75 25 8;
                     20 70 6;
                     50 20 6;
                     85 60 9];
        assignin('base','obstacles',obstacles);

        theta = linspace(0,2*pi,64);
        for i=1:size(obstacles,1)
            patch(obstacles(i,1)+obstacles(i,3)*cos(theta),...
                  obstacles(i,2)+obstacles(i,3)*sin(theta),...
                  [0.7 0.7 0.7],'EdgeColor','k','FaceAlpha',0.9);
        end
    end
end

%% === Helper Collision Functions ===
%%
function hit = segmentCollisionWithCircles(p1,p2,circles)
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