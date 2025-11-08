function MultiUAV()
% MultiUAV_Framework
% Integrated multi-UAV simulation: Path planning (Floyd-Warshall or RRT),
% flocking (Reynolds), collision avoidance, UI controls, visualization & metrics.
%
% Save as MultiUAV_Framework.m and run.
%
% Author: ChatGPT (adapted for a research-quality demo)

close all; clearvars -except; clc;
rng(1);

%% === Global simulation parameters (defaults) ===
world.bounds = [0 100 0 100];   % [xmin xmax ymin ymax]
world.obstacles = [];           % circular obstacles: [x y r] rows
world.nodes = [];               % graph nodes (for Floyd)
world.walls = [world.bounds(1) world.bounds(2) world.bounds(3) world.bounds(4)];

sim.dt = 0.05;                  % simulation timestep
sim.maxSteps = 2000;

%% === Agent defaults ===
defaults.numAgents = 8;
defaults.maxSpeed = 3.0;   % units per second
defaults.maxAccel = 2.0;
defaults.targetSpacing = 5; % desired inter-agent spacing
defaults.avoidRadius = 3;   % local avoidance radius
defaults.flockingWeights = struct('cohesion', 1.0, 'alignment', 1.0, 'separation', 1.5);
defaults.collisionWeight = 3.0;  % repulsive strength

%% === RRT defaults ===
rrtDefaults.stepSize = 3.5;
rrtDefaults.goalBias = 0.10;
rrtDefaults.maxIters = 2000;

%% === UI & Figure ===
h.fig = figure('Name','Multi-UAV Integrated Framework','NumberTitle','off','Color','w',...
    'Position',[50 50 1300 800]);

% Axes for world
h.ax = axes('Parent',h.fig,'Units','normalized','Position',[0.02 0.05 0.68 0.92]);
axis(h.ax, [world.bounds(1) world.bounds(2) world.bounds(3) world.bounds(4)]);
axis(h.ax,'equal'); grid(h.ax,'on'); hold(h.ax,'on');
xlabel(h.ax,'X'); ylabel(h.ax,'Y'); title(h.ax,'Environment');

% Info text area (right panel)
panelX = 0.72; panelW = 0.26;
h.panel = uipanel('Parent',h.fig,'Units','normalized','Position',[panelX 0.02 panelW 0.96],...
    'Title','Controls & Info','FontSize',11);

% Controls inside panel
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.92 0.9 0.05],...
    'String','Path Planner','FontWeight','bold','BackgroundColor',[0.94 0.94 0.94]);

h.plannerPopup = uicontrol(h.panel,'Style','popupmenu','Units','normalized',...
    'Position',[0.05 0.88 0.9 0.05], 'String',{'Floyd-Warshall (graph)','RRT (per agent)'},...
    'Value',1);

% Node & obstacle controls
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.82 0.9 0.04],...
    'String','Map Editing (left-click add node, right-click add obstacle)','BackgroundColor',[0.94 0.94 0.94],'FontSize',9);

h.addNodeBtn = uicontrol(h.panel,'Style','togglebutton','Units','normalized', 'Position',[0.05 0.78 0.44 0.05],...
    'String','Add Node','Value',0);
h.addObsBtn = uicontrol(h.panel,'Style','togglebutton','Units','normalized', 'Position',[0.51 0.78 0.44 0.05],...
    'String','Add Obstacle','Value',0);

uicontrol(h.panel,'Style','pushbutton','Units','normalized','Position',[0.05 0.71 0.44 0.05],...
    'String','Generate Random Map','Callback',@cbGenerateRandomMap);
uicontrol(h.panel,'Style','pushbutton','Units','normalized','Position',[0.51 0.71 0.44 0.05],...
    'String','Clear Map','Callback',@cbClearMap);

% Agent controls
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.65 0.9 0.04],...
    'String','Agents & Flocking','BackgroundColor',[0.94 0.94 0.94],'FontWeight','bold');

uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.61 0.5 0.035],'String','Number of Agents:');
h.numAgentsEdit = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.56 0.61 0.39 0.04],...
    'String',num2str(defaults.numAgents));

uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.56 0.5 0.035],'String','Target spacing:');
h.spacingEdit = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.56 0.56 0.39 0.04],...
    'String',num2str(defaults.targetSpacing));

uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.51 0.5 0.035],'String','Avoid radius:');
h.avoidEdit = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.56 0.51 0.39 0.04],...
    'String',num2str(defaults.avoidRadius));

% Flocking sliders
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.45 0.9 0.03],'String','Flocking weights (cohesion / alignment / separation):');
h.cohSlider = uicontrol(h.panel,'Style','slider','Units','normalized','Min',0,'Max',3,'Value',defaults.flockingWeights.cohesion,...
    'Position',[0.05 0.42 0.9 0.04]);
h.alignSlider = uicontrol(h.panel,'Style','slider','Units','normalized','Min',0,'Max',3,'Value',defaults.flockingWeights.alignment,...
    'Position',[0.05 0.37 0.9 0.04]);
h.sepSlider = uicontrol(h.panel,'Style','slider','Units','normalized','Min',0,'Max',4,'Value',defaults.flockingWeights.separation,...
    'Position',[0.05 0.32 0.9 0.04]);

% Planner parameters
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.26 0.9 0.03],'String','RRT Params (if selected):');
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.23 0.45 0.03],'String','Step size:');
h.rrtStep = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.52 0.23 0.43 0.035],'String',num2str(rrtDefaults.stepSize));
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.19 0.45 0.03],'String','Goal bias (0-0.5):');
h.rrtBias = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.52 0.19 0.43 0.035],'String',num2str(rrtDefaults.goalBias));
uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.15 0.45 0.03],'String','Max iters:');
h.rrtIter = uicontrol(h.panel,'Style','edit','Units','normalized','Position',[0.52 0.15 0.43 0.035],'String',num2str(rrtDefaults.maxIters));

% Control buttons
h.planBtn = uicontrol(h.panel,'Style','pushbutton','Units','normalized','Position',[0.05 0.08 0.42 0.06],...
    'String','Plan & Run','FontSize',11,'BackgroundColor',[0.6 1 0.6],'Callback',@cbPlanAndRun);
h.pauseBtn = uicontrol(h.panel,'Style','togglebutton','Units','normalized','Position',[0.53 0.08 0.42 0.06],...
    'String','Pause','FontSize',11,'Callback',@cbPauseToggle);

% Metrics text area
h.metrics = uicontrol(h.panel,'Style','text','Units','normalized','Position',[0.05 0.005 0.9 0.07],...
    'String','Metrics:','HorizontalAlignment','left','BackgroundColor',[0.95 0.95 0.95]);

% Connect mouse clicks for node/obstacle placement
set(h.fig,'WindowButtonDownFcn',@cbMouseClick);

% Storage for plotted objects
plotStore.nodePlots = []; plotStore.obsPlots = []; plotStore.agentPlots = []; plotStore.pathPlots = []; plotStore.correctionQuivers = [];

% Initialize map
generateDefaultMap();

% Simulation state
isRunning = false;
isPaused = false;
simData = []; % will hold agent structs, etc.

%% ================= Callback & Helper functions =================

    function generateDefaultMap()
        % Default set of circular obstacles and some walls
        world.obstacles = [30 40 8; 60 60 10; 40 80 6; 75 25 8; 20 70 6; 50 20 5; 85 60 7];
        world.nodes = [10 90; 90 10; 90 90; 10 10; 50 50]; % sample nodes for Floyd
        redrawMap();
    end

    function cbGenerateRandomMap(~,~)
        % Create some random obstacles (non-overlapping attempt)
        nObs = randi([5 9]);
        obs = zeros(nObs,3);
        for k=1:nObs
            r = randi([4 10]);
            x = r + (world.bounds(2)-2*r)*rand;
            y = r + (world.bounds(4)-2*r)*rand;
            obs(k,:) = [x y r];
        end
        world.obstacles = obs;
        % random nodes
        nNodes = 6;
        nodes = [];
        for k=1:nNodes
            nodes(k,:) = [10 + 80*rand, 10 + 80*rand];
        end
        world.nodes = nodes;
        redrawMap();
    end

    function cbClearMap(~,~)
        world.obstacles = []; world.nodes = [];
        redrawMap();
    end

    function redrawMap()
        axes(h.ax); cla(h.ax); hold(h.ax,'on');
        axis(h.ax, [world.bounds(1) world.bounds(2) world.bounds(3) world.bounds(4)]);
        xlabel(h.ax,'X'); ylabel(h.ax,'Y'); title(h.ax,'Environment');
        % walls border
        plot(h.ax,[world.bounds(1) world.bounds(2) world.bounds(2) world.bounds(1) world.bounds(1)],...
            [world.bounds(3) world.bounds(3) world.bounds(4) world.bounds(4) world.bounds(3)],'k-','LineWidth',2);
        % obstacles
        theta = linspace(0,2*pi,48);
        delete(plotStore.obsPlots); plotStore.obsPlots = [];
        for i=1:size(world.obstacles,1)
            c = world.obstacles(i,:);
            plotStore.obsPlots(i) = fill(h.ax, c(1)+c(3)*cos(theta), c(2)+c(3)*sin(theta), [0.7 0.7 0.7],'EdgeColor','k');
        end
        % nodes
        delete(plotStore.nodePlots); plotStore.nodePlots = [];
        for i=1:size(world.nodes,1)
            plotStore.nodePlots(i) = plot(h.ax, world.nodes(i,1), world.nodes(i,2), 'ks','MarkerFaceColor','y','MarkerSize',8);
            text(world.nodes(i,1)+1, world.nodes(i,2)+1, sprintf('N%d',i),'Parent',h.ax);
        end
        % clear agent/path plots if any
        delete(plotStore.agentPlots); plotStore.agentPlots = [];
        delete(plotStore.pathPlots); plotStore.pathPlots = [];
        delete(plotStore.correctionQuivers); plotStore.correctionQuivers = [];
        drawnow;
    end

    function cbMouseClick(~,~)
        % If user toggled Add Node or Add Obstacle, add at click position
        pt = get(h.ax,'CurrentPoint'); x = pt(1,1); y = pt(1,2);
        if x < world.bounds(1) || x > world.bounds(2) || y < world.bounds(3) || y > world.bounds(4)
            return;
        end
        if get(h.addNodeBtn,'Value')==1
            world.nodes(end+1,:) = [x y];
            redrawMap();
        elseif get(h.addObsBtn,'Value')==1
            % add circular obstacle with dialog for radius
            r = 5; % default
            world.obstacles(end+1,:) = [x y r];
            redrawMap();
        end
    end

    function cbPauseToggle(src,~)
        isPaused = get(src,'Value');
        if isPaused
            set(src,'String','Resume');
        else
            set(src,'String','Pause');
        end
    end

    function cbPlanAndRun(~,~)
        % Read UI parameters
        numAgents = round(max(1,min(20, str2double(get(h.numAgentsEdit,'String')))));
        targetSpacing = max(1, str2double(get(h.spacingEdit,'String')));
        avoidRadius = max(0.1, str2double(get(h.avoidEdit,'String')));
        flockW.coh = get(h.cohSlider,'Value');
        flockW.align = get(h.alignSlider,'Value');
        flockW.sep = get(h.sepSlider,'Value');
        plannerChoice = get(h.plannerPopup,'Value'); % 1=FW,2=RRT
        rrtStep = max(0.5, str2double(get(h.rrtStep,'String')));
        rrtBias = max(0, min(0.5, str2double(get(h.rrtBias,'String'))));
        rrtIters = round(max(100, min(10000, str2double(get(h.rrtIter,'String')))));

        % Build simData (agents)
        simData.N = numAgents;
        simData.dt = sim.dt;
        simData.maxSpeed = defaults.maxSpeed;
        simData.maxAccel = defaults.maxAccel;
        simData.targetSpacing = targetSpacing;
        simData.avoidRadius = avoidRadius;
        simData.flockW = flockW;
        simData.collisionWeight = defaults.collisionWeight;
        simData.positions = zeros(numAgents,2);
        simData.velocities = zeros(numAgents,2);
        simData.goalAssignments = zeros(numAgents,2);
        simData.paths = cell(numAgents,1);
        simData.metrics.pathLengths = zeros(numAgents,1);
        simData.metrics.avoidEvents = zeros(numAgents,1);

        % Initialize agent positions in a small cluster near a corner
        for a=1:numAgents
            simData.positions(a,:) = [10 + 2*randn, 10 + 2*randn];
            simData.velocities(a,:) = 0.5*randn(1,2);
        end

        % Determine global routes (planner)
        if plannerChoice==1
            % Floyd-Warshall: treat world.nodes as graph vertices and connect visible nodes
            if size(world.nodes,1) < 2
                warndlg('Need at least 2 nodes for Floyd-Warshall.','Warning'); return;
            end
            G = buildVisibilityGraph(world.nodes, world.obstacles);
            D = floydWarshallAllPairs(G);
            % For demonstration assign each agent start->random node -> target node (shortest)
            % select random source and destination nodes per agent
            for a=1:numAgents
                sNode = randi(size(world.nodes,1));
                tNode = randi(size(world.nodes,1));
                while tNode==sNode, tNode = randi(size(world.nodes,1)); end
                pathIdx = reconstructFWPath(D.next, sNode, tNode);
                pathCoords = world.nodes(pathIdx,:);
                % prepend agent actual start and append small offset end
                simData.paths{a} = [simData.positions(a,:); pathCoords; pathCoords(end,:)];
                simData.goalAssignments(a,:) = pathCoords(end,:);
            end
        else
            % RRT: compute individual RRT path for each agent to assigned random node
            for a=1:numAgents
                % pick a random goal node (or random point near nodes)
                if isempty(world.nodes)
                    goalPt = [90 90];
                else
                    idx = randi(size(world.nodes,1));
                    goalPt = world.nodes(idx,:);
                end
                % run RRT from simData.positions(a,:) to goalPt
                rrtPath = runRRT(simData.positions(a,:), goalPt, world, rrtStep, rrtBias, rrtIters);
                if isempty(rrtPath)
                    % fallback: straight line
                    simData.paths{a} = [simData.positions(a,:); goalPt];
                    simData.goalAssignments(a,:) = goalPt;
                else
                    simData.paths{a} = rrtPath;
                    simData.goalAssignments(a,:) = rrtPath(end,:);
                end
            end
        end

        % compute each path length metric
        for a=1:numAgents
            P = simData.paths{a};
            simData.metrics.pathLengths(a) = sum(sqrt(sum(diff(P).^2,2)));
        end

        % Plot global paths and agent initial markers
        delete(plotStore.pathPlots); plotStore.pathPlots = [];
        delete(plotStore.agentPlots); plotStore.agentPlots = [];
        colors = lines(numAgents);
        for a=1:numAgents
            P = simData.paths{a};
            if ~isempty(P)
                plotStore.pathPlots(a) = plot(h.ax,P(:,1),P(:,2),'-','Color',colors(a,:),'LineWidth',1.5);
            end
            plotStore.agentPlots(a) = plot(h.ax,simData.positions(a,1),simData.positions(a,2),'o','MarkerFaceColor',colors(a,:),'MarkerEdgeColor','k','MarkerSize',8);
        end

        % start simulation loop
        isRunning = true;
        isPaused = false;
        set(h.pauseBtn,'Value',0,'String','Pause');
        runSimulationLoop(simData);
    end

    function runSimulationLoop(simData)
        % Main sim loop (blocking until stopped). Uses outer scope world, h, etc.
        N = simData.N;
        dt = simData.dt;
        positions = simData.positions;
        velocities = simData.velocities;
        paths = simData.paths;
        maxSteps = sim.maxSteps;
        tStart = tic;
        interAgentGapHistory = zeros(maxSteps,1);
        avoidEvents = zeros(N,1);
        % Precompute path progress indices
        pathIdx = ones(N,1);
        maxPathLen = 0;
        for a=1:N
            maxPathLen = max(maxPathLen, size(paths{a},1));
        end

        for step=1:maxSteps
            if ~isRunning, break; end
            while isPaused
                pause(0.1);
                if ~isRunning, return; end
            end

            % For each agent, compute desired velocity from global path (waypoint follower),
            % flocking forces, and collision avoidance repulsion.
            nextVel = velocities;
            % compute neighbor info (pairwise distances)
            diffs = reshape(positions,[N 1 2]) - reshape(positions,[1 N 2]); % N x N x 2
            dists = sqrt(squeeze(sum(diffs.^2,3))); dists(eye(N)==1) = inf;

            % compute average spacing metric
            avgNearest = mean(min(dists,[],2));
            interAgentGapHistory(step) = avgNearest;

            % For each agent
            for a=1:N
                pos = positions(a,:);
                vel = velocities(a,:);

                % Global path following: target is next waypoint in path
                P = paths{a};
                if isempty(P)
                    desiredV = [0 0];
                else
                    % progress pathIdx if close to current waypoint
                    idxP = pathIdx(a);
                    % cap
                    idxP = min(idxP, size(P,1));
                    wp = P(idxP,:);
                    towp = wp - pos;
                    if norm(towp) < 1.0 && idxP < size(P,1)
                        idxP = idxP + 1;
                    end
                    pathIdx(a) = idxP;
                    wp = P(idxP,:);
                    towp = wp - pos;
                    desiredDir = safeNorm(towp);
                    desiredV = desiredDir * simData.maxSpeed * 0.9; % follow at 90% top speed
                end

                % Flocking: compute neighbors within a radius
                neighRadius = 15;
                neighbors = find(dists(a,:) < neighRadius);
                cohesionV = [0 0]; alignV = [0 0]; sepV = [0 0];
                if ~isempty(neighbors)
                    % Cohesion: toward average neighbor position
                    avgPos = mean(positions(neighbors,:),1);
                    cohesionV = safeNorm(avgPos - pos) * simData.maxSpeed;
                    % Alignment: match average neighbor velocity
                    avgVel = mean(velocities(neighbors,:),1);
                    alignV = safeNorm(avgVel) * simData.maxSpeed;
                    % Separation: away from close neighbors (within avoid radius)
                    closeNeigh = neighbors(dists(a,neighbors) < simData.avoidRadius);
                    if ~isempty(closeNeigh)
                        rep = zeros(1,2);
                        for idx = closeNeigh
                            v = pos - positions(idx,:);
                            if norm(v)>0
                                rep = rep + (v / (norm(v)^2));
                            end
                        end
                        sepV = safeNorm(rep) * simData.maxSpeed;
                    end
                end

                % Collision avoidance (obstacles and agents)
                repObs = [0 0];
                for oi=1:size(world.obstacles,1)
                    c = world.obstacles(oi,1:2); r = world.obstacles(oi,3);
                    v = pos - c; distc = norm(v);
                    if distc < (r + simData.avoidRadius)
                        % repulsive vector magnitude ~ inverse distance
                        repObs = repObs + safeNorm(v)*(simData.collisionWeight*(r+simData.avoidRadius-distc));
                    end
                end
                % agent-agent repulsion beyond separation above
                repAgents = [0 0];
                closeAgents = find(dists(a,:) < simData.avoidRadius);
                if ~isempty(closeAgents)
                    for j=closeAgents
                        v = pos - positions(j,:);
                        d = norm(v);
                        if d>0
                            repAgents = repAgents + (v / (d^2+1e-6));
                        end
                    end
                    repAgents = safeNorm(repAgents)*simData.maxSpeed;
                end

                % Weighted sum of forces
                flockForce = flockW.coh * cohesionV + flockW.align * alignV + flockW.sep * sepV;
                avoidForce = repObs + repAgents;

                % Merge global desired velocity and flocking/avoidance
                % Allow avoidance to override by mixing
                alphaAvoid = 0.9 * min(1, norm(avoidForce)/(simData.maxSpeed+1e-6));
                blendedV = (1-alphaAvoid)*(0.6*desiredV + 0.4*flockForce) + alphaAvoid*avoidForce;

                % Limit acceleration / speed
                accel = (blendedV - vel) / dt;
                accel = max(-simData.maxAccel, min(simData.maxAccel, accel));
                newVel = vel + accel*dt;
                speed = norm(newVel);
                if speed > simData.maxSpeed
                    newVel = newVel * (simData.maxSpeed / speed);
                end

                nextVel(a,:) = newVel;

                % Record avoidance events
                if norm(avoidForce) > 0.5
                    simData.metrics.avoidEvents(a) = simData.metrics.avoidEvents(a) + 1;
                end
            end % for each agent

            % Integrate positions
            positions = positions + nextVel*dt;
            velocities = nextVel;

            % update plots every few steps to keep speed
            if mod(step,1)==0
                % update agent markers
                colors = lines(N);  % define once before the loop
                for a=1:N
                    if a <= numel(plotStore.agentPlots) && isgraphics(plotStore.agentPlots(a))
                        set(plotStore.agentPlots(a),'XData',positions(a,1),'YData',positions(a,2));
                                    else
                        plotStore.agentPlots(a) = plot(h.ax, positions(a,1), positions(a,2),'o', ...
                            'MarkerFaceColor', colors(a,:), 'MarkerEdgeColor','k','MarkerSize',8);
                    end
                end

                % draw local correction arrows (repulsive vectors) optionally
                delete(plotStore.correctionQuivers); plotStore.correctionQuivers = [];
                for a=1:N
                    % draw small arrow from pos in direction of (blendedV - vel)
                    % approximate: we pick small arrow scaled by avoidance magnitude
                    % reuse nextVel and velocities
                    correction = (nextVel(a,:) - velocities(a,:));
                    if norm(correction)>1e-3
                        plotStore.correctionQuivers(a) = quiver(h.ax, positions(a,1), positions(a,2), correction(1), correction(2), 0.25, 'Color','m','MaxHeadSize',2);
                    end
                end
                drawnow limitrate;
            end

            % Stop when all agents within tolerance of their goal assignments (last waypoint)
            doneCount = 0;
            for a=1:N
                if norm(positions(a,:) - simData.goalAssignments(a,:)) < 2.0
                    doneCount = doneCount + 1;
                end
            end
            if doneCount==N
                break;
            end

            % handle pause toggle
            if get(h.pauseBtn,'Value')==1
                isPaused = true;
            else
                isPaused = false;
            end
            if ~ishandle(h.fig), isRunning=false; break; end
            pause(dt); % control real-time progression
        end % step loop

        totalTime = toc(tStart);
        % compute metrics
        avgPathLen = mean(simData.metrics.pathLengths);
        avgSpacing = mean(interAgentGapHistory(interAgentGapHistory>0));
        totalAvoidEvents = sum(simData.metrics.avoidEvents);

        % display in metrics UI
        summary = sprintf('Time: %.2f s | Steps: %d | avg path len: %.2f | avg spacing: %.2f | avoid events(total): %d',...
            totalTime, step, avgPathLen, avgSpacing, totalAvoidEvents);
        set(h.metrics,'String',summary);
        isRunning = false;
    end

%% ================== Planner & utility functions ===================

    function G = buildVisibilityGraph(nodes, obstacles)
        % build adjacency matrix with euclidean distances if straight-line path not colliding with obstacles
        n = size(nodes,1);
        G = inf(n);
        for i=1:n
            G(i,i)=0;
            for j=i+1:n
                a = nodes(i,:); b=nodes(j,:);
                if ~segmentCollisionWithCircles(a,b,obstacles)
                    G(i,j) = norm(a-b);
                    G(j,i) = G(i,j);
                end
            end
        end
    end

    function D = floydWarshallAllPairs(G)
        % Floyd-Warshall with path reconstruction table
        n = size(G,1);
        dist = G;
        next = zeros(n);
        for i=1:n
            for j=1:n
                if isfinite(G(i,j)) && i~=j, next(i,j)=j; end
            end
        end
        for k=1:n
            for i=1:n
                for j=1:n
                    if dist(i,k)+dist(k,j) < dist(i,j)
                        dist(i,j) = dist(i,k)+dist(k,j);
                        next(i,j) = next(i,k);
                    end
                end
            end
        end
        D.dist = dist; D.next = next;
    end

    function pathIdxs = reconstructFWPath(next, s, t)
        % reconstruct node indices from s to t
        if next(s,t)==0
            pathIdxs = [];
            return;
        end
        u = s;
        pathIdxs = u;
        while u~=t
            u = next(u,t);
            pathIdxs(end+1) = u; %#ok<AGROW>
        end
    end

    function path = runRRT(startPt, goalPt, world, stepSize, goalBias, maxIters)
        % Simple RRT: returns path as Nx2 or [] if none
        nodes = startPt;
        parents = 0;
        for it=1:maxIters
            if rand < goalBias
                q = goalPt;
            else
                q = [world.bounds(1) + (world.bounds(2)-world.bounds(1))*rand, world.bounds(3) + (world.bounds(4)-world.bounds(3))*rand];
            end
            [~, idx] = min(sum((nodes - q).^2,2));
            qNearest = nodes(idx,:);
            v = q - qNearest; d = norm(v);
            if d==0, continue; end
            qNew = qNearest + (stepSize/min(stepSize,d))*v;
            if ~segmentCollisionWithCircles(qNearest, qNew, world.obstacles)
                nodes(end+1,:) = qNew; parents(end+1) = idx; %#ok<AGROW>
                if norm(qNew - goalPt) < 3.0
                    nodes(end+1,:) = goalPt; parents(end+1) = size(nodes,1)-1;
                    % reconstruct path
                    p = size(nodes,1);
                    path = nodes(p,:);
                    while parents(p)~=0
                        p = parents(p);
                        path = [nodes(p,:); path]; %#ok<AGROW>
                    end
                    return;
                end
            end
        end
        path = []; % failed
    end

    function v = safeNorm(x)
        % return normalized vector (1x2) in direction of x; zero if x==0
        if norm(x) < 1e-6
            v = [0 0];
        else
            v = x / norm(x);
        end
    end

end % end main function

%% ===== Collision helper functions (standalone) =====
function hit = segmentCollisionWithCircles(p1,p2, circles)
% circles: Nx3 [x y r]
hit = false;
if isempty(circles), return; end
for i=1:size(circles,1)
    c = circles(i,1:2); r = circles(i,3);
    if segmentCircleIntersect(p1,p2,c,r)
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
tf = (d <= r + 1e-8);
end