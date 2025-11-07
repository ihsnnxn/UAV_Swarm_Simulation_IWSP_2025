function BasicAvoidance()
    clc; clear;

    % === PARAMETERS ===
    numUAVs = 6;
    numObstacles = 3;
    simTime = 30;
    dt = 0.05;
    steps = simTime / dt;
    bounds = [-100 100 -100 100];
    speed = 30;

    avoidanceRadius = 10;     % For obstacle avoidance
    uavAvoidRadius = 10;      % For inter-UAV avoidance
    arrow_length = 8;        % Bigger UAV arrows
    arrow_thickness = 3;    % Thicker lines for drones

    % === INITIAL CONDITIONS ===
    pos = 180 * rand(numUAVs, 2) - 90;
    angles = 2 * pi * rand(numUAVs, 1);
    vel = [cos(angles), sin(angles)] * speed;

    % === OBSTACLES (static) ===
    obstacleCenters = [-40 0; 40 0; 0 50];
    obstacleRadius = 15;

    % === FIGURE SETUP ===
    figure('Color','k');
    axis(bounds);
    axis equal;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
    hold on;
    title('UAV Flocking with Static Obstacles & Mutual Avoidance', ...
        'Color','w', 'FontWeight','bold');

    % === DRAW OBSTACLES ===
    theta = linspace(0, 2*pi, 100);
    for i = 1:numObstacles
        x = obstacleCenters(i,1) + obstacleRadius * cos(theta);
        y = obstacleCenters(i,2) + obstacleRadius * sin(theta);
        fill(x, y, [0.5 0.8 1], 'FaceAlpha', 0.25, ...
            'EdgeColor', [0.3 0.8 1], 'LineWidth', 1.5);
    end

    % === DRAW UAVs AS ARROWS ===
    arrows = gobjects(numUAVs,1);
    for i = 1:numUAVs
        arrows(i) = quiver(pos(i,1), pos(i,2), ...
            arrow_length * vel(i,1)/speed, ...
            arrow_length * vel(i,2)/speed, 0, ...
            'Color', [1 1 1], 'LineWidth', arrow_thickness, 'MaxHeadSize', 3);
    end

    % === MAIN SIMULATION LOOP ===
    for step = 1:steps
        for i = 1:numUAVs
            avoidVec = [0 0];

            %% --- OBSTACLE AVOIDANCE ---
            for j = 1:numObstacles
                diff = pos(i,:) - obstacleCenters(j,:);
                dist = norm(diff);
                if dist < (obstacleRadius + avoidanceRadius)
                    sideVec = [-diff(2), diff(1)];
                    if mod(i,2)==0, sideVec = -sideVec; end
                    avoidVec = avoidVec + sideVec / (norm(sideVec) + 1e-6);
                end
            end

            %% --- INTER-UAV AVOIDANCE ---
            for k = 1:numUAVs
                if k ~= i
                    diff = pos(i,:) - pos(k,:);
                    dist = norm(diff);
                    if dist < uavAvoidRadius
                        avoidVec = avoidVec + diff / (dist + 1e-6);
                    end
                end
            end

            %% --- APPLY AVOIDANCE ---
            if norm(avoidVec) > 0
                vel(i,:) = vel(i,:) + 8 * avoidVec; % Increased repulsion
            end

            %% --- NORMALIZE VELOCITY & MOVE ---
            vel(i,:) = speed * vel(i,:) / (norm(vel(i,:)) + 1e-6);
            pos(i,:) = pos(i,:) + vel(i,:) * dt;

            %% --- BOUNDARY BOUNCE ---
            if pos(i,1) <= bounds(1) || pos(i,1) >= bounds(2)
                vel(i,1) = -vel(i,1);
            end
            if pos(i,2) <= bounds(3) || pos(i,2) >= bounds(4)
                vel(i,2) = -vel(i,2);
            end

            %% --- UPDATE PLOT ---
            set(arrows(i), 'XData', pos(i,1), 'YData', pos(i,2), ...
                'UData', arrow_length * vel(i,1)/speed, ...
                'VData', arrow_length * vel(i,2)/speed);
        end

        drawnow;
        pause(dt * 0.8); % near real-time
    end
end