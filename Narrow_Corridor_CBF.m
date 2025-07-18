clc
clear
close all

%% Define Corridor Parameters

envWidth = 15;
envHeight = 30;
passageWidth = 2;

radius = (envWidth - passageWidth) / 2;
center1X = 0;
center2X = envWidth;
centerY = envHeight / 2;

corridor_center = [(center1X + center2X)/2; centerY];


theta = linspace(pi/2, -pi/2, 100);  % Only left half of the circle
leftHemisphereX = center1X + radius * cos(theta);
leftHemisphereY = centerY + radius * sin(theta);

theta = linspace(pi/2, 3*pi/2, 100);  % Only right half of the circle
rightHemisphereX = center2X + radius * cos(theta);
rightHemisphereY = centerY + radius * sin(theta);

StaticObstacleInfo = NaN(3,2);
StaticObstacleInfo(1:2,1) = [center1X, centerY]';
StaticObstacleInfo(3,1) = radius;
StaticObstacleInfo(1:2,2) = [center2X, centerY]';
StaticObstacleInfo(3,2) = radius;


%% Define the robots
r_a = 1;
n = 0;
samples = 1;
detection_radius = 6;
sampleTime = 0.1;

mpc_horizon = 10;  % Prediction horizon
mpc_dt = sampleTime;  % MPC time step

% For LSTM prediction
window_len = 10;

animation = true;
record_video = true;

h_fig = figure('Name', 'Robot Navigation Animation', 'Position', [100, 100, 600, 1200]);
set(gcf, 'DoubleBuffer', 'on');
set(gcf, 'Color', 'white');

if record_video
    videoFilename = 'narrow_corridor_cbf.mp4';
    videoObj = VideoWriter(videoFilename, 'MPEG-4');
    videoObj.FrameRate = 30;  % Set the frame rate (30 fps is standard)
    videoObj.Quality = 100;   % Set the quality (100 is highest)
    open(videoObj);
    
    % Create the figure with higher resolution for better video quality
    frameCount = 0;
end

while n < samples
    robot2_start = generateRandomCoordinate(0, 15, 4, 5);
    robot2_goal = generateRandomCoordinate(0, 15, 26, 28);
    robot3_start = generateRandomCoordinate(0, 15, 25, 30);
    robot3_goal = generateRandomCoordinate(0, 15, 0, 3);

    robot2 = MbRobot_CBF;
    robot2.name = 'Ally';
    robot2.Start = robot2_start;
    robot2.Goal = robot2_goal;
    robot2.SafeRangeColor = [0.8 1 0.8 1];
    robot2.DetectionRangeColor = [0.8 1 0.8 0.1];
    robot2.SafeRadius = r_a;
    robot2.HeadAngle = atan2(robot2.Goal(2) - robot2.Start(2), ...
                                robot2.Goal(1) - robot2.Start(1));
    robot2.CurrentPose = [robot2.Start; robot2.HeadAngle; robot2.AngularVelocity];
    robot2.CurrentCoord = robot2.CurrentPose(1:2);
    robot2.DetectionRadius = detection_radius;
    robot2.StaticObstacle = StaticObstacleInfo;

    robot2.CurrentState = [robot2.CurrentPose; robot2.Velocity];


    robot3 = MbRobot_CBF;
    robot3.name = 'Adversarial';
    robot3.Start = robot3_start;
    robot3.Goal = robot3_goal;
        
    robot3.SafeRangeColor = [0.6 0.8 1 1];
    robot3.DetectionRangeColor = [0.6 0.8 1 0.1];
    robot3.SafeRadius = r_a;
    robot3InitPose = robot3.CurrentPose;
    robot3.HeadAngle = atan2(robot3.Goal(2) - robot3.Start(2), ...
                                robot3.Goal(1) - robot3.Start(1));
    robot3.CurrentPose = [robot3.Start; robot3.HeadAngle; robot3.AngularVelocity];
    robot3.CurrentCoord = robot3.CurrentPose(1:2);
    robot3.DetectionRadius = detection_radius;
    robot3.StaticObstacle = StaticObstacleInfo;
    robot3.CurrentState = [robot3.CurrentPose; robot3.Velocity];
    
    robot2Poses = [];
    robot3Poses = [];

    robot2_interaction_poses = [];
    robot3_interaction_poses = [];

    robot2States = [];
    robot3States = [];

    interaction_state = struct();
    interaction_state.priority_decided = false;

    % figure('Name', 'Robot Navigation Animation', 'Position', [100, 100, 500, 1000]);
    % set(gcf, 'DoubleBuffer', 'on');  % Enable double buffering for smoother animation
    % 

    heading_smoothing_factor = 0.9;
    for t = 0:sampleTime:50
        % Check if both robots reached their goals
        if robot2.atGoal() && robot3.atGoal()
            disp('Both robots reached their goals!');
            break;
        end
        
        % Store previous states
        if ~robot2.atGoal()
            previousHeading2 = robot2.HeadAngle;
            previousCoord2 = robot2.CurrentCoord;
            dynamic_obstacles = [];
            if calcDist(robot2.CurrentCoord, robot3.CurrentCoord) <= robot2.DetectionRadius
                dynamic_obstacles = [dynamic_obstacles, robot3.CurrentCoord];
            end
            [optimal_control, predicted_path] = runCBF_MPC(robot2, dynamic_obstacles, ...
                                                         mpc_horizon, mpc_dt);
            
            % Apply optimal control to robot1
            applyControlToRobot(robot2, optimal_control, sampleTime);
            
            % Smooth heading
            robot2.HeadAngle = heading_smoothing_factor * previousHeading2 + ...
                              (1-heading_smoothing_factor) * robot2.HeadAngle;

            robot2.AngularVelocity = (robot2.HeadAngle - previousHeading2)/sampleTime;
            robot2.Velocity = norm(robot2.CurrentCoord - previousCoord2)/sampleTime;
            robot2.CurrentPose(4) = robot2.AngularVelocity;

        else
            robot2.DefaultLinearVelocity = 0;
        end
        
        if ~robot3.atGoal()
            previousHeading3 = robot3.HeadAngle;
            previousCoord3 = robot3.CurrentCoord;
            dynamic_obstacles = [];
            if calcDist(robot3.CurrentCoord, robot2.CurrentCoord) <= robot3.DetectionRadius
                dynamic_obstacles = [dynamic_obstacles];
            end
            [optimal_control, predicted_path] = runCBF_MPC(robot3, dynamic_obstacles, ...
                                                         mpc_horizon, mpc_dt);
            
            % Apply optimal control to robot1
            applyControlToRobot(robot3, optimal_control, sampleTime);
            
            % Smooth heading
            robot3.HeadAngle = heading_smoothing_factor * previousHeading3 + ...
                              (1-heading_smoothing_factor) * robot3.HeadAngle;

            robot3.AngularVelocity = (robot3.HeadAngle - previousHeading3)/sampleTime;
            robot3.Velocity = norm(robot3.CurrentCoord - previousCoord3)/sampleTime;
            robot3.CurrentPose(4) = robot3.AngularVelocity;

        else
            robot3.DefaultLinearVelocity = 0;
        end
        
        % Calculate if robots can detect each other
        robotsCanDetectEachOther = calcDist(robot2.CurrentCoord, robot3.CurrentCoord) <= robot2.DetectionRadius;
        
       
        
        % STEP 4: Update positions and stay within boundaries
        if ~robot2.atGoal()
            robot2.CurrentCoord(1) = max(0, min(envWidth, robot2.CurrentCoord(1)));
            robot2.CurrentCoord(2) = max(0, min(envHeight, robot2.CurrentCoord(2)));
            robot2States = [robot2States, [t;robot2.CurrentPose; robot2.Velocity]];
        end
        
        if ~robot3.atGoal()
            robot3.CurrentCoord(1) = max(0, min(envWidth, robot3.CurrentCoord(1)));
            robot3.CurrentCoord(2) = max(0, min(envHeight, robot3.CurrentCoord(2)));
            robot3States = [robot3States, [t;robot3.CurrentPose; robot3.Velocity]];
        end



        if animation
            
            clf(h_fig);
            hold on;
        
            % scatter(obstaclePoints(1,:), obstaclePoints(2,:));
            % plot(robot1.Start(1),robot1.Start(2),'go','MarkerSize',10,'MarkerFaceColor',[0.8500 0.3250 0.0980])
            % plot(robot1.Goal(1),robot1.Goal(2),'rp','MarkerSize',10,'MarkerFaceColor',[0.8500 0.3250 0.0980])
            plot(robot2.Start(1),robot2.Start(2),'go','MarkerSize',10,'MarkerFaceColor','g')
            plot(robot2.Goal(1),robot2.Goal(2),'rp','MarkerSize',10,'MarkerFaceColor','g')
            plot(robot3.Start(1),robot3.Start(2),'go','MarkerSize',10,'MarkerFaceColor','b')
            plot(robot3.Goal(1),robot3.Goal(2),'rp','MarkerSize',10,'MarkerFaceColor','b')

            if ~isempty(robot2Poses)
                plot(robot2Poses(1,:),robot2Poses(2,:),'-',...
                    'Color',[0,255,0]/255,'LineWidth',2.5);
            end

            if ~isempty(robot3Poses)
                plot(robot3Poses(1,:),robot3Poses(2,:),'-',...
                    'Color',[0,0,255]/255,'LineWidth',2.5);
            end

            plotCorridorSimulation(envWidth, envHeight, passageWidth)
            robot2.plotRobot(1); % adjust the frame size as needed
            robot3.plotRobot(1);

            

            drawnow;

            if record_video
                pause(0.001);
                frame = getframe(h_fig);
                writeVideo(videoObj, frame);
                
                frameCount = frameCount + 1;
            end
    

        end


    end


    n = n + 1;
end

if record_video
    frame = getframe(gcf);
    writeVideo(videoObj, frame);
end

if animation && record_video
    close(videoObj);
    fprintf('Video saved as "%s"\n', videoFilename);
end

%%
figure
subplot(2,3,[1 4])
plot(robot2States(2,:), robot2States(3,:),'DisplayName','Ally Trajectory', ...
    'Color',[0 1 0], 'LineWidth', 1.5)
hold on
plot(robot3States(2,:), robot3States(3,:),'DisplayName','Enemy Trajectory', ...
    'Color',[0 0 1], 'LineWidth', 1.5)
xlabel('X Coord')
ylabel('Y Coord')
xlim([0, 15])
ylim([0, 30])
legend
grid on
set(gca, 'FontSize',15)

subplot(2,3,[2,3])
plot(robot2States(1,:), robot2States(end,:),'DisplayName','Ally Speed', ...
    'Color',[0 1 0], 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Velocity (p.u./s)')
grid on
set(gca, 'FontSize',15)

subplot(2,3,[5,6])
plot(robot3States(1,:), robot3States(end,:),'DisplayName','Adversarial Speed', ...
    'Color',[0 0 1], 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Velocity (p.u./s)')
grid on
set(gca, 'FontSize',15)

%%
% assume robot2States and robot3States are [time; x; y; …; speed] layouts:
t2 = robot2States(1,:);   x2 = robot2States(2,:);   y2 = robot2States(3,:);   v2 = robot2States(end,:);
t3 = robot3States(1,:);   v3 = robot3States(end,:);

figure('Position',[100 100 1200 600]);

% — Left: big trajectory plot — %
subplot(2,3,[1 4])
hA = plot(x2, y2, 'g-', 'LineWidth',1.5); hold on;
hE = plot(robot3States(2,:), robot3States(3,:), 'b-', 'LineWidth',1.5);
hDot = plot(NaN, NaN, 'go','MarkerFaceColor','r','MarkerSize',8);
hDot_ad = plot(NaN, NaN, 'bo','MarkerFaceColor','r','MarkerSize',8);
xlabel('X Coord'); ylabel('Y Coord'); 
xlim([0 15]); ylim([0 30]); grid on; set(gca,'FontSize',15)
legend([hA hE],{'Ally Traj','Enemy Traj'});

% — Top-right: ally speed — %
subplot(2,3,[2 3])
hV2 = plot(t2, v2, 'g-','LineWidth',1.5); hold on;
ylim2 = ylim;
hLine2 = plot([t2(1) t2(1)], ylim2, 'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Velocity (p.u./s)');
grid on; set(gca,'FontSize',15)

% — Bottom-right: adversarial speed — %
subplot(2,3,[5 6])
hV3 = plot(t3, v3, 'b-','LineWidth',1.5); hold on;
ylim3 = ylim;
hLine3 = plot([t3(1) t3(1)], ylim3, 'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Velocity (p.u./s)');
grid on; set(gca,'FontSize',15)

% — Animation loop — %
nFrames = numel(t2);          % assume t2 and t3 have same length
dt = mean(diff(t2));          % approximate time per frame
for k = 1:nFrames
    % update dot on trajectory
    set(hDot, 'XData', x2(k), 'YData', y2(k))
    try
        set(hDot_ad, 'XData', x3(k), 'YData', y3(k))
    catch
    end
    
    % update vertical lines on the speed plots
    try
        set(hLine2, 'XData', [t2(k) t2(k)])
    catch
    end

    try
        set(hLine3, 'XData', [t3(k) t3(k)])
    catch
    end
    
    drawnow
    pause(dt)   % slow it down to real time; omit or reduce for faster playback
end



%% Functions
function plotCorridorSimulation(envWidth, envHeight, passageWidth)
   
    
    % Hemisphere parameters
    radius = (envWidth - passageWidth) / 2;
    center1X = 0;
    center2X = envWidth;
    centerY = envHeight / 2;
    
    % Create plot area with appropriate limits
    hold on;
    axis([0, envWidth, 0, envHeight]);
    title('Narrow Corridor Environment');
    xlabel('X');
    ylabel('Y');
    
    % Plot boundaries
    % Left and right vertical boundaries
    plot([0, 0], [0, envHeight], 'k-', 'LineWidth', 2);
    plot([envWidth, envWidth], [0, envHeight], 'k-', 'LineWidth', 2);
    % Top and bottom horizontal boundaries
    plot([0, envWidth], [0, 0], 'k-', 'LineWidth', 2);
    plot([0, envWidth], [envHeight, envHeight], 'k-', 'LineWidth', 2);
    
    % Plot the hemispheres (obstacles)
    % Left hemisphere
    theta = linspace(pi/2, -pi/2, 100);  % Only right half of the circle
    leftHemisphereX = center1X + radius * cos(theta);
    leftHemisphereY = centerY + radius * sin(theta);
    fill(leftHemisphereX, leftHemisphereY, 'k');
    
    % Right hemisphere
    theta = linspace(pi/2, 3*pi/2, 100);  % Only left half of the circle
    rightHemisphereX = center2X + radius * cos(theta);
    rightHemisphereY = centerY + radius * sin(theta);
    fill(rightHemisphereX, rightHemisphereY, 'k');
    
    
end

function coord = generateRandomCoordinate(xMin, xMax, yMin, yMax)
    % Set default parameters if not provided
    if nargin < 1
        xMin = 0;
    end
    if nargin < 2
        xMax = 15;
    end
    if nargin < 3
        yMin = 0;
    end
    if nargin < 4
        yMax = 8;
    end
    
    % Generate random x within the range [xMin, xMax]
    x = xMin + (xMax - xMin) * rand();
    
    % Generate random y within the range [yMin, yMax]
    y = yMin + (yMax - yMin) * rand();
    
    coord = [x;y];
end


function obstacles = generateHemisphereObstacles(centerX, centerY, radius, startAngle, endAngle, numPoints)
    % Generate points along the hemisphere boundary to be used as obstacles
    angles = linspace(startAngle, endAngle, numPoints);
    obstacles = zeros(2, numPoints);
    for i = 1:numPoints
        obstacles(1, i) = centerX + radius * cos(angles(i));
        obstacles(2, i) = centerY + radius * sin(angles(i));
    end
end


function [optimal_control, predicted_path] = runCBF_MPC(robot, dynamic_obstacles, horizon, dt)
    % CBF-MPC: Control Barrier Function Model Predictive Control
    
    current_pos = robot.CurrentCoord;
    goal_pos = robot.Goal;
    static_obstacles = robot.StaticObstacle;
    
    % Desired control (unconstrained)
    desired_direction = goal_pos - current_pos;
    desired_direction = desired_direction / max(norm(desired_direction), 0.1);
    desired_speed = 2.5;
    u_desired = desired_speed * desired_direction;
    
    % Setup CBF constraints
    [A_cbf, b_cbf] = setupCBFConstraints(current_pos, static_obstacles, dynamic_obstacles);
    
    % Solve constrained optimization
    if size(A_cbf, 1) > 0
        optimal_control = solveCBFOptimization(u_desired, current_pos, goal_pos, A_cbf, b_cbf);
    else
        optimal_control = u_desired;
    end
    
    % Predict future path
    predicted_path = predictPath(current_pos, optimal_control, horizon, dt);
end

function [A_cbf, b_cbf] = setupCBFConstraints(current_pos, static_obstacles, dynamic_obstacles)
    % Setup barrier function constraints: A_cbf * u >= b_cbf
    
    A_cbf = [];
    b_cbf = [];
    envWidth = 15;
    envHeight = 30;
    gamma = 1.0;
    
    % Boundary constraints
    safety_margin_walls = 0.8;
    
    % Left wall: h1 = x - safety_margin
    h1 = current_pos(1) - safety_margin_walls;
    if h1 < 5.0
        A_cbf = [A_cbf; 1, 0];
        b_cbf = [b_cbf; -gamma * h1];
    end
    
    % Right wall: h2 = (envWidth - x) - safety_margin
    h2 = (envWidth - current_pos(1)) - safety_margin_walls;
    if h2 < 5.0
        A_cbf = [A_cbf; -1, 0];
        b_cbf = [b_cbf; -gamma * h2];
    end
    
    % Top/bottom boundaries
    safety_margin_boundaries = 1.0;
    h3 = current_pos(2) - safety_margin_boundaries;
    if h3 < 5.0
        A_cbf = [A_cbf; 0, 1];
        b_cbf = [b_cbf; -gamma * h3];
    end
    
    h4 = (envHeight - current_pos(2)) - safety_margin_boundaries;
    if h4 < 5.0
        A_cbf = [A_cbf; 0, -1];
        b_cbf = [b_cbf; -gamma * h4];
    end
    
    % Static obstacles (corridor walls)
    safety_margin_static = 0.8;
    for i = 1:size(static_obstacles, 2)
        obs_center = static_obstacles(1:2, i);
        obs_radius = static_obstacles(3, i);
        
        distance_to_obs = norm(current_pos - obs_center);
        h_static = distance_to_obs - obs_radius - safety_margin_static;
        
        if h_static < 5.0
            gradient = (current_pos - obs_center) / distance_to_obs;
            A_cbf = [A_cbf; gradient'];
            b_cbf = [b_cbf; -gamma * h_static];
        end
    end
    
    % Dynamic obstacles (other robots)
    safety_margin_dynamic = 3;
    for i = 1:size(dynamic_obstacles, 2)
        obs_pos = dynamic_obstacles(:, i);
        
        distance_to_obs = norm(current_pos - obs_pos);
        h_dynamic = distance_to_obs - safety_margin_dynamic;
        
        if h_dynamic < 6
            gradient = (current_pos - obs_pos) / distance_to_obs;
            A_cbf = [A_cbf; gradient'];
            b_cbf = [b_cbf; -gamma * h_dynamic];
        end
    end
end

function optimal_control = solveCBFOptimization(u_desired, current_pos, goal_pos, A_cbf, b_cbf)
    % Solve QP: min 0.5*u'*H*u + f'*u subject to A_cbf*u >= b_cbf
    
    control_weight = 1.0;
    goal_weight = 0.5;
    
    % QP matrices
    H = control_weight * eye(2);
    goal_direction = goal_pos - current_pos;
    goal_direction = goal_direction / max(norm(goal_direction), 0.1);
    f = -control_weight * u_desired - goal_weight * goal_direction;
    
    % Convert to quadprog format: A*u <= b
    A_ineq = -A_cbf;
    b_ineq = -b_cbf;
    
    % Velocity bounds
    v_max = 1.5;
    lb = [-v_max; -v_max];
    ub = [v_max; v_max];
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    try
        optimal_control = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
        if isempty(optimal_control)
            optimal_control = [0; 0];
        end
    catch
        optimal_control = [0; 0];
    end
end

function path = predictPath(start_pos, control, horizon, dt)
    % Simple path prediction
    path = zeros(2, horizon+1);
    path(:, 1) = start_pos;
    
    for i = 1:horizon
        path(:, i+1) = path(:, i) + control * dt;
    end
end

function applyControlToRobot(robot, control, dt)
    % Apply the optimal control to the robot
    
    % Update position
    robot.CurrentCoord = robot.CurrentCoord + control * dt;
    
    % Update heading based on velocity direction
    if norm(control) > 0.1
        robot.HeadAngle = atan2(control(2), control(1));
    end
    
    % Update the robot's pose
    robot.CurrentPose(1:2) = robot.CurrentCoord;
    robot.CurrentPose(3) = robot.HeadAngle;
    
end


function progress = calcProgressAlongPath(start, goal, current)
    path_vector = goal - start;
    path_length = norm(path_vector);
    
    % Vector from start to current position
    current_vector = current - start;
    
    % Project current position onto path
    projection = dot(current_vector, path_vector/path_length);
    
    % Normalize to get progress (0-1)
    progress = min(max(projection/path_length, 0), 1);
end