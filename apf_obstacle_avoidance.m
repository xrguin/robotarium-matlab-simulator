%% Artificial Potential Field (APF) Obstacle Avoidance
% This script demonstrates APF-based navigation where robots avoid each other
% while moving towards their goals, replicating the functionality of Simulation_APF.m
% in the Robotarium format.

%% Initialize Robotarium
N = 2; % Number of robots (2 robots avoiding each other)
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Parameters matching original simulation
sampleTime = r.time_step; % Use Robotarium's time step
simulationTime = 30;
iterations = simulationTime / sampleTime;

% APF parameters
detection_radius = 15;
safe_radius = 1;
attraction_factor = 1;
repulsion_factors = [1.2, 1]; % Different repulsion factors for each robot

% Goal radius for checking arrival
goal_radius = 0.5;

% Generate initial positions and goals similar to original
test_site_size = 15;
distance = 8;

% Robot 1 (green robot in original)
rng('shuffle');
start1 = rand(1,2) * test_site_size;
goal1 = generateRandomGoal(start1, distance, test_site_size);

% Robot 2 (blue robot in original) - generate intercepting path
[start2, goal2] = generateInterceptingPath(start1, goal1, test_site_size, distance);

% Convert to Robotarium coordinates (centered at origin)
robotarium_offset = [r.boundaries(2)/2; r.boundaries(4)/2];
start1_r = start1' - test_site_size/2;
start2_r = start2' - test_site_size/2;
goal1_r = goal1' - test_site_size/2;
goal2_r = goal2' - test_site_size/2;

% Set initial conditions
initial_conditions = [start1_r, start2_r; zeros(1, N)];
r.set_poses(initial_conditions);

% Initialize pose history for plotting
pose_history = zeros(3, N, iterations);
pose_idx = 1;

% Create unicycle position controller for comparison
unicycle_controller = create_unicycle_position_controller();

%% Main simulation loop
for t = 1:iterations
    % Get current poses
    x = r.get_poses();
    
    % Store pose history
    pose_history(:, :, pose_idx) = x;
    pose_idx = pose_idx + 1;
    
    % Calculate APF control for each robot
    dxu = zeros(2, N);
    
    for i = 1:N
        % Current position and orientation
        current_pos = x(1:2, i);
        current_theta = x(3, i);
        
        % Goal for this robot
        if i == 1
            goal = goal1_r;
            rep_factor = repulsion_factors(1);
        else
            goal = goal2_r;
            rep_factor = repulsion_factors(2);
        end
        
        % Find other robots (obstacles)
        obstacles = x(1:2, setdiff(1:N, i));
        
        % Calculate APF forces
        [F_att, F_rep] = calculate_apf_forces(current_pos, goal, obstacles, ...
            detection_radius, attraction_factor, rep_factor);
        
        % Combined force
        F_total = F_att + F_rep;
        
        % Normalize
        if norm(F_total) > 0
            F_total = F_total / norm(F_total);
        end
        
        % Convert to unicycle control
        desired_theta = atan2(F_total(2), F_total(1));
        
        % Angular velocity control
        theta_error = desired_theta - current_theta;
        % Wrap angle to [-pi, pi]
        theta_error = atan2(sin(theta_error), cos(theta_error));
        
        % Set velocities (linear and angular)
        v = 1; % Linear velocity matching original
        w = 4 * theta_error; % P controller for angular velocity
        
        dxu(:, i) = [v; w];
    end
    
    % Set velocities and step
    r.set_velocities(1:N, dxu);
    r.step();
    
    % Check if robots reached goals
    x_current = r.get_poses();
    dist1 = norm(x_current(1:2, 1) - goal1_r);
    dist2 = norm(x_current(1:2, 2) - goal2_r);
    
    if dist1 < goal_radius || dist2 < goal_radius
        break;
    end
end

% Plot trajectories
figure();
hold on;

% Plot trajectories
for i = 1:N
    trajectory = squeeze(pose_history(1:2, i, 1:pose_idx-1));
    if i == 1
        plot(trajectory(1,:), trajectory(2,:), '-', 'Color', [0,1,0], 'LineWidth', 2);
    else
        plot(trajectory(1,:), trajectory(2,:), '-', 'Color', [0,0,1], 'LineWidth', 2);
    end
end

% Plot start and goal positions
plot(start1_r(1), start1_r(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal1_r(1), goal1_r(2), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(start2_r(1), start2_r(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(goal2_r(1), goal2_r(2), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Add legends
legend('Robot 1 Trajectory', 'Robot 2 Trajectory', ...
    'Robot 1 Start', 'Robot 1 Goal', 'Robot 2 Start', 'Robot 2 Goal');
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('APF-based Robot Navigation with Obstacle Avoidance');

% Always call debug at the end
r.debug();

%% Helper Functions
function [F_att, F_rep] = calculate_apf_forces(current_pos, goal, obstacles, ...
    detection_radius, zeta, eta)
    % Calculate attractive force
    goal_vec = goal - current_pos;
    F_att = zeta * goal_vec;
    
    % Calculate repulsive force
    F_rep = zeros(2, 1);
    
    for i = 1:size(obstacles, 2)
        obstacle_pos = obstacles(:, i);
        dist = norm(current_pos - obstacle_pos);
        
        if dist <= detection_radius && dist > 0
            obstacle_vec = current_pos - obstacle_pos;
            F_rep = F_rep + eta * (1/dist - 1/detection_radius) * (obstacle_vec / dist);
        end
    end
end

function goal = generateRandomGoal(start, distance, test_site_size)
    % Generate a random goal at specified distance from start
    while true
        x = rand() * test_site_size;
        y = rand() * test_site_size;
        
        if sqrt((x - start(1))^2 + (y - start(2))^2) > distance
            goal = [x; y];
            break;
        end
    end
end

function [newStart, newGoal] = generateInterceptingPath(refStart, refGoal, ...
    test_site_size, safeRadius)
    % Generate an intercepting path for the second robot
    while true
        dir2 = refGoal - refStart';
        t = rand() * 0.5;
        
        interceptPoint = refStart' + t * dir2;
        
        if all(interceptPoint <= test_site_size) && all(interceptPoint >= 0)
            distance = norm(interceptPoint - refStart');
            
            % Generate random point on circle
            theta = 2 * pi * rand();
            newStart = interceptPoint + distance * [cos(theta); sin(theta)];
            
            % Check if within bounds
            if all(newStart >= 0) && all(newStart <= test_site_size)
                if norm(newStart - refStart') > safeRadius
                    dir3 = interceptPoint - newStart;
                    newGoal = interceptPoint + dir3;
                    
                    % Ensure goal is within bounds
                    newGoal = max(0, min(test_site_size, newGoal));
                    break;
                end
            end
        end
    end
    
    newStart = newStart';
    newGoal = newGoal';
end