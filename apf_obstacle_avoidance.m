%% Artificial Potential Field (APF) Obstacle Avoidance
% This script demonstrates APF-based navigation where robots avoid each other
% while moving towards their goals, replicating the functionality of Simulation_APF.m
% in the Robotarium format.

%% Initialize Robotarium
N = 2; % Number of robots (2 robots avoiding each other)

% Parameters matching original simulation
sampleTime = 0.033; % Typical Robotarium time step
simulationTime = 300;
iterations = ceil(simulationTime / sampleTime);

% APF parameters
detection_radius = 0.5;  % Scaled for Robotarium arena
safe_radius = 0.3;       % Minimum safe distance between robots
attraction_factor = 1;
repulsion_factor = 1; % Different repulsion factors for each robot
safe_distance = 1;

% Goal radius for checking arrival
goal_radius = 0.1;

% Robotarium arena bounds
x_bound = 1.5;  % Leave small margin from actual boundary
y_bound = 0.9;  % Leave small margin from actual boundary

% Robot 1 (green robot in original)
rng('shuffle');
start1 = [(rand()*2-1)*x_bound; (rand()*2-1)*y_bound];
% Generate goal at least 1 meter away
angle1 = rand() * 2 * pi;
distance = 1.0 + rand() * 0.5;  % 1 to 1.5 meters
goal1 = start1 + distance * [cos(angle1); sin(angle1)];
% Ensure goal is within bounds
goal1(1) = max(-x_bound, min(x_bound, goal1(1)));
goal1(2) = max(-y_bound, min(y_bound, goal1(2)));

% Robot 2 (blue robot in original) - generate intercepting path
% Robot 2 - generate intercepting path using similar logic to generateInterceptingPath
[start2, goal2, interceptPoint] = generateInterceptingPathRobotarium(start1, goal1, x_bound, y_bound, safe_distance);

goal_all = [goal1, goal2];

% No conversion needed - already in Robotarium coordinates
start1_r = start1;
start2_r = start2;
goal1_r = goal1;
goal2_r = goal2;

% Set initial conditions
initial_conditions = [start1_r, start2_r; zeros(1, N)];

% Initialize Robotarium with initial conditions
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

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

        goal = goal_all(:,i);
        
        % Find other robots (obstacles)
        obstacles = x(1:2, setdiff(1:N, i));
        
        % Calculate APF forces
        [F_att, F_rep] = calculate_apf_forces(current_pos, goal, obstacles, ...
            detection_radius, attraction_factor, repulsion_factor);
        
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
        v = 0.1; % Linear velocity scaled for Robotarium
        w = 2 * theta_error; % P controller for angular velocity
        
        dxu(:, i) = [v; w];
    end
    
    % Set velocities and step
    r.set_velocities(1:N, dxu);
    r.step();
    
    % Check if robots reached goals using current poses
    dist1 = norm(x(1:2, 1) - goal1_r);
    dist2 = norm(x(1:2, 2) - goal2_r);
    pause(0.01)
    
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
axis([-1.6 1.6 -1 1]);  % Set axis to match Robotarium boundaries
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

function F_boundary = calculate_boundary_forces(current_pos, x_bound, y_bound)
    % Calculate repulsive forces from boundaries to keep robot within arena
    F_boundary = zeros(2, 1);
    
    % Boundary detection distance
    boundary_threshold = 0.2;  % Start repulsion when within 20cm of boundary
    boundary_strength = 2.0;   % Boundary repulsion strength
    
    x = current_pos(1);
    y = current_pos(2);
    
    % Left boundary (x = -x_bound)
    dist_left = x - (-x_bound);
    if dist_left < boundary_threshold && dist_left > 0
        F_boundary(1) = F_boundary(1) + boundary_strength * (1/dist_left - 1/boundary_threshold) / dist_left^2;
    end
    
    % Right boundary (x = x_bound)
    dist_right = x_bound - x;
    if dist_right < boundary_threshold && dist_right > 0
        F_boundary(1) = F_boundary(1) - boundary_strength * (1/dist_right - 1/boundary_threshold) / dist_right^2;
    end
    
    % Bottom boundary (y = -y_bound)
    dist_bottom = y - (-y_bound);
    if dist_bottom < boundary_threshold && dist_bottom > 0
        F_boundary(2) = F_boundary(2) + boundary_strength * (1/dist_bottom - 1/boundary_threshold) / dist_bottom^2;
    end
    
    % Top boundary (y = y_bound)
    dist_top = y_bound - y;
    if dist_top < boundary_threshold && dist_top > 0
        F_boundary(2) = F_boundary(2) - boundary_strength * (1/dist_top - 1/boundary_threshold) / dist_top^2;
    end
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathRobotarium(refStart, refGoal, x_bound, y_bound, safeRadius)
    % Generate intercepting path for Robot 2 based on Robot 1's path
    % Adapted from generateInterceptingPath.m for Robotarium coordinates
    
    max_attempts = 100;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % Direction from start to goal
        dir = refGoal - refStart;
        
        % Random point along the path (30-70% of the way)
        t = 0.3 + rand() * 0.4;
        interceptPoint = refStart + t * dir;
        
        % Check if intercept point is within bounds
        if interceptPoint(1) >= -x_bound && interceptPoint(1) <= x_bound && ...
           interceptPoint(2) >= -y_bound && interceptPoint(2) <= y_bound
            
            % Distance from intercept point to robot 1 start
            distance = norm(interceptPoint - refStart);
            
            % Generate start position for robot 2 on circle around intercept point
            newStart = generateRandomPointOnCircleRobotarium(interceptPoint, distance, x_bound, y_bound);
            
            % Check if new start is far enough from robot 1 start
            if norm(newStart - refStart) > safeRadius
                % Goal is on opposite side of intercept point
                dir_to_goal = interceptPoint - newStart;
                newGoal = interceptPoint + dir_to_goal;
                
                % Ensure goal is within bounds
                newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
                newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
                
                % Verify the paths will actually intersect
                if norm(newGoal - interceptPoint) > 0.1  % Goal is reasonably far from intercept
                    return;
                end
            end
        end
    end
    
    % Fallback: simple crossing paths
    newStart = [-refStart(1); -refStart(2)];  % Opposite quadrant
    newGoal = [-refGoal(1); -refGoal(2)];     % Opposite goal
    interceptPoint = (refStart + refGoal) / 2; % Midpoint
end

function point = generateRandomPointOnCircleRobotarium(center, radius, x_bound, y_bound)
    % Generate random point on circle that's within Robotarium bounds
    
    max_attempts = 50;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % Generate random angle
        theta = 2 * pi * rand();
        
        % Calculate point on circle
        x = center(1) + radius * cos(theta);
        y = center(2) + radius * sin(theta);
        
        % Check if point is within bounds
        if x >= -x_bound && x <= x_bound && y >= -y_bound && y <= y_bound
            point = [x; y];
            return;
        end
    end
    
    % Fallback: return a point that's definitely within bounds
    point = [(rand()*2-1)*x_bound*0.8; (rand()*2-1)*y_bound*0.8];
end