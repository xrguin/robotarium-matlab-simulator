%% Control Barrier Function (CBF) Obstacle Avoidance
% This script demonstrates CBF-based navigation where robots avoid each other
% while moving towards their goals using Control Barrier Functions
% Implemented using Option A: Position-based CBF with unicycle conversion
init
%% TUNABLE PARAMETERS - ADJUST THESE TO CHANGE ROBOT BEHAVIOR

% === CBF Parameters ===
gamma_cbf = 2.0;                % CBF conservativeness (higher = more conservative)
activation_distance = 0.8;      % Distance to activate CBF constraints (m)
safety_margin_robots = 0.3;     % Minimum distance between robots (m)
safety_margin_boundaries = 0.2; % Minimum distance to arena boundaries (m)

% === Path Generation Parameters ===
min_path_length = 1;          % Minimum distance between start and goal (m)
max_path_length = 2;          % Maximum distance between start and goal (m)
boundary_buffer = 0.5;          % Keep initial positions away from walls (m)
min_robot_start_separation = 1.5; % Minimum distance between robot start positions (m)
max_generation_attempts = 20;   % Max attempts to generate good paths
time_intersection_tolerance = 0.5; % Max time difference for meaningful intersection (s)

% === Control Parameters ===
heading_gain = 4.0;             % P-gain for angular velocity control
heading_smooth_factor = 0.7;    % Heading smoothing (0-1, higher = smoother)
linear_velocity_gain = 1.0;     % Scaling for linear velocity

% === Stuck Detection Parameters ===
max_time_at_position = 5.0;     % Time before robot considered stuck (s)
position_threshold = 0.05;      % Movement threshold to detect stuck (m)
distance_tracking_threshold = 0.001; % Minimum movement to track for efficiency (m)

% === Velocity Monitoring Parameters ===
monitor_velocity_violations = true; % Track velocity limit violations
velocity_safety_margin = 0.02;     % Safety margin for velocity limits (m/s)

% === Visualization Parameters ===
show_safety_circles = true;     % Show safety margins around robots
show_detection_range = true;    % Show CBF activation range
show_trajectories = true;       % Show real-time trajectory trails
show_intended_paths = true;     % Show dashed lines for intended paths

%% SYSTEM INITIALIZATION

% === Robotarium Constants ===
N = 2; % Number of robots
max_linear_velocity = 0.1;      % From ARobotarium.m
max_angular_velocity = 2*pi;    % Conservative limit
robot_diameter = 0.11;
collision_diameter = 0.135;
boundaries = [-1.6, 1.6, -1, 1]; % [x_min, x_max, y_min, y_max]

% === Simulation Parameters ===
sampleTime = 0.033; % Robotarium time step
simulationTime = 30; % Maximum simulation time
iterations = ceil(simulationTime / sampleTime);
goal_radius = 0.1; % Distance to consider goal reached

% === Arena Bounds ===
x_bound = boundaries(2) - safety_margin_boundaries;
y_bound = boundaries(4) - safety_margin_boundaries;

% Safe generation bounds (using parameters from above)
x_bound_gen = x_bound - boundary_buffer;
y_bound_gen = y_bound - boundary_buffer;

% Path generation bounds (using parameters from above)
% min_path_length and max_path_length defined in tunable parameters

%% PATH GENERATION
rng('shuffle');

% Robot 1 (green) - random start and goal with minimum path length
start1 = [(rand()*2-1)*x_bound_gen; (rand()*2-1)*y_bound_gen];

% Generate goal ensuring minimum distance
path_length = min_path_length + rand() * (max_path_length - min_path_length);
angle1 = rand() * 2 * pi;
goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));

% Regenerate if path is too short after bounding
max_attempts = 20;
attempt = 0;
while norm(goal1 - start1) < min_path_length && attempt < max_attempts
    angle1 = rand() * 2 * pi;
    goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
    goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
    goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));
    attempt = attempt + 1;
end

% Robot 2 (blue) - intercepting path (simplified APF approach)
[start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
    x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);

% Ensure robots don't start too close together (using parameter from above)
attempts = 0;
while norm(start2 - start1) < min_robot_start_separation && attempts < max_generation_attempts
    [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
        x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);
    attempts = attempts + 1;
end

if attempts >= max_generation_attempts
    fprintf('Warning: Could not find well-separated initial positions\n');
end

% Calculate initial headings (facing towards goals)
initial_heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
initial_heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));

% Set initial conditions with proper headings
initial_conditions = [start1, start2; initial_heading1, initial_heading2];

% Initialize Robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% SIMULATION INITIALIZATION

% === Data Storage ===
pose_history = zeros(3, N, iterations);
velocity_history = zeros(2, N, iterations); % [v; omega]
cbf_active_history = zeros(N, iterations);  % Track when CBF is active
pose_idx = 1;

% === Control State Initialization ===
heading_history = zeros(N, 5); % Keep last 5 headings for smoothing
for i = 1:N
    heading_history(i, :) = initial_conditions(3, i);
end

% === Monitoring Initialization ===
stuck_timer = zeros(N, 1);
last_positions = initial_conditions(1:2, :);
stuck_warned = false(N, 1);  % To avoid repeated warnings

% === Performance Metrics ===
path_start_time = zeros(N, 1);
path_end_time = zeros(N, 1);
straight_line_distance = [norm(goal1 - start1); norm(goal2 - start2)];
total_distance_traveled = zeros(N, 1);

% === Velocity Violation Tracking ===
if monitor_velocity_violations
    linear_violations = 0;
    angular_violations = 0;
    total_commands = 0;
end

% === Visualization Setup ===
fig_cbf = figure('Name', 'CBF Constraint Visualization', 'Position', [100, 100, 800, 600]);

%% MAIN SIMULATION LOOP
for t = 1:iterations
    % Get current poses
    x = r.get_poses();
    current_time = t * sampleTime;
    
    % Store pose history
    pose_history(:, :, pose_idx) = x;
    
    % Check for stuck robots and track distance
    for i = 1:N
        movement = norm(x(1:2, i) - last_positions(:, i));
        
        % Always track distance traveled for efficiency calculation
        if movement > distance_tracking_threshold
            total_distance_traveled(i) = total_distance_traveled(i) + movement;
        end
        
        % Separate stuck detection
        if movement < position_threshold
            stuck_timer(i) = stuck_timer(i) + sampleTime;
            if stuck_timer(i) > max_time_at_position && ~stuck_warned(i)
                fprintf('Warning: Robot %d appears stuck at position (%.2f, %.2f) at time %.1fs\n', ...
                        i, x(1,i), x(2,i), current_time);
                stuck_warned(i) = true;
            end
        else
            stuck_timer(i) = 0;
            stuck_warned(i) = false;
        end
        last_positions(:, i) = x(1:2, i);
    end
    
    % Calculate CBF control for each robot
    dxu = zeros(2, N); % [v; omega] for each robot
    
    for i = 1:N
        % Current state
        current_pos = x(1:2, i);
        current_heading = x(3, i);
        
        % Goal for this robot
        if i == 1
            goal = goal1;
            color_idx = 1; % green
        else
            goal = goal2;
            color_idx = 2; % blue
        end
        
        % Find other robots (dynamic obstacles)
        other_robots = x(1:2, setdiff(1:N, i));
        
        % Desired velocity (before CBF)
        goal_direction = goal - current_pos;
        distance_to_goal = norm(goal_direction);
        
        if distance_to_goal > goal_radius
            % Normalize goal direction
            goal_direction = goal_direction / distance_to_goal;
            u_desired = linear_velocity_gain * max_linear_velocity * goal_direction;
        else
            % At goal - stop
            u_desired = [0; 0];
        end
        
        % Setup CBF constraints with symmetric weights for robot-robot avoidance
        [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(current_pos, other_robots, ...
            boundaries, safety_margin_robots, safety_margin_boundaries, ...
            activation_distance, gamma_cbf, true);  % true = symmetric CBF
        
        % Store CBF activation status
        cbf_active_history(i, pose_idx) = cbf_active;
        
        % Solve CBF-QP for safe velocity
        if cbf_active
            u_safe = solveCBFQP(u_desired, A_cbf, b_cbf, max_linear_velocity);
        else
            u_safe = u_desired;
        end
        
        % Apply velocity ramp-up in first second for smooth start
        if current_time < 1.0
            velocity_scale = current_time;  % 0 to 1 over first second
            u_safe = u_safe * velocity_scale;
        end
        
        % Convert to unicycle control
        if norm(u_safe) > 0.01 % Only compute heading if moving
            desired_heading = atan2(u_safe(2), u_safe(1));
            
            % Update heading history
            heading_history(i, :) = [desired_heading, heading_history(i, 1:end-1)];
            
            % Apply heading smoothing
            weights = [0.5, 0.25, 0.15, 0.07, 0.03]';
            smoothed_heading = atan2(sum(sin(heading_history(i,:)).*weights'), ...
                                   sum(cos(heading_history(i,:)).*weights'));
            
            % Compute angular velocity
            heading_error = wrapToPi(smoothed_heading - current_heading);
            omega = heading_gain * heading_error;
            
            % Linear velocity
            v = norm(u_safe);
            
            % Apply smooth factor to linear velocity based on heading error
            v = v * (1 - 0.5*abs(heading_error)/pi); % Slow down during turns
            
            % Allow speed recovery after obstacle avoidance
            % If CBF is not active and we're far from obstacles, gradually return to desired speed
            if ~cbf_active && v < max_linear_velocity * 0.9
                % Gradual speed recovery
                v = min(v * 1.1, max_linear_velocity);
            end
            
            % Post-processing: Scale entire [v,ω] command proportionally if needed
            v_limited = min(v, max_linear_velocity);
            omega_limited = max(-max_angular_velocity, min(max_angular_velocity, omega));
            
            % Check if scaling is needed
            scale_factor = 1.0;
            if v > max_linear_velocity
                scale_factor = min(scale_factor, max_linear_velocity / v);
            end
            if abs(omega) > max_angular_velocity
                scale_factor = min(scale_factor, max_angular_velocity / abs(omega));
            end
            
            % Apply proportional scaling
            v = v * scale_factor;
            omega = omega * scale_factor;
            
            % Track violations for performance metrics
            if monitor_velocity_violations
                total_commands = total_commands + 1;
                if v_limited < v / scale_factor
                    linear_violations = linear_violations + 1;
                end
                if abs(omega_limited) < abs(omega / scale_factor)
                    angular_violations = angular_violations + 1;
                end
            end
        else
            v = 0;
            omega = 0;
        end
        
        % Set control inputs
        dxu(:, i) = [v; omega];
        velocity_history(:, i, pose_idx) = [v; omega];
    end
    
    % Update real-time visualization
    updateRealtimeVisualization(r.figure_handle, x, pose_history, pose_idx, ...
        [start1, start2], [goal1, goal2], cbf_active_history);
    
    % Set velocities and step
    r.set_velocities(1:N, dxu);
    r.step();
    
    pose_idx = pose_idx + 1;
    
    % Check if robots reached goals and record times
    dist1 = norm(x(1:2, 1) - goal1);
    dist2 = norm(x(1:2, 2) - goal2);
    
    % Record goal reaching times
    if dist1 < goal_radius && path_end_time(1) == 0
        path_end_time(1) = current_time;
        fprintf('Robot 1 reached goal at %.2f seconds\n', current_time);
    end
    if dist2 < goal_radius && path_end_time(2) == 0
        path_end_time(2) = current_time;
        fprintf('Robot 2 reached goal at %.2f seconds\n', current_time);
    end
    
    if dist1 < goal_radius && dist2 < goal_radius
        fprintf('\nBoth robots reached their goals!\n');
        % Print performance metrics
        for i = 1:N
            efficiency = straight_line_distance(i) / total_distance_traveled(i);
            fprintf('Robot %d - Path efficiency: %.2f%% (traveled %.2fm for %.2fm straight-line)\n', ...
                i, efficiency*100, total_distance_traveled(i), straight_line_distance(i));
        end
        
        % Print velocity violation statistics
        if monitor_velocity_violations && total_commands > 0
            linear_violation_rate = (linear_violations / total_commands) * 100;
            angular_violation_rate = (angular_violations / total_commands) * 100;
            fprintf('\nVelocity Violation Statistics:\n');
            fprintf('  Linear velocity violations: %d/%d (%.1f%%)\n', ...
                    linear_violations, total_commands, linear_violation_rate);
            fprintf('  Angular velocity violations: %d/%d (%.1f%%)\n', ...
                    angular_violations, total_commands, angular_violation_rate);
            fprintf('  Total commands processed: %d\n', total_commands);
        end
        break;
    end
end

%% POST-SIMULATION ANALYSIS

% Generate final visualization
visualizeCBFConstraints(fig_cbf, pose_history(:,:,1:pose_idx-1), ...
    velocity_history(:,:,1:pose_idx-1), cbf_active_history(:,1:pose_idx-1), ...
    [start1, start2], [goal1, goal2], boundaries, ...
    safety_margin_robots, safety_margin_boundaries, activation_distance);

% Clean up Robotarium
r.debug();

%% HELPER FUNCTIONS

function [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(pos, other_robots, ...
    boundaries, margin_robots, margin_boundaries, activation_dist, gamma, symmetric)
    % Setup CBF constraint matrices A_cbf * u >= b_cbf
    % symmetric: if true, use symmetric weights for robot-robot avoidance
    
    if nargin < 8
        symmetric = false;
    end
    
    A_cbf = [];
    b_cbf = [];
    cbf_active = false;
    
    % Boundary constraints (full weight - no symmetry needed)
    x = pos(1);
    y = pos(2);
    
    % Left boundary: h = x - x_min - margin
    h_left = x - boundaries(1) - margin_boundaries;
    if h_left < activation_dist
        A_cbf = [A_cbf; 1, 0];
        b_cbf = [b_cbf; -gamma * h_left];
        cbf_active = true;
    end
    
    % Right boundary: h = x_max - x - margin
    h_right = boundaries(2) - x - margin_boundaries;
    if h_right < activation_dist
        A_cbf = [A_cbf; -1, 0];
        b_cbf = [b_cbf; -gamma * h_right];
        cbf_active = true;
    end
    
    % Bottom boundary: h = y - y_min - margin
    h_bottom = y - boundaries(3) - margin_boundaries;
    if h_bottom < activation_dist
        A_cbf = [A_cbf; 0, 1];
        b_cbf = [b_cbf; -gamma * h_bottom];
        cbf_active = true;
    end
    
    % Top boundary: h = y_max - y - margin
    h_top = boundaries(4) - y - margin_boundaries;
    if h_top < activation_dist
        A_cbf = [A_cbf; 0, -1];
        b_cbf = [b_cbf; -gamma * h_top];
        cbf_active = true;
    end
    
    % Robot constraints with optional symmetric weights
    for j = 1:size(other_robots, 2)
        robot_pos = other_robots(:, j);
        distance = norm(pos - robot_pos);
        h_robot = distance - margin_robots;
        
        if h_robot < activation_dist && distance > 0
            gradient = (pos - robot_pos) / distance;
            
            % Apply symmetric weight for robot-robot avoidance
            if symmetric
                weight = 0.5;  % Each robot takes half responsibility
                A_cbf = [A_cbf; weight * gradient'];
                b_cbf = [b_cbf; -gamma * h_robot * weight];
            else
                A_cbf = [A_cbf; gradient'];
                b_cbf = [b_cbf; -gamma * h_robot];
            end
            cbf_active = true;
        end
    end
end

function u_safe = solveCBFQP(u_desired, A_cbf, b_cbf, v_max)
    % Solve QP: min ||u - u_desired||^2 subject to A_cbf * u >= b_cbf
    % Uses conservative velocity bounds to prevent actuator limit violations
    
    % Get velocity safety margin from base workspace
    velocity_safety_margin = evalin('base', 'velocity_safety_margin');
    
    % QP formulation: min 0.5*u'*H*u + f'*u
    H = eye(2);
    f = -u_desired;
    
    % Convert CBF constraints to QP format (A*u <= b)
    A_ineq = -A_cbf;
    b_ineq = -b_cbf;
    
    % Conservative velocity bounds (to ensure ||u|| <= v_max)
    % Use v_max/sqrt(2) to ensure the 2-norm constraint is satisfied
    v_max_conservative = (v_max - velocity_safety_margin) / sqrt(2);
    lb = [-v_max_conservative; -v_max_conservative];
    ub = [v_max_conservative; v_max_conservative];
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    try
        % Don't use initial guess to let solver find feasible solution
        u_safe = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
        if isempty(u_safe) || any(isnan(u_safe))
            u_safe = [0; 0]; % Stop if no solution
        end
    catch
        % If QP fails, try a simple feasible solution
        u_safe = [0; 0];
    end
end

function visualizeCBFConstraints(fig, poses, velocities, cbf_active, starts, goals, ...
    boundaries, margin_robots, margin_boundaries, activation_dist)
    % Create visualization of trajectories and CBF constraint regions
    
    figure(fig);
    clf;
    
    % Main trajectory plot
    subplot(2,2,[1,3]);
    hold on;
    
    % Plot boundaries
    rectangle('Position', [boundaries(1), boundaries(3), ...
        boundaries(2)-boundaries(1), boundaries(4)-boundaries(3)], ...
        'EdgeColor', 'k', 'LineWidth', 2);
    
    % Plot CBF activation boundaries (inner rectangle)
    inner_bounds = [boundaries(1)+margin_boundaries, boundaries(3)+margin_boundaries, ...
        boundaries(2)-margin_boundaries, boundaries(4)-margin_boundaries];
    rectangle('Position', [inner_bounds(1), inner_bounds(2), ...
        inner_bounds(3)-inner_bounds(1), inner_bounds(4)-inner_bounds(2)], ...
        'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1);
    
    % Plot trajectories
    colors = {[0,1,0], [0,0,1]}; % Green and blue
    h_legends = [];
    legend_labels = {};
    
    % Plot boundary and CBF boundary
    h1 = plot(NaN, NaN, 'k-', 'LineWidth', 2);
    h2 = plot(NaN, NaN, 'r--', 'LineWidth', 1);
    h_legends = [h1, h2];
    legend_labels = {'Boundary', 'CBF Boundary'};
    
    for i = 1:size(poses, 2)
        trajectory = squeeze(poses(1:2, i, :));
        h_traj = plot(trajectory(1,:), trajectory(2,:), '-', 'Color', colors{i}, 'LineWidth', 2);
        h_legends = [h_legends, h_traj];
        legend_labels{end+1} = sprintf('Robot %d', i);
        
        % Start and goal markers
        plot(starts(1,i), starts(2,i), 'o', 'Color', colors{i}, ...
            'MarkerSize', 10, 'MarkerFaceColor', colors{i});
        plot(goals(1,i), goals(2,i), 'p', 'Color', colors{i}, ...
            'MarkerSize', 12, 'MarkerFaceColor', colors{i});
    end
    
    % Add start/goal to legend
    h_start = plot(NaN, NaN, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    h_goal = plot(NaN, NaN, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    h_legends = [h_legends, h_start, h_goal];
    legend_labels{end+1} = 'Start';
    legend_labels{end+1} = 'Goal';
    
    % % Mark points where CBF was active (combined for both robots)
    % cbf_active_any = any(cbf_active > 0, 1);
    % if any(cbf_active_any)
    %     for i = 1:size(poses, 2)
    %         trajectory = squeeze(poses(1:2, i, :));
    %         cbf_points = trajectory(:, cbf_active(i,:) > 0);
    %         if ~isempty(cbf_points)
    %             plot(cbf_points(1,:), cbf_points(2,:), '.', 'Color', 'r', ...
    %                 'MarkerSize', 8);
    %         end
    %     end
    %     h_cbf = plot(NaN, NaN, 'r.', 'MarkerSize', 8);
    %     h_legends = [h_legends, h_cbf];
    %     legend_labels{end+1} = 'CBF Active';
    % end
    
    % Add closest approach point
    min_distance = inf;
    min_idx = 1;
    for k = 1:size(poses, 3)
        dist = norm(poses(1:2, 1, k) - poses(1:2, 2, k));
        if dist < min_distance
            min_distance = dist;
            min_idx = k;
        end
    end
    plot(poses(1, :, min_idx), poses(2, :, min_idx), 'rx', ...
        'MarkerSize', 15, 'LineWidth', 3);
    h_closest = plot(NaN, NaN, 'rx', 'MarkerSize', 15, 'LineWidth', 3);
    h_legends = [h_legends, h_closest];
    legend_labels{end+1} = 'Closest Approach';
    
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Robot Trajectories with CBF Activation Points');
    legend(h_legends, legend_labels, 'Location', 'best');
    axis equal;
    grid on;
    
    % Velocity profiles
    time_vec = (0:size(velocities,3)-1) * 0.033;
    
    subplot(2,2,2);
    hold on;
    h_vel_legends = [];
    vel_legend_labels = {};
    
    for i = 1:size(velocities, 2)
        v = squeeze(velocities(1, i, :));
        h_v = stairs(time_vec, v, 'Color', colors{i}, 'LineWidth', 2);
        h_vel_legends = [h_vel_legends, h_v];
        vel_legend_labels{end+1} = sprintf('Robot %d', i);
    end
    
    % Removed CBF active markers per user request
    
    xlabel('Time (s)');
    ylabel('Linear Velocity (m/s)');
    title('Linear Velocity Profiles');
    legend(h_vel_legends, vel_legend_labels);
    grid on;
    
    subplot(2,2,4);
    hold on;
    for i = 1:size(velocities, 2)
        omega = squeeze(velocities(2, i, :));
        stairs(time_vec, omega, 'Color', colors{i}, 'LineWidth', 2);
    end
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Angular Velocity Profiles');
    legend('Robot 1', 'Robot 2');
    grid on;
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius)
    % Generate intercepting path using simple APF approach
    % Revert to proven simple method for reliability
    
    max_attempts = 100;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % Direction from start to goal for robot 1
        dir = refGoal - refStart;
        
        % Simple intercept point selection (40-60% along path)
        t = 0.4 + rand() * 0.2;
        interceptPoint = refStart + t * dir;
        
        % Check if intercept point is within bounds
        if interceptPoint(1) >= -x_bound && interceptPoint(1) <= x_bound && ...
           interceptPoint(2) >= -y_bound && interceptPoint(2) <= y_bound
            
            % Distance from intercept point to robot 1 start (this is our radius)
            radius = norm(interceptPoint - refStart);
            
            % Generate robot 2 start position on circle around intercept point (APF method)
            newStart = generateRandomPointOnCircleBounded(interceptPoint, radius, x_bound, y_bound);
            
            % Simple separation check (like original APF)
            if norm(newStart - refStart) > safeRadius
                % Goal on opposite side of intercept point
                dir_to_goal = interceptPoint - newStart;
                newGoal = interceptPoint + dir_to_goal;
                
                % Ensure goal is within bounds
                newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
                newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
                
                % Simple success check - just verify minimum path length
                if norm(newGoal - newStart) >= min_path_length
                    return;  % Success!
                end
            end
        end
    end
    
    % Fallback: simple crossing paths
    center = [0; 0];
    angle1 = atan2(refGoal(2) - refStart(2), refGoal(1) - refStart(1));
    angle2 = angle1 + pi/2;  % Perpendicular
    
    fallback_radius = max(min_path_length/2, safeRadius/2);
    newStart = center + fallback_radius * [cos(angle2 + pi); sin(angle2 + pi)];
    newGoal = center + fallback_radius * [cos(angle2); sin(angle2)];
    
    % Ensure within bounds
    newStart(1) = max(-x_bound, min(x_bound, newStart(1)));
    newStart(2) = max(-y_bound, min(y_bound, newStart(2)));
    newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
    newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
    
    interceptPoint = center;
end

function point = generateRandomPointOnCircleBounded(center, radius, x_bound, y_bound)
    % Generate random point on circle within bounds (restored APF method)
    
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
    
    % Fallback: return a point that's within bounds (reduced radius)
    fallback_radius = min(radius, min(x_bound, y_bound) * 0.8);
    angle = rand() * 2 * pi;
    point = center + fallback_radius * [cos(angle); sin(angle)];
end


function updateRealtimeVisualization(fig_handle, current_poses, pose_history, pose_idx, starts, goals, cbf_history)
    % Update real-time visualization elements
    % Access parameters from base workspace to avoid global variables
    
    show_safety_circles = evalin('base', 'show_safety_circles');
    show_detection_range = evalin('base', 'show_detection_range');
    show_intended_paths = evalin('base', 'show_intended_paths');
    show_trajectories = evalin('base', 'show_trajectories');
    safety_margin_robots = evalin('base', 'safety_margin_robots');
    activation_distance = evalin('base', 'activation_distance');
    
    if show_safety_circles || show_detection_range || show_intended_paths
        figure(fig_handle);
        
        % Clear previous dynamic elements
        delete(findobj(gca, 'Tag', 'SafetyVis'));
        delete(findobj(gca, 'Tag', 'IntendedPath'));
        delete(findobj(gca, 'Tag', 'StartGoal'));
        
        % Show intended paths
        if show_intended_paths
            plot([starts(1,1), goals(1,1)], [starts(2,1), goals(2,1)], 'g--', ...
                'LineWidth', 1.5, 'Tag', 'IntendedPath');
            plot([starts(1,2), goals(1,2)], [starts(2,2), goals(2,2)], 'b--', ...
                'LineWidth', 1.5, 'Tag', 'IntendedPath');
        end
        
        % Show start and goal positions
        colors = {[0,1,0], [0,0,1]};
        markers = {'o', 'p'};
        sizes = [10, 12];
        
        for i = 1:2
            plot(starts(1,i), starts(2,i), markers{1}, 'MarkerSize', sizes(1), ...
                'MarkerFaceColor', colors{i}, 'Tag', 'StartGoal');
            plot(goals(1,i), goals(2,i), markers{2}, 'MarkerSize', sizes(2), ...
                'MarkerFaceColor', colors{i}, 'Tag', 'StartGoal');
        end
        
        % Show safety and detection circles
        theta_circle = linspace(0, 2*pi, 50);
        for i = 1:size(current_poses, 2)
            color = colors{i};
            pos = current_poses(1:2, i);
            
            % Safety circle
            if show_safety_circles
                safety_circle = pos + safety_margin_robots * [cos(theta_circle); sin(theta_circle)];
                plot(safety_circle(1,:), safety_circle(2,:), '-', ...
                    'Color', color, 'LineWidth', 2, 'Tag', 'SafetyVis');
            end
            
            % Detection range when CBF is active
            if show_detection_range && cbf_history(i, pose_idx)
                detection_circle = pos + activation_distance * [cos(theta_circle); sin(theta_circle)];
                plot(detection_circle(1,:), detection_circle(2,:), '--', ...
                    'Color', color * 0.7, 'LineWidth', 1, 'Tag', 'SafetyVis');
            end
        end
    end
    
    % Show real-time trajectories
    if show_trajectories && pose_idx > 1
        figure(fig_handle);
        delete(findobj(gca, 'Tag', 'Trajectory'));
        
        colors = {[0,1,0,0.5], [0,0,1,0.5]};
        for i = 1:size(current_poses, 2)
            trajectory = squeeze(pose_history(1:2, i, 1:pose_idx));
            plot(trajectory(1,:), trajectory(2,:), '-', ...
                'Color', colors{i}, 'LineWidth', 2, 'Tag', 'Trajectory');
        end
    end
end

function angle = wrapToPi(angle)
    % Wrap angle to [-pi, pi]
    angle = atan2(sin(angle), cos(angle));
end