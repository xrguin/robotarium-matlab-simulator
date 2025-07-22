%% Control Barrier Function (CBF) Obstacle Avoidance - Robotarium Online Version
% This script demonstrates CBF-based navigation for real Robotarium deployment
% Adapted for online web interface with data saving capabilities
init
%% TUNABLE PARAMETERS
% === CBF Parameters ===
gamma_cbf = 2.0;                % CBF conservativeness (higher = more conservative)
activation_distance = 0.8;      % Distance to activate CBF constraints (m)
safety_margin_robots = 0.3;     % Minimum distance between robots (m)
safety_margin_boundaries = 0.2; % Minimum distance to arena boundaries (m)

% === Path Generation Parameters ===
min_path_length = 1.0;          % Minimum distance between start and goal (m)
max_path_length = 2.0;          % Maximum distance between start and goal (m)
boundary_buffer = 0.5;          % Keep initial positions away from walls (m)
min_robot_start_separation = 1.2; % Minimum distance between robot start positions (m)
max_generation_attempts = 20;   % Max attempts to generate good paths

% === Control Parameters ===
heading_gain = 4.0;             % P-gain for angular velocity control
heading_smooth_factor = 0.7;    % Heading smoothing (0-1, higher = smoother)
linear_velocity_gain = 1.0;     % Scaling for linear velocity

% === Deadlock Resolution Parameters ===
enable_deadlock_resolution = true;  % Enable perturbation-based deadlock resolution
deadlock_detection_time = 1.5;      % Time of low movement before applying perturbation (s)
deadlock_movement_threshold = 0.03; % Movement threshold for deadlock detection (m/s)
perturbation_magnitude = 0.03;      % Magnitude of random perturbation (m/s)
perturbation_duration = 1.0;        % How long to apply perturbation (s)
perturbation_cooldown = 2.0;        % Cooldown before next perturbation (s)

% === Monitoring Parameters ===
monitor_velocity_violations = true; % Track velocity limit violations
velocity_safety_margin = 0.01;     % Safety margin for velocity limits (m/s)

% === Visualization Parameters (Arena Projection) ===
enable_visualization = true;     % Master switch for all visualization
show_safety_circles = true;     % Show safety margins around robots
show_detection_range = true;    % Show CBF activation range
show_trajectories = true;       % Show real-time trajectory trails
show_intended_paths = true;     % Show dashed lines for intended paths
trajectory_fade_time = 5.0;     % Time for trajectory trails to fade (seconds)
trajectory_max_points = 150;    % Maximum points to keep in trajectory trail
update_frequency = 15;           % Update visualization every N iterations (higher = faster)

%% ROBOTARIUM SETUP

% === Robot Configuration ===
N = 2; % Number of robots
max_linear_velocity = 0.1;     % Conservative limit for online platform (3/4 of 0.2)
max_angular_velocity = 2*pi;    % Conservative angular limit
boundaries = [-1.6, 1.6, -1, 1]; % Robotarium arena bounds

% === Simulation Parameters ===
sampleTime = 0.033;             % Robotarium sample time (30 Hz)
iterations = 2000;              % Maximum iterations (adjust for desired runtime)
goal_radius = 0.1;              % Distance to consider goal reached

%% PATH GENERATION

% Arena bounds with safety margin
x_bound = boundaries(2) - safety_margin_boundaries;
y_bound = boundaries(4) - safety_margin_boundaries;
x_bound_gen = x_bound - boundary_buffer;
y_bound_gen = y_bound - boundary_buffer;

rng('shuffle');

% Robot 1 (green) - random start and goal with minimum path length
start1 = [(rand()*2-1)*x_bound_gen; (rand()*2-1)*y_bound_gen];
path_length = min_path_length + rand() * (max_path_length - min_path_length);
angle1 = rand() * 2 * pi;
goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));

% Ensure minimum path length
attempt = 0;
while norm(goal1 - start1) < min_path_length && attempt < max_generation_attempts
    angle1 = rand() * 2 * pi;
    goal1 = start1 + path_length * [cos(angle1); sin(angle1)];
    goal1(1) = max(-x_bound_gen, min(x_bound_gen, goal1(1)));
    goal1(2) = max(-y_bound_gen, min(y_bound_gen, goal1(2)));
    attempt = attempt + 1;
end

% Robot 2 (blue) - intercepting path using simplified approach
[start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
    x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);

% Ensure robots don't start too close together
attempts = 0;
while norm(start2 - start1) < min_robot_start_separation && attempts < max_generation_attempts
    [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
        x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);
    attempts = attempts + 1;
end

% Calculate initial headings
initial_heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
initial_heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));

% Set initial conditions
initial_conditions = [start1, start2; initial_heading1, initial_heading2];

%% INITIALIZE ROBOTARIUM

% Create Robotarium instance
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% VISUALIZATION SETUP FOR ARENA PROJECTION

% Get figure handle for arena projections
figure_handle = r.figure_handle;
hold on;

% Colors for robots (green and blue with transparency)
robot_colors = {[0, 1, 0], [0, 0, 1]};
colors_rgb = [0, 1, 0; 0, 0, 1]; % For trajectory storage

% Initialize trajectory storage
if show_trajectories
    trajectory_points = cell(N, 1);
    trajectory_times = cell(N, 1);
    trajectory_handles = cell(N, 1);
    for i = 1:N
        trajectory_points{i} = [];
        trajectory_times{i} = [];
        trajectory_handles{i} = [];
    end
end

% Plot intended paths if enabled
if show_intended_paths
    plot([start1(1), goal1(1)], [start1(2), goal1(2)], '--', ...
        'Color', [robot_colors{1}, 0.5], 'LineWidth', 2);
    plot([start2(1), goal2(1)], [start2(2), goal2(2)], '--', ...
        'Color', [robot_colors{2}, 0.5], 'LineWidth', 2);
end

% Plot start and goal markers
plot(start1(1), start1(2), 'o', 'Color', robot_colors{1}, ...
    'MarkerSize', 12, 'MarkerFaceColor', robot_colors{1});
plot(goal1(1), goal1(2), 'p', 'Color', robot_colors{1}, ...
    'MarkerSize', 15, 'MarkerFaceColor', robot_colors{1});
plot(start2(1), start2(2), 'o', 'Color', robot_colors{2}, ...
    'MarkerSize', 12, 'MarkerFaceColor', robot_colors{2});
plot(goal2(1), goal2(2), 'p', 'Color', robot_colors{2}, ...
    'MarkerSize', 15, 'MarkerFaceColor', robot_colors{2});

% Initialize handles for dynamic elements
safety_circle_handles = gobjects(N, 1);
detection_circle_handles = gobjects(N, 1);

%% DATA COLLECTION SETUP

% Initialize data storage
pose_history = zeros(3, N, iterations);
velocity_history = zeros(2, N, iterations);
cbf_active_history = zeros(N, iterations);
pose_idx = 1;

% Heading smoothing history
heading_history = zeros(N, 5);
for i = 1:N
    heading_history(i, :) = initial_conditions(3, i);
end

% Performance metrics
path_start_time = zeros(N, 1);
path_end_time = zeros(N, 1);
straight_line_distance = [norm(goal1 - start1); norm(goal2 - start2)];
total_distance_traveled = zeros(N, 1);
last_positions = initial_conditions(1:2, :);

% Deadlock resolution state
deadlock_timer = zeros(N, 1);          % Time each robot has been moving slowly
perturbation_active = false(N, 1);     % Whether perturbation is active
perturbation_start_time = zeros(N, 1); % When perturbation started
perturbation_direction = zeros(2, N);  % Random perturbation direction
last_perturbation_time = zeros(N, 1);  % For cooldown tracking
velocity_history_short = zeros(N, 10); % Short history for average velocity

% Velocity violation tracking
if monitor_velocity_violations
    linear_violations = 0;
    angular_violations = 0;
    total_commands = 0;
end

% Start timer
start_time = tic;

%% MAIN CONTROL LOOP

for t = 1:iterations
    % Get current poses
    x = r.get_poses();
    current_time = toc(start_time);
    
    % Store pose data
    pose_history(:, :, pose_idx) = x;
    
    % Track distance traveled
    for i = 1:N
        movement = norm(x(1:2, i) - last_positions(:, i));
        if movement > 0.001
            total_distance_traveled(i) = total_distance_traveled(i) + movement;
        end
        last_positions(:, i) = x(1:2, i);
    end
    
    % Calculate CBF control for each robot
    dxu = zeros(2, N);
    
    for i = 1:N
        current_pos = x(1:2, i);
        current_heading = x(3, i);
        
        % Select goal
        if i == 1
            goal = goal1;
        else
            goal = goal2;
        end
        
        % Find other robots
        other_robots = x(1:2, setdiff(1:N, i));
        
        % Desired velocity
        goal_direction = goal - current_pos;
        distance_to_goal = norm(goal_direction);
        
        if distance_to_goal > goal_radius
            goal_direction = goal_direction / distance_to_goal;
            u_desired = linear_velocity_gain * max_linear_velocity * goal_direction;
        else
            u_desired = [0; 0];
        end
        
        % Apply velocity ramp-up
        if current_time < 1.0
            velocity_scale = current_time;
            u_desired = u_desired * velocity_scale;
        end
        
        % Setup CBF constraints
        [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(current_pos, other_robots, ...
            boundaries, safety_margin_robots, safety_margin_boundaries, ...
            activation_distance, gamma_cbf, true);
        
        cbf_active_history(i, pose_idx) = cbf_active;
        
        % Solve CBF-QP
        if cbf_active
            u_safe = solveCBFQP(u_desired, A_cbf, b_cbf, max_linear_velocity, velocity_safety_margin);
        else
            u_safe = u_desired;
        end
        
        % Deadlock detection and resolution
        if enable_deadlock_resolution && distance_to_goal > goal_radius
            % Update velocity history
            current_velocity = norm(u_safe);
            velocity_history_short(i, :) = [current_velocity, velocity_history_short(i, 1:end-1)];
            avg_velocity = mean(velocity_history_short(i, :));
            
            % Check if in deadlock (low average velocity)
            if avg_velocity < deadlock_movement_threshold && pose_idx > 10
                deadlock_timer(i) = deadlock_timer(i) + sampleTime;
                
                % Apply perturbation if deadlock persists and cooldown passed
                if deadlock_timer(i) > deadlock_detection_time && ...
                   (current_time - last_perturbation_time(i)) > perturbation_cooldown
                    
                    if ~perturbation_active(i)
                        % Start new perturbation
                        perturbation_active(i) = true;
                        perturbation_start_time(i) = current_time;
                        
                        % Random perpendicular direction
                        angle = atan2(u_safe(2), u_safe(1)) + (rand()-0.5)*pi;
                        perturbation_direction(:, i) = perturbation_magnitude * [cos(angle); sin(angle)];
                        
                        fprintf('Robot %d: Applying deadlock perturbation at %.1fs\n', i, current_time);
                    end
                end
            else
                % Reset deadlock timer if moving well
                deadlock_timer(i) = 0;
            end
            
            % Apply perturbation if active
            if perturbation_active(i)
                if (current_time - perturbation_start_time(i)) < perturbation_duration
                    % Add perturbation to safe velocity
                    u_safe = u_safe + perturbation_direction(:, i);
                else
                    % End perturbation
                    perturbation_active(i) = false;
                    last_perturbation_time(i) = current_time;
                    deadlock_timer(i) = 0;
                end
            end
        end
        
        % Convert to unicycle control
        if norm(u_safe) > 0.01
            desired_heading = atan2(u_safe(2), u_safe(1));
            
            % Heading smoothing
            heading_history(i, :) = [desired_heading, heading_history(i, 1:end-1)];
            weights = [0.5, 0.25, 0.15, 0.07, 0.03]';
            smoothed_heading = atan2(sum(sin(heading_history(i,:)).*weights'), ...
                                   sum(cos(heading_history(i,:)).*weights'));
            
            heading_error = wrapToPi(smoothed_heading - current_heading);
            omega = heading_gain * heading_error;
            
            v = norm(u_safe);
            v = v * (1 - 0.5*abs(heading_error)/pi);
            
            % Speed recovery after obstacle avoidance
            if ~cbf_active && v < max_linear_velocity * 0.9
                v = min(v * 1.1, max_linear_velocity);
            end
            
            % Proportional scaling if limits exceeded
            scale_factor = 1.0;
            if v > max_linear_velocity
                scale_factor = min(scale_factor, max_linear_velocity / v);
            end
            if abs(omega) > max_angular_velocity
                scale_factor = min(scale_factor, max_angular_velocity / abs(omega));
            end
            
            v = v * scale_factor;
            omega = omega * scale_factor;
            
            % Track violations
            if monitor_velocity_violations
                total_commands = total_commands + 1;
                if v > max_linear_velocity * 1.01
                    linear_violations = linear_violations + 1;
                end
                if abs(omega) > max_angular_velocity * 1.01
                    angular_violations = angular_violations + 1;
                end
            end
        else
            v = 0;
            omega = 0;
        end
        
        dxu(:, i) = [v; omega];
        velocity_history(:, i, pose_idx) = [v; omega];
    end
    
    % Update arena projections (with frequency control)
    if enable_visualization && mod(t-1, update_frequency) == 0
        updateArenaProjections(x, current_time);
    end
    
    % Robotarium velocity thresholding (critical for online platform)
    norms = arrayfun(@(x) norm(dxu(:, x)), 1:N);
    threshold = r.max_linear_velocity;  % Robotarium requirement
    to_thresh = norms > threshold;
    dxu(:, to_thresh) = threshold * dxu(:, to_thresh) ./ norms(to_thresh);
    
    % Apply velocities
    r.set_velocities(1:N, dxu);
    r.step();
    
    pose_idx = pose_idx + 1;
    
    % Check goal completion
    dist1 = norm(x(1:2, 1) - goal1);
    dist2 = norm(x(1:2, 2) - goal2);
    
    if dist1 < goal_radius && path_end_time(1) == 0
        path_end_time(1) = current_time;
    end
    if dist2 < goal_radius && path_end_time(2) == 0
        path_end_time(2) = current_time;
    end
    
    if dist1 < goal_radius && dist2 < goal_radius
        break;
    end
end

%% DATA SAVING

% Trim unused data
final_poses = pose_history(:, :, 1:pose_idx-1);
final_velocities = velocity_history(:, :, 1:pose_idx-1);
final_cbf_active = cbf_active_history(:, 1:pose_idx-1);

% Save experimental data
save('CBF_Trajectory_Data.mat', 'final_poses', 'final_velocities', 'final_cbf_active', ...
     'start1', 'start2', 'goal1', 'goal2', 'interceptPoint', ...
     'straight_line_distance', 'total_distance_traveled', 'path_end_time', ...
     'boundaries', 'safety_margin_robots', 'safety_margin_boundaries', 'activation_distance');

% Performance summary
fprintf('Experiment completed in %.2f seconds\\n', toc(start_time));
for i = 1:N
    if total_distance_traveled(i) > 0
        efficiency = straight_line_distance(i) / total_distance_traveled(i);
        fprintf('Robot %d - Path efficiency: %.2f%% (traveled %.2fm for %.2fm straight-line)\\n', ...
                i, efficiency*100, total_distance_traveled(i), straight_line_distance(i));
    end
end

if monitor_velocity_violations && total_commands > 0
    linear_violation_rate = (linear_violations / total_commands) * 100;
    angular_violation_rate = (angular_violations / total_commands) * 100;
    fprintf('\\nVelocity Statistics:\\n');
    fprintf('  Linear violations: %d/%d (%.1f%%)\\n', ...
            linear_violations, total_commands, linear_violation_rate);
    fprintf('  Angular violations: %d/%d (%.1f%%)\\n', ...
            angular_violations, total_commands, angular_violation_rate);
end

% Required Robotarium debug call
r.debug();

%% ARENA PROJECTION FUNCTION

function updateArenaProjections(current_poses, current_time)
    % Update visual elements projected on the arena floor
    % Access variables from parent workspace
    
    N = evalin('caller', 'N');
    show_safety_circles = evalin('caller', 'show_safety_circles');
    show_detection_range = evalin('caller', 'show_detection_range');
    show_trajectories = evalin('caller', 'show_trajectories');
    safety_margin_robots = evalin('caller', 'safety_margin_robots');
    activation_distance = evalin('caller', 'activation_distance');
    robot_colors = evalin('caller', 'robot_colors');
    cbf_active_history = evalin('caller', 'cbf_active_history');
    pose_idx = evalin('caller', 'pose_idx');
    
    % Get handle arrays
    safety_circle_handles = evalin('caller', 'safety_circle_handles');
    detection_circle_handles = evalin('caller', 'detection_circle_handles');
    
    % Circle parameters for smooth circles
    theta_circle = linspace(0, 2*pi, 30);
    
    % Update safety circles and detection ranges
    for i = 1:N
        pos = current_poses(1:2, i);
        color = robot_colors{i};
        
        % Delete previous circles
        if isvalid(safety_circle_handles(i))
            delete(safety_circle_handles(i));
        end
        if isvalid(detection_circle_handles(i))
            delete(detection_circle_handles(i));
        end
        
        % Draw safety circle
        if show_safety_circles
            safety_circle_x = pos(1) + safety_margin_robots * cos(theta_circle);
            safety_circle_y = pos(2) + safety_margin_robots * sin(theta_circle);
            h_safety = fill(safety_circle_x, safety_circle_y, color, ...
                'FaceAlpha', 0.1, 'EdgeColor', color, 'LineWidth', 2);
            safety_circle_handles(i) = h_safety;
        end
        
        % Draw detection range when CBF is active
        if show_detection_range && pose_idx > 0 && cbf_active_history(i, pose_idx)
            detection_circle_x = pos(1) + activation_distance * cos(theta_circle);
            detection_circle_y = pos(2) + activation_distance * sin(theta_circle);
            h_detection = plot(detection_circle_x, detection_circle_y, '--', ...
                'Color', color, 'LineWidth', 1.5);
            detection_circle_handles(i) = h_detection;
        end
    end
    
    % Update handle arrays in caller workspace
    assignin('caller', 'safety_circle_handles', safety_circle_handles);
    assignin('caller', 'detection_circle_handles', detection_circle_handles);
    
    % Update trajectory trails
    if show_trajectories
        trajectory_fade_time = evalin('caller', 'trajectory_fade_time');
        trajectory_max_points = evalin('caller', 'trajectory_max_points');
        
        % Get all trajectory data at once
        trajectory_points = evalin('caller', 'trajectory_points');
        trajectory_times = evalin('caller', 'trajectory_times');
        trajectory_handles = evalin('caller', 'trajectory_handles');
        
        for i = 1:N
            pos = current_poses(1:2, i);
            color = robot_colors{i};
            
            % Get trajectory data for this robot
            traj_points = trajectory_points{i};
            traj_times = trajectory_times{i};
            traj_handles = trajectory_handles{i};
            
            % Add new point
            traj_points = [traj_points, pos];
            traj_times = [traj_times, current_time];
            
            % Remove old points (beyond fade time or max points)
            keep_idx = (current_time - traj_times) <= trajectory_fade_time;
            if sum(keep_idx) > trajectory_max_points
                keep_idx = false(size(keep_idx));
                keep_idx(end-trajectory_max_points+1:end) = true;
            end
            
            traj_points = traj_points(:, keep_idx);
            traj_times = traj_times(keep_idx);
            
            % Delete old trajectory handle
            if ~isempty(traj_handles) && isvalid(traj_handles)
                delete(traj_handles);
            end
            
            % Draw new trajectory with fading effect
            if size(traj_points, 2) > 1
                % Calculate alpha values based on time
                alphas = 1 - (current_time - traj_times) / trajectory_fade_time;
                alphas = max(0.1, alphas); % Minimum visibility
                
                % Draw trajectory segments with varying opacity
                for j = 1:size(traj_points, 2)-1
                    alpha = alphas(j);
                    plot(traj_points(1, j:j+1), traj_points(2, j:j+1), '-', ...
                        'Color', [color, alpha], 'LineWidth', 3);
                end
                
                % Store handle of last segment for deletion next time
                h_traj = plot(traj_points(1, end-1:end), traj_points(2, end-1:end), '-', ...
                    'Color', color, 'LineWidth', 3);
                traj_handles = h_traj;
            end
            
            % Update cell arrays
            trajectory_points{i} = traj_points;
            trajectory_times{i} = traj_times;
            trajectory_handles{i} = traj_handles;
        end
        
        % Update all trajectory data in caller workspace
        assignin('caller', 'trajectory_points', trajectory_points);
        assignin('caller', 'trajectory_times', trajectory_times);
        assignin('caller', 'trajectory_handles', trajectory_handles);
    end
end

%% HELPER FUNCTIONS

function [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(pos, other_robots, ...
    boundaries, margin_robots, margin_boundaries, activation_dist, gamma, symmetric)
    
    if nargin < 8
        symmetric = false;
    end
    
    A_cbf = [];
    b_cbf = [];
    cbf_active = false;
    
    x = pos(1);
    y = pos(2);
    
    % Boundary constraints
    h_left = x - boundaries(1) - margin_boundaries;
    if h_left < activation_dist
        A_cbf = [A_cbf; 1, 0];
        b_cbf = [b_cbf; -gamma * h_left];
        cbf_active = true;
    end
    
    h_right = boundaries(2) - x - margin_boundaries;
    if h_right < activation_dist
        A_cbf = [A_cbf; -1, 0];
        b_cbf = [b_cbf; -gamma * h_right];
        cbf_active = true;
    end
    
    h_bottom = y - boundaries(3) - margin_boundaries;
    if h_bottom < activation_dist
        A_cbf = [A_cbf; 0, 1];
        b_cbf = [b_cbf; -gamma * h_bottom];
        cbf_active = true;
    end
    
    h_top = boundaries(4) - y - margin_boundaries;
    if h_top < activation_dist
        A_cbf = [A_cbf; 0, -1];
        b_cbf = [b_cbf; -gamma * h_top];
        cbf_active = true;
    end
    
    % Robot constraints with symmetric weights
    for j = 1:size(other_robots, 2)
        robot_pos = other_robots(:, j);
        distance = norm(pos - robot_pos);
        h_robot = distance - margin_robots;
        
        if h_robot < activation_dist && distance > 0
            gradient = (pos - robot_pos) / distance;
            
            if symmetric
                weight = 0.5;
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

function u_safe = solveCBFQP(u_desired, A_cbf, b_cbf, v_max, velocity_safety_margin)
    
    H = eye(2);
    f = -u_desired;
    
    % Conservative velocity bounds
    v_max_conservative = (v_max - velocity_safety_margin) / sqrt(2);
    lb = [-v_max_conservative; -v_max_conservative];
    ub = [v_max_conservative; v_max_conservative];
    
    % Convert CBF constraints
    A_ineq = -A_cbf;
    b_ineq = -b_cbf;
    
    options = optimoptions('quadprog', 'Display', 'off');
    try
        u_safe = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
        if isempty(u_safe) || any(isnan(u_safe))
            u_safe = [0; 0];
        end
    catch
        u_safe = [0; 0];
    end
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius)
    
    max_attempts = 100;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        dir = refGoal - refStart;
        t = 0.4 + rand() * 0.2;
        interceptPoint = refStart + t * dir;
        
        if interceptPoint(1) >= -x_bound && interceptPoint(1) <= x_bound && ...
           interceptPoint(2) >= -y_bound && interceptPoint(2) <= y_bound
            
            radius = norm(interceptPoint - refStart);
            newStart = generateRandomPointOnCircleBounded(interceptPoint, radius, x_bound, y_bound);
            
            if norm(newStart - refStart) > safeRadius
                dir_to_goal = interceptPoint - newStart;
                newGoal = interceptPoint + dir_to_goal;
                
                newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
                newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
                
                if norm(newGoal - newStart) >= min_path_length
                    return;
                end
            end
        end
    end
    
    % Fallback
    center = [0; 0];
    angle1 = atan2(refGoal(2) - refStart(2), refGoal(1) - refStart(1));
    angle2 = angle1 + pi/2;
    
    fallback_radius = max(min_path_length/2, safeRadius/2);
    newStart = center + fallback_radius * [cos(angle2 + pi); sin(angle2 + pi)];
    newGoal = center + fallback_radius * [cos(angle2); sin(angle2)];
    
    newStart(1) = max(-x_bound, min(x_bound, newStart(1)));
    newStart(2) = max(-y_bound, min(y_bound, newStart(2)));
    newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
    newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
    
    interceptPoint = center;
end

function point = generateRandomPointOnCircleBounded(center, radius, x_bound, y_bound)
    
    max_attempts = 50;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        theta = 2 * pi * rand();
        x = center(1) + radius * cos(theta);
        y = center(2) + radius * sin(theta);
        
        if x >= -x_bound && x <= x_bound && y >= -y_bound && y <= y_bound
            point = [x; y];
            return;
        end
    end
    
    % Fallback
    fallback_radius = min(radius, min(x_bound, y_bound) * 0.8);
    angle = rand() * 2 * pi;
    point = center + fallback_radius * [cos(angle); sin(angle)];
end

function angle = wrapToPi(angle)
    angle = atan2(sin(angle), cos(angle));
end