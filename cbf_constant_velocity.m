%% Control Barrier Function (CBF) Obstacle Avoidance with Constant Velocity
% This script demonstrates CBF-based navigation where robots avoid each other
% while moving towards their goals using Control Barrier Functions
% Modified to maintain CONSTANT LINEAR VELOCITY throughout the simulation
% Multi-trial version for data collection (up to 7000 trials)
init

%% Test Settings (adapted from simulation_APF.m)
annimation = true;        % Set true to show real-time animation
simulation = true;        % Set true to run simulation
record_data = false;      % Set true to save data to .mat file

%% Turning Detection Parameters
turn_threshold = 0.2;     % Angular change threshold to start recording (radians)
n = 0;                    % Trial counter
samples = 3;           % Number of trials to run
cbf_constant_vel_data = cell(samples, 2); % Data storage: {trial, robot} format

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

% === CONSTANT VELOCITY Control Parameters ===
constant_linear_velocity = 0.1;  % CONSTANT linear velocity (m/s) - NEVER CHANGES
heading_gain = 4.0;              % P-gain for angular velocity control
heading_smooth_factor = 0.7;     % Heading smoothing (0-1, higher = smoother)
enable_velocity_ramp = true;     % Enable smooth start (ramp up to constant velocity)
ramp_duration = 1.0;             % Time to ramp up to constant velocity (s)

% === Deadlock Resolution Parameters ===
enable_deadlock_resolution = true;  % Enable perturbation-based deadlock resolution
deadlock_detection_time = 2.0;      % Time of low angular movement before applying perturbation (s)
deadlock_angular_threshold = 0.1;   % Angular velocity threshold for deadlock detection (rad/s)
perturbation_angular_magnitude = 0.5; % Magnitude of angular perturbation (rad/s)
perturbation_duration = 1.0;        % How long to apply perturbation (s)
perturbation_cooldown = 2.0;        % Cooldown before next perturbation (s)

% === Position Tracking Parameters ===
distance_tracking_threshold = 0.001; % Minimum movement to track for efficiency (m)

% === Velocity Monitoring Parameters ===
monitor_velocity_violations = true; % Track velocity limit violations
velocity_safety_margin = 0.02;     % Safety margin for velocity limits (m/s)

% === Visualization Parameters ===
show_safety_circles = true;     % Show safety margins around robots
show_detection_range = true;    % Show CBF activation range
show_trajectories = true;       % Show real-time trajectory trails
show_intended_paths = true;     % Show dashed lines for intended paths
show_velocity_vectors = true;   % Show velocity direction vectors

%% SYSTEM INITIALIZATION

% === Robotarium Constants ===
N = 2; % Number of robots
max_linear_velocity = 0.1;      % From ARobotarium.m
max_angular_velocity = 3;    % Conservative limit
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

%% MULTI-TRIAL SIMULATION LOOP
if simulation
    fprintf('Starting CBF constant velocity simulation with %d trials\n', samples);
    rng('shuffle'); % Ensure randomness across trials
    
    while n < samples
        %% PATH GENERATION FOR THIS TRIAL
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
        
        % Robot 2 (blue) - intercepting path
        [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
            x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);
        
        % Ensure robots don't start too close together
        attempts = 0;
        while norm(start2 - start1) < min_robot_start_separation && attempts < max_generation_attempts
            [start2, goal2, interceptPoint] = generateInterceptingPathSimple(start1, goal1, ...
                x_bound_gen, y_bound_gen, min_path_length, safety_margin_robots*2);
            attempts = attempts + 1;
        end
        
        if attempts >= max_generation_attempts && n == 0
            fprintf('Warning: Could not find well-separated initial positions\n');
        end
        
        % Calculate initial headings (facing towards goals)
        initial_heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
        initial_heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));
        
        % Set initial conditions with proper headings
        initial_conditions = [start1, start2; initial_heading1, initial_heading2];
        
        % Create new Robotarium instance for this trial
        if n == 0
            % First trial - create with animation if enabled
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', annimation, 'InitialConditions', initial_conditions);
        else
            % Subsequent trials - close previous and create new
            if exist('r', 'var')
                r.debug(); % Clean up previous instance
            end
            % Keep animation setting for all trials if enabled
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', annimation, 'InitialConditions', initial_conditions);
        end

        %% TRIAL INITIALIZATION
        % === Data Storage for this trial ===
        pose_history = zeros(3, N, iterations);
        velocity_history = zeros(2, N, iterations); % [v; omega]
        cbf_active_history = zeros(N, iterations);  % Track when CBF is active
        desired_heading_history = zeros(N, iterations); % Track desired heading
        pose_idx = 1;
        
        % === Turning Detection Storage ===
        poses_robot1_cbf = [];  % Only record when turning
        poses_robot2_cbf = [];  % Only record when turning
        recording = false;      % Start recording when significant turning detected
        
        % Store previous angles for turning detection
        prev_angle1 = initial_heading1;
        prev_angle2 = initial_heading2;
        
        % === Control State Initialization ===
        heading_history = zeros(N, 5); % Keep last 5 headings for smoothing
        for i = 1:N
            heading_history(i, :) = initial_conditions(3, i);
        end
        
        % === Position Tracking ===
        last_positions = initial_conditions(1:2, :);
        
        % === Deadlock Resolution State (modified for constant velocity) ===
        deadlock_timer = zeros(N, 1);          % Time each robot has been turning slowly
        perturbation_active = false(N, 1);     % Whether perturbation is active
        perturbation_start_time = zeros(N, 1); % When perturbation started
        perturbation_direction = zeros(N, 1);  % Angular perturbation direction
        last_perturbation_time = zeros(N, 1);  % For cooldown tracking
        angular_velocity_history = zeros(N, 10); % Short history for average angular velocity
        
        % === Warning States ===
        deadlock_warned = false(N, 1);         % To avoid repeated deadlock warnings
        recovery_warned = false(N, 1);         % To track recovery messages
        
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

        %% MAIN SIMULATION LOOP FOR THIS TRIAL
        for t = 1:iterations
            % Get current poses
            x = r.get_poses();
            current_time = t * sampleTime;
            
            % Store pose history
            pose_history(:, :, pose_idx) = x;
            
            % Turning detection (similar to simulation_APF.m)
            if current_time >= 0.5  % Start checking after 0.5 seconds
                current_angle1 = x(3, 1);
                current_angle2 = x(3, 2);
                
                dTheta1 = abs(current_angle1 - prev_angle1);
                dTheta2 = abs(current_angle2 - prev_angle2);
                
                % Start recording when significant turning is detected
                if ~recording && (dTheta1 > turn_threshold || dTheta2 > turn_threshold)
                    recording = true;
                end
                
                % Record poses only when turning is happening
                if recording
                    poses_robot1_cbf = [poses_robot1_cbf, x(:, 1)];  % [x; y; theta]
                    poses_robot2_cbf = [poses_robot2_cbf, x(:, 2)];  % [x; y; theta]
                end
                
                % Update previous angles
                prev_angle1 = current_angle1;
                prev_angle2 = current_angle2;
            end
            
            % Track distance traveled for efficiency calculation
            for i = 1:N
                movement = norm(x(1:2, i) - last_positions(:, i));
                if movement > distance_tracking_threshold
                    total_distance_traveled(i) = total_distance_traveled(i) + movement;
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
        
        % Check if at goal
        distance_to_goal = norm(goal - current_pos);
        
        if distance_to_goal > goal_radius
            % === CONSTANT VELOCITY CONTROL ===
            
            % Desired velocity direction (towards goal)
            goal_direction = (goal - current_pos) / distance_to_goal;
            u_desired = constant_linear_velocity * goal_direction;
            
            % Setup CBF constraints with symmetric weights for robot-robot avoidance
            [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(current_pos, other_robots, ...
                boundaries, safety_margin_robots, safety_margin_boundaries, ...
                activation_distance, gamma_cbf, true);  % true = symmetric CBF
            
            % Store CBF activation status
            cbf_active_history(i, pose_idx) = cbf_active;
            
            % Solve CBF-QP for safe velocity DIRECTION (magnitude will be kept constant)
            if cbf_active
                u_safe_direction = solveCBFQP(u_desired, A_cbf, b_cbf, constant_linear_velocity);
            else
                u_safe_direction = u_desired;
            end
            
            % === FORCE CONSTANT VELOCITY MAGNITUDE ===
            if norm(u_safe_direction) > 0.01
                % Normalize to maintain constant speed
                u_safe = constant_linear_velocity * (u_safe_direction / norm(u_safe_direction));
                desired_heading = atan2(u_safe(2), u_safe(1));
            else
                % If CBF completely blocks motion, move towards goal anyway
                u_safe = u_desired;
                desired_heading = atan2(goal_direction(2), goal_direction(1));
            end
            
            % Store desired heading for analysis
            desired_heading_history(i, pose_idx) = desired_heading;
            
            % Apply velocity ramp-up in first second for smooth start
            if enable_velocity_ramp && current_time < ramp_duration
                velocity_scale = current_time / ramp_duration;  % 0 to 1 over ramp duration
                linear_velocity = constant_linear_velocity * velocity_scale;
                u_safe = linear_velocity * (u_safe / norm(u_safe));
            else
                linear_velocity = constant_linear_velocity;
            end
            
            % === HEADING CONTROL ===
            % Update heading history for smoothing
            heading_history(i, :) = [desired_heading, heading_history(i, 1:end-1)];
            
            % Apply heading smoothing
            weights = [0.5, 0.25, 0.15, 0.07, 0.03]';
            smoothed_heading = atan2(sum(sin(heading_history(i,:)).*weights'), ...
                                   sum(cos(heading_history(i,:)).*weights'));
            
            % Compute angular velocity for heading control
            heading_error = wrapToPi(smoothed_heading - current_heading);
            omega = heading_gain * heading_error;
            
            % === DEADLOCK RESOLUTION (modified for constant velocity) ===
            if enable_deadlock_resolution
                % Update angular velocity history
                angular_velocity_history(i, :) = [abs(omega), angular_velocity_history(i, 1:end-1)];
                avg_angular_velocity = mean(angular_velocity_history(i, :));
                
                % Check if in deadlock (low average angular velocity = not turning much)
                if avg_angular_velocity < deadlock_angular_threshold && pose_idx > 10
                    deadlock_timer(i) = deadlock_timer(i) + sampleTime;
                    
                    % Warning: Deadlock detected (once per episode)
                    if deadlock_timer(i) > deadlock_detection_time && ~deadlock_warned(i)
                        fprintf('Warning: Robot %d deadlock detected (low turning) at position (%.2f, %.2f) at time %.1fs\n', ...
                                i, x(1,i), x(2,i), current_time);
                        deadlock_warned(i) = true;
                        recovery_warned(i) = false; % Allow recovery message for this episode
                    end
                    
                    % Apply angular perturbation if deadlock persists and cooldown passed
                    if deadlock_timer(i) > deadlock_detection_time && ...
                       (current_time - last_perturbation_time(i)) > perturbation_cooldown
                        
                        if ~perturbation_active(i)
                            % Start new angular perturbation
                            perturbation_active(i) = true;
                            perturbation_start_time(i) = current_time;
                            
                            % Random angular perturbation direction
                            perturbation_direction(i) = perturbation_angular_magnitude * (2*rand() - 1);
                            
                            fprintf('Robot %d: Applying angular perturbation at time %.1fs\n', i, current_time);
                        end
                    end
                else
                    % Reset deadlock timer if turning well
                    if deadlock_timer(i) > 0
                        % Warning: Robot recovered (once per episode)
                        if deadlock_warned(i) && ~recovery_warned(i)
                            fprintf('Robot %d: Recovered from deadlock at time %.1fs\n', i, current_time);
                            recovery_warned(i) = true;
                        end
                        deadlock_timer(i) = 0;
                        deadlock_warned(i) = false;
                    end
                end
                
                % Apply angular perturbation if active
                if perturbation_active(i)
                    if (current_time - perturbation_start_time(i)) < perturbation_duration
                        % Add angular perturbation
                        omega = omega + perturbation_direction(i);
                    else
                        % End perturbation
                        perturbation_active(i) = false;
                        last_perturbation_time(i) = current_time;
                        deadlock_timer(i) = 0;
                    end
                end
            end
            
            % === MAINTAIN CONSTANT LINEAR VELOCITY ===
            v = norm(u_safe);  % This should always be constant_linear_velocity (or ramped version)
            
            % Apply velocity limits (angular only, linear is pre-constrained)
            omega = max(-max_angular_velocity, min(max_angular_velocity, omega));
            
            % Track violations for performance metrics
            if monitor_velocity_violations
                total_commands = total_commands + 1;
                if abs(omega) > max_angular_velocity - velocity_safety_margin
                    angular_violations = angular_violations + 1;
                end
            end
        else
            % At goal - stop
            v = 0;
            omega = 0;
        end
        
        % Set control inputs
        dxu(:, i) = [v; omega];
        velocity_history(:, i, pose_idx) = [v; omega];
    end
    
            % Update real-time visualization with CBF-style animation (only if animation is enabled)
            if annimation
                updateRealtimeVisualizationCBF(r.figure_handle, x, pose_history, pose_idx, ...
                    [start1, start2], [goal1, goal2], cbf_active_history, desired_heading_history);
            end
    
            % Set velocities and step
            r.set_velocities(1:N, dxu);
            r.step();
            
            pose_idx = pose_idx + 1;
    
            % Check if robots reached goals and record times
            dist1 = norm(x(1:2, 1) - goal1);
            dist2 = norm(x(1:2, 2) - goal2);
    
            % Record goal reaching times (only print for first few trials or when animation is on)
            if dist1 < goal_radius && path_end_time(1) == 0
                path_end_time(1) = current_time;
                if annimation || n < 5
                    fprintf('Trial %d: Robot 1 reached goal at %.2f seconds\n', n+1, current_time);
                end
            end
            if dist2 < goal_radius && path_end_time(2) == 0
                path_end_time(2) = current_time;
                if annimation || n < 5
                    fprintf('Trial %d: Robot 2 reached goal at %.2f seconds\n', n+1, current_time);
                end
            end
    
            if dist1 < goal_radius && dist2 < goal_radius
                if annimation || n < 5
                    fprintf('Trial %d: Both robots reached their goals!\n', n+1);
                end
                break;
            end
            
            % Safety check - break if simulation runs too long
            if current_time > simulationTime
                if annimation || n < 5
                    fprintf('Trial %d: Simulation timeout reached\n', n+1);
                end
                break;
            end
        end
        
        %% STORE TRIAL DATA
        % Store comprehensive data for later analysis
        trial_data = struct();
        trial_data.poses_robot1_turning = poses_robot1_cbf;  % Robot 1 poses during turning
        trial_data.poses_robot2_turning = poses_robot2_cbf;  % Robot 2 poses during turning
        trial_data.start_positions = [start1, start2];      % Start positions
        trial_data.goal_positions = [goal1, goal2];         % Goal positions
        trial_data.pose_history = pose_history(:,:,1:pose_idx-1);  % Full pose history
        trial_data.velocity_history = velocity_history(:,:,1:pose_idx-1); % Full velocity history
        trial_data.cbf_active_history = cbf_active_history(:,1:pose_idx-1); % CBF activation
        trial_data.simulation_time = (pose_idx-1) * sampleTime; % Actual simulation time
        trial_data.constant_velocity = constant_linear_velocity; % Target velocity
        
        % Store in cell array (maintain compatibility with original format)
        cbf_constant_vel_data{n+1, 1} = poses_robot1_cbf;  % For backward compatibility
        cbf_constant_vel_data{n+1, 2} = poses_robot2_cbf;  % For backward compatibility
        
        % Store full data in separate structure
        if n == 0
            cbf_full_data = cell(samples, 1);
        end
        cbf_full_data{n+1} = trial_data;
        
        % Increment trial counter
        n = n + 1;
        
        % Progress report every 1000 trials
        if mod(n, 1000) == 0
            fprintf('Completed Trials: %d\n', n);
        end
    end
end

%% POST-SIMULATION DATA HANDLING

% Save data if requested
if record_data
    save("constant_velocity_turn_only.mat", "cbf_constant_vel_data", "cbf_full_data");
    fprintf('Data saved to constant_velocity_turn_only.mat\n');
    fprintf('  - cbf_constant_vel_data: turning data only (backward compatibility)\n');
    fprintf('  - cbf_full_data: comprehensive trial data\n');
end

% Plot comprehensive results from stored data (after data collection is complete)
if exist('cbf_full_data', 'var') && n > 0
    % Find a good sample trial with turning data
    sample_trial = min(n, 100);  % Use trial 100 or last trial if fewer
    
    % Look for a trial with actual turning data
    for trial_idx = sample_trial:-1:1
        if ~isempty(cbf_full_data{trial_idx}) && ...
           ~isempty(cbf_full_data{trial_idx}.poses_robot1_turning)
            sample_trial = trial_idx;
            break;
        end
    end
    
    if ~isempty(cbf_full_data{sample_trial})
        data = cbf_full_data{sample_trial};
        
        % Create comprehensive visualization
        fig_results = figure('Name', sprintf('CBF Constant Velocity Analysis (Trial %d)', sample_trial), ...
                           'Position', [100, 100, 1200, 800]);
        
        % 1. Main trajectory plot
        subplot(2,3,[1,4]);
        hold on;
        
        % Plot boundaries
        rectangle('Position', [boundaries(1), boundaries(3), ...
            boundaries(2)-boundaries(1), boundaries(4)-boundaries(3)], ...
            'EdgeColor', 'k', 'LineWidth', 2);
        
        % Plot full trajectories
        colors = {[0,1,0], [0,0,1]}; % Green and blue
        for i = 1:2
            if size(data.pose_history, 3) > 1
                trajectory = squeeze(data.pose_history(1:2, i, :));
                plot(trajectory(1,:), trajectory(2,:), '-', 'Color', colors{i}, 'LineWidth', 2);
            end
        end
        
        % Plot turning segments (highlighted)
        if ~isempty(data.poses_robot1_turning)
            plot(data.poses_robot1_turning(1,:), data.poses_robot1_turning(2,:), '-', ...
                'Color', [0,0.7,0], 'LineWidth', 3);
        end
        if ~isempty(data.poses_robot2_turning)
            plot(data.poses_robot2_turning(1,:), data.poses_robot2_turning(2,:), '-', ...
                'Color', [0,0,0.7], 'LineWidth', 3);
        end
        
        % Plot start and goal positions
        plot(data.start_positions(1,1), data.start_positions(2,1), 'o', 'Color', colors{1}, ...
            'MarkerSize', 12, 'MarkerFaceColor', colors{1}, 'MarkerEdgeColor', 'k');
        plot(data.goal_positions(1,1), data.goal_positions(2,1), 'p', 'Color', colors{1}, ...
            'MarkerSize', 15, 'MarkerFaceColor', colors{1}, 'MarkerEdgeColor', 'k');
        plot(data.start_positions(1,2), data.start_positions(2,2), 'o', 'Color', colors{2}, ...
            'MarkerSize', 12, 'MarkerFaceColor', colors{2}, 'MarkerEdgeColor', 'k');
        plot(data.goal_positions(1,2), data.goal_positions(2,2), 'p', 'Color', colors{2}, ...
            'MarkerSize', 15, 'MarkerFaceColor', colors{2}, 'MarkerEdgeColor', 'k');
        
        xlabel('X (m)'); ylabel('Y (m)');
        title(sprintf('CBF Constant Velocity (%.3f m/s) - Full Trajectory', data.constant_velocity));
        legend('Robot 1 Full', 'Robot 2 Full', 'Robot 1 Turning', 'Robot 2 Turning', ...
               'Start 1', 'Goal 1', 'Start 2', 'Goal 2', 'Location', 'best');
        axis equal; grid on;
        
        % 2. Linear velocity profiles
        if size(data.velocity_history, 3) > 1
            time_vec = (0:size(data.velocity_history,3)-1) * sampleTime;
            
            subplot(2,3,2);
            hold on;
            for i = 1:2
                v = squeeze(data.velocity_history(1, i, :));
                plot(time_vec, v, 'Color', colors{i}, 'LineWidth', 2);
            end
            % Reference line for constant velocity
            plot([0, max(time_vec)], [data.constant_velocity, data.constant_velocity], ...
                'k--', 'LineWidth', 2);
            xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
            title('Linear Velocity (Should be Constant)');
            legend('Robot 1', 'Robot 2', 'Target', 'Location', 'best');
            grid on;
            
            % 3. Angular velocity profiles  
            subplot(2,3,3);
            hold on;
            for i = 1:2
                omega = squeeze(data.velocity_history(2, i, :));
                plot(time_vec, omega, 'Color', colors{i}, 'LineWidth', 2);
            end
            xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
            title('Angular Velocity Profiles');
            legend('Robot 1', 'Robot 2', 'Location', 'best');
            grid on;
            
            % 4. CBF activation timeline
            subplot(2,3,5);
            hold on;
            for i = 1:2
                cbf_active = squeeze(data.cbf_active_history(i, :));
                stairs(time_vec, cbf_active * i, 'Color', colors{i}, 'LineWidth', 2);
            end
            xlabel('Time (s)'); ylabel('Robot ID (when CBF active)');
            title('CBF Activation Timeline');
            legend('Robot 1', 'Robot 2', 'Location', 'best');
            grid on; ylim([0, 3]);
            
            % 5. Distance between robots over time
            subplot(2,3,6);
            distances = zeros(1, length(time_vec));
            for k = 1:length(time_vec)
                distances(k) = norm(data.pose_history(1:2, 1, k) - data.pose_history(1:2, 2, k));
            end
            plot(time_vec, distances, 'r-', 'LineWidth', 2);
            hold on;
            plot([0, max(time_vec)], [safety_margin_robots, safety_margin_robots], ...
                'k--', 'LineWidth', 1);
            xlabel('Time (s)'); ylabel('Distance (m)');
            title('Inter-Robot Distance');
            legend('Actual Distance', 'Safety Margin', 'Location', 'best');
            grid on;
        end
        
        % Print trial statistics
        fprintf('\\n=== Trial %d Statistics ===\\n', sample_trial);
        fprintf('Simulation time: %.2f seconds\\n', data.simulation_time);
        fprintf('Target velocity: %.3f m/s (constant)\\n', data.constant_velocity);
        fprintf('Turning data points: Robot1=%d, Robot2=%d\\n', ...
                size(data.poses_robot1_turning,2), size(data.poses_robot2_turning,2));
        if size(data.pose_history, 3) > 1
            % Calculate minimum distance
            min_dist = min(distances);
            fprintf('Minimum inter-robot distance: %.3f m\\n', min_dist);
            fprintf('Safety margin maintained: %s\\n', ...
                    string(min_dist >= safety_margin_robots));
        end
    end
end

% Clean up Robotarium
r.debug();

fprintf('\nCBF Constant Velocity Multi-Trial Simulation Complete\n');
fprintf('Total trials completed: %d\n', n);
if record_data
    fprintf('Data saved to: constant_velocity_turn_only.mat\n');
end

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
    % Modified for constant velocity - constrains direction, not magnitude
    
    % Get velocity safety margin from base workspace
    velocity_safety_margin = evalin('base', 'velocity_safety_margin');
    
    % QP formulation: min 0.5*u'*H*u + f'*u
    H = eye(2);
    f = -u_desired;
    
    % Convert CBF constraints to QP format (A*u <= b)
    A_ineq = -A_cbf;
    b_ineq = -b_cbf;
    
    % Velocity bounds with safety margin
    v_max_safe = v_max - velocity_safety_margin;
    lb = [-v_max_safe; -v_max_safe];
    ub = [v_max_safe; v_max_safe];
    
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
            
            % Generate robot 2 start position on circle around intercept point
            newStart = generateRandomPointOnCircleBounded(interceptPoint, radius, x_bound, y_bound);
            
            % Simple separation check
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
    % Generate random point on circle within bounds
    
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

function updateRealtimeVisualizationCBF(fig_handle, current_poses, pose_history, pose_idx, starts, goals, cbf_history, desired_heading_history)
    % Update real-time visualization with CBF-style elements
    % Based on updateRealtimeVisualization from cbf_obstacle_avoidance.m
    
    % Access parameters from base workspace
    show_safety_circles = evalin('base', 'show_safety_circles');
    show_detection_range = evalin('base', 'show_detection_range');
    show_intended_paths = evalin('base', 'show_intended_paths');
    show_trajectories = evalin('base', 'show_trajectories');
    show_velocity_vectors = evalin('base', 'show_velocity_vectors');
    safety_margin_robots = evalin('base', 'safety_margin_robots');
    activation_distance = evalin('base', 'activation_distance');
    constant_linear_velocity = evalin('base', 'constant_linear_velocity');
    
    if show_safety_circles || show_detection_range || show_intended_paths || show_velocity_vectors
        figure(fig_handle);
        
        % Clear previous dynamic elements
        delete(findobj(gca, 'Tag', 'SafetyVis'));
        delete(findobj(gca, 'Tag', 'IntendedPath'));
        delete(findobj(gca, 'Tag', 'StartGoal'));
        delete(findobj(gca, 'Tag', 'VelocityVector'));
        
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
        
        % Show safety and detection circles + velocity vectors
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
            if show_detection_range && pose_idx <= size(cbf_history, 2) && cbf_history(i, pose_idx)
                detection_circle = pos + activation_distance * [cos(theta_circle); sin(theta_circle)];
                plot(detection_circle(1,:), detection_circle(2,:), '--', ...
                    'Color', color * 0.7, 'LineWidth', 1, 'Tag', 'SafetyVis');
            end
            
            % Constant velocity vector (show current heading direction)
            if pose_idx > 1
                current_heading = current_poses(3, i);
                vel_scale = 0.3; % Scale factor for velocity visualization
                
                % Current velocity vector (constant magnitude, green for robot 1, blue for robot 2)
                vel_x = constant_linear_velocity * cos(current_heading) * vel_scale;
                vel_y = constant_linear_velocity * sin(current_heading) * vel_scale;
                quiver(pos(1), pos(2), vel_x, vel_y, 0, ...
                    'Color', color, 'LineWidth', 3, 'MaxHeadSize', 0.3, 'Tag', 'VelocityVector');
                
                % Desired heading vector (dashed) if available
                if pose_idx <= size(desired_heading_history, 2) && desired_heading_history(i, pose_idx) ~= 0
                    desired_heading = desired_heading_history(i, pose_idx);
                    des_vel_x = constant_linear_velocity * cos(desired_heading) * vel_scale;
                    des_vel_y = constant_linear_velocity * sin(desired_heading) * vel_scale;
                    quiver(pos(1), pos(2), des_vel_x, des_vel_y, 0, ...
                        'Color', color * 0.5, 'LineWidth', 2, 'LineStyle', '--', ...
                        'MaxHeadSize', 0.2, 'Tag', 'VelocityVector');
                end
            end
        end
    end
    
    % Show real-time trajectories
    if show_trajectories && pose_idx > 1
        figure(fig_handle);
        delete(findobj(gca, 'Tag', 'Trajectory'));
        
        colors_alpha = {[0,1,0,0.7], [0,0,1,0.7]};
        for i = 1:size(current_poses, 2)
            if pose_idx <= size(pose_history, 3)
                trajectory = squeeze(pose_history(1:2, i, 1:pose_idx));
                plot(trajectory(1,:), trajectory(2,:), '-', ...
                    'Color', colors_alpha{i}(1:3), 'LineWidth', 2, 'Tag', 'Trajectory');
            end
        end
    end
end

% Complex final visualization function removed - keep it simple for data collection

function angle = wrapToPi(angle)
    % Wrap angle to [-pi, pi]
    angle = atan2(sin(angle), cos(angle));
end