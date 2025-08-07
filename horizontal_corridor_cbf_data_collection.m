%% Horizontal Corridor CBF Data Collection Script
% This script collects trajectory data from multiple trials of robots navigating
% a horizontal narrow corridor using asymmetric CBF control.
% Based on horizontal_corridor_cbf_fixed.m with multi-trial data collection.

init;

%% Data Collection Settings
num_trials = 2000;                    % Number of trials to collect
animation = false;                    % Set true to show animation (for debugging)
save_interval = 100;                  % Save data every N trials
output_filename = 'horizontal_corridor_cbf_data.mat';

%% TUNABLE PARAMETERS (from horizontal_corridor_cbf_fixed.m)

% === Environment Parameters ===
passage_width = 0.4; % Width of the horizontal corridor (m)

% === Robot & Path Generation ===
N = 2; % Number of robots
ally_start_zone = [-1.5, -1.2];
ally_goal_zone = [1.2, 1.5];
adv_start_zone = [1.2, 1.5];
adv_goal_zone = [-1.5, -1.2];
static1 = [-1.5, 1];
static2 = [-1.5, -1];
y_range = [-0.8, 0.8];

% === CBF-QP Parameters ===
gamma_cbf = 2.0;
safety_margin_robots = 0.3; % Actual safety radius (m)
safety_margin_walls = 0.1;
cbf_activation_distance = 0.8; % Distance at which to start applying CBF (m)

% === Control Parameters ===
max_linear_velocity = 0.1;
max_angular_velocity = 3.5;
heading_gain = 4.0;
heading_smoothing_factor = 0.7; % Smoothing factor for heading updates

% === Simulation & Visualization ===
simulation_time = 60;
goal_radius = 0.1;

%% SETUP: ENVIRONMENT CONSTANTS
sample_time = 0.033;
iterations = ceil(simulation_time / sample_time);
robotarium_height = 2.0;
radius = (robotarium_height - passage_width) / 2;
center_top_y = passage_width/2 + radius;
center_bottom_y = -passage_width/2 - radius;
static_obstacles = [0, center_top_y, radius; 0, center_bottom_y, radius]';

%% Initialize Data Storage
corridor_cbf_data = cell(num_trials, 1);

%% MAIN DATA COLLECTION LOOP
fprintf('Starting CBF corridor data collection with %d trials\n', num_trials);
successful_trials = 0;

for trial = 1:num_trials
    try
        %% Generate Random Initial Conditions for This Trial
        start1 = [ally_start_zone(1) + rand()*(ally_start_zone(2)-ally_start_zone(1)); 
                  y_range(1) + rand()*(y_range(2)-y_range(1))];
        goal1 = [ally_goal_zone(1) + rand()*(ally_goal_zone(2)-ally_goal_zone(1)); 
                 y_range(1) + rand()*(y_range(2)-y_range(1))];
        start2 = [adv_start_zone(1) + rand()*(adv_start_zone(2)-adv_start_zone(1)); 
                  y_range(1) + rand()*(y_range(2)-y_range(1))];
        goal2 = [adv_goal_zone(1) + rand()*(adv_goal_zone(2)-adv_goal_zone(1)); 
                 y_range(1) + rand()*(y_range(2)-y_range(1))];
        
        initial_heading1 = atan2(goal1(2)-start1(2), goal1(1)-start1(1));
        initial_heading2 = atan2(goal2(2)-start2(2), goal2(1)-start2(1));
        initial_conditions = [start1, start2; initial_heading1, initial_heading2];
        
        % Create Robotarium instance
        if trial == 1 && animation
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);
        else
            if exist('r', 'var')
                r.debug(); % Clean up previous instance
            end
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', animation, 'InitialConditions', initial_conditions);
        end
        
        %% Initialize Trial Data Storage
        pose_history = zeros(3, N, iterations);
        velocity_history = zeros(2, N, iterations);
        angular_velocity_history = zeros(N, iterations);
        retreat_state_history = zeros(1, iterations); % Track retreat mode state
        
        % Retreat state for ally robot
        ally_retreat_mode = false;
        retreat_start_pos = [];
        retreat_target = [];
        
        % Previous headings for smoothing
        previous_headings = zeros(N, 1);
        
        % Visualization setup if animation is enabled
        if animation
            plot_horizontal_corridor(r.figure_handle, passage_width, radius, center_top_y, center_bottom_y);
            plot(start1(1), start1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot(goal1(1), goal1(2), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot(start2(1), start2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
            plot(goal2(1), goal2(2), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        end
        
        %% SIMULATION LOOP FOR THIS TRIAL
        actual_iterations = 0;
        x = []; % Initialize x outside loop for scope
        
        for t = 1:iterations
            x = r.get_poses();
            pose_history(:, :, t) = x;
            dxu = zeros(2, N);
            
            % Store retreat state
            retreat_state_history(t) = ally_retreat_mode;
            
            for i = 1:N
                current_pos = x(1:2, i);
                current_heading = x(3, i);
                goal = (i==1) * goal1 + (i==2) * goal2;
                dynamic_obstacles = x(1:2, setdiff(1:N, i));
                
                goal_direction = goal - current_pos;
                dist_to_goal = norm(goal_direction);
                
                % RETREAT LOGIC for ally robot (from horizontal_corridor_cbf_fixed.m)
                if i == 1  % Only ally robot retreats
                    if ~isempty(dynamic_obstacles)
                        adv_pos = dynamic_obstacles(:, 1);
                        dist_to_adv = norm(current_pos - adv_pos);
                        avoid_direction = current_pos - adv_pos;
                        
                        % Check if in narrow corridor
                        in_corridor = abs(current_pos(1)) < 0.6;
                        
                        % Enter retreat mode if blocked
                        if ~ally_retreat_mode && in_corridor && dist_to_adv < 0.5
                            % Check if adversary is blocking the path
                            to_adv = adv_pos - current_pos;
                            if dot(to_adv, goal_direction) > 0  % Adversary is between robot and goal
                                ally_retreat_mode = true;
                                retreat_start_pos = current_pos;
                                
                                % Choose retreat target based on which corridor entrance is closer
                                corridor_left_entrance = [-1.2; 0];
                                corridor_right_entrance = [1.2; 0];
                                
                                dist_to_left = norm(current_pos - corridor_left_entrance);
                                dist_to_right = norm(current_pos - corridor_right_entrance);
                                
                                if dist_to_left < dist_to_right
                                    retreat_target = corridor_left_entrance;
                                else
                                    retreat_target = corridor_right_entrance;
                                end
                            end
                        end
                        
                        % Exit retreat mode when safe
                        if ally_retreat_mode
                            adv_passed = abs(current_pos(1) - adv_pos(1)) > 0.4;
                            safe_distance = dist_to_adv > 0.8;
                            reached_retreat_target = norm(current_pos - retreat_target) < 0.1;
                            
                            if adv_passed || safe_distance || reached_retreat_target
                                ally_retreat_mode = false;
                                retreat_start_pos = [];
                                retreat_target = [];
                            end
                        end
                    end
                    
                    % Set desired velocity based on mode
                    if ally_retreat_mode && ~isempty(retreat_target)
                        retreat_direction = retreat_target - current_pos;
                        retreat_distance = norm(retreat_direction);
                        u_desired = max_linear_velocity * (retreat_direction / retreat_distance);
                    else
                        % Normal: move toward goal
                        u_desired = (dist_to_goal > goal_radius) * max_linear_velocity * (goal_direction / dist_to_goal);
                    end
                else
                    % Adversary always goes to goal
                    u_desired = (dist_to_goal > goal_radius) * max_linear_velocity * (goal_direction / dist_to_goal);
                end
                
                % ASYMMETRIC CBF: Ally (robot 1) avoids adversary (robot 2)
                [A_cbf, b_cbf, cbf_active] = setup_asymmetric_cbf(current_pos, static_obstacles, ...
                    dynamic_obstacles, safety_margin_walls, safety_margin_robots, gamma_cbf, ...
                    cbf_activation_distance, i);
                u_safe = (size(A_cbf,1) > 0) * solve_cbf_qp(u_desired, A_cbf, b_cbf, max_linear_velocity) + ...
                         (size(A_cbf,1) == 0) * u_desired;
                
                if norm(u_safe) > 0.01
                    desired_heading = atan2(u_safe(2), u_safe(1));
                    
                    % Apply heading smoothing for ally robot
                    heading_error = wrapToPi(desired_heading - current_heading);
                    previous_headings(i) = desired_heading;
                    
                    v = norm(u_safe);
                    omega = heading_gain * heading_error;
                else
                    v = 0; omega = 0;
                end
                
                v = min(max(v, 0), max_linear_velocity);
                omega = min(max(omega, -max_angular_velocity), max_angular_velocity);
                dxu(:, i) = [v; omega];
                velocity_history(:, i, t) = [v; omega];
                angular_velocity_history(i, t) = omega;
            end
            
            r.set_velocities(1:N, dxu);
            r.step();
            
            actual_iterations = t;
            
            % Check if robots reached goals
            if norm(x(1:2, 1) - goal1) < goal_radius && norm(x(1:2, 2) - goal2) < goal_radius
                break;
            end
        end
        
        %% Detect First Retreat and Store Trial Data
        % Parameters for retreat-based data collection
        min_data_after_retreat = 50; % minimum timesteps after entering retreat
        
        % Find when ally first enters retreat mode
        first_retreat_idx = find(retreat_state_history(1:actual_iterations) == 1, 1, 'first');
        
        % Check if both robots reached their goals
        ally_goal_reached = norm(x(1:2, 1) - goal1) < goal_radius;
        adversary_goal_reached = norm(x(1:2, 2) - goal2) < goal_radius;
        both_goals_reached = ally_goal_reached && adversary_goal_reached;
        
        % Check if ally had a retreat situation
        retreat_occurred = ~isempty(first_retreat_idx);
        
        % Only store data if ALL conditions are met:
        % 1. Ally had a retreat situation
        % 2. Sufficient data after retreat (at least 50 timesteps)
        % 3. Both robots reached their goals
        if retreat_occurred && (actual_iterations - first_retreat_idx + 1) >= min_data_after_retreat && both_goals_reached
            % Store data from first retreat onwards
            trial_data = struct();
            trial_data.ally_trajectory = pose_history(:, 1, first_retreat_idx:actual_iterations);      % [x; y; theta]
            trial_data.adversary_trajectory = pose_history(:, 2, first_retreat_idx:actual_iterations); % [x; y; theta]
            trial_data.ally_velocity = velocity_history(:, 1, first_retreat_idx:actual_iterations);    % [v; omega]
            trial_data.adversary_velocity = velocity_history(:, 2, first_retreat_idx:actual_iterations); % [v; omega]
            trial_data.ally_angular_velocity = angular_velocity_history(1, first_retreat_idx:actual_iterations);
            trial_data.adversary_angular_velocity = angular_velocity_history(2, first_retreat_idx:actual_iterations);
            trial_data.retreat_state = retreat_state_history(first_retreat_idx:actual_iterations);     % Retreat mode history
            trial_data.start_positions = [start1, start2];
            trial_data.goal_positions = [goal1, goal2];
            trial_data.first_retreat_timestep = first_retreat_idx; % Store when retreat started
            
            % Store positions at retreat start
            trial_data.retreat_start_positions = [pose_history(1:2, 1, first_retreat_idx), pose_history(1:2, 2, first_retreat_idx)];
            trial_data.simulation_time = actual_iterations * sample_time;
            trial_data.goal_reached = [ally_goal_reached; adversary_goal_reached];
            
            % Environment parameters
            trial_data.passage_width = passage_width;
            trial_data.static_obstacles = static_obstacles;
            trial_data.safety_margin_robots = safety_margin_robots;
            trial_data.safety_margin_walls = safety_margin_walls;
            
            % Store in main data array
            corridor_cbf_data{trial} = trial_data;
            successful_trials = successful_trials + 1;
            
            % Visualize the collected training data
            if animation || trial <= 5  % Show first 5 trials
                figure('Name', sprintf('Trial %d - Training Data from Retreat', trial));
                hold on;
                
                % Plot full trajectories in light colors
                full_ally_traj = squeeze(pose_history(1:2, 1, 1:actual_iterations));
                full_adv_traj = squeeze(pose_history(1:2, 2, 1:actual_iterations));
                plot(full_ally_traj(1,:), full_ally_traj(2,:), 'g:', 'LineWidth', 1, 'DisplayName', 'Ally Full Path');
                plot(full_adv_traj(1,:), full_adv_traj(2,:), 'b:', 'LineWidth', 1, 'DisplayName', 'Adversary Full Path');
                
                % Plot training data trajectories in bold
                training_ally_traj = trial_data.ally_trajectory(1:2, :);
                training_adv_traj = trial_data.adversary_trajectory(1:2, :);
                plot(training_ally_traj(1,:), training_ally_traj(2,:), 'g-', 'LineWidth', 3, 'DisplayName', 'Ally Training Data');
                plot(training_adv_traj(1,:), training_adv_traj(2,:), 'b-', 'LineWidth', 3, 'DisplayName', 'Adversary Training Data');
                
                % Mark key positions
                plot(start1(1), start1(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Ally Start');
                plot(start2(1), start2(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Adversary Start');
                plot(goal1(1), goal1(2), 'g^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Ally Goal');
                plot(goal2(1), goal2(2), 'b^', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Adversary Goal');
                
                % Mark retreat start position
                plot(trial_data.retreat_start_positions(1,1), trial_data.retreat_start_positions(2,1), ...
                     'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Retreat Start');
                plot(trial_data.retreat_start_positions(1,2), trial_data.retreat_start_positions(2,2), ...
                     'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
                
                % Draw corridor
                plot_horizontal_corridor(gcf, passage_width, radius, center_top_y, center_bottom_y);
                
                xlim([-1.7, 1.7]); ylim([-1.1, 1.1]);
                xlabel('X (m)'); ylabel('Y (m)');
                title(sprintf('Trial %d: Training Data (Bold) from t=%d onwards', trial, first_retreat_idx));
                legend('Location', 'best');
                grid on; axis equal;
                
                drawnow;
            end
        else
            % Skip this trial - conditions not met for data collection
            corridor_cbf_data{trial} = [];
            if ~retreat_occurred
                fprintf('Trial %d: Skipped - No retreat detected (retreat is required for training data)\n', trial);
            elseif (actual_iterations - first_retreat_idx + 1) < min_data_after_retreat
                fprintf('Trial %d: Skipped - Retreat at t=%d but insufficient data after retreat\n', trial, first_retreat_idx);
            elseif ~both_goals_reached
                fprintf('Trial %d: Skipped - Retreat occurred but not both robots reached goals (Ally: %d, Adversary: %d)\n', ...
                        trial, ally_goal_reached, adversary_goal_reached);
            end
        end
        
        % Progress report
        if mod(trial, 50) == 0
            fprintf('Completed %d/%d trials (%.1f%% success rate)\n', ...
                    trial, num_trials, 100*successful_trials/trial);
        end
        
        % Periodic saving
        if mod(trial, save_interval) == 0
            save(output_filename, 'corridor_cbf_data', 'trial');
            fprintf('Data saved at trial %d\n', trial);
        end
        
    catch ME
        fprintf('Error in trial %d: %s\n', trial, ME.message);
        fprintf('Error location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
        % Store empty data for failed trial
        corridor_cbf_data{trial} = [];
    end
end

%% Final Save and Cleanup
if exist('r', 'var')
    r.debug();
end

% Remove empty trials
corridor_cbf_data = corridor_cbf_data(~cellfun('isempty', corridor_cbf_data));

% Save final data
save(output_filename, 'corridor_cbf_data');
fprintf('\nData collection complete!\n');
fprintf('Successful trials: %d/%d (%.1f%%)\n', successful_trials, num_trials, ...
        100*successful_trials/num_trials);
fprintf('Data saved to: %s\n', output_filename);

% Display sample statistics
if ~isempty(corridor_cbf_data)
    fprintf('\nSample statistics from collected data:\n');
    retreat_counts = 0;
    total_retreat_time = 0;
    min_distances = [];
    retreat_detected_count = 0;
    avg_retreat_time = 0;
    
    for i = 1:length(corridor_cbf_data)
        if ~isempty(corridor_cbf_data{i})
            data = corridor_cbf_data{i};
            retreat_detected_count = retreat_detected_count + 1;
            
            % Record when retreat was first detected
            if isfield(data, 'first_retreat_timestep')
                avg_retreat_time = avg_retreat_time + data.first_retreat_timestep * sample_time;
            end
            
            % Count retreat events
            retreat_transitions = diff([0, data.retreat_state]);
            retreat_counts = retreat_counts + sum(retreat_transitions == 1);
            total_retreat_time = total_retreat_time + sum(data.retreat_state) * sample_time;
            
            % Calculate minimum distance between robots
            ally_traj = data.ally_trajectory(1:2, :);
            adv_traj = data.adversary_trajectory(1:2, :);
            distances = sqrt(sum((ally_traj - adv_traj).^2, 1));
            min_distances = [min_distances, min(distances)];
        end
    end
    
    fprintf('Trials with retreat interactions: %d/%d (%.1f%%)\n', retreat_detected_count, ...
            length(corridor_cbf_data), 100*retreat_detected_count/length(corridor_cbf_data));
    if retreat_detected_count > 0
        fprintf('Average time to first retreat: %.2f seconds\n', avg_retreat_time/retreat_detected_count);
        fprintf('Average retreats per valid trial: %.2f\n', retreat_counts/retreat_detected_count);
        fprintf('Average retreat time per valid trial: %.2f seconds\n', total_retreat_time/retreat_detected_count);
        fprintf('Average minimum inter-robot distance: %.3f m\n', mean(min_distances));
        fprintf('Safety violations (distance < %.2f m): %d\n', safety_margin_robots, ...
                sum(min_distances < safety_margin_robots));
    end
end

%% HELPER FUNCTIONS (from horizontal_corridor_cbf_fixed.m)

function [A, b, active] = setup_asymmetric_cbf(pos, static_obs, dynamic_obs, margin_wall, margin_robot, gamma, activation_dist, robot_index)
    A = zeros(0, 2); b = zeros(0, 1); active = false;
    % Static Obstacles (Walls) - Both robots avoid walls equally
    for i = 1:size(static_obs, 2)
        obs_center = static_obs(1:2, i); obs_radius = static_obs(3, i);
        h = norm(pos - obs_center) - obs_radius - margin_wall;
        if h < activation_dist
            grad_h = (pos - obs_center)' / norm(pos - obs_center);
            A = [A; grad_h]; b = [b; -gamma * h]; active = true;
        end
    end
    % Dynamic Obstacles (Other Robots) - Asymmetric Avoidance
    for i = 1:size(dynamic_obs, 2)
        obs_pos = dynamic_obs(:, i);
        h = norm(pos - obs_pos) - margin_robot;
        if h < activation_dist
            active = true;
            % ONLY ALLY (Robot 1) GETS A CONSTRAINT TO AVOID THE ADVERSARY
            if robot_index == 1
                grad_h = (pos - obs_pos)' / norm(pos - obs_pos);
                A = [A; grad_h];      % Ally takes 100% of the responsibility
                b = [b; -gamma * h];
            end
            % Adversary (Robot 2) gets no constraint and ignores the ally.
        end
    end
end

function u_safe = solve_cbf_qp(u_desired, A_cbf, b_cbf, v_max)
    H = 1 * eye(2); f = -1 * u_desired;
    A_ineq = -A_cbf; b_ineq = -b_cbf;
    lb = [-v_max; -v_max]; ub = [v_max; v_max];
    options = optimoptions('quadprog', 'Display', 'off');
    try
        u_safe = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
        if isempty(u_safe), u_safe = [0; 0]; end
    catch
        u_safe = [0; 0];
    end
end

function plot_horizontal_corridor(fig_handle, ~, radius, center_top_y, center_bottom_y)
    ax = get(fig_handle, 'CurrentAxes');
    if isempty(ax), ax = axes('Parent', fig_handle); end
    axes(ax); hold on;
    theta_top = linspace(pi, 2*pi, 100);
    top_hemisphere_x = 0 + radius * cos(theta_top);
    top_hemisphere_y = center_top_y + radius * sin(theta_top);
    fill(top_hemisphere_x, top_hemisphere_y, 'k');
    theta_bottom = linspace(0, pi, 100);
    bottom_hemisphere_x = 0 + radius * cos(theta_bottom);
    bottom_hemisphere_y = center_bottom_y + radius * sin(theta_bottom);
    fill(bottom_hemisphere_x, bottom_hemisphere_y, 'k');
    rectangle('Position', [-1.6, -1, 3.2, 2], 'EdgeColor', 'k', 'LineWidth', 2);
    axis equal; xlim([-1.7, 1.7]); ylim([-1.1, 1.1]);
end

