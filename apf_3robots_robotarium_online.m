%% APF 3-Robot Obstacle Avoidance with Neural Network Prediction - Robotarium Online Version
% This script demonstrates APF-based navigation with robot prediction for real Robotarium deployment
% Robot 1 (ego) uses actual Robot 2 and predicted Robot 3 positions for navigation
% Adapted for online web interface with data saving capabilities

init

%% SYSTEM INFORMATION LOGGING
fprintf('=== SYSTEM INFORMATION ===\n');
system_info = struct();

% MATLAB version
matlab_version = version;
system_info.matlab_version = matlab_version;
fprintf('MATLAB Version: %s\n', matlab_version);

% MATLAB release
matlab_release = version('-release');
system_info.matlab_release = matlab_release;
fprintf('MATLAB Release: %s\n', matlab_release);

% Check for Deep Learning Toolbox
try
    deep_learning_version = ver('Deep Learning Toolbox');
    if ~isempty(deep_learning_version)
        system_info.deep_learning_toolbox = deep_learning_version.Version;
        fprintf('Deep Learning Toolbox Version: %s\n', deep_learning_version.Version);
    else
        system_info.deep_learning_toolbox = 'Not installed';
        fprintf('Deep Learning Toolbox: Not installed\n');
    end
catch
    system_info.deep_learning_toolbox = 'Check failed';
    fprintf('Deep Learning Toolbox: Check failed\n');
end

% Check for Neural Network Toolbox (older name)
try
    nnet_version = ver('Neural Network Toolbox');
    if ~isempty(nnet_version)
        system_info.neural_network_toolbox = nnet_version.Version;
        fprintf('Neural Network Toolbox Version: %s\n', nnet_version.Version);
    else
        system_info.neural_network_toolbox = 'Not installed';
        fprintf('Neural Network Toolbox: Not installed\n');
    end
catch
    system_info.neural_network_toolbox = 'Check failed';
    fprintf('Neural Network Toolbox: Check failed\n');
end

% Operating system
system_info.operating_system = computer;
fprintf('Operating System: %s\n', computer);

% Java version
try
    java_version = version('-java');
    system_info.java_version = java_version;
    fprintf('Java Version: %s\n', java_version);
catch
    system_info.java_version = 'Check failed';
    fprintf('Java Version: Check failed\n');
end

% Current date and time
system_info.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
fprintf('Timestamp: %s\n', system_info.timestamp);

% Save system info immediately
save('System_Info_APF.mat', 'system_info');
fprintf('System information saved to: System_Info_APF.mat\n');
fprintf('===========================\n\n');

%% LOAD TRAINED MODEL
fprintf('Loading trained robot prediction model...\n');
model_file = 'apf_robot2_predictor_2025-07-28.mat';
if exist(model_file, 'file')
    load(model_file, 'trainedModel');
    net_predictor = trainedModel.net;
    muX_train = trainedModel.normParams.muX;
    sigmaX_train = trainedModel.normParams.sigmaX;
    muT_train = trainedModel.normParams.muT;
    sigmaT_train = trainedModel.normParams.sigmaT;
    offset = 29; % Updated offset for new training model
    fprintf('Model loaded successfully!\n');
    fprintf('Model offset: %d\n', offset);
else
    error('Trained model file not found: %s', model_file);
end

%% APF PARAMETERS
% Robot-specific APF parameters (matched to training data roles)
attraction_factor_robot1 = 1;    % Robot 1 (ego) 
repulsion_factor_robot1 = 1.5;   % Robot 1 (ego)
attraction_factor_robot2 = 1;    % Robot 2 (ally)
repulsion_factor_robot2 = 1.7;   % Robot 2 (ally)
attraction_factor_robot3 = 1;    % Robot 3 (adversarial)
repulsion_factor_robot3 = 1.5;   % Robot 3 (adversarial)

detection_radius = 1;            % APF detection range
safe_radius = 0.3;               % Safety radius around robots
safe_radius_ego = 0.2;           % Safety radius for ego robot
constant_linear_velocity = 0.1;  % Constant linear velocity (m/s)

%% ROBOTARIUM SETUP
% Robot Configuration
N = 3; % Number of robots
max_linear_velocity = 0.1;      % Conservative limit for online platform
max_angular_velocity = 3.5;     % Conservative angular limit
boundaries = [-1.6, 1.6, -1, 1]; % Robotarium arena bounds

% Simulation Parameters
sampleTime = 0.033;             % Robotarium sample time (30 Hz)
max_iterations = 2000;          % Maximum iterations
goal_radius = 0.1;              % Distance to consider goal reached

% Arena bounds with safety margin
boundary_buffer = 0.3;
x_bound = boundaries(2) - boundary_buffer;
y_bound = boundaries(4) - boundary_buffer;

%% FIXED INITIAL POSITIONS (Case 5 from validation)
% Start positions
start1 = [-0.4231; -0.4558];   % Robot 1 (ego) - orange
start2 = [-1.0281; -0.3617];   % Robot 2 (ally) - green
start3 = [-0.051; 0.0305];     % Robot 3 (adversarial) - blue

% Goal positions
goal1 = [-0.874; 0.5];
goal2 = [-0.3175; 0.5];
goal3 = [-1.1; 0.1715];

% Calculate initial headings
heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));
heading3 = atan2(goal3(2) - start3(2), goal3(1) - start3(1));

% Set initial conditions
initial_conditions = [start1, start2, start3; heading1, heading2, heading3];

fprintf('Robot positions: R1[%.2f,%.2f] R2[%.2f,%.2f] R3[%.2f,%.2f]\n', ...
    start1(1), start1(2), start2(1), start2(2), start3(1), start3(2));

%% INITIALIZE ROBOTARIUM
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% VISUALIZATION SETUP
figure_handle = r.figure_handle;
hold on;

% Robot colors: orange for R1, green for R2, blue for R3
robot_colors = {[1, 0.5, 0], [0, 1, 0], [0, 0, 1]};
robot_names = {'Ego (R1)', 'Ally (R2)', 'Adversarial (R3)'};

% Plot start and goal markers
for i = 1:N
    % Start positions (circles)
    plot(initial_conditions(1, i), initial_conditions(2, i), 'o', ...
        'MarkerSize', 12, 'MarkerFaceColor', robot_colors{i}, ...
        'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    % Goal positions (pentagrams)
    if i == 1
        goal = goal1;
    elseif i == 2
        goal = goal2;
    else
        goal = goal3;
    end
    plot(goal(1), goal(2), 'p', 'MarkerSize', 15, ...
        'MarkerFaceColor', robot_colors{i}, 'MarkerEdgeColor', 'k', 'LineWidth', 2);
end

% Initialize trajectory storage
trajectory_handles = gobjects(N, 1);
trajectory_points = cell(N, 1);
for i = 1:N
    trajectory_points{i} = [];
end

% Initialize safety circle handles
safety_circle_handles = gobjects(N, 1);
theta_circle = linspace(0, 2*pi, 30);

% Initialize prediction visualization handles
pred_pos_handle = plot(nan, nan, 's', 'MarkerSize', 12, ...
    'MarkerFaceColor', [1, 0, 1], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
pred_error_handle = plot(nan, nan, 'k--', 'LineWidth', 2);
pred_traj_handle = plot(nan, nan, ':', 'Color', [1, 0, 1], 'LineWidth', 3);

% Legend
legend_items = [trajectory_handles; pred_traj_handle; pred_error_handle];
legend_labels = [robot_names, 'Predicted R3 Path', 'Prediction Error'];

% Arena boundaries
rectangle('Position', [-1.6, -1, 3.2, 2], 'EdgeColor', 'k', 'LineWidth', 2);
axis equal;
xlim([-1.8, 1.8]);
ylim([-1.2, 1.2]);
xlabel('X (m)');
ylabel('Y (m)');
title('APF 3-Robot Online - Network Prediction');

%% DATA COLLECTION SETUP
% Prediction variables
window_size = offset + 1; % Prediction window size (30 for offset=29)
robot2_pose_window = zeros(4, window_size); % Store Robot 2 poses for prediction
prediction_buffer = zeros(5, 2); % FILO buffer for prediction smoothing
buffer_idx = 1;
recording = false;

% Speed filtering variables
prevRobot3Pos = [];
prevPredictedPos = [];
speedThresholdMultiplier = 1.5;  % Threshold = 1.5 × actual robot3 speed
minSpeedForFiltering = 0.05;  % Only apply filtering when robot3 speed < this value

% Data storage
pose_history = zeros(3, N, max_iterations);
velocity_history = zeros(2, N, max_iterations);
angular_velocity_history = zeros(N, max_iterations);
prediction_history = [];
actual_robot3_history = [];
all_predictions = [];
all_actual_robot3 = [];

% Performance tracking
pose_idx = 1;
prev_angles = [heading1, heading2, heading3];
goals_reached = [false, false, false];
straight_line_distance = [norm(goal1 - start1); norm(goal2 - start2); norm(goal3 - start3)];
total_distance_traveled = zeros(N, 1);
last_positions = initial_conditions(1:2, :);

% Start timer
start_time = tic;

%% MAIN CONTROL LOOP
for t = 1:max_iterations
    % Get current poses
    x = r.get_poses();
    current_time = toc(start_time);
    
    % Calculate angular velocities
    angular_velocities = zeros(1, N);
    if current_time >= 0.1 % Start calculating after 0.1 seconds
        for i = 1:N
            current_angle = x(3, i);
            angular_change = wrapToPi(current_angle - prev_angles(i));
            angular_velocities(i) = angular_change / sampleTime;
            prev_angles(i) = current_angle;
        end
    end
    
    % Store data
    pose_history(:, :, pose_idx) = x;
    angular_velocity_history(:, pose_idx) = angular_velocities;
    
    % Track distance traveled
    for i = 1:N
        movement = norm(x(1:2, i) - last_positions(:, i));
        if movement > 0.001
            total_distance_traveled(i) = total_distance_traveled(i) + movement;
        end
        last_positions(:, i) = x(1:2, i);
    end
    
    % Check for turning detection (start prediction after window filled)
    if current_time >= 0.5
        is_turning = abs(angular_velocities(2)) > 0.2; % Robot 2 turning threshold
        
        if ~recording && is_turning
            recording = true;
            fprintf('Prediction started at t=%.2fs (Robot 2 ω=%.2f)\n', current_time, angular_velocities(2));
        end
    end
    
    %% Robot 2 and 3 APF Navigation (Ground Truth)
    dxu = zeros(2, N);
    
    % Robot 2 (ally) - uses Robot 3 as obstacle
    robot2_pos = x(1:2, 2);
    robot2_heading = x(3, 2);
    distance_to_goal2 = norm(goal2 - robot2_pos);
    
    if distance_to_goal2 > goal_radius
        obstacles_robot2 = x(1:2, 3); % Only Robot 3
        [F_att2, F_rep2, F_combined2] = calculate_apf_forces(robot2_pos, goal2, ...
            obstacles_robot2, detection_radius, attraction_factor_robot2, repulsion_factor_robot2);
        
        if norm(F_combined2) > 0.01
            desired_heading2 = atan2(F_combined2(2), F_combined2(1));
        else
            goal_direction2 = goal2 - robot2_pos;
            desired_heading2 = atan2(goal_direction2(2), goal_direction2(1));
        end
        
        heading_error2 = wrapToPi(desired_heading2 - robot2_heading);
        omega2 = 2.0 * heading_error2;
        omega2 = max(-max_angular_velocity, min(max_angular_velocity, omega2));
        v2 = min(constant_linear_velocity, max_linear_velocity);
    else
        v2 = 0; omega2 = 0;
        goals_reached(2) = true;
    end
    
    % Robot 3 (adversarial) - only sees Robot 2 (ally), ignores ego Robot 1
    robot3_pos = x(1:2, 3);
    robot3_heading = x(3, 3);
    distance_to_goal3 = norm(goal3 - robot3_pos);
    
    if distance_to_goal3 > goal_radius
        obstacles_robot3 = x(1:2, 2); % Only Robot 2
        [F_att3, F_rep3, F_combined3] = calculate_apf_forces(robot3_pos, goal3, ...
            obstacles_robot3, detection_radius, attraction_factor_robot3, repulsion_factor_robot3);
        
        if norm(F_combined3) > 0.01
            desired_heading3 = atan2(F_combined3(2), F_combined3(1));
        else
            goal_direction3 = goal3 - robot3_pos;
            desired_heading3 = atan2(goal_direction3(2), goal_direction3(1));
        end
        
        heading_error3 = wrapToPi(desired_heading3 - robot3_heading);
        omega3 = 2.0 * heading_error3;
        omega3 = max(-max_angular_velocity, min(max_angular_velocity, omega3));
        v3 = min(constant_linear_velocity, max_linear_velocity);
    else
        v3 = 0; omega3 = 0;
        goals_reached(3) = true;
    end
    
    %% Robot 3 Prediction System
    predicted_robot3_pos = [];
    
    % Update Robot 2 pose window for prediction
    if pose_idx <= window_size
        robot2_pose_window(:, pose_idx) = [x(:, 2); angular_velocities(2)];
    else
        robot2_pose_window(:, 1:end-1) = robot2_pose_window(:, 2:end);
        robot2_pose_window(:, end) = [x(:, 2); angular_velocities(2)];
        
        % Make prediction if we have turning data
        if recording
            % Normalize input (only x, y, theta)
            input_data = robot2_pose_window(1:3, :)'; % Transpose to match training format
            input_norm = (input_data - muX_train) ./ sigmaX_train;
            
            % Predict Robot 3 position
            prediction_norm = predict(net_predictor, input_norm);
            predicted_robot3_raw = prediction_norm .* sigmaT_train + muT_train;
            
            % Speed filtering before FILO buffer
            currentRobot3Pos = x(1:2, 3);
            if ~isempty(prevRobot3Pos) && current_time > 0
                actualRobot3Speed = norm(currentRobot3Pos - prevRobot3Pos) / sampleTime;
                
                % Only apply filtering when robot3 is moving slowly
                if actualRobot3Speed < minSpeedForFiltering && ~isempty(prevPredictedPos)
                    predictedSpeed = norm(predicted_robot3_raw' - prevPredictedPos) / sampleTime;
                    speedThreshold = speedThresholdMultiplier * actualRobot3Speed;
                    
                    % If predicted speed exceeds threshold, use previous prediction
                    if predictedSpeed > speedThreshold
                        predicted_robot3_raw = prevPredictedPos';
                        fprintf('Filtered prediction at t=%.2fs (pred speed=%.3f > threshold=%.3f)\n', ...
                            current_time, predictedSpeed, speedThreshold);
                    end
                end
            end
            
            % Store current positions for next iteration
            prevRobot3Pos = currentRobot3Pos;
            prevPredictedPos = predicted_robot3_raw';
            
            % Apply FILO filter for smoothing
            prediction_buffer(buffer_idx, :) = predicted_robot3_raw;
            if buffer_idx < 5
                predicted_robot3_pos = mean(prediction_buffer(1:buffer_idx, :), 1)';
                buffer_idx = buffer_idx + 1;
            else
                predicted_robot3_pos = mean(prediction_buffer, 1)';
                % Circular buffer
                prediction_buffer(1:end-1, :) = prediction_buffer(2:end, :);
                prediction_buffer(end, :) = predicted_robot3_raw;
            end
            
            % Store for analysis
            prediction_history = [prediction_history, predicted_robot3_pos];
            actual_robot3_history = [actual_robot3_history, x(1:2, 3)];
            all_predictions = [all_predictions, predicted_robot3_pos];
            all_actual_robot3 = [all_actual_robot3, x(1:2, 3)];
        end
    end
    
    %% Robot 1 (Ego) Navigation with Prediction
    robot1_pos = x(1:2, 1);
    robot1_heading = x(3, 1);
    distance_to_goal1 = norm(goal1 - robot1_pos);
    
    if distance_to_goal1 > goal_radius
        % Build obstacle list: actual Robot 2 + predicted Robot 3 (if available)
        obstacles_robot1 = x(1:2, 2); % Always include actual Robot 2
        
        if ~isempty(predicted_robot3_pos)
            obstacles_robot1 = [obstacles_robot1, predicted_robot3_pos];
        end
        
        [F_att1, F_rep1, F_combined1] = calculate_apf_forces(robot1_pos, goal1, ...
            obstacles_robot1, detection_radius, attraction_factor_robot1, repulsion_factor_robot1);
        
        if norm(F_combined1) > 0.01
            desired_heading1 = atan2(F_combined1(2), F_combined1(1));
        else
            goal_direction1 = goal1 - robot1_pos;
            desired_heading1 = atan2(goal_direction1(2), goal_direction1(1));
        end
        
        heading_error1 = wrapToPi(desired_heading1 - robot1_heading);
        omega1 = 2.0 * heading_error1;
        omega1 = max(-max_angular_velocity, min(max_angular_velocity, omega1));
        v1 = min(constant_linear_velocity, max_linear_velocity);
    else
        v1 = 0; omega1 = 0;
        goals_reached(1) = true;
    end
    
    % Set control inputs
    dxu(:, 1) = [v1; omega1];
    dxu(:, 2) = [v2; omega2];
    dxu(:, 3) = [v3; omega3];
    
    % Store velocities
    velocity_history(:, :, pose_idx) = dxu;
    
    % Final velocity safety check
    for i = 1:N
        if abs(dxu(1, i)) > max_linear_velocity
            dxu(1, i) = sign(dxu(1, i)) * max_linear_velocity;
        end
        if abs(dxu(2, i)) > max_angular_velocity
            dxu(2, i) = sign(dxu(2, i)) * max_angular_velocity;
        end
    end
    
    %% Update Visualization
    % Update trajectories
    for i = 1:N
        trajectory_points{i} = [trajectory_points{i}, x(1:2, i)];
        
        % Delete old trajectory handle
        if isvalid(trajectory_handles(i))
            delete(trajectory_handles(i));
        end
        
        % Draw new trajectory
        if size(trajectory_points{i}, 2) > 1
            trajectory_handles(i) = plot(trajectory_points{i}(1, :), ...
                trajectory_points{i}(2, :), '-', 'Color', robot_colors{i}, 'LineWidth', 2);
        end
    end
    
    % Update safety circles
    for i = 1:N
        if isvalid(safety_circle_handles(i))
            delete(safety_circle_handles(i));
        end
        
        pos = x(1:2, i);
        if i == 1
            radius = safe_radius_ego;
        else
            radius = safe_radius;
        end
        
        circle_x = pos(1) + radius * cos(theta_circle);
        circle_y = pos(2) + radius * sin(theta_circle);
        
        % Use lighter color for safety circles
        light_color = robot_colors{i} * 0.5 + [0.5, 0.5, 0.5];
        safety_circle_handles(i) = plot(circle_x, circle_y, ':', ...
            'Color', light_color, 'LineWidth', 1);
    end
    
    % Update predicted trajectory
    if ~isempty(all_predictions) && size(all_predictions, 2) > 1
        set(pred_traj_handle, 'XData', all_predictions(1, :), ...
            'YData', all_predictions(2, :));
    end
    
    % Update current prediction
    if ~isempty(predicted_robot3_pos)
        set(pred_pos_handle, 'XData', predicted_robot3_pos(1), ...
            'YData', predicted_robot3_pos(2));
        
        % Update prediction error line
        actual_pos = x(1:2, 3);
        set(pred_error_handle, 'XData', [actual_pos(1), predicted_robot3_pos(1)], ...
            'YData', [actual_pos(2), predicted_robot3_pos(2)]);
    end
    
    drawnow;
    
    %% Safety Checks
    dist_12 = norm(x(1:2, 1) - x(1:2, 2));
    dist_13 = norm(x(1:2, 1) - x(1:2, 3));
    dist_23 = norm(x(1:2, 2) - x(1:2, 3));
    
    if dist_12 < safe_radius_ego || dist_13 < safe_radius_ego || dist_23 < safe_radius
        fprintf('WARNING: Near collision at t=%.2fs! Distances: 1-2=%.3f, 1-3=%.3f, 2-3=%.3f\n', ...
            current_time, dist_12, dist_13, dist_23);
    end
    
    % Apply velocities
    r.set_velocities(1:N, dxu);
    r.step();
    
    pose_idx = pose_idx + 1;
    
    % Check termination conditions
    if all(goals_reached)
        fprintf('All robots reached goals at t=%.2fs\n', current_time);
        break;
    end
    
    if current_time > 60 % 60 second timeout
        fprintf('Simulation timeout at t=%.2fs\n', current_time);
        break;
    end
end

%% DATA SAVING
% Trim unused data
final_poses = pose_history(:, :, 1:pose_idx-1);
final_velocities = velocity_history(:, :, 1:pose_idx-1);
final_angular_velocities = angular_velocity_history(:, 1:pose_idx-1);

% Calculate prediction errors
if ~isempty(prediction_history) && ~isempty(actual_robot3_history)
    prediction_errors = sqrt(sum((prediction_history - actual_robot3_history).^2, 1));
    mean_prediction_error = mean(prediction_errors);
    max_prediction_error = max(prediction_errors);
    fprintf('\nPrediction Results:\n');
    fprintf('  Mean Error: %.4f m\n', mean_prediction_error);
    fprintf('  Max Error: %.4f m\n', max_prediction_error);
    fprintf('  Total Predictions: %d\n', length(prediction_errors));
else
    prediction_errors = [];
    mean_prediction_error = NaN;
    max_prediction_error = NaN;
    fprintf('\nNo predictions made during experiment\n');
end

% Save experimental data
save('APF_3Robot_Trajectory_Data.mat', 'final_poses', 'final_velocities', ...
     'final_angular_velocities', 'prediction_history', 'actual_robot3_history', ...
     'prediction_errors', 'mean_prediction_error', 'max_prediction_error', ...
     'start1', 'start2', 'start3', 'goal1', 'goal2', 'goal3', ...
     'straight_line_distance', 'total_distance_traveled', 'goals_reached', ...
     'boundaries', 'safe_radius', 'safe_radius_ego', 'detection_radius');

% Performance summary
fprintf('\nExperiment completed in %.2f seconds\n', toc(start_time));
for i = 1:N
    if total_distance_traveled(i) > 0
        efficiency = straight_line_distance(i) / total_distance_traveled(i);
        fprintf('Robot %d - Path efficiency: %.2f%% (traveled %.2fm for %.2fm straight-line)\n', ...
                i, efficiency*100, total_distance_traveled(i), straight_line_distance(i));
    end
end

% Required Robotarium debug call
r.debug();

%% HELPER FUNCTIONS

function [F_att, F_rep, F_combined] = calculate_apf_forces(current_pos, goal, obstacles, ...
    detection_radius, attraction_factor, repulsion_factor)
    % Calculate APF forces using the same method as validation script
    
    % Attraction force
    goal_vector = goal - current_pos;
    if norm(goal_vector) > 0
        F_att = attraction_factor * (goal_vector / norm(goal_vector));
    else
        F_att = [0; 0];
    end
    
    % Repulsion force
    F_rep = [0; 0];
    if ~isempty(obstacles)
        distances = zeros(1, size(obstacles, 2));
        for i = 1:size(obstacles, 2)
            distances(i) = norm(obstacles(:, i) - current_pos);
        end
        
        [min_distance, nearest_idx] = min(distances);
        
        if min_distance <= detection_radius && min_distance > 0
            obstacle_vector = current_pos - obstacles(:, nearest_idx);
            F_rep = (obstacle_vector / norm(obstacle_vector)) * ...
                   (detection_radius - min_distance) / detection_radius;
            F_rep = repulsion_factor * F_rep;
        end
    end
    
    % Combined force
    F_combined = F_att + F_rep;
    
    % Normalize
    if norm(F_combined) > 0
        F_combined = F_combined / norm(F_combined);
    end
end

function [newStart, newGoal, interceptPoint] = generateInterceptingPathSimple(refStart, refGoal, ...
    x_bound, y_bound, min_path_length, safeRadius)
    % Generate intercepting path (simplified version)
    
    max_attempts = 50;
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        dir = refGoal - refStart;
        t = 0.4 + rand() * 0.2; % Intercept at 40-60% along path
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
    
    fallback_radius = max(min_path_length/2, safeRadius);
    newStart = center + fallback_radius * [cos(angle2 + pi); sin(angle2 + pi)];
    newGoal = center + fallback_radius * [cos(angle2); sin(angle2)];
    
    newStart(1) = max(-x_bound, min(x_bound, newStart(1)));
    newStart(2) = max(-y_bound, min(y_bound, newStart(2)));
    newGoal(1) = max(-x_bound, min(x_bound, newGoal(1)));
    newGoal(2) = max(-y_bound, min(y_bound, newGoal(2)));
    
    interceptPoint = center;
end

function point = generateRandomPointOnCircleBounded(center, radius, x_bound, y_bound)
    % Generate random point on circle within bounds
    
    max_attempts = 30;
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

function [egoStart, egoGoal] = generateEgoRobotPath(allyStart, enemyStart, interceptPoint, x_bound, y_bound, safeRadius)
    % Generate ego robot path using ValidationEnvironment_APF algorithm
    
    max_attempts = 10000;
    for attempt = 1:max_attempts
        % Calculate distance from allyStart to interceptPoint
        distance = norm(interceptPoint - allyStart);
        
        % Vector from enemy to intercept point
        vect_to_intercept = interceptPoint - enemyStart;
        if norm(vect_to_intercept) > 0
            vect_to_intercept = vect_to_intercept / norm(vect_to_intercept);
        else
            vect_to_intercept = [1; 0]; % Default direction
        end
        
        % Random angle for positioning
        angle = -pi * rand();
        
        % Rotation matrix
        rotation_matrix = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        % Calculate ego start position
        egoStart = enemyStart + 2 * distance * (rotation_matrix * vect_to_intercept);
        
        % Check if egoStart is far enough from both robots and within bounds
        if norm(egoStart - allyStart) > safeRadius && ...
           norm(egoStart - enemyStart) > safeRadius && ...
           egoStart(1) >= -x_bound && egoStart(1) <= x_bound && ...
           egoStart(2) >= -y_bound && egoStart(2) <= y_bound
            
            % Calculate egoGoal
            dir = interceptPoint - egoStart;
            egoGoal = interceptPoint + dir;
            
            % Check if egoGoal is within bounds
            if egoGoal(1) >= -x_bound && egoGoal(1) <= x_bound && ...
               egoGoal(2) >= -y_bound && egoGoal(2) <= y_bound
                return;
            end
        end
    end
    
    % Fallback if no valid configuration found
    vec_enemy_to_ally = allyStart - enemyStart;
    if norm(vec_enemy_to_ally) > 0
        vec_enemy_to_ally = vec_enemy_to_ally / norm(vec_enemy_to_ally);
    else
        vec_enemy_to_ally = [1; 0];
    end
    
    % Perpendicular vector for crossing
    perp_vec = [-vec_enemy_to_ally(2); vec_enemy_to_ally(1)];
    
    % Place ego robot perpendicular to enemy-ally line
    egoStart = interceptPoint + safeRadius * 2 * perp_vec;
    egoGoal = interceptPoint - safeRadius * 2 * perp_vec;
    
    % Bound to arena
    egoStart(1) = max(-x_bound, min(x_bound, egoStart(1)));
    egoStart(2) = max(-y_bound, min(y_bound, egoStart(2)));
    egoGoal(1) = max(-x_bound, min(x_bound, egoGoal(1)));
    egoGoal(2) = max(-y_bound, min(y_bound, egoGoal(2)));
end

function angle = wrapToPi(angle)
    % Wrap angle to [-pi, pi]
    angle = atan2(sin(angle), cos(angle));
end