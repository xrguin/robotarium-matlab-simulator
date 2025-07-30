%% APF 3-Robot Validation Environment for Robotarium
% Based on ValidationEnvironment_APF.m but adapted for Robotarium simulator
% Tests trained network's ability to predict Robot 3 position from Robot 2 trajectory
% Robot 1 (ego) uses both actual Robot 2 and predicted Robot 3 for navigation

% Initialize Robotarium paths
addpath(genpath('utilities'));
addpath(genpath('patch_generation'));
addpath('./');

clc
clear
close all

%% Program Settings
animation = true;        % Set true to show real-time animation
save_video = false;       % Set true to save validation videos
record_data = false;      % Set true to save validation results
samples = 1;             % Number of validation scenarios to test

%% Load Trained Model
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

%% Validation Parameters
N = 3; % Number of robots
window_size = offset + 1; % Prediction window size (30 for offset=29)

% APF Parameters (matched to training data roles)
attraction_factor_robot1 = 1;    % Robot 1 (ego) 
repulsion_factor_robot1 = 1.5;   % Robot 1 (ego)
attraction_factor_robot2 = 1;    % Robot 2 (ally) - matches training Robot 1 (input)
repulsion_factor_robot2 = 1.7;   % Robot 2 (ally) - matches training Robot 1 (input)
attraction_factor_robot3 = 1;    % Robot 3 (adversarial) - matches training Robot 2 (predicted)
repulsion_factor_robot3 = 1.5;   % Robot 3 (adversarial) - matches training Robot 2 (predicted)

detection_radius = 1;            % APF detection range
safe_radius = 0.3;               % Safety radius around robots
safe_radius_ego = 0.2;
constant_linear_velocity = 0.1;  % Constant linear velocity (m/s)

% Robotarium Constants
max_linear_velocity = 0.08;
max_angular_velocity = 3.5;
actual_max_linear_velocity = 0.1;  % Actual robot maximum speed restriction
robot_diameter = 0.11;
boundaries = [-1.6, 1.6, -1, 1]; % [x_min, x_max, y_min, y_max]

% Simulation Parameters
sampleTime = 0.033;              % Robotarium time step
simulationTime = 30;             % Maximum simulation time
iterations = ceil(simulationTime / sampleTime);
goal_radius = 0.1;               % Distance to consider goal reached

% Arena bounds
boundary_buffer = 0.3;
x_bound = boundaries(2) - boundary_buffer;
y_bound = boundaries(4) - boundary_buffer;
x_bound_gen = x_bound - 0.2;
y_bound_gen = y_bound - 0.2;

% Path generation parameters
min_path_length = 1;
max_path_length = 2;
min_robot_separation = 1.0;
max_generation_attempts = 20;

%% Validation Results Storage
validation_results = cell(samples, 1);
collision_count = 0;
prediction_errors = [];

%% Video Setup
if save_video
    video_folder = fullfile(pwd, 'validation_videos');
    if ~exist(video_folder, 'dir')
        mkdir(video_folder);
    end
end

%% Main Validation Loop
fprintf('Starting APF 3-robot validation with %d scenarios\n', samples);

for n = 1:samples
    fprintf('\n=== Validation Scenario %d/%d ===\n', n, samples);
    
    %% Generate Initial Conditions (following apf_obstacle_avoidance.m pattern)
    % Robot 2 (ally) - primary trajectory (same as Robot 1 in apf_obstacle_avoidance.m)
    start2 = [(rand()*2-1)*x_bound_gen; (rand()*2-1)*y_bound_gen];
    
    % Generate goal ensuring minimum distance
    path_length = min_path_length + rand() * (max_path_length - min_path_length);
    angle2 = rand() * 2 * pi;
    goal2 = start2 + path_length * [cos(angle2); sin(angle2)];
    goal2(1) = max(-x_bound_gen, min(x_bound_gen, goal2(1)));
    goal2(2) = max(-y_bound_gen, min(y_bound_gen, goal2(2)));
    
    % Regenerate if path is too short after bounding (same as apf_obstacle_avoidance.m)
    max_attempts = 20;
    attempt = 0;
    while norm(goal2 - start2) < min_path_length && attempt < max_attempts
        angle2 = rand() * 2 * pi;
        goal2 = start2 + path_length * [cos(angle2); sin(angle2)];
        goal2(1) = max(-x_bound_gen, min(x_bound_gen, goal2(1)));
        goal2(2) = max(-y_bound_gen, min(y_bound_gen, goal2(2)));
        attempt = attempt + 1;
    end
    
    % Robot 3 (adversarial/obstacle) - intercepting path (same as Robot 2 in apf_obstacle_avoidance.m)
    [start3, goal3, interceptPoint] = generateInterceptingPathSimple(start2, goal2, ...
        x_bound_gen, y_bound_gen, min_path_length, safe_radius*2);
    
    % Ensure robots don't start too close together (same as apf_obstacle_avoidance.m)
    attempts = 0;
    while norm(start3 - start2) < min_robot_separation && attempts < max_generation_attempts
        [start3, goal3, interceptPoint] = generateInterceptingPathSimple(start2, goal2, ...
            x_bound_gen, y_bound_gen, min_path_length, safe_radius*2);
        attempts = attempts + 1;
    end
    
    if attempts >= max_generation_attempts
        fprintf('Warning: Could not find well-separated initial positions for robots 2&3\n');
    end
    
    % Robot 1 (ego) - generate using specific algorithm from ValidationEnvironment_APF
    [start1, goal1] = generateEgoRobotPath(start2, start3, interceptPoint, x_bound_gen, y_bound_gen, safe_radius);
    
    % Calculate initial headings
    heading1 = atan2(goal1(2) - start1(2), goal1(1) - start1(1));
    heading2 = atan2(goal2(2) - start2(2), goal2(1) - start2(1));
    heading3 = atan2(goal3(2) - start3(2), goal3(1) - start3(1));
    
    initial_conditions = [start1, start2, start3; heading1, heading2, heading3];
    
    %% Initialize Robotarium
    if n == 1
        r = Robotarium('NumberOfRobots', N, 'ShowFigure', animation, 'InitialConditions', initial_conditions);
    else
        if exist('r', 'var')
            r.debug();
        end
        r = Robotarium('NumberOfRobots', N, 'ShowFigure', animation, 'InitialConditions', initial_conditions);
    end
    
    %% Video Setup for This Scenario
    if save_video
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        video_filename = fullfile(video_folder, sprintf('validation_scenario_%d_%s.mp4', n, timestamp));
        try
            v = VideoWriter(video_filename, 'MPEG-4');
        catch
            video_filename = fullfile(video_folder, sprintf('validation_scenario_%d_%s.avi', n, timestamp));
            v = VideoWriter(video_filename, 'Motion JPEG AVI');
        end
        v.FrameRate = 10;
        open(v);
    end
    
    %% Scenario Data Storage
    pose_history = zeros(4, N, iterations); % [x; y; theta; omega]
    velocity_history = zeros(2, N, iterations); % [v; omega]
    prediction_history = [];
    actual_robot3_history = [];
    
    % Prediction variables
    robot2_pose_window = zeros(4, window_size); % Store Robot 2 poses for prediction
    prediction_buffer = zeros(5, 2); % FILO buffer for prediction smoothing
    buffer_idx = 1;
    recording = false;
    
    % Speed filtering variables
    prevRobot3Pos = [];
    prevPredictedPos = [];
    speedThresholdMultiplier = 1.5;  % Threshold = 1.5 × actual robot3 speed
    minSpeedForFiltering = 0.05;  % Only apply filtering when robot3 speed < this value
    
    % Store all predictions for trajectory visualization
    all_predictions = [];
    all_actual_robot3 = [];
    
    % Performance tracking
    scenario_collisions = false;
    min_distance_12 = inf;
    min_distance_13 = inf;
    min_distance_23 = inf;
    goals_reached = [false, false, false];
    
    pose_idx = 1;
    prev_angles = [heading1, heading2, heading3];
    
    fprintf('Robot positions: R1[%.2f,%.2f] R2[%.2f,%.2f] R3[%.2f,%.2f]\n', ...
        start1(1), start1(2), start2(1), start2(2), start3(1), start3(2));
    
    %% Main Simulation Loop
    for t = 1:iterations
        % Get current poses
        x = r.get_poses();
        current_time = t * sampleTime;
        
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
        
        % Store pose history
        pose_history(1:3, :, pose_idx) = x;
        pose_history(4, :, pose_idx) = angular_velocities;
        
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
            obstacles_robot2 = x(1:2,  3); % Robot 1 and Robot 3
            [F_att2, F_rep2, F_combined2] = calculate_apf_forces_mbstyle(robot2_pos, goal2, ...
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
            v2 = min(constant_linear_velocity, actual_max_linear_velocity);
        else
            v2 = 0; omega2 = 0;
            goals_reached(2) = true;
        end
        
        % Robot 3 (adversarial) - only sees Robot 2 (ally), ignores ego Robot 1
        robot3_pos = x(1:2, 3);
        robot3_heading = x(3, 3);
        distance_to_goal3 = norm(goal3 - robot3_pos);
        
        if distance_to_goal3 > goal_radius
            obstacles_robot3 = x(1:2, 2); % Only Robot 2 (ally), ignores ego
            [F_att3, F_rep3, F_combined3] = calculate_apf_forces_mbstyle(robot3_pos, goal3, ...
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
            v3 = min(constant_linear_velocity, actual_max_linear_velocity);
        else
            v3 = 0; omega3 = 0;
            goals_reached(3) = true;
        end
        
        %% Robot 2 Prediction System
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
                        % Calculate predicted speed
                   % if ~isempty(prevPredictedPos)
                        predictedSpeed = norm(predicted_robot3_raw' - prevPredictedPos) / sampleTime;
                        speedThreshold = speedThresholdMultiplier * actualRobot3Speed;
                        
                        % If predicted speed exceeds threshold, use previous prediction
                        if predictedSpeed > speedThreshold
                            predicted_robot3_raw = prevPredictedPos';
                            fprintf('Trial %d: Filtered prediction at t=%.2fs (pred speed=%.3f > threshold=%.3f)\n', ...
                                n, current_time, predictedSpeed, speedThreshold);
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
                
                % Store all predictions for trajectory visualization
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
            % else
            %     % Fallback: use actual Robot 3 position if no prediction yet
            %     obstacles_robot1 = [obstacles_robot1, x(1:2, 3)];
            end
            
            [F_att1, F_rep1, F_combined1] = calculate_apf_forces_mbstyle(robot1_pos, goal1, ...
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
            v1 = min(constant_linear_velocity, actual_max_linear_velocity);
        else
            v1 = 0; omega1 = 0;
            goals_reached(1) = true;
        end
        
        % Set control inputs with final velocity constraints
        dxu(:, 1) = [v1; omega1];
        dxu(:, 2) = [v2; omega2];
        dxu(:, 3) = [v3; omega3];
        
        % Final check: ensure velocities don't exceed actual robot limits
        for i = 1:N
            if abs(dxu(1, i)) > actual_max_linear_velocity
                dxu(1, i) = sign(dxu(1, i)) * actual_max_linear_velocity;
                fprintf('Warning: Clamped robot %d linear velocity to %.2f m/s\n', i, actual_max_linear_velocity);
            end
            if abs(dxu(2, i)) > max_angular_velocity
                dxu(2, i) = sign(dxu(2, i)) * max_angular_velocity;
            end
        end
        
        velocity_history(:, :, pose_idx) = dxu;
        
        %% Safety Checks
        dist_12 = norm(x(1:2, 1) - x(1:2, 2));
        dist_13 = norm(x(1:2, 1) - x(1:2, 3));
        dist_23 = norm(x(1:2, 2) - x(1:2, 3));
        
        min_distance_12 = min(min_distance_12, dist_12);
        min_distance_13 = min(min_distance_13, dist_13);
        min_distance_23 = min(min_distance_23, dist_23);
        
        if dist_12 < safe_radius_ego || dist_13 < safe_radius_ego || dist_23 < safe_radius
            scenario_collisions = true;
            fprintf('COLLISION at t=%.2fs! Distances: 1-2=%.3f, 1-3=%.3f, 2-3=%.3f\n', ...
                current_time, dist_12, dist_13, dist_23);
            break;
        end
        
        %% Visualization
        if animation
            updateValidationVisualization(r.figure_handle, x, pose_history, pose_idx, ...
                [start1, start2, start3], [goal1, goal2, goal3], predicted_robot3_pos, safe_radius, ...
                all_predictions, all_actual_robot3);
            
            if save_video && exist('v', 'var')
                frame = getframe(r.figure_handle);
                writeVideo(v, frame);
            end
        end
        
        % Set velocities and step
        r.set_velocities(1:N, dxu);
        r.step();
        
        pose_idx = pose_idx + 1;
        
        % Check termination conditions
        if all(goals_reached)
            fprintf('All robots reached goals at t=%.2fs\n', current_time);
            break;
        end
        
        if current_time > simulationTime
            fprintf('Simulation timeout at t=%.2fs\n', current_time);
            break;
        end
    end
    
    %% Scenario Analysis
    if scenario_collisions
        collision_count = collision_count + 1;
    end
    
    % Calculate prediction errors
    if ~isempty(prediction_history) && ~isempty(actual_robot3_history)
        if size(prediction_history, 2) == size(actual_robot3_history, 2)
            errors = sqrt(sum((prediction_history - actual_robot3_history).^2, 1));
            scenario_mean_error = mean(errors);
            scenario_max_error = max(errors);
            prediction_errors = [prediction_errors, errors];
            
            fprintf('Prediction Results:\n');
            fprintf('  Mean Error: %.4f m\n', scenario_mean_error);
            fprintf('  Max Error: %.4f m\n', scenario_max_error);
            fprintf('  Predictions Made: %d\n', length(errors));
        else
            scenario_mean_error = NaN;
            scenario_max_error = NaN;
            fprintf('Prediction size mismatch - no error calculated\n');
        end
    else
        scenario_mean_error = NaN;
        scenario_max_error = NaN;
        fprintf('No predictions made during scenario\n');
    end
    
    % Store results
    validation_results{n} = struct(...
        'collision', scenario_collisions, ...
        'min_distances', [min_distance_12, min_distance_13, min_distance_23], ...
        'goals_reached', goals_reached, ...
        'simulation_time', (pose_idx-1) * sampleTime, ...
        'prediction_mean_error', scenario_mean_error, ...
        'prediction_max_error', scenario_max_error, ...
        'num_predictions', size(prediction_history, 2), ...
        'initial_positions', [start1, start2, start3], ...
        'goal_positions', [goal1, goal2, goal3]);
    
    fprintf('Scenario %d Summary:\n', n);
    fprintf('  Collision: %s\n', string(scenario_collisions));
    fprintf('  Min Distances: 1-2=%.3f, 1-3=%.3f, 2-3=%.3f\n', min_distance_12, min_distance_13, min_distance_23);
    fprintf('  Goals Reached: R1=%s, R2=%s, R3=%s\n', string(goals_reached));
    
    % Close video for this scenario
    if save_video && exist('v', 'var')
        close(v);
        fprintf('Video saved: %s\n', video_filename);
    end
end % End of for n = 1:samples loop

%% Overall Validation Results
fprintf('\n=== VALIDATION RESULTS SUMMARY ===\n');
fprintf('Total Scenarios: %d\n', samples);
fprintf('Collisions: %d (%.1f%%)\n', collision_count, 100*collision_count/samples);

if ~isempty(prediction_errors)
    fprintf('Prediction Performance:\n');
    fprintf('  Overall Mean Error: %.4f m\n', mean(prediction_errors));
    fprintf('  Overall Std Error: %.4f m\n', std(prediction_errors));
    fprintf('  Overall Max Error: %.4f m\n', max(prediction_errors));
    fprintf('  Total Predictions: %d\n', length(prediction_errors));
end

% Save validation results
if record_data
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    save_filename = sprintf('validation_results_%s.mat', timestamp);
    save(save_filename, 'validation_results', 'prediction_errors', ...
         'collision_count', 'samples');
    fprintf('\nValidation results saved to: %s\n', save_filename);
end

% Clean up
r.debug();
fprintf('\nValidation complete!\n');

%% Helper Functions

function [F_att, F_rep, F_combined] = calculate_apf_forces_mbstyle(current_pos, goal, obstacles, ...
    detection_radius, attraction_factor, repulsion_factor)
    % Calculate APF forces using the same method as apf_obstacle_avoidance.m
    
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
    % Generate intercepting path (simplified version from apf_obstacle_avoidance.m)
    
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

function updateValidationVisualization(fig_handle, current_poses, pose_history, pose_idx, starts, goals, predicted_pos, safe_radius, all_predictions, all_actual)
    % Update visualization for 3-robot validation with persistent legend
    
    persistent plot_handles legend_created legend_handle
    
    figure(fig_handle);
    
    % Robot colors - more distinct colors
    colors = {
        [1, 0, 0],      % Robot 1 (Ego) - Bright Red
        [0, 0.8, 0],    % Robot 2 (Ally) - Bright Green  
        [0, 0, 1]       % Robot 3 (Adversarial) - Bright Blue
    };
    names = {'Ego (R1)', 'Ally (R2)', 'Adversarial (R3)'};
    
    % Initialize plot handles on first call
    if isempty(plot_handles) || ~isvalid(plot_handles.traj(1))
        % Clear only our custom elements, not Robotarium objects
        delete(findobj(gca, 'Tag', 'ValTraj'));
        delete(findobj(gca, 'Tag', 'ValMarker'));
        delete(findobj(gca, 'Tag', 'ValPrediction'));
        delete(findobj(gca, 'Tag', 'ValSafety'));
        delete(findobj(gca, 'Tag', 'ValPredTraj'));
        
        hold on;
        
        % Initialize trajectory handles
        for i = 1:3
            plot_handles.traj(i) = plot(nan, nan, '-', 'Color', colors{i}, 'LineWidth', 2, 'Tag', 'ValTraj');
        end
        
        % Initialize safety circle handles
        theta_circle = linspace(0, 2*pi, 30);
        for i = 1:3
            light_color = colors{i} * 0.5 + [0.5, 0.5, 0.5];
            plot_handles.safety(i) = plot(nan, nan, ':', 'Color', light_color, 'LineWidth', 1, 'Tag', 'ValSafety');
        end
        
        % Initialize marker handles
        for i = 1:3
            plot_handles.start(i) = plot(starts(1,i), starts(2,i), 'o', 'MarkerSize', 12, ...
                'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'Tag', 'ValMarker');
            plot_handles.goal(i) = plot(goals(1,i), goals(2,i), 'p', 'MarkerSize', 15, ...
                'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'Tag', 'ValMarker');
        end
        
        % Initialize prediction handles
        plot_handles.pred_traj = plot(nan, nan, ':', 'Color', [1, 0, 1], 'LineWidth', 3, 'Tag', 'ValPredTraj');
        plot_handles.pred_pos = plot(nan, nan, 's', 'MarkerSize', 12, ...
            'MarkerFaceColor', [1, 0, 1], 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'Tag', 'ValPrediction');
        plot_handles.pred_error = plot(nan, nan, 'k--', 'LineWidth', 2, 'Tag', 'ValPrediction');
        
        % Arena boundaries
        rectangle('Position', [-1.6, -1, 3.2, 2], 'EdgeColor', 'k', 'LineWidth', 2, 'Tag', 'ValMarker');
        
        axis equal;
        xlim([-1.8, 1.8]);
        ylim([-1.2, 1.2]);
        xlabel('X (m)');
        ylabel('Y (m)');
        title('APF 3-Robot Validation - Network Prediction');
        
        legend_created = false;
    end
    
    % Update trajectory data
    if pose_idx > 1
        for i = 1:3
            trajectory = squeeze(pose_history(1:2, i, 1:pose_idx-1));
            if size(trajectory, 2) > 1
                set(plot_handles.traj(i), 'XData', trajectory(1,:), 'YData', trajectory(2,:));
            end
        end
    end
    
    % Update safety circles
    theta_circle = linspace(0, 2*pi, 30);
    for i = 1:3
        pos = current_poses(1:2, i);
        safety_circle = pos + safe_radius * [cos(theta_circle); sin(theta_circle)];
        set(plot_handles.safety(i), 'XData', safety_circle(1,:), 'YData', safety_circle(2,:));
    end
    
    % Update predicted trajectory
    if ~isempty(all_predictions) && size(all_predictions, 2) > 1
        set(plot_handles.pred_traj, 'XData', all_predictions(1,:), 'YData', all_predictions(2,:));
    end
    
    % Update current prediction
    if ~isempty(predicted_pos)
        set(plot_handles.pred_pos, 'XData', predicted_pos(1), 'YData', predicted_pos(2));
        
        % Update prediction error line
        actual_pos = current_poses(1:2, 3);
        set(plot_handles.pred_error, 'XData', [actual_pos(1), predicted_pos(1)], ...
            'YData', [actual_pos(2), predicted_pos(2)]);
        
        % Create legend only once when predictions start
        if ~legend_created
            legend_items = {
                sprintf('%s - Red', names{1}), ...
                sprintf('%s - Green', names{2}), ...
                sprintf('%s - Blue', names{3}), ...
                'Predicted R3 Path (Magenta)', ...
                'Prediction Error (Black)'
            };
            legend_handle = legend([plot_handles.traj, plot_handles.pred_traj, plot_handles.pred_error], ...
                legend_items, 'Location', 'best', 'FontSize', 10);
            legend_created = true;
        end
    else
        % Create basic legend without predictions
        if ~legend_created
            legend_items = {
                sprintf('%s - Red', names{1}), ...
                sprintf('%s - Green', names{2}), ...
                sprintf('%s - Blue', names{3})
            };
            legend_handle = legend(plot_handles.traj, legend_items, 'Location', 'best', 'FontSize', 10);
            legend_created = true;
        end
    end
    
    drawnow;
end

function [egoStart, egoGoal] = generateEgoRobotPath(allyStart, enemyStart, interceptPoint, x_bound, y_bound, safeRadius)
    % Generate ego robot path using ValidationEnvironment_APF algorithm
    % Adapted for Robotarium coordinate system
    
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
                fprintf('Ego robot generated: start=[%.2f, %.2f], goal=[%.2f, %.2f]\n', ...
                    egoStart(1), egoStart(2), egoGoal(1), egoGoal(2));
                return;
            end
        end
    end
    
    % Fallback if no valid configuration found
    fprintf('Warning: Could not generate ego robot with original algorithm, using fallback\n');
    
    % Simple fallback: position ego robot to create crossing path
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