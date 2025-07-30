%% Data Analysis for APF 3-Robot Experiment with Neural Network Prediction
% This script analyzes the experimental data from apf_3robots_robotarium_online.m
% Includes trajectory visualization, prediction accuracy, and performance metrics

clear; close all; clc;

%% Load Data
fprintf('Loading APF 3-Robot experimental data...\n');
data_file = 'APF_3Robot_Trajectory_Data.mat';

if ~exist(data_file, 'file')
    error('Data file not found: %s\nPlease run apf_3robots_robotarium_online.m first.', data_file);
end

load(data_file);
fprintf('Data loaded successfully!\n\n');

%% Extract Data
N = 3; % Number of robots
num_timesteps = size(final_poses, 3);
time_vector = (0:num_timesteps-1) * 0.033; % 30Hz sample rate

% Robot colors
robot_colors = {[1, 0.5, 0], [0, 1, 0], [0, 0, 1]}; % Orange, Green, Blue
robot_names = {'Robot 1 (Ego)', 'Robot 2 (Ally)', 'Robot 3 (Adversarial)'};

%% Calculate Performance Metrics
fprintf('=== PERFORMANCE METRICS ===\n');

% Path efficiency
fprintf('\nPath Efficiency:\n');
for i = 1:N
    if total_distance_traveled(i) > 0
        efficiency = straight_line_distance(i) / total_distance_traveled(i) * 100;
        fprintf('  %s: %.1f%% (traveled %.3fm for %.3fm straight-line)\n', ...
            robot_names{i}, efficiency, total_distance_traveled(i), straight_line_distance(i));
    end
end

% Minimum inter-robot distances
fprintf('\nMinimum Inter-Robot Distances:\n');
min_dist_12 = inf; min_dist_13 = inf; min_dist_23 = inf;
min_time_12 = 0; min_time_13 = 0; min_time_23 = 0;

for t = 1:num_timesteps
    dist_12 = norm(final_poses(1:2, 1, t) - final_poses(1:2, 2, t));
    dist_13 = norm(final_poses(1:2, 1, t) - final_poses(1:2, 3, t));
    dist_23 = norm(final_poses(1:2, 2, t) - final_poses(1:2, 3, t));
    
    if dist_12 < min_dist_12
        min_dist_12 = dist_12;
        min_time_12 = time_vector(t);
    end
    if dist_13 < min_dist_13
        min_dist_13 = dist_13;
        min_time_13 = time_vector(t);
    end
    if dist_23 < min_dist_23
        min_dist_23 = dist_23;
        min_time_23 = time_vector(t);
    end
end

fprintf('  Robot 1-2: %.3fm at t=%.2fs\n', min_dist_12, min_time_12);
fprintf('  Robot 1-3: %.3fm at t=%.2fs\n', min_dist_13, min_time_13);
fprintf('  Robot 2-3: %.3fm at t=%.2fs\n', min_dist_23, min_time_23);

% Prediction accuracy
if ~isempty(prediction_errors)
    fprintf('\nPrediction Accuracy:\n');
    fprintf('  Mean prediction error: %.4fm\n', mean_prediction_error);
    fprintf('  Max prediction error: %.4fm\n', max_prediction_error);
    fprintf('  Std prediction error: %.4fm\n', std(prediction_errors));
    fprintf('  Number of predictions: %d\n', length(prediction_errors));
    
    % Find when predictions started
    pred_start_idx = num_timesteps - length(prediction_errors) + 1;
    pred_start_time = time_vector(pred_start_idx);
    fprintf('  Predictions started at: t=%.2fs\n', pred_start_time);
end

% Goal achievement
fprintf('\nGoal Achievement:\n');
for i = 1:N
    if goals_reached(i)
        fprintf('  %s: Reached goal\n', robot_names{i});
    else
        fprintf('  %s: Did not reach goal\n', robot_names{i});
    end
end

%% Create Figure with Subplots
figure('Position', [100, 100, 1400, 900], 'Name', 'APF 3-Robot Experiment Analysis');

%% Subplot 1: 2D Arena View with Full Trajectories
subplot(2, 3, [1, 2]);
hold on;

% Plot boundaries
rectangle('Position', [-1.6, -1, 3.2, 2], 'EdgeColor', 'k', 'LineWidth', 2);

% Plot trajectories
for i = 1:N
    trajectory = squeeze(final_poses(1:2, i, :));
    plot(trajectory(1, :), trajectory(2, :), '-', 'Color', robot_colors{i}, ...
        'LineWidth', 2, 'DisplayName', robot_names{i});
end

% Plot start positions
plot(start1(1), start1(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', robot_colors{1}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(start2(1), start2(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', robot_colors{2}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(start3(1), start3(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', robot_colors{3}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');

% Plot goal positions
plot(goal1(1), goal1(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', robot_colors{1}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(goal2(1), goal2(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', robot_colors{2}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(goal3(1), goal3(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', robot_colors{3}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');

% Add start/goal labels
text(start1(1)-0.1, start1(2)-0.1, 'S1', 'FontSize', 10, 'FontWeight', 'bold');
text(start2(1)-0.1, start2(2)-0.1, 'S2', 'FontSize', 10, 'FontWeight', 'bold');
text(start3(1)-0.1, start3(2)-0.1, 'S3', 'FontSize', 10, 'FontWeight', 'bold');
text(goal1(1)-0.1, goal1(2)+0.1, 'G1', 'FontSize', 10, 'FontWeight', 'bold');
text(goal2(1)-0.1, goal2(2)+0.1, 'G2', 'FontSize', 10, 'FontWeight', 'bold');
text(goal3(1)-0.1, goal3(2)+0.1, 'G3', 'FontSize', 10, 'FontWeight', 'bold');

axis equal;
xlim([-1.8, 1.8]);
ylim([-1.2, 1.2]);
xlabel('X (m)');
ylabel('Y (m)');
title('Robot Trajectories in Arena');
legend('Location', 'best');
grid on;

%% Subplot 2: Predicted vs Actual Robot 3 Positions
subplot(2, 3, 3);
hold on;

if ~isempty(prediction_history)
    % Plot actual Robot 3 trajectory
    actual_traj = squeeze(final_poses(1:2, 3, :));
    plot(actual_traj(1, :), actual_traj(2, :), '-', 'Color', robot_colors{3}, ...
        'LineWidth', 2, 'DisplayName', 'Actual R3');
    
    % Plot predicted positions
    plot(prediction_history(1, :), prediction_history(2, :), 'm:', ...
        'LineWidth', 2, 'DisplayName', 'Predicted R3');
    
    % Highlight start of predictions
    plot(prediction_history(1, 1), prediction_history(2, 1), 'ms', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Pred Start');
end

% Plot Robot 3 start and goal
plot(start3(1), start3(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', robot_colors{3}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(goal3(1), goal3(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', robot_colors{3}, ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');

axis equal;
xlim([-1.8, 1.8]);
ylim([-1.2, 1.2]);
xlabel('X (m)');
ylabel('Y (m)');
title('Predicted vs Actual Robot 3 Trajectory');
legend('Location', 'best');
grid on;

%% Subplot 3: Prediction Error Over Time
subplot(2, 3, 4);

if ~isempty(prediction_errors)
    pred_time_vector = time_vector(pred_start_idx:pred_start_idx+length(prediction_errors)-1);
    plot(pred_time_vector, prediction_errors, 'r-', 'LineWidth', 2);
    hold on;
    
    % Add mean error line
    mean_error_line = mean_prediction_error * ones(size(pred_time_vector));
    plot(pred_time_vector, mean_error_line, 'k--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Mean: %.3fm', mean_prediction_error));
    
    % Add shaded region for std deviation
    std_upper = mean_error_line + std(prediction_errors);
    std_lower = mean_error_line - std(prediction_errors);
    fill([pred_time_vector, fliplr(pred_time_vector)], ...
         [std_upper, fliplr(std_lower)], 'k', 'FaceAlpha', 0.2, ...
         'EdgeColor', 'none', 'HandleVisibility', 'off');
end

xlabel('Time (s)');
ylabel('Prediction Error (m)');
title('Robot 3 Position Prediction Error');
legend('Location', 'best');
grid on;

%% Subplot 4: Linear Velocity Profiles
subplot(2, 3, 5);
hold on;

for i = 1:N
    velocities = squeeze(final_velocities(1, i, :));
    plot(time_vector, velocities, '-', 'Color', robot_colors{i}, ...
        'LineWidth', 2, 'DisplayName', robot_names{i});
end

% Add max velocity line
plot(time_vector, 0.1*ones(size(time_vector)), 'k--', 'LineWidth', 1, ...
    'DisplayName', 'Max Velocity');

xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Linear Velocity Profiles');
legend('Location', 'best');
grid on;
ylim([-0.02, 0.12]);

%% Subplot 5: Angular Velocity Profiles
subplot(2, 3, 6);
hold on;

for i = 1:N
    omega_velocities = squeeze(final_velocities(2, i, :));
    plot(time_vector, omega_velocities, '-', 'Color', robot_colors{i}, ...
        'LineWidth', 2, 'DisplayName', robot_names{i});
end

% Add max angular velocity lines
plot(time_vector, 3.5*ones(size(time_vector)), 'k--', 'LineWidth', 1, ...
    'HandleVisibility', 'off');
plot(time_vector, -3.5*ones(size(time_vector)), 'k--', 'LineWidth', 1, ...
    'HandleVisibility', 'off');

% Mark prediction start
if ~isempty(prediction_errors)
    plot([pred_start_time, pred_start_time], [-4, 4], 'r:', 'LineWidth', 2, ...
        'DisplayName', 'Prediction Start');
end

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Profiles');
legend('Location', 'best');
grid on;
ylim([-4, 4]);

%% Overall title
sgtitle('APF 3-Robot Experiment with Neural Network Prediction Analysis', ...
    'FontSize', 16, 'FontWeight', 'bold');

%% Print final summary
fprintf('\n=== EXPERIMENT SUMMARY ===\n');
fprintf('Total experiment time: %.2f seconds\n', time_vector(end));
fprintf('Safe radius: %.2fm (ego: %.2fm)\n', safe_radius, safe_radius_ego);
fprintf('Detection radius: %.2fm\n', detection_radius);

if min_dist_12 < safe_radius_ego || min_dist_13 < safe_radius_ego || min_dist_23 < safe_radius
    fprintf('\nWARNING: Minimum distance violation detected!\n');
end

fprintf('\nAnalysis complete!\n');