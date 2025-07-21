%% Plot CBF Trajectory Data
% This script creates a visualization similar to Figure 2 from cbf_obstacle_avoidance.m
% using the collected trajectory data from CBF_Trajectory_Data.mat

clear; close all; clc;

%% Load Data
load('CBF_Trajectory_Data.mat');

% Extract data dimensions
[~, num_robots, num_timesteps] = size(final_poses);
sampleTime = 0.033; % Same as original code
time_vec = (0:num_timesteps-1) * sampleTime;

%% Create Figure
fig = figure('Name', 'CBF Trajectory Analysis', 'Position', [100, 100, 1000, 700]);

% Colors for robots
colors = {[0,1,0], [0,0,1]}; % Green and blue

%% Main Trajectory Plot (positions [1,3])
subplot(2,2,[1,3]);
hold on;

% Plot arena boundary
rectangle('Position', [boundaries(1), boundaries(3), ...
    boundaries(2)-boundaries(1), boundaries(4)-boundaries(3)], ...
    'EdgeColor', 'k', 'LineWidth', 2);

% Plot CBF safety boundaries (inner rectangle)
inner_bounds = [boundaries(1)+safety_margin_boundaries, boundaries(3)+safety_margin_boundaries, ...
    boundaries(2)-boundaries(1)-2*safety_margin_boundaries, boundaries(4)-boundaries(3)-2*safety_margin_boundaries];
rectangle('Position', inner_bounds, ...
    'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1);

% Initialize legend handles and labels
h_legends = [];
legend_labels = {};

% Add boundary legends
h_boundary = plot(NaN, NaN, 'k-', 'LineWidth', 2);
h_cbf_boundary = plot(NaN, NaN, 'r--', 'LineWidth', 1);
h_legends = [h_boundary, h_cbf_boundary];
legend_labels = {'Arena Boundary', 'CBF Safety Boundary'};

% Plot trajectories for each robot
for i = 1:num_robots
    trajectory = squeeze(final_poses(1:2, i, :));
    
    % Plot full trajectory
    h_traj = plot(trajectory(1,:), trajectory(2,:), '-', 'Color', colors{i}, 'LineWidth', 2);
    h_legends = [h_legends, h_traj];
    legend_labels{end+1} = sprintf('Robot %d Trajectory', i);
    
    % Mark CBF active regions
    cbf_active_indices = find(final_cbf_active(i,:) > 0);
    if ~isempty(cbf_active_indices)
        cbf_points = trajectory(:, cbf_active_indices);
        plot(cbf_points(1,:), cbf_points(2,:), '.', 'Color', colors{i}*0.7, ...
            'MarkerSize', 12);
    end
    
    % Plot start and goal positions
    if i == 1
        start_pos = start1;
        goal_pos = goal1;
    else
        start_pos = start2;
        goal_pos = goal2;
    end
    
    plot(start_pos(1), start_pos(2), 'o', 'Color', colors{i}, ...
        'MarkerSize', 10, 'MarkerFaceColor', colors{i});
    plot(goal_pos(1), goal_pos(2), 'p', 'Color', colors{i}, ...
        'MarkerSize', 12, 'MarkerFaceColor', colors{i});
end

% Add CBF active points to legend if any exist
if any(final_cbf_active(:) > 0)
    h_cbf_active = plot(NaN, NaN, '.', 'Color', [0.5, 0.5, 0.5], 'MarkerSize', 12);
    h_legends = [h_legends, h_cbf_active];
    legend_labels{end+1} = 'CBF Active Points';
end

% Add start/goal to legend
h_start = plot(NaN, NaN, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_goal = plot(NaN, NaN, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
h_legends = [h_legends, h_start, h_goal];
legend_labels = [legend_labels, 'Start Position', 'Goal Position'];

xlabel('X (m)');
ylabel('Y (m)');
title('Robot Trajectories with CBF Constraints');
legend(h_legends, legend_labels, 'Location', 'best', 'FontSize', 9);
axis equal;
grid on;

%% Linear Velocity Profiles (position 2)
subplot(2,2,2);
hold on;

h_vel_legends = [];
vel_legend_labels = {};

for i = 1:num_robots
    v = squeeze(final_velocities(1, i, :));
    h_v = stairs(time_vec, v, 'Color', colors{i}, 'LineWidth', 2);
    h_vel_legends = [h_vel_legends, h_v];
    vel_legend_labels{end+1} = sprintf('Robot %d', i);
    
    % Mark CBF active regions on velocity plot
    cbf_active_indices = find(final_cbf_active(i,:) > 0);
    if ~isempty(cbf_active_indices)
        cbf_times = time_vec(cbf_active_indices);
        cbf_velocities = v(cbf_active_indices);
        plot(cbf_times, cbf_velocities, '.', 'Color', colors{i}*0.7, 'MarkerSize', 8);
    end
end

xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Linear Velocity Profiles');
legend(h_vel_legends, vel_legend_labels, 'Location', 'best');
grid on;

%% Angular Velocity Profiles (position 4)
subplot(2,2,4);
hold on;

for i = 1:num_robots
    omega = squeeze(final_velocities(2, i, :));
    stairs(time_vec, omega, 'Color', colors{i}, 'LineWidth', 2);
    
    % Mark CBF active regions on angular velocity plot
    cbf_active_indices = find(final_cbf_active(i,:) > 0);
    if ~isempty(cbf_active_indices)
        cbf_times = time_vec(cbf_active_indices);
        cbf_omegas = omega(cbf_active_indices);
        plot(cbf_times, cbf_omegas, '.', 'Color', colors{i}*0.7, 'MarkerSize', 8);
    end
end

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Profiles');
legend('Robot 1', 'Robot 2', 'Location', 'best');
grid on;

%% Display Performance Metrics
fprintf('=== CBF Trajectory Analysis ===\n');
fprintf('Total simulation time: %.2f seconds\n', time_vec(end));
fprintf('Number of time steps: %d\n', num_timesteps);
fprintf('Sample time: %.3f seconds\n', sampleTime);

fprintf('\nRobot Performance:\n');
for i = 1:num_robots
    if i == 1
        start_pos = start1;
        goal_pos = goal1;
    else
        start_pos = start2;
        goal_pos = goal2;
    end
    
    % Calculate final position
    final_pos = final_poses(1:2, i, end);
    distance_to_goal = norm(final_pos - goal_pos);
    
    % CBF activation statistics
    cbf_activations = sum(final_cbf_active(i,:) > 0);
    cbf_percentage = (cbf_activations / num_timesteps) * 100;
    
    fprintf('  Robot %d:\n', i);
    fprintf('    Start: [%.3f, %.3f]\n', start_pos(1), start_pos(2));
    fprintf('    Goal:  [%.3f, %.3f]\n', goal_pos(1), goal_pos(2));
    fprintf('    Final: [%.3f, %.3f]\n', final_pos(1), final_pos(2));
    fprintf('    Distance to goal: %.3f m\n', distance_to_goal);
    fprintf('    Straight-line distance: %.3f m\n', straight_line_distance(i));
    fprintf('    Total distance traveled: %.3f m\n', total_distance_traveled(i));
    if total_distance_traveled(i) > 0
        efficiency = straight_line_distance(i) / total_distance_traveled(i);
        fprintf('    Path efficiency: %.1f%%\n', efficiency*100);
    end
    fprintf('    CBF activations: %d/%d (%.1f%%)\n', cbf_activations, num_timesteps, cbf_percentage);
    if path_end_time(i) > 0
        fprintf('    Goal reached at: %.2f seconds\n', path_end_time(i));
    else
        fprintf('    Goal not reached within simulation time\n');
    end
    fprintf('\n');
end

% Calculate minimum distance between robots
min_distance = inf;
min_time_idx = 1;
robot_distances = zeros(num_timesteps, 1);

for k = 1:num_timesteps
    dist = norm(final_poses(1:2, 1, k) - final_poses(1:2, 2, k));
    robot_distances(k) = dist;
    if dist < min_distance
        min_distance = dist;
        min_time_idx = k;
    end
end

fprintf('Collision Avoidance:\n');
fprintf('  Minimum distance between robots: %.3f m at t=%.2f s\n', ...
    min_distance, (min_time_idx-1)*sampleTime);
fprintf('  Safety margin (required): %.3f m\n', safety_margin_robots);
if min_distance >= safety_margin_robots
    fprintf('  ✓ No safety violations detected\n');
else
    fprintf('  ⚠ Safety margin violated!\n');
end

%% Save Figure
saveas(fig, 'CBF_Trajectory_Analysis.png');
fprintf('\nFigure saved as CBF_Trajectory_Analysis.png\n');