%% Horizontal Narrow Corridor with Asymmetric CBF-QP in Robotarium
% This script simulates two robots navigating a horizontal narrow corridor.
% The Ally robot (green) is programmed to always yield to the Adversary
% (blue) using an asymmetric Control Barrier Function (CBF) formulation.

init;

%% TUNABLE PARAMETERS

% === Environment Parameters ===
passage_width = 0.4; % Width of the horizontal corridor (m)

% === Robot & Path Generation ===
N = 2; % Number of robots
ally_start_zone = [-1.5, -1.2];
ally_goal_zone = [1.2, 1.5];
adv_start_zone = [1.2, 1.5];
adv_goal_zone = [-1.5, -1.2];
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

% === Simulation & Visualization ===
simulation_time = 60;
goal_radius = 0.1;
record_video = true;

%% SETUP: ENVIRONMENT AND ROBOTS

% Initial conditions and Robotarium instance
sample_time = 0.033;
iterations = ceil(simulation_time / sample_time);
robotarium_height = 2.0;
radius = (robotarium_height - passage_width) / 2;
center_top_y = passage_width/2 + radius;
center_bottom_y = -passage_width/2 - radius;
static_obstacles = [0, center_top_y, radius; 0, center_bottom_y, radius]';

start1 = [ally_start_zone(1) + rand()*(ally_start_zone(2)-ally_start_zone(1)); y_range(1) + rand()*(y_range(2)-y_range(1))];
goal1 = [ally_goal_zone(1) + rand()*(ally_goal_zone(2)-ally_goal_zone(1)); y_range(1) + rand()*(y_range(2)-y_range(1))];
start2 = [adv_start_zone(1) + rand()*(adv_start_zone(2)-adv_start_zone(1)); y_range(1) + rand()*(y_range(2)-y_range(1))];
goal2 = [adv_goal_zone(1) + rand()*(adv_goal_zone(2)-adv_goal_zone(1)); y_range(1) + rand()*(y_range(2)-y_range(1))];
initial_conditions = [start1, start2; atan2(goal1(2)-start1(2), goal1(1)-start1(1)), atan2(goal2(2)-start2(2), goal2(1)-start2(1))];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

% Data Storage
pose_history = zeros(3, N, iterations);
velocity_history = zeros(2, N, iterations);
angular_velocity_history = zeros(N, iterations);

%% SETUP: VISUALIZATION

plot_horizontal_corridor(r.figure_handle, passage_width, radius, center_top_y, center_bottom_y);
plot(start1(1), start1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal1(1), goal1(2), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(start2(1), start2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(goal2(1), goal2(2), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

ally_traj_handle = plot(NaN, NaN, 'g-', 'LineWidth', 2);
adv_traj_handle = plot(NaN, NaN, 'b-', 'LineWidth', 2);

colors = {'g', 'b'};
theta_circle = linspace(0, 2*pi, 50);
safety_handles = gobjects(N, 1);
activation_handles = gobjects(N, 1);
for i=1:N
    safety_handles(i) = plot(NaN, NaN, '-', 'Color', colors{i}, 'LineWidth', 1.5);
    activation_handles(i) = plot(NaN, NaN, '--', 'Color', colors{i}, 'LineWidth', 1);
end

if record_video
    video_filename = 'horizontal_corridor_asymmetric_cbf.mp4';
    video_obj = VideoWriter(video_filename, 'MPEG-4');
    video_obj.FrameRate = 30; video_obj.Quality = 100;
    open(video_obj);
end

%% MAIN SIMULATION LOOP

for t = 1:iterations
    x = r.get_poses();
    pose_history(:, :, t) = x;
    dxu = zeros(2, N);

    for i = 1:N
        current_pos = x(1:2, i);
        current_heading = x(3, i);
        goal = (i==1) * goal1 + (i==2) * goal2;
        dynamic_obstacles = x(1:2, setdiff(1:N, i));

        goal_direction = goal - current_pos;
        dist_to_goal = norm(goal_direction);
        u_desired = (dist_to_goal > goal_radius) * max_linear_velocity * (goal_direction / dist_to_goal);

        % ASYMMETRIC CBF: Ally (robot 1) avoids adversary (robot 2)
        [A_cbf, b_cbf, cbf_active] = setup_asymmetric_cbf(current_pos, static_obstacles, dynamic_obstacles, safety_margin_walls, safety_margin_robots, gamma_cbf, cbf_activation_distance, i);
        u_safe = (~isempty(A_cbf)) * solve_cbf_qp(u_desired, A_cbf, b_cbf, max_linear_velocity) + isempty(A_cbf) * u_desired;

        if norm(u_safe) > 0.01
            desired_heading = atan2(u_safe(2), u_safe(1));
            heading_error = wrapToPi(desired_heading - current_heading);
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
        
        % Update Visualization Circles
        safety_circle = current_pos + safety_margin_robots * [cos(theta_circle); sin(theta_circle)];
        set(safety_handles(i), 'XData', safety_circle(1,:), 'YData', safety_circle(2,:));
        if cbf_active
            activation_circle = current_pos + cbf_activation_distance * [cos(theta_circle); sin(theta_circle)];
            set(activation_handles(i), 'XData', activation_circle(1,:), 'YData', activation_circle(2,:));
        else
            set(activation_handles(i), 'XData', NaN, 'YData', NaN);
        end
    end

    r.set_velocities(1:N, dxu);
    r.step();

    set(ally_traj_handle, 'XData', squeeze(pose_history(1, 1, 1:t)), 'YData', squeeze(pose_history(2, 1, 1:t)));
    set(adv_traj_handle, 'XData', squeeze(pose_history(1, 2, 1:t)), 'YData', squeeze(pose_history(2, 2, 1:t)));
    drawnow;

    if record_video, frame = getframe(r.figure_handle); writeVideo(video_obj, frame); end
    if norm(x(1:2, 1) - goal1) < goal_radius && norm(x(1:2, 2) - goal2) < goal_radius, disp('Goals reached!'); break; end
end

if record_video, close(video_obj); fprintf('Video saved as "%s"\n', video_filename); end
r.debug();

%% POST-SIMULATION ANALYSIS & PLOTTING
% (Plotting code is unchanged)
figure('Name', 'Simulation Results', 'Position', [100, 100, 1200, 800]);
subplot(2, 2, [1, 3]);
hold on;
plot_horizontal_corridor(gcf, passage_width, radius, center_top_y, center_bottom_y);
plot(start1(1), start1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Ally Start');
plot(goal1(1), goal1(2), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Ally Goal');
plot(start2(1), start2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Adversary Start');
plot(goal2(1), goal2(2), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Adversary Goal');
plot(squeeze(pose_history(1, 1, 1:t)), squeeze(pose_history(2, 1, 1:t)), 'g-', 'LineWidth', 2, 'DisplayName', 'Ally Trajectory');
plot(squeeze(pose_history(1, 2, 1:t)), squeeze(pose_history(2, 2, 1:t)), 'b-', 'LineWidth', 2, 'DisplayName', 'Adversary Trajectory');
title('Robot Trajectories'); xlabel('X (m)'); ylabel('Y (m)');
legend('show'); grid on; axis equal;
time_vec = (1:t) * sample_time;
subplot(2, 2, 2);
hold on;
plot(time_vec, squeeze(velocity_history(1, 1, 1:t)), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ally Velocity');
plot(time_vec, squeeze(velocity_history(1, 2, 1:t)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Adversary Velocity');
title('Linear Velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('show'); grid on;
subplot(2, 2, 4);
hold on;
plot(time_vec, angular_velocity_history(1, 1:t), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ally Angular Velocity');
plot(time_vec, angular_velocity_history(2, 1:t), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Adversary Angular Velocity');
title('Angular Velocity'); xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('show'); grid on;

%% HELPER FUNCTIONS

function [A, b, active] = setup_asymmetric_cbf(pos, static_obs, dynamic_obs, margin_wall, margin_robot, gamma, activation_dist, robot_index)
    A = []; b = []; active = false;
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
    H = 2 * eye(2); f = -2 * u_desired;
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