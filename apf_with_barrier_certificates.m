%% APF with Barrier Certificates for Safe Navigation
% This script combines Artificial Potential Fields with barrier certificates
% to ensure collision-free navigation while maintaining APF behavior

%% Initialize Robotarium
N = 3; % Three robots for more complex interactions
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Create barrier certificate for safety
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

% Parameters
sampleTime = r.time_step;
simulationTime = 30;
iterations = simulationTime / sampleTime;

% APF parameters
detection_radius = 2.0; % Smaller for Robotarium scale
safe_radius = 0.3;
attraction_factor = 2;
repulsion_factor = 0.8;
goal_radius = 0.1;

% Generate initial positions using Robotarium's built-in function
initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
r.set_poses(initial_positions);

% Generate random goals
goals = generate_initial_conditions(N, ...
    'Width', r.boundaries(2)-r.boundaries(1)-0.3, ...
    'Height', r.boundaries(4)-r.boundaries(3)-0.3, ...
    'Spacing', 0.5);

% Store trajectories
trajectories = cell(N, 1);
for i = 1:N
    trajectories{i} = zeros(2, iterations);
end

% Colors for robots
colors = {'r', 'g', 'b'};

%% Main simulation loop
for t = 1:iterations
    % Get current poses
    x = r.get_poses();
    
    % Store positions
    for i = 1:N
        trajectories{i}(:, t) = x(1:2, i);
    end
    
    % Calculate APF-based velocities
    dxu = zeros(2, N);
    
    for i = 1:N
        % Current state
        current_pos = x(1:2, i);
        current_theta = x(3, i);
        goal = goals(1:2, i);
        
        % Get positions of other robots as obstacles
        obstacles = x(1:2, setdiff(1:N, i));
        
        % Calculate APF forces
        [F_att, F_rep] = calculate_apf_forces_enhanced(current_pos, goal, ...
            obstacles, detection_radius, attraction_factor, repulsion_factor);
        
        % Combined force
        F_total = F_att + F_rep;
        
        % Convert to desired velocity (SI)
        if norm(F_total) > 0
            desired_vel = F_total / norm(F_total);
        else
            desired_vel = [0; 0];
        end
        
        % Scale velocity
        max_speed = 0.15; % Robotarium speed limit
        desired_vel = max_speed * desired_vel;
        
        % Convert SI to unicycle dynamics
        v = norm(desired_vel);
        desired_theta = atan2(desired_vel(2), desired_vel(1));
        
        % Angular velocity control
        theta_error = desired_theta - current_theta;
        theta_error = atan2(sin(theta_error), cos(theta_error));
        w = 5 * theta_error;
        
        % Limit angular velocity
        w = max(-2*pi, min(2*pi, w));
        
        dxu(:, i) = [v; w];
    end
    
    % Apply barrier certificates for safety
    dxu = uni_barrier_cert(dxu, x);
    
    % Set velocities
    r.set_velocities(1:N, dxu);
    r.step();
    
    % Check if all robots reached their goals
    at_goal = true;
    x_current = r.get_poses();
    for i = 1:N
        if norm(x_current(1:2, i) - goals(1:2, i)) > goal_radius
            at_goal = false;
            break;
        end
    end
    
    if at_goal
        disp('All robots reached their goals!');
        break;
    end
end

%% Plotting
figure('Name', 'APF Navigation with Barrier Certificates');
hold on;

% Plot boundaries
rectangle('Position', [r.boundaries(1), r.boundaries(3), ...
    r.boundaries(2)-r.boundaries(1), r.boundaries(4)-r.boundaries(3)], ...
    'EdgeColor', 'k', 'LineWidth', 2);

% Plot trajectories and goals
for i = 1:N
    % Trajectory
    traj = trajectories{i}(:, 1:t);
    plot(traj(1,:), traj(2,:), [colors{i} '-'], 'LineWidth', 2);
    
    % Start position
    plot(initial_positions(1,i), initial_positions(2,i), ...
        [colors{i} 'o'], 'MarkerSize', 10, 'MarkerFaceColor', colors{i});
    
    % Goal position
    plot(goals(1,i), goals(2,i), [colors{i} 's'], ...
        'MarkerSize', 12, 'MarkerFaceColor', colors{i});
    
    % Current position
    plot(x_current(1,i), x_current(2,i), [colors{i} '^'], ...
        'MarkerSize', 8, 'MarkerFaceColor', colors{i});
end

% Legend
legend_entries = {};
for i = 1:N
    legend_entries = [legend_entries, ...
        {sprintf('Robot %d trajectory', i), ...
         sprintf('Robot %d start', i), ...
         sprintf('Robot %d goal', i), ...
         sprintf('Robot %d current', i)}];
end
legend(legend_entries, 'Location', 'bestoutside');

axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('APF Navigation with Barrier Certificates for Safety');

% Always call debug
r.debug();

%% Enhanced APF force calculation
function [F_att, F_rep] = calculate_apf_forces_enhanced(current_pos, goal, ...
    obstacles, Q_star, zeta, eta)
    
    % Attractive force using gradient of parabolic potential
    d_goal = norm(goal - current_pos);
    F_att = -zeta * (current_pos - goal);
    
    % Limit attractive force magnitude
    max_att_force = 1.0;
    if norm(F_att) > max_att_force
        F_att = max_att_force * F_att / norm(F_att);
    end
    
    % Repulsive force
    F_rep = zeros(2, 1);
    
    for i = 1:size(obstacles, 2)
        obstacle_pos = obstacles(:, i);
        D = norm(current_pos - obstacle_pos);
        
        if D <= Q_star && D > 0
            % Gradient of repulsive potential
            obstacle_vec = current_pos - obstacle_pos;
            
            % Standard APF repulsive force
            F_rep_i = eta * (1/Q_star - 1/D) * (1/D^2) * (obstacle_vec / D);
            
            % Add goal influence to avoid local minima
            goal_influence = 0.2 * d_goal^2;
            F_rep_i = F_rep_i * (1 + goal_influence);
            
            F_rep = F_rep + F_rep_i;
        end
    end
    
    % Limit repulsive force magnitude
    max_rep_force = 2.0;
    if norm(F_rep) > max_rep_force
        F_rep = max_rep_force * F_rep / norm(F_rep);
    end
end