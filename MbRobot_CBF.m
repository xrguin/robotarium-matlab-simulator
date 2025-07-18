%=========================================================================%
% robot = MbRobot(waypoints, color, SafeZone, DetectionZone)
%
%=========================================================================%


%% Robot with xxx Model

classdef MbRobot_CBF < handle
    properties
        MaxLinearVelocity = 1;
        MinLinearVelocity = 0;
        DefaultLinearVelocity = 1;
        Velocity;
        RotationAngle
        CurrentPose
        CurrentCoord
        CurrentState
        HeadAngle = 0;
        heading_history
        AngularVelocity = 0;
        Start
        Goal
        Waypoints
        SafeRangeColor
        DetectionRangeColor
        GoalRadius = 1;
        SafeRadius
        DetectionRadius
        Obstacle
        StaticObstacle; % An array that contains the central point, and the radius of the obstacle
                        % [x1, x2; y1, y2; r1, r2]
        DynamicObstacle;
        attraction_force
        repulsion_force
        combined_force
        name
        LinearVelocity
        detection_indicator = 0;
        
    end

    methods
%% This is for dynamic rotational model
         
      
        

        % Bug algorithm
        function bug_rotate(obj, sampleTime)
            previousHeadAngle = obj.HeadAngle;
            target_point = obj.Goal;
            obstacle_distances = zeros(1, size(obj.Obstacle, 2));
            collision = false;
            
            
            for i = 1:size(obj.Obstacle, 2)
                obstacle_distances(i) = calcDist(obj.Obstacle(:,i), obj.CurrentCoord);
            end
            
            [min_distance, nearest_obstacle_index] = min(obstacle_distances);
            % if collision
            %     obstacle_vector = obj.Obstacle(:,i) - obj.CurrentCoord;
            %     tangent_vector = [-obstacle_vector(2), obstacle_vector(1)];
            %     tangent_vector = tangent_vector / norm(tangent_vector);
            % 
            %     obj.HeadAngle = atan2(tangent_vector(2), tangent_vector(1));
            % else
            %     obj.HeadAngle = atan2(target_point(2) - obj.CurrentCoord(2),...
            %         target_point(1) - obj.CurrentCoord(1));
            % end
            if min_distance < obj.SafeRadius
                collision = true;
                obstacle_vector = obj.Obstacle(:,nearest_obstacle_index) - obj.CurrentCoord;
                tangent_vector = [-obstacle_vector(2), obstacle_vector(1)];
                tangent_vector = tangent_vector / norm(tangent_vector);
                obj.HeadAngle = atan2(tangent_vector(2), tangent_vector(1));
            else
                collision = false;
                obj.HeadAngle = atan2(target_point(2) - obj.CurrentCoord(2),...
                    target_point(1) - obj.CurrentCoord(1));
            end

            obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];
        end

        function wall_follow_bug(obj, sampleTime)
            previousHeadAngle = obj.HeadAngle;
            target_point = obj.Goal;
            obstacle_distances = zeros(1, size(obj.StaticObstacle, 2));
            collision = false;
            
            
            for i = 1:size(obj.StaticObstacle, 2)
                obstacle_distances(i) = calcDist(obj.StaticObstacle(:,i), obj.CurrentCoord);
            end
            
            [min_distance, nearest_obstacle_index] = min(obstacle_distances);
            nearest_obstacle = obj.StaticObstacle(:,nearest_obstacle_index);
            obstacle_vector = nearest_obstacle - obj.CurrentCoord;

            if min_distance < obj.SafeRadius

                on_left_side = (obj.CurrentCoord(1) < 7.5);
                goal_is_above = (obj.Goal(2) > obj.CurrentCoord(2));
                if on_left_side
                    if goal_is_above
                        tangent_vector = [obstacle_vector(2), -obstacle_vector(1)];
                    else
                        tangent_vector = [-obstacle_vector(2), obstacle_vector(1)];
                    end
                else
                    if goal_is_above
                        % Follow left side of right circle (counter-clockwise)
                        tangent_vector = [-obstacle_vector(2), obstacle_vector(1)];
                    else
                        % Follow right side of right circle (clockwise)
                        tangent_vector = [obstacle_vector(2), -obstacle_vector(1)];
                    end
                end

                % Normalize tangent vector
                tangent_vector = tangent_vector / norm(tangent_vector);
                obj.HeadAngle = atan2(tangent_vector(2), tangent_vector(1));
                
            else
                obj.HeadAngle = atan2(target_point(2) - obj.CurrentCoord(2),...
                    target_point(1) - obj.CurrentCoord(1));
            end

            obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];
        end
        

        

        % Calculate weighted averaged repulsive forces from all the
        % obstacles in the detection range.
        function artificial_potential_field(obj, sampleTime, ...
                attraction_factor, repulsive_factor)
            
            previousHeadAngle = obj.HeadAngle;
            target_point = obj.Goal;
            
            % Calculate attraction to goal
            goal_vector = target_point - obj.CurrentCoord;
            obj.attraction_force = goal_vector / norm(goal_vector);


            if ~isempty(obj.Obstacle)
                diff_vectors = obj.Obstacle - obj.CurrentCoord;  % Implicit expansion
                all_distances = sqrt(sum(diff_vectors.^2, 1));
                
                % Find obstacles within detection range
                in_range_idx = find(all_distances < obj.DetectionRadius);

                if isempty(in_range_idx)
                    obj.repulsion_force = [0,0];
                else
                    distances_in_range = all_distances(in_range_idx);
                    obstacles_in_range = obj.Obstacle(:, in_range_idx);
                    
                    repulsion_vectors = obj.CurrentCoord - obstacles_in_range;  % Vector from obstacle to robot
                    
                    repulsion_directions = repulsion_vectors ./ distances_in_range;  % Implicit expansion

                    proximity_factors = (obj.DetectionRadius - distances_in_range) ./ obj.DetectionRadius;
                    repulsion_magnitudes = proximity_factors.^3;  % Cubic scaling for stronger effect
                    
                    
                    danger_close = distances_in_range < (obj.SafeRadius * 1.2);
                    if any(danger_close)
                        repulsion_magnitudes(danger_close) = repulsion_magnitudes(danger_close) * 4;
                    end

                    % This gives closer obstacles more influence on the final direction
                    weights = 1 ./ (distances_in_range.^2);
                    weights = weights / sum(weights);  % Normalize weights
                    
                    % Calculate individual repulsive forces
                    repulsive_forces = repulsion_directions .* repulsion_magnitudes;
                    
                    % Apply weighted average for final repulsion force
                    obj.repulsion_force = repulsive_forces * weights';
                    
                    % % Normalize the repulsion force
                    % if norm(obj.repulsion_force) > 0
                    %     obj.repulsion_force = obj.repulsion_force / norm(obj.repulsion_force);
                    % end
                end
            else
                obj.repulsion_force = [0; 0];
            end

            % Combine forces with proper weighting
            obj.combined_force = attraction_factor * obj.attraction_force + ...
                          repulsive_factor * obj.repulsion_force;
            obj.combined_force = obj.combined_force / norm(obj.combined_force);
            


            
            % Set new heading angle
            obj.HeadAngle = atan2(obj.combined_force(2), obj.combined_force(1));
            
            % Move robot
            obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];

        end


        % obstacles in the detection range.
        function APF_Static(obj, sampleTime, ...
                attraction_factor, repulsive_factor)
            

            previousHeadAngle = obj.HeadAngle;
            target_point = obj.Goal;
            % Calculate attraction to goal
            goal_vector = target_point - obj.CurrentCoord;
            obj.attraction_force = goal_vector / norm(goal_vector);
            ObstacleCoords = obj.StaticObstacle(1:2, :);
            ObstacleRadius = obj.StaticObstacle(3,:);
            if ~isempty(obj.StaticObstacle)
                % Get vectors from robot to obstacles
                diff_vectors = ObstacleCoords - obj.CurrentCoord; % Implicit expansion
                all_distances = sqrt(sum(diff_vectors.^2, 1));
                
                % No need to check for in-range since we have map info
                % Calculate repulsion from all obstacles, adjusted by their size
                
                % Assuming obj.ObstacleRadii contains the radius of each obstacle
                % If not available, you'll need to add this property
                
                % Calculate repulsion directions
                repulsion_vectors = obj.CurrentCoord - ObstacleCoords; % Vector from obstacle to robot
                repulsion_directions = repulsion_vectors ./ all_distances; % Normalize directions
                
                % Adjust distances to consider obstacle radius (distance to edge, not center)
                effective_distances = all_distances - ObstacleRadius;
                effective_distances = max(effective_distances, 0.1); % Prevent division by zero
                
                % Scale repulsion by inverse of effective distance (closer = stronger)
                base_repulsion = 1 ./ (effective_distances.^2);
                
                % Scale by obstacle size - larger obstacles get stronger repulsion
                % Normalize radii by maximum possible radius to get a scaling factor
                max_possible_radius = obj.DetectionRadius; % Or use another reference value
                radius_scaling = ObstacleRadius / max_possible_radius;
                
                % Combine both factors for final repulsion magnitude
                repulsion_magnitudes = base_repulsion .* (1 + radius_scaling);
                
             
                % Calculate individual repulsive forces
                repulsive_forces = repulsion_directions .* repulsion_magnitudes;
                
                % Weight by inverse square of effective distance
                weights = base_repulsion;
                weights = weights / sum(weights); % Normalize weights
                
                % Apply weighted average for final repulsion force
                obj.repulsion_force = repulsive_forces * weights';
            else
                obj.repulsion_force = [0; 0];
            end
            
            % Combine forces with proper weighting
            obj.combined_force = attraction_factor * obj.attraction_force + ...
                repulsive_factor * obj.repulsion_force;
            
            % Normalize the combined force
            if norm(obj.combined_force) > 0
                obj.combined_force = obj.combined_force / norm(obj.combined_force);
            end
            
            % Set new heading angle
            obj.HeadAngle = atan2(obj.combined_force(2), obj.combined_force(1));

            % Keep history of recent headings (assume obj.heading_history exists)
            target_heading = atan2(obj.combined_force(2), obj.combined_force(1));
            
            % If heading_history doesn't exist, create it
            if ~isfield(obj, 'heading_history') || isempty(obj.heading_history)
                obj.heading_history = repmat(target_heading, 5, 1);
            end
            
            % Update heading history
            obj.heading_history = [target_heading; obj.heading_history(1:end-1)];
            
            % Apply weighted smoothing
            weights = [0.5, 0.25, 0.15, 0.07, 0.03]'; % Higher weight for recent headings
            % weights = ones(5,1)/5;
            smoothed_heading = mod(sum(angdiff(zeros(size(obj.heading_history)), obj.heading_history) .* weights), 2*pi);
            
            % Set smoothed heading angle
            obj.HeadAngle = smoothed_heading;
            
            % Move robot
            obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                              obj.HeadAngle;
                              obj.AngularVelocity];
            obj.Velocity = obj.DefaultLinearVelocity;

        end


        function APF_Dynamic(obj, sampleTime, attraction_factor, repulsive_factor, dynamic_obstacles)
            % Store previous heading for smoothing and angular velocity calculation
            previousHeadAngle = obj.HeadAngle;
            target_point = obj.Goal;
            
            % Calculate attraction to goal
            goal_vector = target_point - obj.CurrentCoord;
            obj.attraction_force = goal_vector / norm(goal_vector);
            
            % Process static obstacles
            ObstacleCoords = obj.StaticObstacle(1:2, :);
            ObstacleRadius = obj.StaticObstacle(3,:);

            diff_vectors = ObstacleCoords - obj.CurrentCoord;
            all_distances = sqrt(sum(diff_vectors.^2, 1));
            repulsion_vectors = obj.CurrentCoord - ObstacleCoords;
            repulsion_directions = repulsion_vectors ./ all_distances;
            
            % Initialize repulsion force
            obj.repulsion_force = [0; 0];
            
            % Calculate static obstacle repulsion if they exist
            if ~isempty(obj.StaticObstacle)
                % Standard static obstacle handling (same as in APF_Static)
                
                
                
                effective_distances = all_distances - ObstacleRadius;
                effective_distances = max(effective_distances, 0.1); % Prevent division by zero
                
                base_repulsion = 1 ./ (effective_distances.^2);
                max_possible_radius = obj.DetectionRadius;
                radius_scaling = ObstacleRadius / max_possible_radius;
                
                repulsion_magnitudes = base_repulsion .* (1 + radius_scaling);
                repulsive_forces = repulsion_directions .* repulsion_magnitudes;
                
                weights = base_repulsion;
                weights = weights / sum(weights);
                
                static_repulsion = repulsive_forces * weights';
                obj.repulsion_force = static_repulsion;
            end
            
            % Process dynamic obstacles (other agents)
            dynamic_repulsion = [0; 0];
            speed_factor = 1.0; % Default full speed
            
            if ~isempty(dynamic_obstacles)
                % Calculate repulsion from dynamic obstacles
                dyn_diff_vectors = dynamic_obstacles - obj.CurrentCoord;
                dyn_distances = sqrt(sum(dyn_diff_vectors.^2, 1));

                
                % Check if any dynamic obstacle is within detection range
                in_range_idx = find(dyn_distances < obj.DetectionRadius);
                proximity_factor = 1;
                if ~isempty(in_range_idx)
                    % Process obstacles in range
                    for i = in_range_idx
                        obstacle_coord = dynamic_obstacles(:, i);
                        distance = dyn_distances(i);
                        
                        % Agent size (can be parameterized)
                        agent_radius = obj.SafeRadius;
                        
                        % Calculate effective distance (edge-to-edge)
                        effective_distance = distance - agent_radius * 2;
                        effective_distance = max(effective_distance, 0.1);
                        
                        % Calculate repulsion vector and strength
                        repulsion_vector = obj.CurrentCoord - obstacle_coord;
                        repulsion_direction = repulsion_vector / norm(repulsion_vector);
                        
                        % Scale factor to ensure repulsion can exceed 1 at close distances
                        scale_factor = 2.0; 
                        
                        % Base repulsion that increases rapidly as distance decreases
                        repulsion_strength = scale_factor * (obj.DetectionRadius / effective_distance)^2;
                        
                        % Optional: Add a minimum distance threshold for extremely strong repulsion
                        if effective_distance < 1.0
                            % Add an exponential component for very close distances
                            repulsion_strength = repulsion_strength + 2.0 * exp(2.0 * (1.0 - effective_distance));
                        end
                        
                        % Add to dynamic repulsion
                        dynamic_repulsion = repulsion_direction * repulsion_strength;
                        
                        % Determine speed adjustment based on proximity
                        % The closer the other agent, the slower we move
                    
                        % Calculate speed factor (ranges from 0.2 to 1.0)
                        proximity_factor = distance / obj.DetectionRadius;
                        current_speed_factor = proximity_factor;
                        
                        % Take the minimum speed (most cautious approach)
                        speed_factor = min(speed_factor, current_speed_factor);
                        
                    end
                    
                end
            end
            
            % Add dynamic repulsion to static repulsion
            obj.repulsion_force = obj.repulsion_force + dynamic_repulsion;
            obj.repulsion_force = obj.repulsion_force / norm(obj.repulsion_force);
      
            
            obj.combined_force = attraction_factor * proximity_factor * obj.attraction_force + ...
                          repulsive_factor * obj.repulsion_force;
            
            % Normalize the combined force
            if norm(obj.combined_force) > 0
                obj.combined_force = obj.combined_force / norm(obj.combined_force);
            end
            
            % Set new heading angle
            target_heading = atan2(obj.combined_force(2), obj.combined_force(1));
            
            % Apply heading smoothing
            if ~isfield(obj, 'heading_history') || isempty(obj.heading_history)
                obj.heading_history = repmat(target_heading, 5, 1);
            end
            
            obj.heading_history = [target_heading; obj.heading_history(1:end-1)];
            
            weights = [0.5, 0.25, 0.15, 0.07, 0.03]';
            smoothed_heading = mod(sum(angdiff(zeros(size(obj.heading_history)), obj.heading_history) .* weights), 2*pi);
            obj.HeadAngle = smoothed_heading;
            
            % Move robot with adjusted speed
            obj.Velocity = obj.DefaultLinearVelocity * speed_factor;

            next_predicted_coord = obj.CurrentCoord + obj.Velocity * [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            predict_diff_vectors = ObstacleCoords - next_predicted_coord;
            predict_all_distances = sqrt(sum(predict_diff_vectors.^2, 1));

            if any(predict_all_distances < ObstacleRadius + obj.SafeRadius)
                [~,closest_idx] = min(all_distances);
                % Get repulsion direction from the closest obstacle
                repulsion_dir = repulsion_directions(:,closest_idx);
                
                % Calculate both possible tangential directions
                tangent_ccw = [-repulsion_dir(2); repulsion_dir(1)];  % Counter-clockwise
                tangent_cw = [repulsion_dir(2); -repulsion_dir(1)];   % Clockwise
                
                % Calculate goal direction
                goal_dir = (obj.Goal - obj.CurrentCoord);
                if norm(goal_dir) > 0
                    goal_dir = goal_dir / norm(goal_dir);
                end
                % Calculate goal direction
                goal_dir = (obj.Goal - obj.CurrentCoord);
                if norm(goal_dir) > 0
                    goal_dir = goal_dir / norm(goal_dir);
                end
                
                % Determine which tangential direction is closer to the goal direction
                dot_product_ccw = dot(tangent_ccw, goal_dir);
                dot_product_cw = dot(tangent_cw, goal_dir);
                
                dyn_diff_vectors = dynamic_obstacles - obj.CurrentCoord;
                dyn_distances = sqrt(sum(dyn_diff_vectors.^2, 1));
                if dyn_distances > 2*obj.SafeRadius
                    if dot_product_ccw > dot_product_cw 
                        tangent_vector = tangent_ccw;  % Counter-clockwise is closer to goal
                    else
                        tangent_vector = tangent_cw;   % Clockwise is closer to goal
                    end
                else % Go furthe from the goal because avoiding is more important
                    if dot_product_ccw <= dot_product_cw 
                        tangent_vector = tangent_ccw;  
                    else
                        tangent_vector = tangent_cw;  
                    end
                end
                proximity_factor = min(dyn_distances) / obj.DetectionRadius;
                current_speed_factor = proximity_factor;
                obj.HeadAngle = atan2(tangent_vector(2), tangent_vector(1));
                obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
                obj.Velocity = obj.DefaultLinearVelocity;

            else
                obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
                obj.Velocity = obj.DefaultLinearVelocity;

            end
            
            
            % obj.CurrentCoord = obj.CurrentCoord + velocity * [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord; obj.HeadAngle; obj.AngularVelocity];
        end









        % Update the position when the robot considers estimated position
        % of the moving obstacle: Predictive maneuvering zone (PMZ)
        function apf(obj, sampleTime, zeta, eta)
            obstacle_distances = zeros(1, size(obj.Obstacle, 2));
            previousHeadAngle = obj.HeadAngle;
            q = obj.CurrentCoord;
            theta = obj.Goal';
            
            Q_star = obj.DetectionRadius;

            F_att = [0;0];
            F_rep = [0;0];

            d_goal = calcDist(theta,q);
            U_att = 0.5 * zeta * d_goal^2;
            F_att = zeta * (q - theta);
            obj.attraction_force = -F_att;
            % F_att = F_att/norm(F_att);

            
            % Calculate distances to all obstacles (robot2 and robot3)
            for i = 1:size(obj.Obstacle, 2)
                obstacle_distances(i) = calcDist(obj.Obstacle(:,i), obj.CurrentCoord);
            end
            
            % Find the nearest obstacle
            [D, nearest_obstacle_index] = min(obstacle_distances);
            % d is the distance between ego robot and the nearest obstacle

            if D <= Q_star
                obstacle_vector = q - obj.Obstacle(:,nearest_obstacle_index);
                U_rep = 0.5 * eta * (1/D - 1/Q_star)^2;
                % F_rep = eta * (1/Q_star - 1/D) * (1/D^2) * (obstacle_vector / D);
                F_rep = eta * (1/Q_star - 1/D) * (obstacle_vector / D);
                obj.repulsion_force = -F_rep;
                % F_rep = F_rep / norm(F_rep);
            end

            F = -F_att - F_rep;

            F = F/norm(F);
            obj.combined_force = F;

            % Set new heading angle
            obj.HeadAngle = atan2(F(2), F(1));
        
            % Move robot
            obj.CurrentCoord = obj.CurrentCoord + obj.DefaultLinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                               obj.HeadAngle;
                               obj.AngularVelocity];


        end

        function MPC_Controller(obj, ...
                                sampleTime, ...
                                horizonSteps, ...
                                obstacleWeight, ...
                                goalWeight, ...
                                controlWeight, ...
                                dynamic_obstacles)


            % Model Predictive Control for robot navigation with obstacle avoidance
            %
            % Inputs:
            %   sampleTime - Time step for simulation
            %   horizonSteps - Number of steps in the prediction horizon
            %   obstacleWeight - Weight for obstacle avoidance in cost function
            %   goalWeight - Weight for goal attraction in cost function
            %   controlWeight - Weight for control effort in cost function
            %   dynamic_obstacles - Array of dynamic obstacles [x1,x2,...; y1,y2,...]
            %                      If dynamic obstacles have velocity information, it should be
            %                      [x1,x2,...; y1,y2,...; vx1,vx2,...; vy1,vy2,...]

            % Store previous heading for angular velocity calculation
            previousHeadAngle = obj.HeadAngle;
            previousVelocity = obj.Velocity;
            if isempty(previousVelocity) || isnan(previousVelocity)
                previousVelocity = obj.DefaultLinearVelocity;
            end

            % Current state
            current_x = obj.CurrentCoord(1);
            current_y = obj.CurrentCoord(2);
            current_theta = obj.HeadAngle;
            
            % Goal state
            goal_x = obj.Goal(1);
            goal_y = obj.Goal(2);


            % Define the control space (possible combinations of heading changes and velocities)
            num_theta_controls = 9; % Number of discrete heading control options
            num_vel_controls = 5;   % Number of discrete velocity control options
            
            % Range of heading angle changes
            delta_theta_range = linspace(-pi/3, pi/3, num_theta_controls);
            
            % Range of velocity changes (as percentage of default velocity)
            vel_factor_range = linspace(0.2, 1.0, num_vel_controls);
            
            % Initialize cost matrix for all control combinations
            costs = zeros(num_theta_controls, num_vel_controls);

            % Store static obstacles information
            static_obstacles = [];
            static_obstacle_radii = [];
            if ~isempty(obj.StaticObstacle)
                static_obstacles = obj.StaticObstacle(1:2, :);
                static_obstacle_radii = obj.StaticObstacle(3, :);
            end
            
            % Store previous dynamic obstacles for velocity estimation
            persistent previous_dynamic_obstacles;
            if isempty(previous_dynamic_obstacles)
                previous_dynamic_obstacles = dynamic_obstacles;
            end
            
            % Calculate velocity estimates for dynamic obstacles
            dynamic_obstacle_velocities = [];
            if ~isempty(dynamic_obstacles) && ~isempty(previous_dynamic_obstacles) && ...
               size(dynamic_obstacles, 2) == size(previous_dynamic_obstacles, 2)
                % Calculate velocity for each obstacle
                dynamic_obstacle_velocities = (dynamic_obstacles - previous_dynamic_obstacles) / sampleTime;
            else
                % If we don't have previous positions, set velocities to zero
                if ~isempty(dynamic_obstacles)
                    dynamic_obstacle_velocities = zeros(2, size(dynamic_obstacles, 2));
                end
            end
    
            % Update previous_dynamic_obstacles for next iteration
            previous_dynamic_obstacles = dynamic_obstacles;


            % Evaluate all control combinations
            for i_theta = 1:num_theta_controls
                for i_vel = 1:num_vel_controls
                    % Extract control inputs for this combination
                    delta_theta = delta_theta_range(i_theta);
                    vel_factor = vel_factor_range(i_vel);
                    current_velocity = obj.DefaultLinearVelocity * vel_factor;
                    
                    % Initialize trajectory cost
                    total_cost = 0;
                    
                    % Initialize predicted state
                    pred_x = current_x;
                    pred_y = current_y;
                    pred_theta = current_theta + delta_theta; % Apply heading change immediately
                    
                    % Simulate trajectory over prediction horizon
                    for step = 1:horizonSteps
                        % Update predicted state using kinematic model
                        pred_x = pred_x + current_velocity * cos(pred_theta) * sampleTime;
                        pred_y = pred_y + current_velocity * sin(pred_theta) * sampleTime;
                        
                        % Calculate goal cost (distance to goal)
                        dist_to_goal = sqrt((pred_x - goal_x)^2 + (pred_y - goal_y)^2);
                        goal_cost = dist_to_goal^2;
                        
                        % Calculate obstacle cost
                        obstacle_cost = 0;
                        
                        % Static obstacles cost
                        if ~isempty(static_obstacles)
                            for j = 1:size(static_obstacles, 2)
                                obs_x = static_obstacles(1, j);
                                obs_y = static_obstacles(2, j);
                                obs_radius = static_obstacle_radii(j);
                                
                                % Distance to obstacle (from edge)
                                dist_to_obs = sqrt((pred_x - obs_x)^2 + (pred_y - obs_y)^2) - obs_radius - obj.SafeRadius;
                                dist_to_obs = max(dist_to_obs, 0.1); % Prevent division by zero
                                
                                % Add inverse square distance to cost
                                obstacle_cost = obstacle_cost + 1 / (dist_to_obs^2);
                            end
                        end
                        
                        % Dynamic obstacles cost
                        if ~isempty(dynamic_obstacles)
                            for j = 1:size(dynamic_obstacles, 2)
                                % Predict obstacle position at this time step
                                obs_x = dynamic_obstacles(1, j) + dynamic_obstacle_velocities(1, j) * step * sampleTime;
                                obs_y = dynamic_obstacles(2, j) + dynamic_obstacle_velocities(2, j) * step * sampleTime;
                                
                                % Distance to obstacle (from edge to edge, assuming circular robot and obstacle)
                                dist_to_obs = sqrt((pred_x - obs_x)^2 + (pred_y - obs_y)^2) - obj.SafeRadius - obj.SafeRadius;
                                dist_to_obs = max(dist_to_obs, 0.1); % Prevent division by zero
                                
                                % Add inverse square distance to cost
                                obstacle_cost = obstacle_cost + 2 / (dist_to_obs^2); % Higher weight for dynamic obstacles
                            end
                        end
                        
                        % Calculate control cost (penalize changes from current controls)
                        control_cost = delta_theta^2 + (current_velocity - previousVelocity)^2;
                        
                        % Time-weighted cost (penalize earlier steps more)
                        time_weight = 1 / (step);
                        
                        % Add weighted costs to total
                        total_cost = total_cost + time_weight * (...
                            goalWeight * goal_cost +... 
                            obstacleWeight * obstacle_cost +... 
                            controlWeight * control_cost...
                        );
                    end
                    
                    % Store the total cost for this control combination
                    costs(i_theta, i_vel) = total_cost;
                end
            end

            % Find the control combination with minimum cost
            [~, min_idx] = min(costs(:));
            [min_i_theta, min_i_vel] = ind2sub(size(costs), min_idx);
            
            % Extract optimal control inputs
            optimal_delta_theta = delta_theta_range(min_i_theta);
            optimal_vel_factor = vel_factor_range(min_i_vel);
            
            % Apply optimal control inputs
            obj.HeadAngle = obj.HeadAngle + optimal_delta_theta;
            obj.Velocity = obj.DefaultLinearVelocity * optimal_vel_factor;
            
            % Update robot state
            obj.CurrentCoord = obj.CurrentCoord + obj.Velocity * [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord; obj.HeadAngle; obj.AngularVelocity];

            
        end

 
        function isAtGoal = atGoal(obj)
            % Check if the robot has reached its goal
            distanceToGoal = calcDist(obj.CurrentPose(1:2), obj.Goal(:));
            isAtGoal = distanceToGoal < obj.GoalRadius;
        end

        function plotRobot(obj, frameSize)
            % Plot the robot's current position
            plotTrVec = [obj.CurrentPose(1:2); 0];
            plotRot = obj.custom_axang2quat([0, 0, 1, obj.CurrentPose(3)]);
            plotTransforms(plotTrVec', plotRot, "MeshFilePath", ...
                "groundvehicle.stl", "Parent", gca, "View", "2D", ...
                "FrameSize", frameSize);
            hold on
            centerX = obj.CurrentPose(1);
            centerY = obj.CurrentPose(2);
            r = obj.SafeRadius;          % rectangle('Position',[centerX-rd, ...
            %     centerY-rd, 2*rd, 2*rd],...
            %     'Curvature',[1,1], 'EdgeColor',[0.5 0.5 0.5], ...
            %     'FaceColor',obj.LightColor);
            rectangle('Position',[centerX-r, ...
                centerY-r, 2*r, 2*r],...
                'Curvature',[1,1], 'EdgeColor','none', ...
                'FaceColor',obj.SafeRangeColor, ...
                'FaceAlpha',obj.SafeRangeColor(4));
            
            r2 = obj.DetectionRadius;
            rectangle('Position',[centerX-r2, ...
                centerY-r2, 2*r2, 2*r2],...
                'Curvature',[1,1],...
                'EdgeColor','none',...
                'FaceColor',obj.DetectionRangeColor, ...
                'FaceAlpha',obj.DetectionRangeColor(4));
            % Visualize forces
            scale = 1; % Adjust this value to change the length of the force arrows
            
            % % Attraction force (green)
            % quiver(centerX, centerY, ...
            %     obj.attraction_force(1)*scale, obj.attraction_force(2)*scale, ...
            %     0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Attraction Force');
            % 
            % % Repulsion force (red)
            % quiver(centerX, centerY, ...
            %     obj.repulsion_force(1)*scale, obj.repulsion_force(2)*scale, ...
            %     0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Repulsion Force');
            % 
            % % Combined force (blue)
            % quiver(centerX, centerY, ...
            %     obj.combined_force(1)*scale, obj.combined_force(2)*scale, ...
            %     0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Combined Force');

            hold off



        end

        function q = custom_axang2quat(obj,axang)
            % Extract axis and angle from input
            axis = axang(1:3);
            angle = axang(end);
            
            % Normalize the axis vector
            axis = axis / norm(axis);
            
            % Compute half angle
            half_angle = angle / 2;
            
            % Compute sine and cosine of half angle
            s = sin(half_angle);
            c = cos(half_angle);
            
            % Compute quaternion
            q = [c, axis(1)*s, axis(2)*s, axis(3)*s];
        end

        function [in_range_idx, distances] = find_obstacles_in_range(obj,obstacles)
            % Calculate distances to all static obstacles within detection range
            %
            % Returns:
            %   in_range_idx: Indices of obstacles within detection range
            %   distances: Array of distances to all obstacles
            
            % Initialize outputs
            in_range_idx = [];
            distances = [];
            
            % If no static obstacles, return empty arrays
            if isempty(obstacles)
                return;
            end
            
            % Calculate distances to all obstacles
            diff_vectors = obstacles - obj.CurrentCoord;  % Using implicit expansion
            distances = sqrt(sum(diff_vectors.^2, 1));
            
            % Find obstacles within detection range
            in_range_idx = find(distances <= obj.DetectionRadius);
        end
    end
end