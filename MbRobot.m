%=========================================================================%
% robot = MbRobot(waypoints, color, SafeZone, DetectionZone)
%
%=========================================================================%


%% Robot with xxx Model

classdef MbRobot < handle
    properties
        LinearVelocity = 1;
        RotationAngle
        CurrentPose
        CurrentCoord
        HeadAngle = 0;
        AngularVelocity = 0;
        Start
        Goal
        Waypoints
        LightColor
        DarkColor
        GoalRadius = 0.5;
        SafeRadius
        DetectionRadius
        Obstacle
        attraction_force
        repulsion_force
        combined_force
        name
        
    end

    methods
%% This is for dynamic rotational model
         
        % The first obstacle avoidance method: Directly rotate a degree of
        % rotationAnlge
        function fixed_rotate(obj, sampleTime, rotationAngle)
            collision = false;
            previousHeadAngle = obj.HeadAngle;
            rotation = [cos(rotationAngle), -sin(rotationAngle);
                        sin(rotationAngle), cos(rotationAngle)];

            for i = 1:size(obj.Obstacle, 2)
                if calcDist(obj.Obstacle(:,i), obj.CurrentCoord) < obj.SafeRadius
                    collision = true;
                   break
                end
            end
            
            if collision
                direction = rotation * [cos(obj.HeadAngle); sin(obj.HeadAngle)];
            else
                direction = (obj.Goal - obj.CurrentCoord)...
                    /calcDist(obj.Goal, obj.CurrentCoord);
            end
            obj.CurrentCoord = obj.CurrentCoord + ...
                        direction * obj.LinearVelocity * sampleTime;
            obj.HeadAngle = atan2(direction(2),direction(1));
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];
        end
        

        % The second obstacle avoidance method: Bug algorithm
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

            obj.CurrentCoord = obj.CurrentCoord + obj.LinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];
        end
        

        % For the ego robot
        function artificial_potential_field(obj, sampleTime, ...
                attraction_factor, repulsive_factor)
            % For robot1 (ego robot)
            previousHeadAngle = obj.HeadAngle;
            obstacle_distances = zeros(1, size(obj.Obstacle, 2));
            target_point = obj.Goal;
      
            % Calculate distances to all obstacles (robot2 and robot3)
            for i = 1:size(obj.Obstacle, 2)
                obstacle_distances(i) = calcDist(obj.Obstacle(:,i), obj.CurrentCoord);
            end
            
            % Find the nearest obstacle
            [min_distance, nearest_obstacle_index] = min(obstacle_distances);
            
            % if min_distance < extended_safe_radius
            obstacle_vector = obj.CurrentCoord - obj.Obstacle(:,nearest_obstacle_index);
        
            obj.repulsion_force = obstacle_vector / norm(obstacle_vector);
            obj.repulsion_force = obj.repulsion_force * (obj.DetectionRadius - min_distance) / obj.DetectionRadius;
            
            % Calculate attraction to goal
            goal_vector = target_point' - obj.CurrentCoord;
            obj.attraction_force = goal_vector / norm(goal_vector);
            
            % Combine forces
            obj.combined_force = attraction_factor * obj.attraction_force + ...
                    repulsive_factor * obj.repulsion_force;
            
            obj.combined_force = obj.combined_force / norm(obj.combined_force);
            % Set new heading angle
            obj.HeadAngle = atan2(obj.combined_force(2), obj.combined_force(1));
            
            % Move robot
            obj.CurrentCoord = obj.CurrentCoord + obj.LinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                                obj.HeadAngle; 
                                obj.AngularVelocity];

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
            obj.CurrentCoord = obj.CurrentCoord + obj.LinearVelocity * ...
                [cos(obj.HeadAngle); sin(obj.HeadAngle)] * sampleTime;
            obj.AngularVelocity = (obj.HeadAngle - previousHeadAngle) / sampleTime;
            obj.CurrentPose = [obj.CurrentCoord;
                               obj.HeadAngle;
                               obj.AngularVelocity];


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
                'Curvature',[1,1], 'EdgeColor',[0.5 0.5 0.5], ...
                'FaceColor',obj.DarkColor);

            % Visualize forces
            scale = 1; % Adjust this value to change the length of the force arrows
            
            % Attraction force (green)
            quiver(centerX, centerY, ...
                obj.attraction_force(1)*scale, obj.attraction_force(2)*scale, ...
                0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Attraction Force');
            
            % Repulsion force (red)
            quiver(centerX, centerY, ...
                obj.repulsion_force(1)*scale, obj.repulsion_force(2)*scale, ...
                0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Repulsion Force');
            
            % Combined force (blue)
            quiver(centerX, centerY, ...
                obj.combined_force(1)*scale, obj.combined_force(2)*scale, ...
                0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Combined Force');

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
    end
end