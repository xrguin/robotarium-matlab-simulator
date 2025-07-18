% Last Modified: 5/21/2024
%%
clc
clear
close all
%%
% Settting avoid and detection range
r_a = 1;
r_d = 0;
distance = 8;
%%
n = 1;
samples = 5;
augmented_pq_r4 = cell(samples,2);
annimation = true;
record_collision_condition = true;

test_site_size = 15;

load('Sliced_No_Seg_apf_3layer.mat','multiSliceNet')

collision_conditions_mz = {};

while n <= samples

    %======Comment this section to test use saved initial locations========
    rng('shuffle')
   
    % Generate ally robot: robot2
    robot2 = MbRobot;
    robot2.name = 'Ally';
    robot2.Start = rand(1,2)*test_site_size;
    robot2.Goal = generateRandomCoords(1, robot2.Start, distance, test_site_size);
    % robot2.Start = [2, 2];
    % robot2.Goal = [13, 13];
    lightColor2 = [0.8 1 0.8 0.2];
    % lightColor2 = [1 1 1 0.2];
    darkColor2 = [0.5 1 0.5 0.2];
    robot2.LightColor = lightColor2;
    robot2.DarkColor = lightColor2;
    robot2.SafeRadius = r_a;
    robot2.HeadAngle = atan2(robot2.Goal(2) - robot2.Start(2), ...
                                robot2.Goal(1) - robot2.Start(1));
    robot2.CurrentPose = [robot2.Start'; robot2.HeadAngle; robot2.AngularVelocity];
    robot2.CurrentCoord = robot2.CurrentPose(1:2);
    robot2.DetectionRadius = 15;
    recordCount = 1;
    % Generate enemy robot: robot3
    % Ensure the distance between start and goal for enemy robot is larger
    % than 5, and the robot3 trajectory is longer than robot2
    % while true
        % robot3Start = rand(1,2)*test_site_size;
    
%%
    robot3 = MbRobot;
    robot3.name = 'Adversarial';
    [robot3.Start, robot3.Goal, interceptPoint] = generateInterceptingPath(robot2.Start, robot2.Goal, ...
        test_site_size, robot2.SafeRadius);
        % if calcDist(robot3Start, robot3Goal) > ...
        %         calcDist(robot2Start,robot2Goal) && ...
        %         calcDist(robot2Start,robot3Start) > robot2.SafeRadius && ...
        %         doLinesIntersect(robot2Start,robot2Goal,robot3Start,robot3Goal)
        %     break
        % end
    % end
    % robot3.Start = [13,2];
    % robot3.Goal = [2, 13];
    lightColor3 = [0.6 0.8 1 0.2];
    % lightColor3 = [1 1 1 0.2];
    darkColor3 = [0.4 0.6 1 0.2];
    robot3.LightColor = lightColor3;
    robot3.DarkColor = lightColor3;
    robot3.SafeRadius = r_a;
    robot3InitPose = robot3.CurrentPose;
    robot3.HeadAngle = atan2(robot3.Goal(2) - robot3.Start(2), ...
                                robot3.Goal(1) - robot3.Start(1));
    robot3.CurrentPose = [robot3.Start'; robot3.HeadAngle; robot3.AngularVelocity];
    robot3.CurrentCoord = robot3.CurrentPose(1:2);
    robot3.DetectionRadius = 15;
    % Generate ego robot, robot1
    existCoords = [robot2.Start; robot2.Goal; robot3.Start; robot3.Goal];
    
    robot1 = MbRobot;
    robot1.name = 'Ego';
    try
        [robot1.Start, robot1.Goal] = generateEgoRobotPath(robot2.Start, robot3.Start, ...
            interceptPoint,test_site_size,robot3.SafeRadius);
        % origins = [robot2.Start; robot3.Start];
        % robot1_locations = generateRandomCoords(2, origins, distance, test_site_size);
        % robot1.Start = robot1_locations(1,:);
        % robot1.Goal = robot1_locations(2,:);
        %     interceptPoint,test_site_size,robot3.SafeRadius);
        % robot1.Start = [2,13];
        % robot1.Goal = [13,2];
        lightColor1 = [255, 178, 102]/255 ; 
        robot1.LightColor = lightColor1;
        robot1.DarkColor = lightColor1;
        robot1.SafeRadius = 1;
        robot1.HeadAngle = atan2(robot1.Goal(2) - robot1.Start(2), ...
                                    robot1.Goal(1) - robot1.Start(1));
        robot1.CurrentPose = [robot1.Start'; robot1.HeadAngle; robot1.AngularVelocity];
        robot1.CurrentCoord = robot1.CurrentPose(1:2);
        robot1.DetectionRadius = 15;

            % === Generate second ego robot: robot4 ===
        robot4 = MbRobot;
        robot4.name = 'Ego2';

        [robot4.Start, robot4.Goal] = generateEgoRobotPatha(robot1.Start,robot2.Start, robot3.Start, ...
            interceptPoint,test_site_size,robot3.SafeRadius);

        % 保证robot4的起点不与robot1、robot2、robot3重合
      %  existing_positions = [robot1.Start; robot2.Start; robot3.Start];
      %  while true
      %      candidateStart = rand(1,2)*test_site_size;
      %      if all(vecnorm(existing_positions - candidateStart, 2, 2) > robot1.SafeRadius)
      %          break;
       %     end
        %end

        % 生成一个足够远的goal
       % robot4.Start = candidateStart;
       % robot4.Goal = generateRandomCoords(1, robot4.Start, distance, test_site_size);

        lightColor1 = [255, 178, 102]/255 ; 
        robot4.LightColor = lightColor1;
        robot4.DarkColor = lightColor1;
        robot4.SafeRadius = r_a;
        robot4.HeadAngle = atan2(robot4.Goal(2) - robot4.Start(2), ...
                              robot4.Goal(1) - robot4.Start(1));
        robot4.CurrentPose = [robot4.Start'; robot4.HeadAngle; robot4.AngularVelocity];
        robot4.CurrentCoord = robot4.CurrentPose(1:2);
        robot4.DetectionRadius = 15;


        %==============================================================================
        collision_conditions_mz{n,1} = robot1.Start;
        collision_conditions_mz{n,2} = robot1.Goal;
        collision_conditions_mz{n,3} = robot2.Start;
        collision_conditions_mz{n,4} = robot2.Goal;
        collision_conditions_mz{n,5} = robot3.Start;
        collision_conditions_mz{n,6} = robot3.Goal;
        collision_conditions_mz{n,7} = robot4.Start;
        collision_conditions_mz{n,8} = robot4.Goal;
    
        % Record if ego robot has collided with any other agnet
        collision_conditions_mz{n,9} = 0;
        % % Record the average prediction error 
        % collision_conditions_mz{n,8} = 0;
%%
        estm_pos3 = [];
        estm_plot = [];
        poses1 = [];
        poses2 = [];
        poses3 = [];
        poses4 = [];
%%
        % Define sample time, and simulation time
        sampleTime = 0.1;
        simulationTime = 30;
        
        % Use the robot2.Pose data to predict robot3.Pose data
        recordSize = 10;
        robot2OnlinePose = zeros(4,recordSize);
        k = 1;
        bufferSize = 1;
        filo_length = 5;  % Length of the moving average window
        buffer = zeros(filo_length,2);  % Circular buffer for moving average
        buffer_sum = [0,0];  % Sum of values in the buffer
%%
     % Simulation Loop
        for t = 0:sampleTime:simulationTime
    
            robot1Pose = robot1.CurrentPose;  
            robot2Pose = robot2.CurrentPose;
            robot3Pose = robot3.CurrentPose;
            robot4Pose = robot4.CurrentPose;
            if t == sampleTime
                robot2Pose(4) = 0;
                robot3Pose(4) = 0;
            end
            poses2 = [poses2, [robot2Pose; t]];
            poses3 = [poses3, [robot3Pose; t]];
            poses1 = [poses1, [robot1Pose; t]];
            poses4 = [poses4, [robot4Pose; t]];
            
            obstaclePose2 = [robot3Pose(1:2,:),robot4Pose(1:2,:),robot1Pose(1:2,:)];
            obstaclePose3 = [robot2Pose(1:2,:)];
            robot2.Obstacle = obstaclePose2;
            
%%
            robot2.artificial_potential_field(sampleTime,1,1);
    
            robot3.Obstacle = obstaclePose3;
            robot3.artificial_potential_field(sampleTime,1,1.2);

            % Use the robot2.Pose data to predict robot3.Pose data
            % To predict the location of obstacles
            if recordCount <= recordSize
                robot2OnlinePose(: , recordCount) = robot2.CurrentPose;
                recordCount = recordCount + 1;
                obstaclePosAll = [robot2Pose(1:2,:)];
            else
           
                robot2OnlinePose(:,bufferSize:end-bufferSize) = ...
                    robot2OnlinePose(:,1+bufferSize:end);
                robot2OnlinePose(:,end) = robot2.CurrentPose;
    
                estmRobot3_raw = predict(multiSliceNet,robot2OnlinePose(1:3,:)');
                multiSliceNet.resetState();
                % estmRobot3 = estmRobot3';
                
                % estm_pos3 = [estm_pos3, [estmRobot3(1); estmRobot3(2)]];
            % estm_pos3 = [estm_pos3, robot3Pose(1:2,:)];
                if k <= filo_length
                    buffer(k,:) = estmRobot3_raw;
                    buffer_sum = sum(buffer,1);
                    estmRobot3 = buffer_sum / k; % For the first few points
                    k = k + 1;
                else
                    buffer(1:filo_length-1,:) = buffer(2:filo_length,:);
                    buffer(end,:) = estmRobot3_raw;
                    buffer_sum = sum(buffer,1);
                    estmRobot3 = buffer_sum / filo_length;  % Full window average
                    
                end
      
                estmRobot3 = estmRobot3';
                obstaclePosAll = [robot2Pose(1:2,:), estmRobot3];
            end

            % obstaclePosAll = estmRobot3;
            % obstaclePosAll = [robot2Pose(1:2,:)];
            robot1.Obstacle = [obstaclePosAll, robot4Pose(1:2,:)];
            robot1.artificial_potential_field(sampleTime, 1, 1.2);
            robot4.Obstacle = [obstaclePosAll, robot1Pose(1:2,:)];
            robot4.artificial_potential_field(sampleTime, 1, 1.2);
            
            if calcDist(robot1.CurrentCoord, robot2.CurrentCoord) < robot1.SafeRadius ||...
                calcDist(robot1.CurrentCoord, robot3.CurrentCoord) < robot1.SafeRadius ||...
                calcDist(robot1.CurrentCoord, robot4.CurrentCoord) < robot1.SafeRadius ||...
                calcDist(robot4.CurrentCoord, robot2.CurrentCoord) < robot4.SafeRadius || ...
                calcDist(robot4.CurrentCoord, robot1.CurrentCoord) < robot4.SafeRadius || ...
                calcDist(robot4.CurrentCoord, robot3.CurrentCoord) < robot4.SafeRadius
                collision_conditions_mz{n,9} = 1;
                break;
            end
            % If any agent reaches the goal then the simulation stops
            if robot1.atGoal() && robot4.atGoal() 
                break;
            elseif robot2.atGoal()
                robot2.LinearVelocity = 0;
                robot2.AngularVelocity = 0;
            elseif robot3.atGoal()
                robot3.LinearVelocity = 0;
                robot3.AngularVelocity = 0;
            elseif robot1.atGoal() && ~robot4.atGoal()
                robot1.LinearVelocity = 0;
                robot1.AngularVelocity = 0;
            elseif robot4.atGoal() && ~robot1.atGoal()
                robot4.LinearVelocity = 0;
                robot4.AngularVelocity = 0;
            end
            
%%
            if annimation
                % Plot the robot's position
                figure(1)
                set(gcf,'DoubleBuffer','off','Renderer','opengl')
                clf; % clear current figure
                robot2.plotRobot(1); % adjust the frame size as needed
                robot3.plotRobot(1);
                robot1.plotRobot(1);
                robot4.plotRobot(1);
        
                hold on
                % plot(robot1.Start(1),robot1.Start(2),'go','MarkerSize',10,'MarkerFaceColor','g')
                % plot(robot1.Goal(1),robot1.Goal(2),'rp','MarkerSize',10,'MarkerFaceColor','r')
                scatter(poses1(1,1:2:end),poses1(2,1:2:end),800,robot1.LightColor);
                % plot(poses2(1,:),poses2(2,:),'-',...
                %     'Color',[1,50,32]/255,'LineWidth',1);   % Orange: [1 0.5 0]
                scatter(poses2(1,1:2:end),poses2(2,1:2:end),800,robot2.LightColor(1:3));
                scatter(poses3(1,1:2:end),poses3(2,1:2:end),800,robot3.LightColor(1:3));
                scatter(poses4(1,1:2:end),poses4(2,1:2:end),800,robot4.LightColor);
                % plot(poses3(1,:),poses3(2,:),'-',...
                %     'Color',[0.5,0.5,0.5],'LineWidth',1);
                try
                    % plot(estm_plot(1,:),estm_plot(2,:),'--',...
                    %     'Color',[0 0 1],'LineWidth',1)
                    
                    if calcDist(estmRobot3, robot3.CurrentCoord) < robot1.SafeRadius || calcDist(estmRobot3, robot3.CurrentCoord) < robot4.SafeRadius
                        estm_pos3 = [estm_pos3, estmRobot3];
                        % scatter(estmRobot3(1), estmRobot3(2),120,'blue','square','filled')
                        % scatter(estm_pos3(1,1:2:end),estm_pos3(2,1:2:end),800,'blue')
                    % else
                        % scatter(estmRobot3(1), estmRobot3(2),120,'cyan','square','filled')
                    end

                catch 
                    
                end
                try
                    scatter(estm_pos3(1,1:2:end),estm_pos3(2,1:2:end),800,'blue')
                end
                axis equal;
                xlim([-5, 20]);
                ylim([-5, 20]);
                drawnow
            
%%
            end
        end
    

%%
%     if ~robot2.atGoal()
%         disp('Robot did not reach the goal within the simulation time.');
%     end
    n = n + 1;
    augmented_pq_r4{n,1} = poses2;
    augmented_pq_r4{n,2} = poses3;
    if mod(n-1,100) == 0
        fprintf('Completed Trials: %d\n', n)
    end
    % allWaypoints_pq_r2{n,3} = estm_poses3;
    catch ME
        fprintf('An error occurred: %s\n', ME.message);
        fprintf('Skipped, new random start generated.\n')
        fprintf('Completed Trials: %d\n', n-1)
    end
end
%%
if record_collision_condition
    save('Data/SimulationData/demo_mz.mat','collision_conditions_mz');
end
%%
% Get the last column of the cell array
lastColumn = collision_conditions_mz(:, end);

% Convert the cell array column to a numeric array
numericColumn = cell2mat(lastColumn);

% Count the number of 1s
num_collide_mz = sum(numericColumn == 1)
collision_rate_mz = num_collide_mz / samples
%%
% save('Sata/SimulationData/demo_robot1.mat','poses1')
% save('Sata/SimulationData/demo_robot2.mat','poses2')
% save('Sata/SimulationData/demo_robot3.mat','poses3')
%%
% poses3InWindow = poses3(:,timeIndices);
% prediction_error = poses3InWindow(1:2,:) - estm_plot(1:2,:);
% avg_error = mean(sqrt(prediction_error(1,:).^2 + prediction_error(2,:).^2))
function [egoStart, egoGoal] = generateEgoRobotPatha(partnerStart, allyStart, enemyStart, interceptPoint, test_site_size, safeRadius)
    max_attempts = 10000;
    for attempt = 1:max_attempts
        % Calculate distance from robot2Start to interceptPoint
        distance = calcDist(interceptPoint, allyStart);
        
        vect_to_intercept = interceptPoint - partnerStart;
        vect_to_intercept = vect_to_intercept / norm(vect_to_intercept);
        
        angle = -pi * rand();
        egoStart = partnerStart + 2 * distance * vect_to_intercept * ...
            [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        
        % Check if egoStart is far enough from both robot2Start and robot3Start
        if norm(egoStart - allyStart) > safeRadius && norm(egoStart - enemyStart) > safeRadius && norm(egoStart-partnerStart) > safeRadius
            % Calculate egoGoal
            dir = interceptPoint - egoStart;
            egoGoal = interceptPoint + dir;
            
            % Check if egoGoal is within the test site
            if all(egoGoal >= 0 & egoGoal <= test_site_size)
                return
            end
        end
    end
    error('Unable to find a suitable configuration for ego robot after %d attempts', max_attempts);
end
