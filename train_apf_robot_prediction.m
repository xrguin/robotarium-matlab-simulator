%% Train a Network to Predict Robot 2 Position from Robot 1 Turning Trajectory
% Based on APF_Training_10.m approach for APF constant velocity data

clc
clear
close all

% Program Settings
todayDate = datestr(datetime('today'), 'yyyy-mm-dd');
doTrain = true;
newShuffle = true;

%% Load and Prepare Data

% Load the APF turning data
load('apf_robotarium_simulation.mat', 'apf_constant_vel_data', 'apf_full_data');

% apf_constant_vel_data format: {trial, robot} where each cell contains
% turning trajectory as [x; y; theta; omega] (4×N matrix)

% Transpose and extract only x,y,theta,omega (similar to APF_Training_10.m)
raw_data = cell(size(apf_constant_vel_data));
for i = 1:size(apf_constant_vel_data, 1)
    for j = 1:size(apf_constant_vel_data, 2)
        if ~isempty(apf_constant_vel_data{i, j})
            % Transpose from 4×N to N×4 format
            raw_data{i, j} = apf_constant_vel_data{i, j}';
        end
    end
end

numObservations = size(raw_data, 1);
fprintf('Total number of trials: %d\n', numObservations);

%% Plot a Few Sample Trajectories

figure('Name', 'Sample Turning Trajectories');
validSamples = 0;
plotIdx = 1;

for i = 1:numObservations
    if ~isempty(raw_data{i,1}) && ~isempty(raw_data{i,2}) && validSamples < 4
        robot1_data = raw_data{i,1};
        robot2_data = raw_data{i,2};
        
        if size(robot1_data, 1) > 20 && size(robot2_data, 1) > 20
            validSamples = validSamples + 1;
            subplot(2, 2, validSamples);
            
            % Extract x,y coordinates
            robot1_coord = robot1_data(:, 1:2);
            robot2_coord = robot2_data(:, 1:2);
            
            plot(robot1_coord(:,1), robot1_coord(:,2), 'g-', 'LineWidth', 2);
            hold on;
            plot(robot2_coord(:,1), robot2_coord(:,2), 'b-', 'LineWidth', 2);
            plot(robot1_coord(1,1), robot1_coord(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            plot(robot2_coord(1,1), robot2_coord(1,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            
            xlim([-2, 2]);
            ylim([-1.5, 1.5]);
            xlabel('X (m)');
            ylabel('Y (m)');
            title(sprintf('Trial %d Turning Trajectories', i));
            legend('Robot 1', 'Robot 2', 'Location', 'best');
            grid on;
            hold off;
        end
    end
end

%% Filter Trajectories by Length

% Find trajectories with sufficient length (both robots must have data)
minLength = 50; % Minimum trajectory length for meaningful training
isValid = false(numObservations, 1);

for i = 1:numObservations
    if ~isempty(raw_data{i,1}) && ~isempty(raw_data{i,2})
        % Check if both robots have sufficient turning data
        if size(raw_data{i,1}, 1) >= minLength && size(raw_data{i,2}, 1) >= minLength
            isValid(i) = true;
        end
    end
end

% Extract valid trajectories
validTrajectories = raw_data(isValid, :);
fprintf('Valid trajectories with length > %d: %d\n', minLength, size(validTrajectories, 1));

%% Create Sliding Window Dataset

windowSize = 10; % Use 10 timesteps of robot 1 data to predict robot 2
XAll = {}; % Input: Robot 1 trajectory windows
TAll = {}; % Target: Robot 2 position at next timestep
allData = {};
k = 1; % Index for storing valid windows

for n = 1:size(validTrajectories, 1)
    robot1_data = validTrajectories{n, 1};
    robot2_data = validTrajectories{n, 2};
    
    % Ensure we have synchronized data
    minLen = min(size(robot1_data, 1), size(robot2_data, 1));
    
    % Create sliding windows
    numWindows = minLen - windowSize - 1;
    
    for t = 1:numWindows
        % Extract window of robot 1 data (10 timesteps)
        robot1_window = robot1_data(t:t+windowSize-1, :); % All 4 features
        
        % Check if window contains actual turning (non-zero angular velocities)
        angular_velocities = robot1_window(:, 4);
        if any(abs(angular_velocities) > 0.1) % At least some turning in the window
            % Input: Robot 1 trajectory over window
            XAll{k, 1} = robot1_window;
            
            % Target: Robot 2 position at next timestep
            TAll{k, 1} = robot2_data(t+windowSize, 1:2); % Only x,y position
            
            % Store together for shuffling
            allData{k, 1} = XAll{k, 1};
            allData{k, 2} = TAll{k, 1};
            
            k = k + 1;
        end
    end
end

fprintf('Total number of valid windows: %d\n', size(XAll, 1));

%% Shuffle the Data

if newShuffle
    randOrder = randperm(size(XAll, 1));
    % Optionally save shuffle order for reproducibility
    % save('shuffleOrder_apf_robot_prediction.mat', 'randOrder');
else
    % Load existing shuffle order if needed
    % load('shuffleOrder_apf_robot_prediction.mat', 'randOrder');
    randOrder = 1:size(XAll, 1);
end

shuffledData = allData(randOrder, :);

%% Split Data into Train/Validation/Test Sets

totalSamples = size(shuffledData, 1);
iTrain = 1:floor(0.7 * totalSamples);
iVal = floor(0.7 * totalSamples) + 1 : floor(0.9 * totalSamples);
iTest = floor(0.9 * totalSamples) + 1 : totalSamples;

XTrain = shuffledData(iTrain, 1);
XVal = shuffledData(iVal, 1);
XTest = shuffledData(iTest, 1);

TTrain = cell2mat(shuffledData(iTrain, 2));
TVal = cell2mat(shuffledData(iVal, 2));
TTest = cell2mat(shuffledData(iTest, 2));

fprintf('\nData split:\n');
fprintf('Training samples: %d\n', length(iTrain));
fprintf('Validation samples: %d\n', length(iVal));
fprintf('Test samples: %d\n', length(iTest));

%% Normalize the Data

% Calculate normalization parameters from training data
muX_train = mean(cell2mat(XTrain));
sigmaX_train = std(cell2mat(XTrain), 0);

muT_train = mean(TTrain);
sigmaT_train = std(TTrain, 0);

% Normalize training data
XTrain_norm = cell(size(XTrain));
TTrain_norm = zeros(size(TTrain));
for n = 1:numel(XTrain)
    XTrain_norm{n} = (XTrain{n} - muX_train) ./ sigmaX_train;
    TTrain_norm(n, :) = (TTrain(n, :) - muT_train) ./ sigmaT_train;
end

% Normalize validation data
XVal_norm = cell(size(XVal));
TVal_norm = zeros(size(TVal));
for n = 1:numel(XVal)
    XVal_norm{n} = (XVal{n} - muX_train) ./ sigmaX_train;
    TVal_norm(n, :) = (TVal(n, :) - muT_train) ./ sigmaT_train;
end

% Normalize test data
XTest_norm = cell(size(XTest));
TTest_norm = zeros(size(TTest));
for n = 1:numel(XTest)
    XTest_norm{n} = (XTest{n} - muX_train) ./ sigmaX_train;
    TTest_norm(n, :) = (TTest(n, :) - muT_train) ./ sigmaT_train;
end

%% Define and Train the Network

numChannels = 4; % x, y, theta, omega for robot 1
numOutputs = 2;  % x, y for robot 2

if doTrain
    % Define LSTM network architecture (similar to APF_Training_10.m)
    layers = [...
        sequenceInputLayer(numChannels)
        lstmLayer(32, 'OutputMode', 'sequence')
        dropoutLayer(0.2)
        lstmLayer(128, 'OutputMode', 'last')
        fullyConnectedLayer(numOutputs)
        ];
    
    % Training options
    options = trainingOptions('adam', ...
        'ExecutionEnvironment', 'auto', ... % Use 'gpu' if available
        'MaxEpochs', 300, ...
        'MiniBatchSize', 128, ...
        'InitialLearnRate', 0.001, ...
        'LearnRateSchedule', 'piecewise', ...
        'LearnRateDropPeriod', 100, ...
        'LearnRateDropFactor', 0.2, ...
        'SequencePaddingDirection', 'left', ...
        'Shuffle', 'every-epoch', ...
        'GradientThreshold', 0.5, ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'ValidationData', {XVal_norm, TVal_norm}, ...
        'ValidationFrequency', 50, ...
        'ValidationPatience', 20);
    
    % Train the network
    fprintf('\nTraining LSTM network...\n');
    net_apf_robot_prediction = trainnet(XTrain_norm, TTrain_norm, layers, 'mse', options);
    
    % Save trained model and normalization parameters
    trainedModel = struct();
    trainedModel.net = net_apf_robot_prediction;
    trainedModel.muX = muX_train;
    trainedModel.sigmaX = sigmaX_train;
    trainedModel.muT = muT_train;
    trainedModel.sigmaT = sigmaT_train;
    trainedModel.windowSize = windowSize;
    trainedModel.trainingDate = todayDate;
    
    save(sprintf('apf_robot2_predictor_%s.mat', todayDate), 'trainedModel');
    fprintf('\nModel saved to: apf_robot2_predictor_%s.mat\n', todayDate);
end

%% Test the Model

if exist('net_apf_robot_prediction', 'var')
    % Predict on test set
    YTest_norm = predict(net_apf_robot_prediction, XTest_norm);
    
    % Denormalize predictions
    YTest = YTest_norm .* sigmaT_train + muT_train;
    
    % Calculate test error
    testError = mean(sqrt(sum((YTest - TTest).^2, 2)));
    fprintf('\nTest Results:\n');
    fprintf('Mean Euclidean Error: %.4f meters\n', testError);
    
    % Visualize some predictions
    figure('Name', 'Test Set Predictions');
    numPlots = min(4, size(YTest, 1));
    
    for i = 1:numPlots
        subplot(2, 2, i);
        
        % Plot robot 1 input trajectory
        robot1_traj = XTest{i} * sigmaX_train(1:2) + muX_train(1:2);
        plot(robot1_traj(:, 1), robot1_traj(:, 2), 'g-', 'LineWidth', 2);
        hold on;
        
        % Plot actual vs predicted robot 2 position
        plot(TTest(i, 1), TTest(i, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        plot(YTest(i, 1), YTest(i, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        
        % Calculate error for this sample
        error = norm(YTest(i, :) - TTest(i, :));
        
        xlim([-2, 2]);
        ylim([-1.5, 1.5]);
        xlabel('X (m)');
        ylabel('Y (m)');
        title(sprintf('Sample %d - Error: %.3f m', i, error));
        legend('Robot 1 Path', 'Robot 2 Actual', 'Robot 2 Predicted', 'Location', 'best');
        grid on;
        hold off;
    end
    
    % Distribution of errors
    figure('Name', 'Prediction Error Distribution');
    errors = sqrt(sum((YTest - TTest).^2, 2));
    histogram(errors, 30);
    xlabel('Prediction Error (m)');
    ylabel('Frequency');
    title('Distribution of Prediction Errors on Test Set');
    grid on;
    
    fprintf('\nError Statistics:\n');
    fprintf('Mean Error: %.4f m\n', mean(errors));
    fprintf('Std Error: %.4f m\n', std(errors));
    fprintf('Min Error: %.4f m\n', min(errors));
    fprintf('Max Error: %.4f m\n', max(errors));
    fprintf('Median Error: %.4f m\n', median(errors));
end

fprintf('\nTraining complete!\n');
