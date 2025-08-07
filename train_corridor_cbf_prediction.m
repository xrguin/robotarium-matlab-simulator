%% Train a Network to Predict Adversary Position in Narrow Corridor
% Based on train_apf_robot_prediction.m approach adapted for CBF corridor data
% This script trains an LSTM network to predict the adversary's next position
% given the ally robot's trajectory history in a narrow corridor environment

clc
clear
close all

% Program Settings
todayDate = datestr(datetime('today'), 'yyyy-mm-dd');
doTrain = true;
newShuffle = true;

%% Load and Prepare Data

% Load the CBF corridor data
fprintf('Loading corridor CBF data...\n');
load('horizontal_corridor_cbf_data.mat', 'corridor_cbf_data');

% Extract number of valid trials
totalTrials = length(corridor_cbf_data);
fprintf('Total number of trials available: %d\n', totalTrials);

% Use only first 2000 trials
maxTrialsToUse = 2000;
numTrials = min(totalTrials, maxTrialsToUse);
corridor_cbf_data = corridor_cbf_data(1:numTrials);
fprintf('Using %d trials for training\n', numTrials);

%% Convert Data Format
% Convert from structured format to cell array format compatible with training

raw_data = cell(numTrials, 2);
validTrialCount = 0;

for i = 1:numTrials
    if ~isempty(corridor_cbf_data{i})
        trial_data = corridor_cbf_data{i};
        
        % Extract ally trajectory [x; y; theta]
        ally_traj = trial_data.ally_trajectory; % Should be 3×N
        
        % Extract adversary trajectory
        adv_traj = trial_data.adversary_trajectory; % Should be 3×N
        
        % Check dimensions and handle accordingly
        if ndims(ally_traj) > 2
            % If it's a 3D array, it's likely only one time series, so squeeze it
            ally_traj = squeeze(ally_traj);
            adv_traj = squeeze(adv_traj);
        end
        
        % Ensure correct orientation (should be 3×N for trajectory)
        if size(ally_traj, 1) > size(ally_traj, 2)
            ally_traj = ally_traj';
        end
        if size(adv_traj, 1) > size(adv_traj, 2)
            adv_traj = adv_traj';
        end
        
        % Transpose to get N×3 format [x, y, theta]
        ally_data = ally_traj'; % N×3
        adv_data = adv_traj'; % N×3
        
        raw_data{i, 1} = ally_data;
        raw_data{i, 2} = adv_data;
        validTrialCount = validTrialCount + 1;
    end
end

fprintf('Valid trials with data: %d\n', validTrialCount);

%% Plot a Few Sample Trajectories

figure('Name', 'Sample Corridor Trajectories');
validSamples = 0;

for i = 1:numTrials
    if ~isempty(raw_data{i,1}) && ~isempty(raw_data{i,2}) && validSamples < 4
        ally_data = raw_data{i,1};
        adv_data = raw_data{i,2};
        
        if size(ally_data, 1) > 20 && size(adv_data, 1) > 20
            validSamples = validSamples + 1;
            subplot(2, 2, validSamples);
            
            % Extract x,y coordinates
            ally_coord = ally_data(:, 1:2);
            adv_coord = adv_data(:, 1:2);
            
            % Plot trajectories
            plot(ally_coord(:,1), ally_coord(:,2), 'g-', 'LineWidth', 2);
            hold on;
            plot(adv_coord(:,1), adv_coord(:,2), 'b-', 'LineWidth', 2);
            
            % Mark start positions
            plot(ally_coord(1,1), ally_coord(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            plot(adv_coord(1,1), adv_coord(1,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            
            % Draw corridor obstacles (from original script)
            passage_width = 0.4;
            robotarium_height = 2.0;
            radius = (robotarium_height - passage_width) / 2;
            center_top_y = passage_width/2 + radius;
            center_bottom_y = -passage_width/2 - radius;
            
            % Draw static obstacles
            theta_obs = linspace(0, 2*pi, 50);
            obs_top = [0 + radius*cos(theta_obs); center_top_y + radius*sin(theta_obs)];
            obs_bottom = [0 + radius*cos(theta_obs); center_bottom_y + radius*sin(theta_obs)];
            fill(obs_top(1,:), obs_top(2,:), 'k', 'FaceAlpha', 0.3);
            fill(obs_bottom(1,:), obs_bottom(2,:), 'k', 'FaceAlpha', 0.3);
            
            xlim([-1.6, 1.6]);
            ylim([-1, 1]);
            xlabel('X (m)');
            ylabel('Y (m)');
            title(sprintf('Trial %d Corridor Trajectories', i));
            legend('Ally (yields)', 'Adversary', 'Location', 'best');
            grid on;
            hold off;
        end
    end
end

%% Filter Trajectories by Length

% Find trajectories with sufficient length (both robots must have data)
minLength = 50; % Minimum trajectory length for meaningful training
isValid = false(numTrials, 1);

for i = 1:numTrials
    if ~isempty(raw_data{i,1}) && ~isempty(raw_data{i,2})
        % Check if both robots have sufficient data
        if size(raw_data{i,1}, 1) >= minLength && size(raw_data{i,2}, 1) >= minLength
            isValid(i) = true;
        end
    end
end

% Extract valid trajectories
validTrajectories = raw_data(isValid, :);
fprintf('Valid trajectories with length > %d: %d\n', minLength, size(validTrajectories, 1));

%% Split trajectories for training and full trajectory testing

% Take 5% of complete trajectories for full trajectory testing
numTrajectories = size(validTrajectories, 1);
numTestTrajectories = ceil(0.05 * numTrajectories);

% Randomly select test trajectory indices
rng(42); % For reproducibility
testTrajIndices = randperm(numTrajectories, min(numTestTrajectories, numTrajectories));
trainTrajIndices = setdiff(1:numTrajectories, testTrajIndices);

% Store test trajectories for later evaluation
testTrajectories = validTrajectories(testTrajIndices, :);
trainTrajectories = validTrajectories(trainTrajIndices, :);

fprintf('Using %d trajectories for training, %d for full trajectory testing\n', ...
    length(trainTrajIndices), length(testTrajIndices));

%% Create Sliding Window Dataset from Training Trajectories

offset = 29; % Use 30 timesteps total (same as APF)
windowSize = offset + 1; % Total window size is 30
XAll = {}; % Input: Ally trajectory windows
TAll = {}; % Target: Adversary position at next timestep
allData = {};
k = 1; % Index for storing valid windows

for n = 1:size(trainTrajectories, 1)
    ally_data = trainTrajectories{n, 1};
    adv_data = trainTrajectories{n, 2};
    
    % Ensure we have synchronized data
    minLen = min(size(ally_data, 1), size(adv_data, 1));
    
    % Create sliding windows
    numWindows = minLen - offset - 1;
    
    for t = 1:numWindows
        % Extract window of ally data (30 timesteps, x,y,theta)
        ally_window = ally_data(t:t+offset, 1:3);
        
        % Input: Ally trajectory over window (x,y,theta)
        XAll{k, 1} = ally_window;
        
        % Target: Adversary position at next timestep
        TAll{k, 1} = adv_data(t+offset+1, 1:2); % Only x,y position
        
        % Store together for shuffling
        allData{k, 1} = XAll{k, 1};
        allData{k, 2} = TAll{k, 1};
        
        k = k + 1;
    end
end

fprintf('Total number of valid windows: %d\n', size(XAll, 1));

%% Shuffle the Data

if newShuffle
    randOrder = randperm(size(XAll, 1));
    % Save shuffle order for reproducibility
    save('shuffleOrder_corridor_cbf.mat', 'randOrder');
else
    % Load existing shuffle order if needed
    if exist('shuffleOrder_corridor_cbf.mat', 'file')
        load('shuffleOrder_corridor_cbf.mat', 'randOrder');
    else
        randOrder = 1:size(XAll, 1);
    end
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

% Avoid division by zero
sigmaX_train(sigmaX_train == 0) = 1;
sigmaT_train(sigmaT_train == 0) = 1;

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

% Check GPU availability
if gpuDeviceCount > 0
    gpuDevice(1);
    reset(gpuDevice);
    executionEnvironment = 'gpu';
else
    executionEnvironment = 'cpu';
    fprintf('WARNING: No GPU detected, training on CPU (will be slower)\n');
end

numChannels = 3; % x, y, theta for ally
numOutputs = 2;  % x, y for adversary

if doTrain
    % Define LSTM network architecture (similar to APF)
    layers = [...
        sequenceInputLayer(numChannels)
        lstmLayer(64, 'OutputMode', 'sequence')
        dropoutLayer(0.2)
        lstmLayer(256, 'OutputMode', 'last')
        fullyConnectedLayer(numOutputs)
        ];
    
    % Training options
    options = trainingOptions('adam', ...
        'ExecutionEnvironment', executionEnvironment, ...
        'MaxEpochs', 500, ...
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
        'ValidationPatience', 1000, ...
        'CheckpointPath', pwd, ...
        'OutputFcn', @customTrainingMonitor);
    
    % Train the network
    fprintf('\nTraining LSTM network for corridor adversary prediction...\n');
    net_corridor_cbf = trainnet(XTrain_norm, TTrain_norm, layers, 'mse', options);
    
    % Create structure to store network and training data
    trainedModel = struct();
    trainedModel.net = net_corridor_cbf;
    trainedModel.normParams = struct();
    trainedModel.normParams.muX = muX_train;
    trainedModel.normParams.sigmaX = sigmaX_train;
    trainedModel.normParams.muT = muT_train;
    trainedModel.normParams.sigmaT = sigmaT_train;
    trainedModel.trainingInfo = struct();
    trainedModel.trainingInfo.offset = offset;
    trainedModel.trainingInfo.windowSize = windowSize;
    trainedModel.trainingInfo.numChannels = numChannels;
    trainedModel.trainingInfo.numOutputs = numOutputs;
    trainedModel.trainingInfo.trainDate = todayDate;
    trainedModel.trainingInfo.dataSize = struct();
    trainedModel.trainingInfo.dataSize.train = numel(XTrain);
    trainedModel.trainingInfo.dataSize.val = numel(XVal);
    trainedModel.trainingInfo.dataSize.test = numel(XTest);
    trainedModel.trainingInfo.environment = 'narrow_corridor_cbf';
    
    save(sprintf('corridor_adversary_predictor_%s.mat', todayDate), 'trainedModel');
    fprintf('\nModel saved to: corridor_adversary_predictor_%s.mat\n', todayDate);
end

%% Test the Model

if exist('net_corridor_cbf', 'var')
    % Predict on test set
    YTest_norm = zeros(numel(XTest_norm), numOutputs);
    for i = 1:numel(XTest_norm)
        YTest_norm(i,:) = predict(net_corridor_cbf, XTest_norm{i});
    end
    
    % Denormalize predictions
    YTest = YTest_norm .* sigmaT_train + muT_train;
    
    % Calculate test error
    testError = mean(sqrt(sum((YTest - TTest).^2, 2)));
    fprintf('\nTest Results:\n');
    fprintf('Mean Euclidean Error: %.4f meters\n', testError);
    
    % Visualize some predictions
    figure('Name', 'Test Set Predictions - Corridor');
    numPlots = min(4, size(YTest, 1));
    
    for i = 1:numPlots
        subplot(2, 2, i);
        
        % Plot ally input trajectory
        ally_traj = XTest{i} * sigmaX_train(1:2) + muX_train(1:2);
        plot(ally_traj(:, 1), ally_traj(:, 2), 'g-', 'LineWidth', 2);
        hold on;
        
        % Plot actual vs predicted adversary position
        plot(TTest(i, 1), TTest(i, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        plot(YTest(i, 1), YTest(i, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        
        % Draw corridor obstacles
        passage_width = 0.4;
        robotarium_height = 2.0;
        radius = (robotarium_height - passage_width) / 2;
        center_top_y = passage_width/2 + radius;
        center_bottom_y = -passage_width/2 - radius;
        
        theta_obs = linspace(0, 2*pi, 50);
        obs_top = [0 + radius*cos(theta_obs); center_top_y + radius*sin(theta_obs)];
        obs_bottom = [0 + radius*cos(theta_obs); center_bottom_y + radius*sin(theta_obs)];
        fill(obs_top(1,:), obs_top(2,:), 'k', 'FaceAlpha', 0.2);
        fill(obs_bottom(1,:), obs_bottom(2,:), 'k', 'FaceAlpha', 0.2);
        
        % Calculate error for this sample
        error = norm(YTest(i, :) - TTest(i, :));
        
        xlim([-1.6, 1.6]);
        ylim([-1, 1]);
        xlabel('X (m)');
        ylabel('Y (m)');
        title(sprintf('Sample %d - Error: %.3f m', i, error));
        legend('Ally Path', 'Adversary Actual', 'Adversary Predicted', 'Location', 'best');
        grid on;
        hold off;
    end
    
    % Distribution of errors
    figure('Name', 'Prediction Error Distribution - Corridor');
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

%% Evaluate on Full Test Trajectories

% Load existing model if not just trained
if ~exist('net_corridor_cbf', 'var') && exist(sprintf('corridor_adversary_predictor_%s.mat', todayDate), 'file')
    load(sprintf('corridor_adversary_predictor_%s.mat', todayDate), 'trainedModel');
    net_corridor_cbf = trainedModel.net;
    muX_train = trainedModel.normParams.muX;
    sigmaX_train = trainedModel.normParams.sigmaX;
    muT_train = trainedModel.normParams.muT;
    sigmaT_train = trainedModel.normParams.sigmaT;
    offset = trainedModel.trainingInfo.offset;
end

if exist('net_corridor_cbf', 'var') && ~isempty(testTrajectories)
    fprintf('\n===== Evaluating on Full Test Trajectories =====\n');
    
    % Select first test trajectory for detailed visualization
    testIdx = 1;
    if size(testTrajectories, 1) >= testIdx
        testTrajectory = testTrajectories(testIdx, :);
        ally_test = testTrajectory{1};
        adv_test = testTrajectory{2};
        
        numTimeSteps = size(ally_test, 1);
        numPredictionTimeSteps = numTimeSteps - offset - 1;
        
        if numPredictionTimeSteps > 0
            Y_pred = nan(numPredictionTimeSteps, 2);
            Y_filter = nan(size(Y_pred)); % Filtered predictions
            
            % FILO filter parameters
            filo_length = 5;
            buffer = zeros(filo_length, 2);
            
            % Reset network state before trajectory prediction
            net_corridor_cbf.resetState();
            
            for t = 1:numPredictionTimeSteps
                % Extract window
                Xwindow = ally_test(t:t+offset, 1:3);
                
                % Normalize input
                Xwindow_norm = (Xwindow - muX_train) ./ sigmaX_train;
                
                % Predict
                Y_norm = predict(net_corridor_cbf, Xwindow_norm);
                
                % Denormalize
                Y_pred(t, :) = Y_norm .* sigmaT_train + muT_train;
                
                % Apply FILO filter
                if t <= filo_length
                    buffer(t, :) = Y_pred(t, :);
                    Y_filter(t, :) = mean(buffer(1:t, :), 1);
                else
                    buffer(1:filo_length-1, :) = buffer(2:filo_length, :);
                    buffer(end, :) = Y_pred(t, :);
                    Y_filter(t, :) = mean(buffer, 1);
                end
                
                % Reset network state for next prediction
                net_corridor_cbf.resetState();
            end
            
            % Plot full trajectory results
            figure('Name', 'Full Trajectory Prediction - Corridor');
            
            % Create color map for time steps
            c = linspace(1, 10, length(Y_filter));
            
            % Draw corridor obstacles first
            passage_width = 0.4;
            robotarium_height = 2.0;
            radius = (robotarium_height - passage_width) / 2;
            center_top_y = passage_width/2 + radius;
            center_bottom_y = -passage_width/2 - radius;
            
            theta_obs = linspace(0, 2*pi, 50);
            obs_top = [0 + radius*cos(theta_obs); center_top_y + radius*sin(theta_obs)];
            obs_bottom = [0 + radius*cos(theta_obs); center_bottom_y + radius*sin(theta_obs)];
            fill(obs_top(1,:), obs_top(2,:), 'k', 'FaceAlpha', 0.2);
            hold on;
            fill(obs_bottom(1,:), obs_bottom(2,:), 'k', 'FaceAlpha', 0.2);
            
            % Plot trajectories
            scatter(adv_test(offset+2:end, 1), adv_test(offset+2:end, 2), [], c, ...
                'DisplayName', 'Actual Adversary Positions');
            
            % Starting points
            scatter(adv_test(1, 1), adv_test(1, 2), 60, 'filled', 'd', ...
                'DisplayName', 'Adversary Start', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
            scatter(ally_test(1, 1), ally_test(1, 2), 60, 'filled', 'd', ...
                'DisplayName', 'Ally Start', 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
            
            % Trajectories
            plot(ally_test(:, 1), ally_test(:, 2), 'Color', [0 0.7 0], ...
                'LineWidth', 2, 'DisplayName', 'Ally Trajectory');
            plot(adv_test(:, 1), adv_test(:, 2), '--', ...
                'DisplayName', 'Adversary Actual Trajectory');
            
            % Predicted positions
            scatter(Y_filter(:, 1), Y_filter(:, 2), [], c, 'filled', ...
                'DisplayName', 'Predicted Adversary Positions');
            
            xlabel('X (m)');
            ylabel('Y (m)');
            xlim([-1.6, 1.6]);
            ylim([-1, 1]);
            
            colorbar;
            colormap('parula');
            cb = colorbar;
            cb.Label.String = 'Prediction Steps';
            cb.Limits = [1, 10];
            cb.Ticks = [1, 10];
            cb.TickLabels = {'1', num2str(numPredictionTimeSteps)};
            
            legend('Location', 'best');
            grid on;
            ax = gca;
            ax.FontSize = 12;
            title('Full Trajectory Prediction Results - Narrow Corridor');
            hold off;
            
            % Calculate trajectory prediction error
            actual_positions = adv_test(offset+2:offset+1+numPredictionTimeSteps, 1:2);
            trajectory_errors = sqrt(sum((Y_filter - actual_positions).^2, 2));
            
            fprintf('\nFull Trajectory Results:\n');
            fprintf('Mean Trajectory Error: %.4f m\n', mean(trajectory_errors));
            fprintf('Max Trajectory Error: %.4f m\n', max(trajectory_errors));
            fprintf('Final Position Error: %.4f m\n', trajectory_errors(end));
        end
    end
end

fprintf('\nTraining complete!\n');

%% Custom Training Monitor Function
function [stop, options] = customTrainingMonitor(trainingState, options)
    persistent bestValidationLoss bestEpoch epochsWithoutImprovement
    
    % Initialize persistent variables
    if isempty(bestValidationLoss)
        bestValidationLoss = inf;
        bestEpoch = 0;
        epochsWithoutImprovement = 0;
    end
    
    % Only execute during validation
    if ~isempty(trainingState.ValidationLoss)
        % Check current validation loss
        if trainingState.ValidationLoss < bestValidationLoss
            % Save best model
            bestValidationLoss = trainingState.ValidationLoss;
            bestEpoch = trainingState.Epoch;
            epochsWithoutImprovement = 0;
            
            % Save current best model
            save(fullfile(pwd, 'bestModelCheckpoint_corridor_cbf.mat'), 'trainingState');
            fprintf('Epoch %d: New best validation loss - %.6f\n', ...
                trainingState.Epoch, trainingState.ValidationLoss);
        else
            % Record epochs without improvement
            epochsWithoutImprovement = epochsWithoutImprovement + 1;
            fprintf('Epoch %d: Validation loss not improved (Best: %.6f, No improvement for %d epochs)\n', ...
                trainingState.Epoch, bestValidationLoss, epochsWithoutImprovement);
        end
        
        % Determine if training should stop
        if epochsWithoutImprovement >= 1000 && bestValidationLoss < 5
            fprintf('Early stopping triggered: Validation loss has not improved for %d epochs\n', ...
                epochsWithoutImprovement);
            stop = true;
            return;
        end
    end
    
    stop = false;
end