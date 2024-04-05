%PID Loops
% Setup gain variables
% Initialize integral, previous error, and previous time terms
% Initialize or perform calculations for targets/values your loop will be working with
% tic
% small delay to prevent toc from blowing up derivative term on first loop
% begin while
	% calculate dt based on toc and previous time
	% update previous time
	% take reading of data
	% Perform conditional checking whether we want to exit the loop (e.g. read all black on reflectance array during line following)
		% exit the loop if so, and perform any needed end behaviors (e.g. setting motors to zero)
	% calculate error term
	% use error to calculate/update integral and derivative terms
	% calculate control signal based on error, integral, and derivative
	% update previous error
	% calculate desired setpoint quantities from control signal
	% apply limits to setpoints if needed
	% apply setpoint to plant (e.g. setting duty cycle for motors)

%Gesture Classification
% initialize variable to track whether we’ve given valid input
% define list of possible output values, in the order that the network was trained on
% while we haven’t given valid input
	% perform gesture classification as we had done in labs 4 and 5 (outputs index of classified output according to the output value list)
	% let user know what was gesture was classified as, and ask to verify
	% perform gesture classification again
	% check if the first gesture was verified according to the second gesture.
		% if so, set valid input variable such that we exit the loop on the next iteration
		% if not, proceed to the next iteration of the loop
%% Used data from Intelligence Lab 4: Introduction to Convolutional Neural Networks & Network Architecture

% the goal of this lab is to train a neural network to detect numbers gestured
% with an accelerometer. At a minimum you need to be able to reliably
% distinguish between ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT,
% and NINE with at least 10% accuracy. You will be responsible for
% modifying the neural network architecture to achieve this goal

clear; clc; close all; %initialization
r = nanobot('/dev/cu.usbmodem11301', 115200, 'serial'); %connect to MKR
r.ledWrite(0);

%% Specify initial parameters:
trialCount = 50; % specify how many times you will make each gesture
digits = [0:6]; % specify which digits you will be gesturing
gesturechar = ['1', '0' ,'_', '\', '/', 'y', 'n'];
numreads = 100; % about 1.5 seconds (on serial)
Trainvals = zeros(3,numreads);
%% Collect Multiple Gesture
gestureCount = length(digits); % determine the number of gestures
data = cell(gestureCount, trialCount+1); % preallocate cell array
for i = 1:gestureCount
    data{i,1} = digits(i);
end

countdown("Beginning in", 3); %display a countdown to prep user to move accelerometer 

for a = 1:gestureCount % iterate through all the gestures
    b = 1; %index for trials
    while b <= trialCount % iterate through all the trials
        fprintf("Draw a %c (%d of %d)",gesturechar(a), b, trialCount); % Displays the prompt in the command window
        r.ledWrite(1);       % Begin recording data

        % Gesture is performed during the segement below
        for i = 1:numreads
            valr = r.accelRead();
            Trainvals(1,i) = valr.x;
            Trainvals(2,i) = valr.y;
            Trainvals(3,i) = valr.z;
        end

        r.ledWrite(0);          % Set the LED to red          % Set the LED to red
        try
            data{a,b+1} = [Trainvals(1,:);Trainvals(2,:);Trainvals(3,:)]; % Truncate the data
            b = b + 1;
        catch
            disp("Data capture failed. Trying again in 3 seconds")
            pause(3);
        end
        clc; % clear the command line
        pause(1); % wait one second
    end
end

pause(1); % wait a second
clc; % clear command line

if menu("Would you like to save the dataset you just recorded?", "Yes", "No") == 1
    t = clock;
    filename = sprintf("%d%d%d_%d%d%d_TrainingSet_%dGestures%dTrials",t(1),t(2),t(3),t(4),t(5),round(t(6)),height(data),width(data)-1);
    save(filename, "data");
end

%% Store Data as "Images" for Neural Network
gestureCount = height(data);
trialCount = width(data)-1;

TrainingFeatures = zeros(3,100,1,gestureCount*trialCount); %features are stored as a stack of 3D images (4D array)
labels = zeros(1,gestureCount*trialCount); %labels are stored as a simple number (1D array)

k=1; %simpler counter
for a = 1:gestureCount %iterate through gestures
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = data{a,b+1}; %put data into image stack
        labels(k) = data{a,1}; %put each label into label stack
        k = k + 1; %increment
    end
end
labels = categorical(labels); %convert labels into categorical

% Split Training and Testing Data
selection = ones(1,gestureCount*trialCount); %allocate logical array
selectionIndices = []; %initialization
for b = 1:gestureCount %save 1/4 of the data for testing
    selectionIndices = [selectionIndices,  round(linspace(1,trialCount,round(trialCount/4))) + (trialCount*(b-1))];
end
selection(selectionIndices) = 0; %set logical to zero

%training data
xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on
%testing data
xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on

% Define Neural Network

[inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

%%%%%%%%%%%%%%%%%%%%%% YOUR MODIFICATIONS GO HERE %%%%%%%%%%%%%%%%%%%%%%%%%%

learnRate = 0.02989; %how quickly network makes changes and learns
maxEpoch = 100; %how long the network learns

layers= [ ... %NN architecture for a conv net
    imageInputLayer([inputsize1,inputsize2,1])
    convolution2dLayer([2,40],38) %size of convolution (e.g., 3x10 means across all accelerometers and 10 time steps)
    batchNormalizationLayer
    reluLayer
    convolution2dLayer([2,40],38) %size of convolution (e.g., 3x10 means across all accelerometers and 10 time steps)
    batchNormalizationLayer
    reluLayer
    fullyConnectedLayer(70)
    dropoutLayer(0.44) %rate of dropout (e.g., 0.01 = 1%). 
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%

options = trainingOptions('sgdm','InitialLearnRate', learnRate, 'MaxEpochs', maxEpoch,...
    'Shuffle','every-epoch','Plots','training-progress', 'ValidationData',{xTest,yTest}); %options for NN
% Train Neural Network
[myNeuralNetwork, info] = trainNetwork(xTrain,yTrain,layers,options); %output is the trained NN


%% Test Neural Network

t = 1:length(info.TrainingAccuracy);
figure();
subplot(2,2,1);
plot(info.TrainingAccuracy,'LineWidth',2,'Color',"#0072BD");
hold on;
plot(t(~isnan(info.ValidationAccuracy)),info.ValidationAccuracy(~isnan(info.ValidationAccuracy)),'--k','LineWidth',2,'Marker','o');
title("Training Accuracy")
legend("Training Accuracy","Validation Accuracy");
xlabel("Iterations");
ylabel("Accuracy (%)");

subplot(2,2,3);
plot(info.TrainingLoss,'LineWidth',2,'Color',"#D95319");
hold on;
plot(t(~isnan(info.ValidationLoss)),info.ValidationLoss(~isnan(info.ValidationLoss)),'--k','LineWidth',2,'Marker','o');
title("Training Loss")
legend("Training Loss","Validation Loss");
xlabel("Iterations");
ylabel("RMSE");

predictions = classify(myNeuralNetwork, xTest)'; %classify testing data using NN
disp("The Neural Network Predicted:"); disp(predictions); %display predictions
disp("Correct Answers"); disp(yTest); % display correct answers
subplot(2,2,[2,4]); confusionchart(yTest,predictions); % plot a confusion matrix
title("Confusion Matrix")

% View Neural Network

figure(); plot(myNeuralNetwork); % visualize network connections

%% Run-Time Predictions (copy of lab 1 code with determiation replaced with nn code)

% make sure NN exists
if(~exist('myNeuralNetwork'))
    error("You have not yet created your neural network! Be sure you run this section AFTER your neural network is created.");
end

% collect gesture
% collect gesture
clear r;
r = nanobot('/dev/cu.usbmodem11301', 115200, 'serial'); %connect to MKR
r.ledWrite(0);
pause(.5);
countdown("Beginning in", 3);
disp("Make A Gesture!");
r.ledWrite(1); % Begin recording data
numreads = 100; % about 1.5 seconds (on serial)

% Gesture is performed during the segement below
for i = 1:numreads
    valr = r.accelRead();
    Trainvals(1,i) = valr.x;
    Trainvals(2,i) = valr.y;
    Trainvals(3,i) = valr.z;
end

r.ledWrite(0); % Set the LED to red

rtdata = [Trainvals(1,:);Trainvals(2,:);Trainvals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,100,1,1);
xTestLive(:,:,1,1) = rtdata;

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);

% Plot with label
display(prediction);

   
figure(); plot(rtdata', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(prediction)); %title plot with the label



%% Line Following
% Define a motor speed compensation factor if needed
% Set up PID loop
	% After reading data from the necessary sensors, you should check important states for the robot (e.g. reading all black, meaning a stop was encountered, or all white, meaning you lost the line) and handle them appropriately.
	% Error should be calculated such that the robot seeks to center the reflectance array on the line
	% The robot should move at a specified speed forward, and use the control signal adjust the speed to each motor to turn and keep itself on the line

%% Wall Following
%(True distance-maintaining)
% Set up PID loop
	% Check for all black condition (made a complete circle)
	% Calculate error as the difference between the distance you intend to follow and the measured ultrasonic distance
	% Use prior values of error and/or motor setpoints/encoder rates to determine if the robot is reading a point in front of or behind it, and correct the control signal/motor setpoints accordingly to stabilize.

%OR

%(Hacky solution)
% Move forward while reading sensors
	% If reflectance is all black, exit loop
	% if the side ultrasonic reading exceeds a certain value
		% turn by a specified amount (or turn slowly, reading the ultrasonic until you read a new value greater than or equal to the current exceeded value), then proceed

%% Odometry
%(Straight-line)
% Set up PID loop
	% Using measurements of wheel geometry, calculate the needed encoder counts that each motor needs to go
	% Define a default speed for the robot to move at
	% Calculate the error as being the difference between the motor encoder rates (either countspersec or counts divided by a measured dt)
	% Keep track of the total number of encoder counts for one or both motors and exit the loop and stop the motors once you reach the number of encoder counts needed to travel a certain distance.
		% Alternatively, you could just drive and continue until another condition is met (e.g. the RGB sensor reads red)

%(Angular)
% Set up PID loop
	% Using measurements of wheel geometry, calculate the needed encoder counts that each motor needs to go
	% Similarly to the straight line case, we can monitor the encoder counts/rates until we match the needed value, or come within a certain threshold of it.
%% END