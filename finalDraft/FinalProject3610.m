%% Final project Working code 

%% init gesture and Robot. 

%init robot
clc
clear all
% Create an instance of the nanobot class
%network name ECE3610k22
%password ece3610k22
 %nb = nanobot('COM49', 115200, 'wifi');
%here is for the hand gesture functions
r1 = nanobot('/dev/cu.usbmodem101', 115200, 'serial'); %connect to MKR
r1.ledWrite(0);
%init ultrasonic sensors
%nb.initUltrasonic1('D2','D3')
%nb.initUltrasonic2('D4','D5')
%init colour sensors 
%//nb.initColor(); 
% init gesture material 
[file, path] = uigetfile('.mat');
load(fullfile(path,file));

[h, w] = size(data);
gestureCount = h;
trialCount = w-1;
digits = [data{:,1}];

% Store Data as "Images" for Neural Network
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
% Store Data as "Images" for Neural Network
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


%% Main execution loop. 
state = 1; 
degree = 0;
readVal = 0; 
gest =  1;
while(state == 1)
    
    %fprintf( "Working here\n");
    %check gesture
    %if linefollowing do linefollowing function here. 
    %in linefollowing check if gesture motion is checked if so trigger a
    %stop  current action and check in motion and proceeed. 
    %check if wall following needed. 
    %forward(nb);
    %//pause(1);
    %//stop(nb);
    if (gest == 1)
        % make sure NN exists
        if(~exist('myNeuralNetwork'))
        error("You have not yet created your neural network! Be sure you run this section AFTER your neural network is created.");
        end

        % collect gesture
        % collect gesture
        clear r1;
        r1 = nanobot('/dev/cu.usbmodem101', 115200, 'serial'); %connect to MKR
        r1.ledWrite(0);
        pause(.5);
        countdown("Beginning in", 3);
        disp("Make A Gesture!");
        r1.ledWrite(1); % Begin recording data
        numreads = 100; % about 1.5 seconds (on serial)

        % Gesture is performed during the segement below
    for i = 1:numreads
        valr = r1.accelRead();
        Trainvals(1,i) = valr.x;
        Trainvals(2,i) = valr.y;
        Trainvals(3,i) = valr.z;
    end

    r1.ledWrite(0); % Set the LED to red

    rtdata = [Trainvals(1,:);Trainvals(2,:);Trainvals(3,:)];

    % put accelerometer data into NN input form
    xTestLive = zeros(3,100,1,1);
    xTestLive(:,:,1,1) = rtdata;

    %  Prediction based on NN
    prediction = classify(myNeuralNetwork,xTestLive);

    % Plot with label
    %display(prediction);

    % 
    % figure(); plot(rtdata', 'LineWidth', 1.5); %plot accelerometer traces
    % legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
    % title("Classification:", string(prediction)); %title plot with the label
    words = string(prediction);
    yesNo = string(prediction);

    switch words
        case "0"
            fprintf("line following\n");
        case "1"
            fprintf("stopping\n");
        case "2"
            fprintf("forward\n");
        case "3"
            fprintf("right 45\n");
        case "4"
            fprintf("left 45\n");
        otherwise
            fprintf("ERROR CRITICAL!\n");
        end 
        gest = 0;
        state = 0;
        end
    
end
%% Line following function 
%LINEF
function outputLineState = lineFollow()
    outputLineState = 0;
end
%% Forward movment function 

function [] = forward(nb1)
    tic 
    while(toc < 3)
        nb1.setMotor(1,  10);
        nb1.setMotor(2,  10);
    end
end
%% backwords movment function
function [] = backwords(nb1)
    while(toc < 3)
            nb1.setMotor(1,  10);
            nb1.setMotor(2,  10);
      end
end
 %% turn function 
 function [outDeg] = turn(degreeGoTo, currentDeg)
 
 end
 %% stop function 

     function [] = stop(nb1)
    nb1.setMotor(1, 0);
    nb1.setMotor(2, 0);
 end
%% Color sensing helper function 
%functions are always on the bottom. 
%this was pulled from nanobot_demo.m
function [R, G, B] = ColorSenseRGB()
%init the colour sensor 
nb.initColor();
%Take a single RGB color sensor reading
values = nb.colorRead();
%The sensor values are saved as fields in a structure:
R = values.red;
G = values.green;
B = values.blue;

end
%% Wall Following Function

function degFinal = wallfollow(currentDeg)


end
%% attempt to center 
function []  =  center()

end
%% gesture init 
function gestInit()
[file, path] = uigetfile('.mat');
load(fullfile(path,file));

[h, w] = size(data);
gestureCount = h;
trialCount = w-1;
digits = [data{:,1}];
end
%% Gesture contorl function. 
function [output]  = GestureCon()


end
%% check motion function
function [] = checkMotion

end
