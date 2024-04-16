%%%%%%%%%%%%%
% ECE 3610
% Final -- PID-Based Line Following
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, you will be working in teams to create a PID feedback loop
% and use it to tune a robot to smoothly follow a black line on a white
% background. Line following will be a core part of your final project, so
% it's good to get some experience with it early!
%
% Deliverables:
%   - Demonstrate that your robot accurately and smoothly follows a provided line
%   without losing tracking.
%%%%%%%%%%%%%%

%% WIRING
% This lab has lots of wiring! Be sure to check out the wiring diagram on
% Canvas to map all of the wires where they need to go. Double check this
% before connecting the Arduino to power.

%% 1. CONNECT TO YOUR NANOBOT (SUPER SPECIAL WIFI EDITION)
% INIT SECTION. 
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

% NOTE: This is the first time we will be connecting to our robot via
% Wi-Fi. Make sure to change string to specify 'wifi' as the third parameter in
% the nanobot() function. Your port should be identical to the one you've
% been using in previous labs.
% Make sure you're connected to the Arduino's wifi network before you
% connect here.

clc
clear all
nb = nanobot('/dev/cu.usbmodem1301', 115200, 'serial');%nanobot('COM9', 115200, 'wifi');
% r1 = nanobot('/dev/cu.usbmodem143201', 115200, 'serial'); %connect to MKR
% r1.ledWrite(0);
%init ultrasonic sensors
nb.initUltrasonic1('D2','D3')
nb.initUltrasonic2('D4','D5')
%init colour sensors 
nb.initColor(); 

% RUN IF GOING TO USE CALIBRATED SENSOR VALUES - Calibrate Reflectance Minimums (white background)

nb.initReflectance();
vals = nb.reflectanceRead();
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n', vals.one, vals.two, vals.three, vals.four, vals.five, vals.six);
minReflectance  = [85,72,60,60,60,72]; % Set me to minimum reflectance values for each sensor
maxReflectance = [1602, 1266, 1161, 1058, 1219, 1750];

calibratedVals.one = 1000 * (vals.one - minReflectance(1)) / (maxReflectance(1) - minReflectance(1))
    calibratedVals.two = 1000 * (vals.two - minReflectance(2)) / (maxReflectance(2) - minReflectance(2))
    calibratedVals.three = 1000 * (vals.three - minReflectance(3)) / (maxReflectance(3) - minReflectance(3))
    calibratedVals.four = 1000 * (vals.four - minReflectance(4)) / (maxReflectance(4) - minReflectance(4))
    calibratedVals.five = 1000 * (vals.five - minReflectance(5)) / (maxReflectance(5) - minReflectance(5))
    calibratedVals.six = 1000 * (vals.six - minReflectance(6)) / (maxReflectance(6) - minReflectance(6))
% init gesture material 

%% here is the ML SECTION . 
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



 %% 2.a TESTING FOR ULTRASONIC SENSORS
%Test for ultra-sonic sensor

%Take a single ultrasonic reading
FrontUS = nb.ultrasonicRead1;
SideUS = nb.ultrasonicRead2;
fprintf('FrontUS = %i\n', FrontUS)
fprintf('SideUS = %i\n', SideUS)

%% LINE FOLLOWING PID LOOP

% nb.initReflectance();

% TUNING:
% Start small (ESPECIALLY with the reflectance values, error can range from zero to several thousand!)
% Tip: when tuning kd, it must be the opposite sign of kp to damp
kp = 0.001;
ki = 0.0000;
kd = -0.00;

% kp = 0.00006;
% ki = 0.0000;
% kd = -0.000005;

% Basic initialization
vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;
whiteThresh = 50;
blkThresh = 100;
turnOffset = 0

% Determine a threshold to detect when white is present on all sensors
% whiteThresh = '?'; % Max value detected for all white

% The base duty cycle "speed" you wish to travel down the line with
% (recommended values are 9 or 10)
% baseSpeed = 11;
baseSpeed = 9;
motor1BaseSpeed = baseSpeed;
motor2BaseSpeed = baseSpeed;
maxDuty = 17;

tic
nb.setMotor(1, 9);
nb.setMotor(2, 9.2);
pause(0.03);
black_tape_marker = 0;
while (toc < 120)  % Adjust me if you want to stop your line following earlier, or let it run longer.
    
    % TIME STEP
    dt = toc - prevTime;
    prevTime = toc;
    
    [calibratedVals, error] = getError(nb, minReflectance, maxReflectance);

    % Calculate P, I, and D terms
    integral = integral + (error*dt);

    derivative = (error - prevError)/dt;

    % Set PID
    control = kp*error + ki*integral + kd*derivative + turnOffset;

    % STATE CHECKING - stops robot if all sensors read white (lost tracking):
    if (calibratedVals.one < whiteThresh && ...
            calibratedVals.two < whiteThresh && ...
            calibratedVals.three < whiteThresh && ...
            calibratedVals.four < whiteThresh && ...
            calibratedVals.five < whiteThresh && ...
            calibratedVals.six < whiteThresh)
        % Stop the motors and exit the while loop
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        pause(.1)
        if prevError > 0
            nb.setMotor(2, -10);
        elseif prevError < 0
            nb.setMotor(1, -10.2);
        end
        % while abs(error)<2000
        %     [calibratedVals, error] = getError(nb, minReflectance, maxReflectance);;
        % pause(.01)
        % end
        % while abs(error)>500
        %     [calibratedVals, error] = getError(nb, minReflectance, maxReflectance);;
        % pause(.01)
        % end
        pause(.2);
    elseif (calibratedVals.one > blkThresh && ...
            calibratedVals.two > blkThresh && ...
            calibratedVals.three > blkThresh && ...
            calibratedVals.four > blkThresh && ...
            calibratedVals.five > blkThresh && ...
            calibratedVals.six > blkThresh && black_tape_marker)
        % Stop the motors and exit the while loop
             %mod here. 
        [R,G, B] = ColorSenseRGB(nb);
        % %check if color is detected. 
        colorDet =0;
          % %checking if red
        if ((R > 150) & (G >= 50) & (B >= 50))
            turn(nb, 60);
            forward(nb, 1.2)
            turn(nb, -180);
            forward(nb, .9);
            colorDet  = 1; 
        end
        if ((R >= 50) & (R < 90) & (G >= 90) & (B >= 90))
            turn(nb, -50);
            forward(nb, 1.2);
            turn(nb, 180);
            forward(nb, .9);
            colorDet = 1; 
        end 
         if (colorDet  == 1)
             fprintf("Red :%d Blue:%d Green:%G \n", R, G, B);
             state =0;
         else
             clc;
             fprintf("Red :%d Blue:%d Green:%G \n", R, G, B);
             fprintf("NADA, NIL, KAMPF\n");
             %forward(nb);
         end
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        break
    elseif (calibratedVals.one > blkThresh && ...
            calibratedVals.two > blkThresh && ...
            calibratedVals.three > blkThresh && ...
            calibratedVals.four > blkThresh && ...
            calibratedVals.five > blkThresh && ...
            calibratedVals.six > blkThresh && ~black_tape_marker)
        % Stop the motors and exit the while loop
        black_tape_marker = 1;
        nb.setMotor(1, 0);
        nb.setMotor(2,0);
        pause(.3)
        nb.setMotor(1, -10)
        nb.setMotor(2,10.2)
        pause(1.25)
        nb.setMotor(1,0)
        nb.setMotor(2,0)
        pause(0.3)
        nb.setMotor(1, 10);
        nb.setMotor(2,10.2);
    else
        % LINE DETECTED:
        %MODIFCATION HERE
         %this needs to be called everytime in execution. 

        %END OF MOD. 
        % Remember, we want to travel around a fixed speed down the line,
        % and the control should make minor adjustments that allow the
        % robot to stay centered on the line as it moves.
        m1Duty = motor1BaseSpeed+control+(turnOffset>0);
        m2Duty = motor2BaseSpeed-control+(turnOffset<0);
       
        if m1Duty > maxDuty
            m1Duty = maxDuty;
        elseif m1Duty < 8
            m1Duty = 8;
        end

        if m2Duty > maxDuty
            m2Duty = maxDuty;
        elseif m2Duty < 8
            m2Duty = 8;
        end
        % If you're doing something with encoders to derive control, you
        % may want to make sure the duty cycles of the motors don't exceed
        % the maximum speed so that your counts stay accurate.

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
        turnOffset = 0;
        prevError = error;
    end

    
 end
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% EMERGENCY MOTOR SHUT OFF
% If this doesn't work, turn off the power switch on your motor carrier
% board.

% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);
nb.initReflectance();
tic
while(toc<10)
% error = 6*vals.one + 1*vals.two + 0*vals.three - 0*vals.four - 1*vals.five - 6*vals.six;
error = getError(nb, minReflectance, maxReflectance)
end

%%
mOffScale = 1.2;
m1Duty = 9;
m2Duty = 9;
nb.setMotor(1, 15);
nb.setMotor(2, 15);
pause(.03);
nb.setMotor(1, m2Duty);
nb.setMotor(2, m2Duty);
pause(2);
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% testing wall following 
wallfollow(nb);


%% ML CODE execution loop. 
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
        r1 = nanobot('/dev/cu.usbmodem1401', 115200, 'serial'); %connect to MKR
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
while 1
    switch words
        case "0"
            fprintf("line following\n");
            %here is the numo numo. 
        case "1"
            fprintf("stopping\n");
            stop();
        case "2"
            fprintf("forward\n");
            foward(nb, 0.25);
            stop();
        case "3"
            fprintf("right 45\n");
            turn(nb, 45);
        case "4"
            fprintf("left 45\n");
            turn(nb, -45)
        otherwise
            fprintf("ERROR CRITICAL!\n");
        end 
        gest = 0;
        state = 0;
        end
    end   
end


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

%% Wall Following Function
function wallfollow(nb1)
    locState = 1;
    fprintf("hello working");
    while locState == 1
        if(nb1.ultrasonicRead1  < 500)
        turn(nb1, 50);
        forward(nb1, 1.2);
        pause(1);
        end
        stop(nb1)
        locState = 0;
    end
end
%% stop function 
function stop(nb1)
nb1.setMotor(1, 0);
nb1.setMotor(2, 0);
end

%% turn function
function  endDeg = turn(nb1, dege)
% here is the timing functiin =
endDeg = dege;
timeCal  = abs(dege * (1/120));
tic
    while toc < timeCal
        if  (dege > 0)
            nb1.setMotor(1, 10);
            nb1.setMotor(2, -10);
        end
        if(dege < 0)
            nb1.setMotor(1, -10);
            nb1.setMotor(2, 10);
        end
    end
    nb1.setMotor(1, 0);
    nb1.setMotor(2, 0);
end
%%
function [calVals, error] = getError(nb, minReflectance, maxReflectance)
vals = nb.reflectanceRead();
    calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0, 'five', 0, 'six', 0);
    calibratedVals.one = 1000 * (vals.one - minReflectance(1)) / (maxReflectance(1) - minReflectance(1))
    calibratedVals.two = 1000 * (vals.two - minReflectance(2)) / (maxReflectance(2) - minReflectance(2))
    calibratedVals.three = 1000 * (vals.three - minReflectance(3)) / (maxReflectance(3) - minReflectance(3))
    calibratedVals.four = 1000 * (vals.four - minReflectance(4)) / (maxReflectance(4) - minReflectance(4))
    calibratedVals.five = 1000 * (vals.five - minReflectance(5)) / (maxReflectance(5) - minReflectance(5))
    calibratedVals.six = 1000 * (vals.six - minReflectance(6)) / (maxReflectance(6) - minReflectance(6))
    calVals = calibratedVals;
    error = 3*calibratedVals.one + 2*calibratedVals.two + 1*calibratedVals.three - 1*calibratedVals.four - 2*calibratedVals.five - 3*calibratedVals.six
end
%% Color sensing helper function 
%functions are always on the bottom. 
%this was pulled from nanobot_demo.m
function [R, G, B] = ColorSenseRGB(nb1)
%Take a single RGB color sensor reading
values = nb1.colorRead();
%The sensor values are saved as fields in a structure:
R = values.red;
G = values.green;
B = values.blue;
end
%% forward motion
function forward(nb1, tim)
   % PID Tuning to match a desired RPM lab 11.
    maxEncoderDuty = 17;
    % Modify these (Tip: tune proportional first, then integral or derivative)
    %for motor 1. 
    kp = 3.9;%proportional gain
    ki = 0.3;           %integral gain
    kd = -0.07; %derivative gain

    %for motor 2. 
    kp2 = 3.9;
    ki2 = 0.3;
    kd2 = -0.07;

    rpm_targ = 100;  %The goal RPM

    %target RPM 
    target_rpm = 100;
    integral = 0;
    integral2 = 0;
    prevError = 0;
    prevError2 = 0;
    vals = 0;
    vals2 = 0;

    % For graphing
    rpms = 0;
    times = 0;
    rmps2 =0;
    times2 = 0;

    % Change me to change how long the program runs
    runtime = tim;

    vals = nb1.encoderRead(2);
    vals2 = nb1.encoderRead(2);
    tic
    pause(0.03); % Small delay to avoid initial case dt blowing up
    while toc < runtime
    pause(0.014);
    vals = nb1.encoderRead(1); % Get the counts since the last time we called
    vals2 = nb1.encoderRead(2);
   
    times(end+1) = toc; % Collect for graphing
    %times2(end+1) = toc; % collect for graphing 2
    dt = times(end) - times(end - 1); % Compute the time difference between the last loop and this one.
    dt2 = times(end) - times(end -1); % compute the time diffrence bwteen two for second 
    %Calculate RPM:
    % Hint 1: You can get your counts per second either by dividing your
    % counts since last read by dt, or use the val.countspersec
    % estimation multiplied by 100.

    % Hint 2: There are about 1440 encoder counts per revolution of the wheel 
    
    % Hint 3: Remember, we want this result in revolutions per minute (RPM)
    rpm = (abs(vals.counts*60)/(1440*dt));
    rpm2 = (abs(vals2.counts*60)/(1440*dt));
    
    rpms(end+1) = rpm; % Collect for graphing
    %rpms2(end+1) = rpm2;
    
    error = target_rpm - rpm; % Difference between our target and current rpm 
    error2 =  target_rpm - rpm2;
    integral = integral + (error * dt); % Running sum adding the error over the time step
    integral2 = integral + (error2* dt2);
    derivative = (error-prevError) / dt; % Rate of change estimate using the difference
    derivative2  =  (error-prevError) / dt2;                     % between the current and prior error over the
                                         % time between them
                                         
    prevError = error;
    prevError2 = error2;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control = error*kp + integral*ki + derivative*kd; % Put the error terms and coefficients together
                                                                  % into one control signal!
    control2 = error2*kp2 + integral2*ki2 + derivative2*kd2;
    % Caps the motor duty cycle at +/- max encoder duty cycle to prevent
    % aliasing
    if control > maxEncoderDuty
        control = maxEncoderDuty;
    end 
    if control < -maxEncoderDuty
        control = -maxEncoderDuty;
    end
     if control2 > maxEncoderDuty
        control2 = maxEncoderDuty;
    end 
    if control2 < -maxEncoderDuty
        control2 = -maxEncoderDuty;
    end
    nb1.setMotor(1, control); % Send the control signal to the motor.
    nb1.setMotor(2, control2);
    end
    stop(nb1);
end



