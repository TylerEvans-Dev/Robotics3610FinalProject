%%%%%%%%%%%%%
% ECE 3610
% FINAL PROJECT
%%%%%%%%%%%%%

%NOTES[4|1]
% - Connected Reflectance Sensor and Ultrasonic sensor to robot
% - Ultrasonic front working (D3,D2)
% - Ultrasonic front working (D4,D5)

%%%%%%%%%%%%%%

%% 1. CONNECT TO NANOBOT
clc
clear all
nb = nanobot('COM5', 115200, 'serial');
%% 2.a TESTING FOR ULTRASONIC SENSORS
%Test for ultra-sonic sensor
nb.initUltrasonic('D2','D3')
%Take a single ultrasonic reading
val = nb.ultrasonicRead();
fprintf('val = %i\n', val)
%% 2.b TESTING FOR REFLECTANCE SENSORS
%Reflectance Array
%Initialize with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead;

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', val.one, val.two, val.three, val.four);

%% 3. DC MOTOR TESTING BASED ON SENSOR & LINE DETECTION
%  Using readings from reflectance array to change direction of your DC motor
%  based on the the amount of error.

motor1BaseSpeed, motor2BaseSpeed = 9; % set to minimum duty cycle
maxDuty = 15; %maximum Duty cycle

tic

nb.setMotor(1, 10);
nb.setMotor(2, 10);
pause(0.03);

%PID Params
kp=0.000003;
ki=0; %leave at zero for testing
kd=0; %leave at zero for testing
prevError = 0;

while(toc < 10)
   

    %TO-DO: Refactor PID code for real code
    val = nb.reflectanceRead();
    error = -3*val.one-2*val.two-val.three+val.four+2*val.five+3*val.six;
    stopCheck = -3*val.one + 3*val.six; 

    integral = integral + error * dt;
    derivative = (error - prevError) / dt;
    control = kp*error + ki*integral + kd*derivative;
    
    %Error should be 0 when perfectly centered on line
    if(stopCheck<5 && stopCheck>-5)
        nb.setMotor(1,0); % set right motor to 0
        nb.setMotor(2,0); %set left motor to 0
        fprintf('Break out of while loop');
        break;
    else
        RightMotorDuty = motor1BaseSpeed - control;
        LeftMotorDuty = motor2BaseSpeed + control;

        if RightMotorDuty > maxDuty
            RightMotorDuty = maxDuty;
        elseif RightMotorDuty < 6
            RightMotorDuty = 6;
        end

        if LeftMotorDuty > maxDuty
            LeftMotorDuty = maxDuty;
        elseif LeftMotorDuty < 6
            LeftMotorDuty = 6;
        end

        nb.setMotor(1,RightMotorDuty); % set right motor
        nb.setMotor(2,LeftMotorDuty); %set left motor
    end
   prevError = error;
end

%% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0)

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all