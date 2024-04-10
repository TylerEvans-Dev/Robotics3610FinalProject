%% init the arduino
clear all 
clc
nb = nanobot('/dev/cu.usbmodem14101', 115200, 'wifi');

%% Motor testing

% move forward
r.setMotor(1,10.3) %right motor on Arduino side
r.setMotor(2,10) %left motor on Arduino side
pause(1)
r.setMotor(1,0)
r.setMotor(2,0)
pause(0.5)
% move backward
r.setMotor(1,-10)
r.setMotor(2,-10)
pause(1)
r.setMotor(1,0)
r.setMotor(2,0)
pause(0.5)
% turn around
r.setMotor(1,10)
r.setMotor(2,-10)
pause(1.25)
r.setMotor(1,0)
r.setMotor(2,0)

%% Wall following
nb.initUltrasonic2('D4','D5')
nb.initReflectance();
while(toc < 50)
r.setMotor(1,9.8)
r.setMotor(2,10)
US_reading = nb.ultrasonicRead2();
reflect_reading = nb.reflectanceRead();
if (reading < 650)
    r.setMotor(2,9.8)
end
if(reflect_reading < 5 && reflect_reading > -5)
    break
end
end
pause(0.5)
r.setMotor(1,0)
r.setMotor(2,0)

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
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f, five: %.2f, six: %.2f\n' , val.one, val.two, val.three, val.four, val.five, val.six);
%% Test Error
nb.initReflectance();
while(1)
    val=nb.reflectanceRead();
    error = (-3*val.one)-(2*val.two)-(val.three)+(val.four)+(2*val.five)+(3*val.six);
    %fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f, five: %.2f, six: %.2f\n' , val.one, val.two, val.three, val.four, val.five, val.six);
    fprintf("Error: %d\n", error);
    pause(.5);
end
%% 3. DC MOTOR TESTING BASED ON SENSOR & LINE DETECTION
%  Using readings from reflectance array to change direction of your DC motor
%  based on the the amount of error.

motor1BaseSpeed = 9;
motor2BaseSpeed = 9; % set to minimum duty cycle
maxDuty = 12; %maximum Duty cycle
nb.initReflectance();

tic

% nb.setMotor(1, 10);
% nb.setMotor(2, 10);
% pause(0.03);

%PID Params
kp=0.01;
ki=0; %leave at zero for testing
kd=0; %leave at zero for testing
prevError = 0;
prevTime = 0;
intgral = 0;

while(toc < 5)
   dt = toc - prevTime;
   prevTime = toc;

    %TO-DO: Refactor PID code for real code
    val = nb.reflectanceRead()
    error = -3*val.one -2*val.two -val.three + val.four+2*val.five+ 3*val.six;
    stopCheck = -3*val.one + 3*val.six; 

    intgral = intgral + (error * dt);
    derivative = (error - prevError) / dt;
    control = abs(kp*error + ki*intgral + kd*derivative);
    
    %Reverse on all white
    % if(val.one<100 && val.two<100 && val.three<100 && val.four<100 && val.five<100 && val.six<100 ) 
    %     nb.setMotor(1,-10)
    %     nb.setMotor(2,-10*1.2)
    %     pause(1)
    %     nb.setMotor(1,0)
    %     nb.setMotor(2,0)
    %     pause(0.5)
    % else
        RightMotorDuty = 2+ motor1BaseSpeed + control;
        LeftMotorDuty = 2+ motor1BaseSpeed - control;
        fprintf('error: %.2d, control: %.2f, rightMotor: %.2f, leftMotor: %.2f\n',error,control, RightMotorDuty,LeftMotorDuty);

        if RightMotorDuty > maxDuty
            RightMotorDuty = maxDuty;
        elseif RightMotorDuty < motor2BaseSpeed
            RightMotorDuty = motor2BaseSpeed;
        end

        if LeftMotorDuty > maxDuty
            LeftMotorDuty = maxDuty;
        elseif LeftMotorDuty < motor1BaseSpeed
            LeftMotorDuty = motor1BaseSpeed;
        end

        nb.setMotor(2,RightMotorDuty*1.03); % set right motor
        nb.setMotor(1,LeftMotorDuty); %set left motor
    % end
   prevError = error;
end

nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all