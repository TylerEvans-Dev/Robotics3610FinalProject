%% init the arduino
clear all 
clc
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');

%% Motor testing

% move forward
nb.setMotor(1,10.3) %right motor on Arduino side
nb.setMotor(2,10) %left motor on Arduino side
pause(1)
nb.setMotor(1,0)
nb.setMotor(2,0)
pause(0.5)
% move backward
nb.setMotor(1,-10)
nb.setMotor(2,-10)
pause(1)
nb.setMotor(1,0)
nb.setMotor(2,0)
pause(0.5)
% turn around
nb.setMotor(1,10)
nb.setMotor(2,-10)
pause(1.25)
nb.setMotor(1,0)
nb.setMotor(2,0)
%% checking the len of the wire. 
tic 
while (toc < 3)
    nb.setMotor(1, 10);
    nb.setMotor(2, 10);
end
nb.setMotor(1,0)
nb.setMotor(2,0)
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

motor1BaseSpeed = 10;
motor2BaseSpeed = 10; % set to minimum duty cycle
maxDuty = 10; %maximum Duty cycle
nb.initReflectance();

tic

% nb.setMotor(1, 10);
% nb.setMotor(2, 10);
% pause(0.03);

%PID Params
kp=3.9;
ki=2.80; %leave at zero for testing
kd=-2.50; %leave at zero for testing
prevError = 0;
prevTime = 0;
intgral = 0;

while(toc < 3.5)
   dt = toc - prevTime;
   prevTime = toc;

    %TO-DO: Refactor PID code for real code
    val = nb.reflectanceRead()
    error = (-3*val.one) + (-2*val.two) + (-1* val.three) + val.four+ (2*val.five)+ (3*val.six);
    %stopCheck = -3*val.one + 3*val.six; 

    intgral = intgral + (error * dt);
    derivative = (error - prevError) / dt;
    control = (kp*error) + (ki*intgral) + (kd*derivative);
    
    %Reverse on all white
    % if(val.one<100 && val.two<100 && val.three<100 && val.four<100 && val.five<100 && val.six<100 ) 
    %     nb.setMotor(1,-10)
    %     nb.setMotor(2,-10*1.2)
    %     pause(1)
    %     nb.setMotor(1,0)
    %     nb.setMotor(2,0)
    %     pause(0.5)
    % else
        RightMotorDuty =  motor1BaseSpeed + control;
        LeftMotorDuty = motor1BaseSpeed - control;
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

         nb.setMotor(2,RightMotorDuty*.98); % set right motor
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
