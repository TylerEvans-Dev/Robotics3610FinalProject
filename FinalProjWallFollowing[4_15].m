%% init the arduino
r = nanobot('/dev/cu.usbmodem14101', 115200, 'wifi');

%% Motion testing

% move forward
r.setMotor(1,9.8) %right motor on Arduino side
r.setMotor(2,10) %left motor on Arduino side
pause(0.5)
r.setMotor(1,0)
r.setMotor(2,0)
pause(0.5)
% move backward
r.setMotor(1,-10)
r.setMotor(2,-10)
pause(0.5)
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
nb.init
frontUF_distance_threshold = 650;
leftUF_desired_distance = 20; 

while(toc < 50)
r.setMotor(1,9.8)
r.setMotor(2,10)
front_reading = nb.ultrasonicRead1(); 
left_reading = nb.ultrasonicRead2(); 
 if (front_reading < frontUF_distance_threshold) %Robot infront of wall
        if (left_reading > desired_distance) %too far from wall, turn right
            r.setMotor(1,9.8)
            r.setMotor(2,10)
        elseif (left_reading < desired_distance)%too close to wall, turn left
            r.setMotor(1,10)
            r.setMotor(2,9.8)
        else % left sensor reads at the desired distance, go straight
            r.setMotor(1,9.8)
            r.setMotor(2,10)
        end
 else
    % Front sensor doesn't detect the wall, move forward
    r.setMotor(1,9.8)
    r.setMotor(2,10)
end
end
pause(0.5)
r.setMotor(1,0)
r.setMotor(2,0)


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all