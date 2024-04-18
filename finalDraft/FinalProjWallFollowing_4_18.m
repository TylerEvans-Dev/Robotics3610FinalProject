%% init the arduino
%r = nanobot('/dev/cu.usbmodem14101', 115200, 'wifi');
clc
clear all
r = nanobot('COM4', 115200, 'serial');
%% init ultrasonic sensors
r.initUltrasonic1('D2','D3');
r.initUltrasonic2('D4','D5');

frontVal = r.ultrasonicRead1();
fprintf('Front val = %i\n', frontVal);
sideVal = r.ultrasonicRead2();
fprintf('Side val = %i\n', sideVal);

%% TESTING
%%turn right and move forward
    fprintf("Turning right and moving\n");
    r.setMotor(1, 0);
    r.setMotor(2, 12);
    pause(0.5);
    r.setMotor(1,0); 
    r.setMotor(2,0);
    r.setMotor(1,10.5) ;
    r.setMotor(2,9);
    pause(1); 
    r.setMotor(1,0); 
    r.setMotor(2,0);

%% move forward for short time
    fprintf("Move forward\n");
    r.setMotor(1,10.5) ;
    r.setMotor(2,9);
    pause(0.5);
    r.setMotor(1, 0);
    r.setMotor(2, 0);
    pause(1);
%% turn left and move forward
    fprintf("Turning right and moving\n");
    r.setMotor(1, 12);
    r.setMotor(2, 0);
    pause(0.5);
    r.setMotor(1,0); 
    r.setMotor(2,0);
    r.setMotor(1,10.5) ;
    r.setMotor(2,9);
    pause(0.5); 
    r.setMotor(1,0); 
    r.setMotor(2,0);

%% ACTUAL TEST
% The process is that when the wall is sensed,an acknowledgement is printed out and
% then it waits for one second before turning right and moving forward. It then senses if
% it's far enough from the wall and turn's left if so, otherwise it goes straight

frontValThresh = 750;
sideValThresh = 500;
firstRightTurn = true;
tic

while(toc<30)
   if(frontVal < frontValThresh)
       %start process
       if(firstRightTurn)
           fprintf("Begin turning right\n");
           pause(2.5);
           %begins right turn process
            fprintf("Turning right and moving\n");
            r.setMotor(1, 0);
            r.setMotor(2, 12);
            pause(0.5);
            r.setMotor(1,0); 
            r.setMotor(2,0);
            r.setMotor(1,10.5) ;
            r.setMotor(2,9);
            pause(1); 
            r.setMotor(1,0); 
            r.setMotor(2,0);
            firstRightTurn = false;
       end
       if(sideVal > sideValThresh)
           %slight left turn and move forward short
       else
           %move forward
       end
   end
end
