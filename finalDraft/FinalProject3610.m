%% Final project Working code 

%% init gesture and Robot. 

%init robot
clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM49', 115200, 'wifi');
%here is for the hand gesture functions
r = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); %connect to MKR
r.ledWrite(0);

%% Main execution loop. 
state = 1; 
degree = 0;
while(state == 1)
    fprintf( "Working here\n");
    %check gesture
    %if linefollowing do linefollowing function here. 
    %in linefollowing check if gesture motion is checked if so trigger a
    %stop  current action and check in motion and proceeed. 
    %
    %check if wall following needed. 
    
end
%% Line following function 
%LINEF
function [outputLineState] = lineFollow()
    outputLineState = 0;
end
%% Forward movment function 

function [] = foward()
    nb.setMotor(1,  10);
    nb.setMotor(2,  10);
end
 %% turn function 
 function [outDeg] = turn(degreeGoTo, currentDeg)
 
 end
 %% stop function 

 function [] = stop()
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
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

function [degFinal] = wallfollow(currentDeg)


end
%% attempt to center 
function []  =  center()

end
%% Gesture contorl function. 

function []  = GestureCon()

%GESTURE STUFF here. 
end
%% check motion function
function [] = checkMotion

end