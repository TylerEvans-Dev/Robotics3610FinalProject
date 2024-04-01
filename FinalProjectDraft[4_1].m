%%%%%%%%%%%%%
% ECE 3610
% FINAL PROJECT
%%%%%%%%%%%%%

%NOTES[4|1]
% - Connected Reflectance Sensor and Ultrasonic sensor to robot
% - 
% -

%%%%%%%%%%%%%%

%% 1. CONNECT TO NANOBOT
clc
clear all
nb = nanobot('COM5', 115200, 'serial');
%% 2. TESTING FOR ULTRASONIC SENSORS
%Test for ultra-sonic sensor
nb.initUltrasonic('D8','D7')
%Take a single ultrasonic reading
val = nb.ultrasonicRead();
fprintf('val = %i\n', val)
%% TESTING FOR REFLECTANCE SENSORS
%Reflectance Array
%Initialize with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead;

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', val.one, val.two, val.three, val.four);

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all