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
nb = nanobot('?', 115200, 'wifi');
%% 2. TESTING FOR ULTRASONIC SENSORS
%Test for ultra-sonic sensor
nb.initUltrasonic1('D2','D3')
%Take a single ultrasonic reading
FrontUS = nb.ultrasonicRead1();
fprintf('FrontUS = %i\n', FrontUS)

nb.initUltrasonic2('D4','D5')
%Take a single ultrasonic reading
SideUS = nb.ultrasonicRead2();
fprintf('SideUS = %i\n', SideUS)

%% TESTING FOR REFLECTANCE SENSORS
%Reflectance Array
%Initialize with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead();

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f, four: %.2f, five: %.2f, six: %.2f\n', val.one, val.two, val.three, val.four, val.five, val.six);

%% IR Testing
nb.initReflectance();
tic
while(toc < 10)
    val = nb.reflectanceRead();
    error = ((1)*val.one + (1)*val.two) - (0.002*val.three - 0.002*val.four) -  ((1)*val.five + (1)*val.six);
    fprintf("Error: %d\n", error);
    pause(0.5);
end


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
