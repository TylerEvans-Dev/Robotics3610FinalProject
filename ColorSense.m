%% init
% clear; clc; close all; %initialization
nb = nanobot('/dev/cu.usbmodem11301', 115200, 'serial'); %connect to MKR
nb.ledWrite(0);
%% Testing func.
while (1)
    %this needs to be called everytime in execution. 
    Color = ColorSenseRGB();
    if(R > 250)
        fprintf("RED MELON HEADS!\n");
    end
    if(G > 250)
        fprintf("Greeny beany!\n");
    end
    if(B > 250)
        fprintf("Blue the Hue!\n");
    end
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

%% end of code
