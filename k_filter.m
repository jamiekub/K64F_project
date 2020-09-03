%clear existing variables, close any figure windows, clear the command line
close all
clear all
clc
% Clear any existing serial ports
delete(instrfind);

% Create a serial port object
serialPort = serial('COM4', 'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none', 'StopBit', 1);
fopen(serialPort);

for n = 0 : 200
%read data from sensors
%data rate = 6.25 Hz
%Make sure blue LED is on, or else we're losing data
accel_x = str2double(fgetl(serialPort));
accel_y = str2double(fgetl(serialPort));
accel_z = str2double(fgetl(serialPort));
magn_x = str2double(fgetl(serialPort));
magn_y = str2double(fgetl(serialPort));
magn_z = str2double(fgetl(serialPort));

%Convert to mg
accel_x = accel_x * 0.488
accel_y = accel_y * 0.488
accel_z = accel_z * 0.488
%Convert to uT
magn_x = magn_x * 0.1
magn_y = magn_y * 0.1
magn_z = magn_z * 0.1


end
fclose(serialPort);