%clear existing variables, close any figure windows, clear the command line
close all
clear all
clc
% Clear any existing serial ports
delete(instrfind);

sample = 1000;

accel_x = zeros(1, sample);
accel_y = zeros(1, sample);
accel_z = zeros(1, sample);
magn_x = zeros(1, sample);
magn_y = zeros(1, sample);
magn_z = zeros(1, sample);

% Create a serial port object
serialPort = serial('COM4', 'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none', 'StopBit', 1);
fopen(serialPort);

for n = 1 : sample
%read data from sensors
%data rate = 6.25 Hz
%Make sure blue LED is on, or else we're losing data
ax = str2double(fgetl(serialPort));
ay = str2double(fgetl(serialPort));
az = str2double(fgetl(serialPort));
mx = str2double(fgetl(serialPort));
my = str2double(fgetl(serialPort));
mz = str2double(fgetl(serialPort));

accel_x(n) = ax;
accel_y(n) = ay;
accel_z(n) = az;
magn_x(n) = mx;
magn_y(n) = my;
magn_z(n) = mz;
n
end
fclose(serialPort);

figure
histogram(accel_x);
figure
histogram(accel_y);
figure
histogram(accel_z);
figure

histogram(magn_x);
figure
histogram(magn_y);
figure
histogram(magn_z);