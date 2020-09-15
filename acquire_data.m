function z = acquire_data(num_samples)
    % Clear any existing serial ports
    delete(instrfind);
    
    % Create a serial port object
    serialPort = serial('COM4', 'BaudRate', 256000, 'DataBits', 8, 'Parity', 'none', 'StopBit', 1);
    fopen(serialPort);

    fwrite(serialPort, 'a', 'char', 'async');
    
    %measurement data
    z = zeros(6, num_samples);
    
    for k = 1 : num_samples
        %read data from sensors
        %data rate = 100 Hz
        %Make sure blue LED is on, or else we're losing data
        ax = str2double(fgetl(serialPort));
        ay = str2double(fgetl(serialPort));
        az = str2double(fgetl(serialPort));
        mx = str2double(fgetl(serialPort));
        my = str2double(fgetl(serialPort));
        mz = str2double(fgetl(serialPort));

        z(1, k) = ax;
        z(2, k) = ay;
        z(3, k) = az;
        z(4, k) = mx;
        z(5, k) = my;
        z(6, k) = mz;
    end
    fclose(serialPort);
end