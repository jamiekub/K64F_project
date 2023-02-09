# K64F_project
Experimentation with obtaining orientation through sensor fusion of accelerometer and magnetometer

1. Intereface with NXP FXOS8700CQ 3D accelerometer and magnetometer on FRDM K64F development board
    a. Setup development environment (git repo, uvision project, Matlab license)
	b. Copy initialization code for UART, LEDs, and buttons from IDE labs
	c. Research how to initialize accelerometer and magnetometer gpio/i2c ports and access the data
	d. Send data over UART to verify sensors work
2. Develope Kalman filter for sensor fusion and orientation estimation
	a. Use Matlab to receive sensor data
	b. Create filter
	c. Tune filter
	d. Port filter code to run on uController
3. Characterize Error of Kalman filter
