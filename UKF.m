function [x_post, P_post] = UKF(sample, z_data) %#codegen
    r_acc = 0.0001; %accelerometer covariance
    r_mag = 0.00001; %magnetometer covariance
    q_0 = 0.01; %initial motion model covariance
    epsilon = 0.1;
    lambda = 0; %spread of sigma points
    alpha = 0.3; %initial lowpass filter factor
    beta = 0.5; %final lowpass filter factor
    zeta = 100; %adaptive factor
    tau = 0.3; %angular velocity estimate factor
    g = [0; 0; 1000]; %gravitational field vector (mg)
    B = 51.25; %magnetic field intensity at 40 41' 30" N, 74 39' 12" W (uT)
    delta = -65; %magnetic field inclination at 40 41' 30" N, 74 39' 12" W (deg)
    b = [0; B * cosd(delta); B * sind(delta)]; %magnetic field vector at 40 41' 30" N, 74 39' 12" W (uT)
    n = 3; %number of state dimensions
    nu = sqrt(n + lambda);
    R = [r_acc * eye(3), zeros(3); zeros(3), r_mag * eye(3)]; %measurement covariance
    
    %measurement data
    z = z_data;

    %state estimate
    x_post = zeros(3, sample);

    %covariance of state estimate
    P_post = zeros(3, 3, sample);

    for k = 1 : sample
        %low pass filter on accel data
        if(k > 1)
            z(1:3, k) = alpha * z(1:3, k-1) + (1 - alpha) * z(1:3, k);
        end

        %adapt motion model covariance
        if(abs(norm(z(1:3,k)) - norm(g)) < epsilon)
            q = q_0;
        else
            q = q_0 / (1 + zeta * abs(norm(z(1:3, k)) - norm(g)));
        end

        %predict
        if(k > 2)
            x_pri = x_post(:, k-1) + tau * (x_post(:, k-1) - x_post(:, k-2));
        elseif(k > 1)
            x_pri = x_post(:, k-1);
        else 
            x_pri = zeros(3, 1);
        end

        if(k > 1)
            P_pri = P_post(:, :, k-1) + q * eye(3);
        else
            P_pri = eye(3);
        end

        %Compute sigma points
        L = nu * chol(validateCovMatrix(P_pri)); %need to perform square root decomposition
        X = [x_pri, x_pri + L(:, 1), x_pri + L(:, 2), x_pri + L(:, 3), ...
             x_pri - L(:, 1), x_pri - L(:, 2), x_pri - L(:, 3)]; 
        Z = [myrotx(X(1, 1)) * myroty(X(2, 1)) * myrotz(X(3, 1)) * g, ...
             myrotx(X(1, 2)) * myroty(X(2, 2)) * myrotz(X(3, 2)) * g, ...
             myrotx(X(1, 3)) * myroty(X(2, 3)) * myrotz(X(3, 3)) * g, ...
             myrotx(X(1, 4)) * myroty(X(2, 4)) * myrotz(X(3, 4)) * g, ...
             myrotx(X(1, 5)) * myroty(X(2, 5)) * myrotz(X(3, 5)) * g, ...
             myrotx(X(1, 6)) * myroty(X(2, 6)) * myrotz(X(3, 6)) * g, ...
             myrotx(X(1, 7)) * myroty(X(2, 7)) * myrotz(X(3, 7)) * g;
             myrotx(X(1, 1)) * myroty(X(2, 1)) * myrotz(X(3, 1)) * b, ...
             myrotx(X(1, 2)) * myroty(X(2, 2)) * myrotz(X(3, 2)) * b, ...
             myrotx(X(1, 3)) * myroty(X(2, 3)) * myrotz(X(3, 3)) * b, ...
             myrotx(X(1, 4)) * myroty(X(2, 4)) * myrotz(X(3, 4)) * b, ...
             myrotx(X(1, 5)) * myroty(X(2, 5)) * myrotz(X(3, 5)) * b, ...
             myrotx(X(1, 6)) * myroty(X(2, 6)) * myrotz(X(3, 6)) * b, ...
             myrotx(X(1, 7)) * myroty(X(2, 7)) * myrotz(X(3, 7)) * b];

        %Predict Measurement using sigma points
        z_pri = zeros(6, 1); %predicted measurement data
        P_z = zeros(6); %covarinace of predicted measurement data
        P_x_z = zeros(3, 6);
        for ii = 1 : 2*n+1
            if(ii == 1)
                w = lambda / (n + lambda);
            else
                w = 1 / (2 * (n + lambda));
            end

            z_pri = z_pri + w * Z(:, ii);

            P_z = P_z + w * ((Z(:, ii) - z_pri) * (Z(:, ii) - z_pri)');

            P_x_z = P_x_z + w * ((X(:, ii) - x_pri) * (Z(:, ii) - z_pri)');
        end

        %Update with measurement data
        K = P_x_z / (P_z + R); %inverse is required

        x_post(:, k) = x_pri + K * (z(:, k) - z_pri);

        P_post(:, :, k) = P_pri - K * (P_z + R) * K';

        %final lowpass filter
        if(k > 1)
            x_post(:, k) = beta * x_post(:, k-1) + (1 - beta) * x_post(:, k);
        end
    end
    
    %make sure roll and yaw are between -180 and 180,
    %and pitch is between -90 and 90
    %N = [360*ones(1, sample); 180*ones(1, sample); 360*ones(1, sample)];
    %rem = x_post ./ N;
    %x_post = x_post - round(rem) .* N;
end