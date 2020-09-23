sample = 10000; %number of samples

z_data = acquire_data(sample);
[x_post, P_post] = UKF(sample, z_data);

roll = atan2d(z_data(2, :), z_data(3, :));
pitch = atand((-1 .* z_data(1, :)) ./ (z_data(2, :) .* sind(roll) + z_data(3, :) .* cosd(roll)));
yaw = atan2d((z_data(6, :) .* sind(roll) - z_data(5, :) .* cosd(roll)), (z_data(4, :) .* cosd(pitch) + z_data(5, :) .* sind(pitch) .* sind(roll) + z_data(6, :) .* sind(pitch) .* cosd(roll)));

t = [0.01:0.01:sample*0.01];

figure
plot(t, roll);
hold on
plot(t, x_post(1, :));
%legend('R raw', 'R filtered')
hold on
%figure
plot(t, pitch);
hold on
plot(t, x_post(2, :));
%legend('P raw', 'P filtered')
hold on
%figure
plot(t, yaw);
hold on
plot(t, x_post(3, :));
legend('R raw', 'R filtered', 'P raw', 'P filtered', 'Y raw', 'Y filtered')

figure
plot(t, squeeze(P_post(1, 1, :))');
hold on
plot(t, squeeze(P_post(2, 2, :))');
hold on
plot(t, squeeze(P_post(3, 3, :))');
legend('R cov', 'P cov', 'Y cov')