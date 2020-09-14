sample = 1000;

%measurement data
%z = acquire_data(sample);

figure
histogram(z(1, :));
figure
histogram(z(2, :));
figure
histogram(z(3, :));

figure
histogram(z(4, :));
figure
histogram(z(5, :));
figure
histogram(z(6, :));

figure
scatter(z(4, :), z(5, :), [], ones(1, sample));
hold on
scatter(z(4, :), z(6, :), [], 2*ones(1, sample));
hold on
scatter(z(5, :), z(6, :), [], 3*ones(1, sample));
legend('XY', 'XZ', 'YZ')
axis equal